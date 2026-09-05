from __future__ import annotations

import json
import math
import os
import shlex
import subprocess
import tempfile
from collections import deque
from pathlib import Path

import rclpy
from agt_robot_interfaces.msg import MapStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String, Header
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalRelocalization(Node):
    def __init__(self):
        super().__init__('agt_global_relocalization')
        p = self.declare_parameter
        p('scan_topic', '/agt/livox/points')
        p('request_topic', '/agt/relocalization/request')
        p('output_pose_topic', '/agt/relocalization/pose')
        p('status_topic', '/agt/global_relocalization/status')
        p('map_frame', 'map')
        p('query_frame', 'base_link')
        p('tf_timeout_sec', 0.10)
        p('follow_map_manager', True)
        p('map_status_topic', '/agt/map/status')
        p('sdk_command', '')
        p('candidate_sdk_command', '')
        p('global_map', '')
        p('relocalization_assets', '')
        p('backend_local_map_radius_xy', 35.0)
        p('backend_local_map_half_height', 8.0)
        p('backend_min_local_map_points', 800)
        p('work_dir', '~/.ros/agt_global_relocalization')
        p('sdk_timeout_sec', 10.0)
        p('accumulate_clouds', 5)
        p('min_points', 2000)
        p('max_points', 250000)
        p('query_min_range_m', 0.5)
        p('query_max_range_m', 30.0)
        p('query_voxel_leaf_m', 0.25)
        p('require_stationary', True)
        p('local_odom_topic', '/agt/odometry/local')
        p('odom_freshness_sec', 0.50)
        p('stationary_linear_threshold_mps', 0.05)
        p('stationary_angular_threshold_rps', 0.08)
        p('min_score', 0.50)
        p('max_fitness', 1.00)
        p('min_overlap', 0.20)
        p('best_position_std_m', 0.15)
        p('worst_position_std_m', 0.80)
        p('best_yaw_std_deg', 3.0)
        p('worst_yaw_std_deg', 15.0)
        p('auto_request', False)

        self.clouds = deque(maxlen=max(1, int(self.get_parameter('accumulate_clouds').value)))
        self.busy = False
        self.auto_requested = False
        self.auto_timer = None
        self.latest_odom = None
        self.latest_odom_rx_ns = 0
        self.latest_motion = None
        self.active_map_status = None
        self.last_query_raw_points = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('output_pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.query_pub = self.create_publisher(PointCloud2, '/agt/relocalization/query_cloud', 10)
        self.coarse_cloud_pub = self.create_publisher(PointCloud2, '/agt/relocalization/coarse_aligned_cloud', 10)
        self.aligned_cloud_pub = self.create_publisher(PointCloud2, '/agt/relocalization/aligned_cloud', 10)
        self.coarse_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/agt/relocalization/coarse_pose', 10)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self.on_odom, 50)
        self.create_subscription(Empty, self.get_parameter('request_topic').value, self.on_request, 10)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            MapStatus,
            self.get_parameter('map_status_topic').value,
            self.on_map_status,
            map_qos,
        )

    def on_cloud(self, msg):
        self.clouds.append(msg)
        if (bool(self.get_parameter('auto_request').value)
                and not self.auto_requested
                and len(self.clouds) >= int(self.get_parameter('accumulate_clouds').value)):
            self.auto_requested = True
            self.auto_timer = self.create_timer(0.5, self._auto_request)

    def _auto_request(self):
        if self.clouds and not self.busy:
            self.auto_timer.cancel()
            self.on_request(Empty())

    def _rearm_auto_request(self, clear_clouds=True):
        """Allow a fresh automatic attempt after any precondition/backend failure.

        The auto timer is one-shot in practice: _auto_request() cancels it before
        calling on_request(). Therefore every early return must clear the
        auto_requested latch, otherwise a transient moving/stale/no-cloud gate
        permanently disables automatic relocalization until node restart.
        """
        if self.auto_timer is not None:
            self.auto_timer.cancel()
            self.auto_timer = None
        if clear_clouds:
            self.clouds.clear()
        self.auto_requested = False

    def on_map_status(self, msg):
        previous_generation = int(self.active_map_status.generation) if self.active_map_status else -1
        self.active_map_status = msg
        if msg.active and int(msg.generation) != previous_generation:
            self._rearm_auto_request(clear_clouds=True)
            self.get_logger().info(
                f'Following active map {msg.map_id}/{msg.map_version} generation={msg.generation}')

    @staticmethod
    def motion_metrics(msg):
        t = msg.twist.twist
        linear = math.sqrt(t.linear.x*t.linear.x + t.linear.y*t.linear.y + t.linear.z*t.linear.z)
        angular = math.sqrt(t.angular.x*t.angular.x + t.angular.y*t.angular.y + t.angular.z*t.angular.z)
        return linear, angular

    @staticmethod
    def pose_delta_motion(previous, current):
        if previous is None:
            return None
        prev_ns = int(previous.header.stamp.sec) * 1_000_000_000 + int(previous.header.stamp.nanosec)
        curr_ns = int(current.header.stamp.sec) * 1_000_000_000 + int(current.header.stamp.nanosec)
        dt = (curr_ns - prev_ns) / 1e9
        if dt <= 1e-4 or dt > 1.0:
            return None
        p0, p1 = previous.pose.pose.position, current.pose.pose.position
        dx, dy, dz = p1.x-p0.x, p1.y-p0.y, p1.z-p0.z
        linear = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
        q0, q1 = previous.pose.pose.orientation, current.pose.pose.orientation
        a = (q0.x,q0.y,q0.z,q0.w)
        b = (q1.x,q1.y,q1.z,q1.w)
        na = math.sqrt(sum(v*v for v in a))
        nb = math.sqrt(sum(v*v for v in b))
        if na <= 1e-12 or nb <= 1e-12:
            return None
        dot = abs(sum(x*y for x,y in zip(a,b)) / (na*nb))
        dot = min(1.0, max(-1.0, dot))
        return linear, 2.0 * math.acos(dot) / dt

    def on_odom(self, msg):
        previous = self.latest_odom
        first_odom = previous is None
        pose_motion = self.pose_delta_motion(previous, msg)
        twist_motion = self.motion_metrics(msg)
        self.latest_odom = msg
        self.latest_odom_rx_ns = self.get_clock().now().nanoseconds
        if pose_motion is None:
            self.latest_motion = None
        else:
            self.latest_motion = (max(twist_motion[0], pose_motion[0]),
                                  max(twist_motion[1], pose_motion[1]))
        if bool(self.get_parameter('require_stationary').value):
            moving = self.latest_motion is None or (
                self.latest_motion[0] > float(self.get_parameter('stationary_linear_threshold_mps').value)
                or self.latest_motion[1] > float(self.get_parameter('stationary_angular_threshold_rps').value)
            )
            if first_odom or moving:
                self.clouds.clear()

    def status(self, state, detail='', **extra):
        m = String()
        payload = {'state': state, 'detail': detail}
        payload.update(extra)
        m.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(m)
        self.get_logger().info(m.data)

    def stationary_gate(self):
        if not bool(self.get_parameter('require_stationary').value):
            return True, 'stationary gate disabled'
        if self.latest_odom is None or self.latest_odom_rx_ns <= 0:
            return False, 'no local odometry available for stationary gate'
        age = (self.get_clock().now().nanoseconds - self.latest_odom_rx_ns) / 1e9
        if age > float(self.get_parameter('odom_freshness_sec').value):
            return False, f'local odometry stale: {age:.3f}s'
        if self.latest_motion is None:
            return False, 'local odometry motion estimate is not ready'
        linear, angular = self.latest_motion
        if linear > float(self.get_parameter('stationary_linear_threshold_mps').value):
            return False, f'robot moving: linear={linear:.3f}m/s'
        if angular > float(self.get_parameter('stationary_angular_threshold_rps').value):
            return False, f'robot rotating: angular={angular:.3f}rad/s'
        return True, f'stationary linear={linear:.3f}m/s angular={angular:.3f}rad/s'

    def on_request(self, _msg):
        if self.busy:
            self.status('BUSY', 'relocalization already running')
            return
        if not self.clouds:
            self.status('FAILED', 'no stationary PointCloud2 scan available yet')
            if bool(self.get_parameter('auto_request').value):
                self._rearm_auto_request(clear_clouds=True)
            return
        stationary, detail = self.stationary_gate()
        if not stationary:
            self.status('FAILED', detail)
            if bool(self.get_parameter('auto_request').value):
                self._rearm_auto_request(clear_clouds=True)
            return
        self.busy = True
        succeeded = False
        try:
            self.status('QUERY_READY', 'stationary query is ready')
            self.run_once()
            succeeded = True
        except Exception as exc:
            self.status('REJECTED', str(exc))
        finally:
            self.busy = False
            if bool(self.get_parameter('auto_request').value) and not succeeded:
                self._rearm_auto_request(clear_clouds=True)

    @staticmethod
    def transform_xyz(x, y, z, transform):
        tr = transform.transform.translation
        qr = transform.transform.rotation
        qx, qy, qz, qw = qr.x, qr.y, qr.z, qr.w
        qn = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if qn < 1e-12:
            raise RuntimeError('invalid zero TF quaternion')
        qx, qy, qz, qw = qx/qn, qy/qn, qz/qn, qw/qn
        tx = 2.0 * (qy*z - qz*y)
        ty = 2.0 * (qz*x - qx*z)
        tz = 2.0 * (qx*y - qy*x)
        rx = x + qw*tx + (qy*tz - qz*ty)
        ry = y + qw*ty + (qz*tx - qx*tz)
        rz = z + qw*tz + (qx*ty - qy*tx)
        return rx + tr.x, ry + tr.y, rz + tr.z

    def merged_points(self):
        rows = []
        max_points = int(self.get_parameter('max_points').value)
        min_range = float(self.get_parameter('query_min_range_m').value)
        max_range = float(self.get_parameter('query_max_range_m').value)
        min_range_sq = min_range * min_range
        max_range_sq = max_range * max_range
        voxel_leaf = float(self.get_parameter('query_voxel_leaf_m').value)
        query_frame = str(self.get_parameter('query_frame').value)
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))

        def finalize():
            self.last_query_raw_points = len(rows)
            if voxel_leaf <= 0.0:
                return rows
            voxels = {}
            for row in rows:
                key = (
                    math.floor(row[0] / voxel_leaf),
                    math.floor(row[1] / voxel_leaf),
                    math.floor(row[2] / voxel_leaf),
                )
                if key not in voxels:
                    voxels[key] = row
            return list(voxels.values())

        for cloud in list(self.clouds):
            if not cloud.header.frame_id:
                raise RuntimeError('relocalization scan has empty frame_id')
            try:
                transform = self.tf_buffer.lookup_transform(
                    query_frame,
                    cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                    timeout=timeout,
                )
            except TransformException as exc:
                raise RuntimeError(
                    f'cannot transform relocalization scan {cloud.header.frame_id} -> {query_frame}: {exc}'
                ) from exc

            names = [f.name for f in cloud.fields]
            requested = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
            for pt in point_cloud2.read_points(cloud, field_names=requested, skip_nans=True):
                x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                intensity = float(pt[3]) if len(pt) > 3 else 0.0
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    range_sq = x*x + y*y + z*z
                    if range_sq < min_range_sq or range_sq > max_range_sq:
                        continue
                    x, y, z = self.transform_xyz(x, y, z, transform)
                    rows.append((x, y, z, intensity))
                if len(rows) >= max_points:
                    return finalize()
        return finalize()

    @staticmethod
    def cloud_message(rows, stamp, frame):
        fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        header = Header()
        header.stamp = stamp
        header.frame_id = frame
        return point_cloud2.create_cloud(header, fields, rows)

    @staticmethod
    def apply_pose(rows, xyz, q):
        qx, qy, qz, qw = q
        out = []
        for x, y, z, intensity in rows:
            tx = 2.0 * (qy*z - qz*y); ty = 2.0 * (qz*x - qx*z); tz = 2.0 * (qx*y - qy*x)
            out.append((x + qw*tx + (qy*tz - qz*ty) + xyz[0],
                        y + qw*ty + (qz*tx - qx*tz) + xyz[1],
                        z + qw*tz + (qx*ty - qy*tx) + xyz[2], intensity))
        return out

    @staticmethod
    def write_ascii_pcd(path: Path, rows):
        with path.open('w', encoding='utf-8') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n')
            f.write('TYPE F F F F\nCOUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(rows)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(rows)}\nDATA ascii\n')
            for x, y, z, intensity in rows:
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {intensity:.3f}\n')

    def resolve_map_inputs(self):
        global_map = os.path.expanduser(str(self.get_parameter('global_map').value)).strip()
        assets_dir = os.path.expanduser(str(self.get_parameter('relocalization_assets').value)).strip()
        map_id = ''
        map_version = ''
        generation = 0

        if bool(self.get_parameter('follow_map_manager').value):
            msg = self.active_map_status
            if msg is not None and msg.active:
                if msg.localization_map_pcd:
                    global_map = msg.localization_map_pcd
                if msg.relocalization_assets_path:
                    assets_dir = msg.relocalization_assets_path
                map_id = msg.map_id
                map_version = msg.map_version
                generation = int(msg.generation)

        return global_map, assets_dir, map_id, map_version, generation

    def verify_map_snapshot(self, map_id, map_version, generation):
        if not bool(self.get_parameter('follow_map_manager').value):
            return
        if generation <= 0:
            return
        current = self.active_map_status
        if current is None or not current.active:
            raise RuntimeError('active map disappeared during relocalization; result discarded')
        if (
            int(current.generation) != generation
            or current.map_id != map_id
            or current.map_version != map_version
        ):
            raise RuntimeError(
                'active map changed during relocalization; old-map result discarded '
                f'(started={map_id}/{map_version}@{generation}, '
                f'current={current.map_id}/{current.map_version}@{int(current.generation)})'
            )

    def run_once(self):
        rows = self.merged_points()
        min_points = int(self.get_parameter('min_points').value)
        if len(rows) < min_points:
            raise RuntimeError(f'not enough scan points: {len(rows)} < {min_points}')
        stamp = self.clouds[-1].header.stamp
        self.query_pub.publish(self.cloud_message(
            rows, stamp, str(self.get_parameter('query_frame').value)))

        global_map, assets_dir, map_id, map_version, generation = self.resolve_map_inputs()
        fallback_command = str(self.get_parameter('sdk_command').value).strip()
        candidate_command = str(self.get_parameter('candidate_sdk_command').value).strip()
        candidate_db = Path(assets_dir) / 'polar_context.db' if assets_dir else None
        use_candidate_backend = bool(candidate_command and candidate_db and candidate_db.is_file())
        command_template = candidate_command if use_candidate_backend else fallback_command
        backend_name = 'polar_context_bbs_gicp' if use_candidate_backend else 'whole_map_bbs_gicp'
        if not command_template:
            raise RuntimeError('relocalization backend command is empty')
        if not global_map or not Path(global_map).is_file():
            raise RuntimeError(f'global_map not found: {global_map!r}')
        if assets_dir and not Path(assets_dir).is_dir():
            raise RuntimeError(f'relocalization_assets directory not found: {assets_dir!r}')

        work = Path(os.path.expanduser(str(self.get_parameter('work_dir').value)))
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='query_', dir=work) as td:
            scan_pcd = Path(td) / 'query_scan.pcd'
            self.write_ascii_pcd(scan_pcd, rows)
            timeout = float(self.get_parameter('sdk_timeout_sec').value)
            assets_arg = f'--assets-dir {shlex.quote(assets_dir)}' if assets_dir else ''
            cmd = command_template.format(
                scan_pcd=str(scan_pcd),
                global_map=global_map,
                timeout_sec=timeout,
                assets_arg=assets_arg,
                local_map_radius_xy=float(self.get_parameter('backend_local_map_radius_xy').value),
                local_map_half_height=float(self.get_parameter('backend_local_map_half_height').value),
                min_local_map_points=int(self.get_parameter('backend_min_local_map_points').value),
            )
            self.status(
                'BBS_SEARCHING',
                'calling 3D relocalization backend',
                points=len(rows),
                raw_points=self.last_query_raw_points,
                query_min_range_m=float(self.get_parameter('query_min_range_m').value),
                query_max_range_m=float(self.get_parameter('query_max_range_m').value),
                query_voxel_leaf_m=float(self.get_parameter('query_voxel_leaf_m').value),
                query_frame=self.get_parameter('query_frame').value,
                assets=bool(assets_dir),
                backend=backend_name,
                map_id=map_id,
                map_version=map_version,
                map_generation=generation,
            )
            proc = subprocess.run(shlex.split(cmd), capture_output=True, text=True, timeout=timeout, check=False)
            if proc.returncode != 0:
                detail = proc.stderr.strip() or proc.stdout.strip() or 'no backend diagnostics'
                raise RuntimeError(f'backend returned {proc.returncode}: {detail}')
            try:
                result = json.loads(proc.stdout.strip().splitlines()[-1])
            except Exception as exc:
                raise RuntimeError(f'backend stdout must end with JSON result: {exc}') from exc

        # A map switch invalidates the semantic frame of the pose even when the
        # backend computation itself succeeded. Never publish an old-map pose.
        self.verify_map_snapshot(map_id, map_version, generation)

        if not bool(result.get('success', False)):
            raise RuntimeError(str(result.get('message', 'backend reported failure')))
        self.status('BBS_COARSE_FOUND', 'BBS coarse pose found')
        score = float(result.get('score', 0.0))
        fitness = float(result.get('fitness', math.inf))
        overlap = float(result.get('overlap', 0.0))
        if score < float(self.get_parameter('min_score').value):
            raise RuntimeError(f'score gate failed: {score:.3f}')
        if fitness > float(self.get_parameter('max_fitness').value):
            raise RuntimeError(f'fitness gate failed: {fitness:.3f}')
        if overlap < float(self.get_parameter('min_overlap').value):
            raise RuntimeError(f'overlap gate failed: {overlap:.3f}')

        q = [float(result[k]) for k in ('qx', 'qy', 'qz', 'qw')]
        qn = math.sqrt(sum(v*v for v in q))
        if qn < 1e-9:
            raise RuntimeError('backend returned invalid quaternion')
        q = [v / qn for v in q]
        cq = [float(result[k]) for k in ('coarse_qx', 'coarse_qy', 'coarse_qz', 'coarse_qw')]
        cp = [float(result.get(k, 0.0)) for k in ('coarse_x', 'coarse_y', 'coarse_z')]
        coarse_pose = PoseWithCovarianceStamped()
        coarse_pose.header.stamp = stamp
        coarse_pose.header.frame_id = str(self.get_parameter('map_frame').value)
        coarse_pose.pose.pose.position.x, coarse_pose.pose.pose.position.y, coarse_pose.pose.pose.position.z = cp
        coarse_pose.pose.pose.orientation.x, coarse_pose.pose.pose.orientation.y, coarse_pose.pose.pose.orientation.z, coarse_pose.pose.pose.orientation.w = cq
        self.coarse_pose_pub.publish(coarse_pose)
        self.coarse_cloud_pub.publish(self.cloud_message(
            self.apply_pose(rows, cp, cq), stamp, str(self.get_parameter('map_frame').value)))
        self.status('GICP_REFINING', 'GICP refinement complete; publishing refined pose')

        score01 = min(1.0, max(0.0, score))
        pos_std = self._lerp('worst_position_std_m', 'best_position_std_m', score01)
        yaw_std = math.radians(self._lerp('worst_yaw_std_deg', 'best_yaw_std_deg', score01))

        # Re-check immediately before publication in case a MapStatus callback
        # arrived while the result was passing quality gates.
        self.verify_map_snapshot(map_id, map_version, generation)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.clouds[-1].header.stamp
        msg.header.frame_id = str(self.get_parameter('map_frame').value)
        msg.pose.pose.position.x = float(result['x'])
        msg.pose.pose.position.y = float(result['y'])
        msg.pose.pose.position.z = float(result.get('z', 0.0))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = pos_std * pos_std
        cov[21] = cov[28] = math.radians(10.0) ** 2
        cov[35] = yaw_std * yaw_std
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        self.aligned_cloud_pub.publish(self.cloud_message(
            self.apply_pose(rows, [float(result['x']), float(result['y']), float(result['z'])], q),
            stamp, str(self.get_parameter('map_frame').value)))
        self.status(
            'SUCCEEDED',
            'global base pose published',
            score=score,
            fitness=fitness,
            overlap=overlap,
            position_std_m=pos_std,
            yaw_std_deg=math.degrees(yaw_std),
            map_id=map_id,
            map_version=map_version,
            map_generation=generation,
            bbs_elapsed_ms=result.get('bbs_elapsed_ms'),
            backend=backend_name,
            candidate_patch=result.get('candidate_patch'),
            descriptor_similarity=result.get('descriptor_similarity'),
            descriptor_yaw_seed_deg=result.get('descriptor_yaw_seed_deg'),
            bbs_assets_loaded=result.get('bbs_assets_loaded'),
            gicp_target_points=result.get('gicp_target_points'),
            gicp_full_map_fallback=result.get('gicp_full_map_fallback'),
        )

    def _lerp(self, low_name, high_name, t):
        low = float(self.get_parameter(low_name).value)
        high = float(self.get_parameter(high_name).value)
        return low + (high - low) * t


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRelocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
