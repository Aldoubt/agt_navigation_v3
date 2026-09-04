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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String
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
        p('sdk_command', '')
        p('global_map', '')
        p('work_dir', '~/.ros/agt_global_relocalization')
        p('sdk_timeout_sec', 10.0)
        p('accumulate_clouds', 5)
        p('min_points', 2000)
        p('max_points', 250000)
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

        self.clouds = deque(maxlen=max(1, int(self.get_parameter('accumulate_clouds').value)))
        self.busy = False
        self.latest_odom = None
        self.latest_odom_rx_ns = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('output_pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self.on_cloud, 10)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self.on_odom, 50)
        self.create_subscription(Empty, self.get_parameter('request_topic').value, self.on_request, 10)

    def on_cloud(self, msg):
        self.clouds.append(msg)

    def on_odom(self, msg):
        self.latest_odom = msg
        self.latest_odom_rx_ns = self.get_clock().now().nanoseconds

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
        t = self.latest_odom.twist.twist
        linear = math.sqrt(t.linear.x*t.linear.x + t.linear.y*t.linear.y + t.linear.z*t.linear.z)
        angular = math.sqrt(t.angular.x*t.angular.x + t.angular.y*t.angular.y + t.angular.z*t.angular.z)
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
            self.status('FAILED', 'no PointCloud2 scan available')
            return
        stationary, detail = self.stationary_gate()
        if not stationary:
            self.status('FAILED', detail)
            return
        self.busy = True
        try:
            self.run_once()
        except Exception as exc:
            self.status('FAILED', str(exc))
        finally:
            self.busy = False

    @staticmethod
    def transform_xyz(x, y, z, transform):
        tr = transform.transform.translation
        qr = transform.transform.rotation
        qx, qy, qz, qw = qr.x, qr.y, qr.z, qr.w
        qn = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if qn < 1e-12:
            raise RuntimeError('invalid zero TF quaternion')
        qx, qy, qz, qw = qx/qn, qy/qn, qz/qn, qw/qn
        # Quaternion rotation expanded to avoid another point-cloud conversion dependency.
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
        query_frame = str(self.get_parameter('query_frame').value)
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
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
                    x, y, z = self.transform_xyz(x, y, z, transform)
                    rows.append((x, y, z, intensity))
                if len(rows) >= max_points:
                    return rows
        return rows

    @staticmethod
    def write_ascii_pcd(path: Path, rows):
        with path.open('w', encoding='utf-8') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n')
            f.write('TYPE F F F F\nCOUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(rows)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(rows)}\nDATA ascii\n')
            for x, y, z, intensity in rows:
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {intensity:.3f}\n')

    def run_once(self):
        rows = self.merged_points()
        min_points = int(self.get_parameter('min_points').value)
        if len(rows) < min_points:
            raise RuntimeError(f'not enough scan points: {len(rows)} < {min_points}')

        command_template = str(self.get_parameter('sdk_command').value).strip()
        global_map = os.path.expanduser(str(self.get_parameter('global_map').value))
        if not command_template:
            raise RuntimeError('relocalization backend command is empty')
        if not global_map or not Path(global_map).is_file():
            raise RuntimeError(f'global_map not found: {global_map!r}')

        work = Path(os.path.expanduser(str(self.get_parameter('work_dir').value)))
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='query_', dir=work) as td:
            scan_pcd = Path(td) / 'query_scan.pcd'
            self.write_ascii_pcd(scan_pcd, rows)
            timeout = float(self.get_parameter('sdk_timeout_sec').value)
            cmd = command_template.format(scan_pcd=str(scan_pcd), global_map=global_map, timeout_sec=timeout)
            self.status('RUNNING', 'calling 3D relocalization backend', points=len(rows), query_frame=self.get_parameter('query_frame').value)
            proc = subprocess.run(shlex.split(cmd), capture_output=True, text=True, timeout=timeout, check=False)
            if proc.returncode != 0:
                raise RuntimeError(f'backend returned {proc.returncode}: {proc.stderr.strip()}')
            try:
                result = json.loads(proc.stdout.strip().splitlines()[-1])
            except Exception as exc:
                raise RuntimeError(f'backend stdout must end with JSON result: {exc}') from exc

        if not bool(result.get('success', False)):
            raise RuntimeError(str(result.get('message', 'backend reported failure')))
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

        score01 = min(1.0, max(0.0, score))
        pos_std = self._lerp('worst_position_std_m', 'best_position_std_m', score01)
        yaw_std = math.radians(self._lerp('worst_yaw_std_deg', 'best_yaw_std_deg', score01))

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
        self.status('SUCCEEDED', 'global base pose published', score=score, fitness=fitness, overlap=overlap,
                    position_std_m=pos_std, yaw_std_deg=math.degrees(yaw_std))

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
