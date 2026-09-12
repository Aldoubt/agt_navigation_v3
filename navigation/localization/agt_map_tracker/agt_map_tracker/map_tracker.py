"""Low-rate local-map tracking without taking TF ownership."""

from __future__ import annotations

import json
import math
import os
import subprocess
import tempfile
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from agt_robot_interfaces.msg import LocalizationStatus


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=float)


def transform_matrix(t):
    m = np.eye(4)
    m[:3, :3] = quat_matrix((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
    m[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return m


def matrix_pose(m):
    # Stable rotation-matrix to quaternion conversion, xyzw.
    tr = float(np.trace(m[:3, :3]))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(m[:3, :3])))
        if i == 0:
            s = math.sqrt(max(1e-12, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            q = [(s / 4.0), (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
        elif i == 1:
            s = math.sqrt(max(1e-12, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            q = [(m[0, 1] + m[1, 0]) / s, s / 4.0, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
        else:
            s = math.sqrt(max(1e-12, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
            q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4.0, (m[1, 0] - m[0, 1]) / s]
        qx, qy, qz, qw = q
    return m[:3, 3], (qx, qy, qz, qw)


@dataclass
class OdomSample:
    stamp_ns: int
    pose: np.ndarray


@dataclass
class CloudSample:
    stamp_ns: int
    points_base: np.ndarray
    odom_pose: np.ndarray


class MapTracker(Node):
    def __init__(self):
        super().__init__('agt_map_tracker')
        self.declare_parameter('scan_topic', '/agt/livox/points')
        self.declare_parameter('local_odom_topic', '/agt/odometry/local')
        self.declare_parameter('localization_status_topic', '/agt/localization/status')
        self.declare_parameter('output_pose_topic', '/agt/map_tracking/pose')
        self.declare_parameter('status_topic', '/agt/map_tracking/status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_map', '')
        for name, default in {
            'tracking_rate_hz': 0.5, 'local_map_radius_xy_m': 20.0,
            'local_map_half_height_m': 5.0, 'query_accumulate_clouds': 3,
            'query_voxel_leaf_m': 0.25, 'max_points': 150000,
            'min_query_points': 1500, 'min_local_map_points': 1000,
            'max_fitness': 0.50, 'min_overlap': 0.40,
            'max_translation_innovation_m': 0.50,
            'max_yaw_innovation_deg': 5.0, 'odom_match_max_skew_sec': 0.10,
            'tf_timeout_sec': 0.10, 'native_timeout_sec': 8.0,
            'degraded_after_rejects': 2, 'recovery_after_rejects': 4,
            'reject_degenerate_hessian': False,
            'max_hessian_condition_number': 1.0e10,
        }.items():
            self.declare_parameter(name, default)
        self._odom = deque(maxlen=6000)
        self._clouds = deque(maxlen=20)
        self._localized = False
        self._correction_valid = False
        self._last_run_ns = 0
        self._consecutive_rejects = 0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('output_pose_topic').value, 10)
        self._status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self._on_odom, 100)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self._on_cloud, 10)
        self.create_subscription(LocalizationStatus, self.get_parameter('localization_status_topic').value, self._on_status, 10)
        hz = max(0.05, float(self.get_parameter('tracking_rate_hz').value))
        self.create_timer(1.0 / hz, self._track)

    @staticmethod
    def _stamp(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_odom(self, msg):
        if msg.header.frame_id != self.get_parameter('odom_frame').value or msg.child_frame_id != self.get_parameter('base_frame').value:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        m = np.eye(4)
        m[:3, :3] = quat_matrix((q.x, q.y, q.z, q.w))
        m[:3, 3] = (p.x, p.y, p.z)
        self._odom.append(OdomSample(self._stamp(msg), m))

    def _nearest_odom(self, stamp_ns: int) -> Optional[OdomSample]:
        if not self._odom:
            return None
        sample = min(self._odom, key=lambda x: abs(x.stamp_ns - stamp_ns))
        if abs(sample.stamp_ns - stamp_ns) > float(self.get_parameter('odom_match_max_skew_sec').value) * 1e9:
            return None
        return sample

    def _on_status(self, msg):
        self._localized = msg.state in (LocalizationStatus.STATE_LOCALIZED,
                                        LocalizationStatus.STATE_DEGRADED)
        self._correction_valid = bool(msg.global_correction_valid)

    def _on_cloud(self, msg):
        odom = self._nearest_odom(self._stamp(msg))
        if odom is None:
            return
        try:
            tf = self._tf.lookup_transform(
                self.get_parameter('base_frame').value, msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
        except TransformException:
            return
        rows = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = (float(p[0]), float(p[1]), float(p[2]))
            if 0.5 <= math.sqrt(x*x + y*y + z*z) <= 30.0:
                rows.append((x, y, z))
        if not rows:
            return
        points = np.asarray(rows, dtype=float)
        sensor_to_base = transform_matrix(tf.transform)
        points = (sensor_to_base[:3, :3] @ points.T).T + sensor_to_base[:3, 3]
        self._clouds.append(CloudSample(self._stamp(msg), points, odom.pose))

    def _write_pcd(self, points: np.ndarray, path: Path):
        points = points[:int(self.get_parameter('max_points').value)]
        with path.open('w', encoding='ascii') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
            f.write('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')

    def _record_reject(self, reason, **fields):
        self._consecutive_rejects += 1
        degraded_after = max(1, int(self.get_parameter('degraded_after_rejects').value))
        recovery_after = max(degraded_after + 1, int(self.get_parameter('recovery_after_rejects').value))
        if self._consecutive_rejects >= recovery_after:
            state = 'RECOVERY_REQUIRED'
        elif self._consecutive_rejects >= degraded_after:
            state = 'DEGRADED'
        else:
            state = 'HOLD'
        self._publish_status(
            state,
            reason=reason,
            consecutive_rejects=self._consecutive_rejects,
            **fields,
        )

    def _record_accept(self, **fields):
        self._consecutive_rejects = 0
        self._publish_status('TRACKING_OK', consecutive_rejects=0, **fields)

    def _track(self):
        if not (self._localized and self._correction_valid):
            self._publish_status('WAIT_LOCALIZED', reason='manager_not_localized')
            return
        if not self._clouds:
            self._publish_status('WAIT_QUERY', reason='no_cloud')
            return
        ref = self._clouds[-1]
        query = []
        inv_ref = np.linalg.inv(ref.odom_pose)
        for item in list(self._clouds)[-int(self.get_parameter('query_accumulate_clouds').value):]:
            relative = inv_ref @ item.odom_pose
            pts = (relative[:3, :3] @ item.points_base.T).T + relative[:3, 3]
            query.append(pts)
        query = np.concatenate(query, axis=0)
        query = self._voxel(query, float(self.get_parameter('query_voxel_leaf_m').value))
        if len(query) < int(self.get_parameter('min_query_points').value):
            self._record_reject('min_query_points', query_points=len(query))
            return
        try:
            map_odom = transform_matrix(self._tf.lookup_transform(
                self.get_parameter('map_frame').value, self.get_parameter('odom_frame').value,
                rclpy.time.Time(), timeout=Duration(seconds=0.2)).transform)
        except TransformException:
            self._publish_status('WAIT_LOCALIZED', reason='map_odom_unavailable')
            return
        predicted = map_odom @ ref.odom_pose
        p, q = matrix_pose(predicted)
        map_path = str(self.get_parameter('global_map').value)
        if not map_path or not os.path.isfile(map_path):
            self._record_reject('global_map_missing')
            return
        with tempfile.TemporaryDirectory(prefix='agt_map_tracker_') as work:
            scan_path = Path(work) / 'query.pcd'
            self._write_pcd(query, scan_path)
            cmd = ['ros2', 'run', 'agt_global_relocalization_native', 'map_gicp_tracker',
                   '--map', map_path, '--scan', str(scan_path), '--x', str(p[0]), '--y', str(p[1]), '--z', str(p[2]),
                   '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                   '--radius', str(self.get_parameter('local_map_radius_xy_m').value),
                   '--half-height', str(self.get_parameter('local_map_half_height_m').value)]
            try:
                result = subprocess.run(cmd, check=False, capture_output=True, text=True,
                                        timeout=float(self.get_parameter('native_timeout_sec').value))
                data = json.loads(result.stdout.strip().splitlines()[-1])
            except (OSError, subprocess.SubprocessError, ValueError, IndexError) as exc:
                self._record_reject(f'native_tracker:{exc}')
                return
        if not data.get('success'):
            self._record_reject(str(data.get('message', 'gicp_failed')))
            return
        fitness = float(data.get('fitness', math.inf))
        overlap = float(data.get('overlap', 0.0))
        hessian_condition = float(data.get('hessian_condition_number', math.inf))
        hessian_degenerate = bool(data.get('hessian_degenerate', False))
        if (bool(self.get_parameter('reject_degenerate_hessian').value)
                and (hessian_degenerate
                     or hessian_condition > float(
                         self.get_parameter('max_hessian_condition_number').value))):
            self._record_reject(
                'degenerate_geometry',
                fitness=fitness,
                overlap=overlap,
                hessian_condition_number=hessian_condition,
                hessian_degenerate=hessian_degenerate,
            )
            return
        if fitness > float(self.get_parameter('max_fitness').value) or overlap < float(self.get_parameter('min_overlap').value):
            self._record_reject('quality_gate', fitness=fitness, overlap=overlap)
            return
        measured = np.eye(4)
        measured[:3, :3] = quat_matrix((float(data['qx']), float(data['qy']), float(data['qz']), float(data['qw'])))
        measured[:3, 3] = (float(data['x']), float(data['y']), float(data['z']))
        translation_innovation = float(np.linalg.norm(measured[:3, 3] - predicted[:3, 3]))
        def yaw_of(matrix):
            return math.atan2(matrix[1, 0], matrix[0, 0])
        yaw_innovation = abs((yaw_of(measured) - yaw_of(predicted) + math.pi) % (2.0 * math.pi) - math.pi)
        if (translation_innovation > float(self.get_parameter('max_translation_innovation_m').value)
                or yaw_innovation > math.radians(float(self.get_parameter('max_yaw_innovation_deg').value))):
            self._record_reject('innovation_gate', fitness=fitness,
                                overlap=overlap, translation_innovation_m=translation_innovation,
                                yaw_innovation_deg=math.degrees(yaw_innovation))
            return
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.get_parameter('map_frame').value
        pose_msg.header.stamp = rclpy.time.Time(nanoseconds=ref.stamp_ns).to_msg()
        pose_msg.pose.pose.position.x = float(data['x']); pose_msg.pose.pose.position.y = float(data['y']); pose_msg.pose.pose.position.z = float(data['z'])
        pose_msg.pose.pose.orientation.x = float(data['qx']); pose_msg.pose.pose.orientation.y = float(data['qy'])
        pose_msg.pose.pose.orientation.z = float(data['qz']); pose_msg.pose.pose.orientation.w = float(data['qw'])
        pose_msg.pose.covariance[0] = max(0.01, fitness)
        pose_msg.pose.covariance[7] = max(0.01, fitness)
        pose_msg.pose.covariance[14] = max(0.01, fitness)
        pose_msg.pose.covariance[35] = max(0.001, fitness / max(overlap, 0.01))
        self._pose_pub.publish(pose_msg)
        self._record_accept(
            fitness=fitness,
            overlap=overlap,
            query_points=len(query),
            map_points=int(data.get('map_points', 0)),
            translation_innovation_m=translation_innovation,
            yaw_innovation_deg=math.degrees(yaw_innovation),
            hessian_condition_number=float(data.get('hessian_condition_number', math.inf)),
            hessian_degenerate=bool(data.get('hessian_degenerate', False)),
        )

    @staticmethod
    def _voxel(points, leaf):
        if len(points) == 0 or leaf <= 0.0:
            return points
        keys = np.floor(points / leaf).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(indices)]

    def _publish_status(self, state, **fields):
        data = {'state': state, **fields}
        msg = String(); msg.data = json.dumps(data, sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
