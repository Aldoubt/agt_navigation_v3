"""Low-rate local-map tracking without taking TF ownership."""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import tempfile
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from agt_robot_interfaces.msg import LocalizationStatus
from agt_navigation_interfaces.msg import GlobalQuality, OdomQuality
from agt_global_relocalization.query_job import CloudSnapshot, file_identity, hash_file, run_command


def quat_matrix(q):
    norm = math.sqrt(sum(v*v for v in q))
    if not all(math.isfinite(v) for v in q) or norm <= 1.0e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = (v / norm for v in q)
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


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    pose: np.ndarray


@dataclass(frozen=True)
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
            'tracking_rate_hz': 0.5, 'backend_threads': 2, 'local_map_radius_xy_m': 20.0,
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
            'apply_correction': True,
            'shadow_mode': False, 'quality_calibrated': False,
            'quality_topic': '/agt/localization/global_observation',
            'odom_quality_topic': '/agt/odometry/quality',
            'map_id': '', 'map_version': '', 'map_hash': '',
            'max_result_age_sec': 2.0, 'deskew_max_odom_gap_sec': 0.05,
            'max_point_offset_sec': 0.20, 'max_normalized_hessian_condition': 1.0e10,
            'save_debug_cloud': False,
            'debug_directory': '~/.ros/agt_map_tracker_debug',
            'max_debug_failure_samples': 20,
        }.items():
            self.declare_parameter(name, default)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='agt_map_tracker')
        self._preprocessor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='agt_tracker_deskew')
        self._cloud_future = None
        self._cloud_context = None
        self._future = None
        self._context = None
        self._cancel = threading.Event()
        self._odom_epoch = 1
        self._odom_quality = None
        self._odom_quality_rx_ns = 0
        self._raw_clouds = deque(maxlen=20)
        self._map_hash_verified = False
        self._odom = deque(maxlen=6000)
        self._clouds = deque(maxlen=20)
        self._localized = False
        self._correction_valid = False
        self._last_run_ns = 0
        self._consecutive_rejects = 0
        # Diagnostics only: these counters make missing tracker attempts
        # distinguishable from an actual GICP rejection.  They deliberately do
        # not participate in the tracking state machine.
        self._attempt_id = 0
        self._cloud_time_sync_errors = 0
        self._cloud_tf_errors = 0
        self._debug_failure_count = 0
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        shadow = bool(self.get_parameter('shadow_mode').value)
        pose_topic = '/agt/map_tracking/shadow/pose' if shadow else self.get_parameter('output_pose_topic').value
        status_topic = '/agt/map_tracking/shadow/status' if shadow else self.get_parameter('status_topic').value
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self._quality_pub = self.create_publisher(GlobalQuality, self.get_parameter('quality_topic').value, 10)
        self.create_subscription(OdomQuality, self.get_parameter('odom_quality_topic').value, self._on_odom_quality, 10)
        self.create_timer(0.05, self._poll_tracking)
        self.create_timer(0.05, self._process_pending_clouds)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self._on_odom, 100)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(LocalizationStatus, self.get_parameter('localization_status_topic').value, self._on_status, 10)
        hz = max(0.05, float(self.get_parameter('tracking_rate_hz').value))
        self.create_timer(1.0 / hz, self._track)

    @staticmethod
    def _stamp(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_odom_quality(self, msg):
        now = self.get_clock().now().nanoseconds
        if int(msg.odom_epoch) == 0 or not 0 <= (now - self._stamp(msg)) / 1e9 <= 0.30:
            return
        if self._odom_quality is not None and self._stamp(msg) <= self._stamp(self._odom_quality):
            return
        if int(msg.odom_epoch) != self._odom_epoch:
            self._cancel.set()
            self._odom.clear()
            self._clouds.clear()
            self._raw_clouds.clear()
            self._odom_epoch = int(msg.odom_epoch)
        self._odom_quality = msg
        self._odom_quality_rx_ns = self.get_clock().now().nanoseconds

    def _on_odom(self, msg):
        if msg.header.frame_id != self.get_parameter('odom_frame').value or msg.child_frame_id != self.get_parameter('base_frame').value:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if self._stamp(msg) <= 0 or not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            return
        if self._odom and self._stamp(msg) == self._odom[-1].stamp_ns:
            return
        if self._odom and self._stamp(msg) < self._odom[-1].stamp_ns:
            self._cancel.set()
            self._odom.clear(); self._clouds.clear(); self._raw_clouds.clear()
            self._odom_quality = None
        m = np.eye(4)
        try:
            m[:3, :3] = quat_matrix((q.x, q.y, q.z, q.w))
        except ValueError:
            return
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
        if bool(self.get_parameter('shadow_mode').value):
            self._localized = bool(msg.local_odom_fresh)
            self._correction_valid = True  # TF lookup still requires a retained anchor.

    def _on_cloud(self, msg):
        if bool(self.get_parameter('shadow_mode').value):
            if self._stamp(msg) <= 0 or (self._raw_clouds and self._stamp(msg) <= self._stamp(self._raw_clouds[-1])):
                return
            self._raw_clouds.append(msg)
            return
        self._consume_cloud(msg, deskew=False)

    def _process_pending_clouds(self):
        if self._cloud_future is not None:
            if not self._cloud_future.done():
                return
            future, (epoch, stamp) = self._cloud_future, self._cloud_context
            self._cloud_future = self._cloud_context = None
            try:
                prepared = future.result()
                if epoch == self._odom_epoch and 0 <= (self.get_clock().now().nanoseconds - stamp)/1e9 <= float(self.get_parameter('max_result_age_sec').value):
                    self._clouds.append(prepared)
            except (ValueError, RuntimeError, TypeError) as exc:
                self._publish_status('WAIT_QUERY', reason='deskew:' + str(exc))
        if not self._raw_clouds or not self._odom:
            return
        while self._raw_clouds:
            msg = self._raw_clouds[0]
            bound = int(float(self.get_parameter('max_point_offset_sec').value) * 1e9)
            if self._stamp(msg) + bound > self._odom[-1].stamp_ns:
                break
            self._raw_clouds.popleft()
            if self._stamp(msg) < self._odom[0].stamp_ns:
                continue
            try:
                tf = self._tf.lookup_transform(self.get_parameter('base_frame').value, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp), timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
                transform = transform_matrix(tf.transform)
            except (TransformException, ValueError):
                self._cloud_tf_errors += 1
                continue
            samples = tuple(OdomSample(item.stamp_ns, item.pose.copy()) for item in self._odom)
            self._cloud_context = (self._odom_epoch, self._stamp(msg))
            self._cloud_future = self._preprocessor.submit(self._prepare_cloud, CloudSnapshot.freeze(msg),
                transform, samples, float(self.get_parameter('deskew_max_odom_gap_sec').value),
                float(self.get_parameter('max_point_offset_sec').value))
            break

    @staticmethod
    def _prepare_cloud(snapshot, transform, samples, gap_sec, offset_sec):
        msg = snapshot.message()
        if 'offset_time' not in [field.name for field in msg.fields]:
            raise ValueError('point_timing_missing')
        rows, offsets = [], []
        for point in point_cloud2.read_points(msg, field_names=('x','y','z','offset_time'), skip_nans=True):
            xyz = tuple(float(point[i]) for i in range(3))
            if all(math.isfinite(v) for v in xyz) and 0.25 <= sum(v*v for v in xyz) <= 900.0:
                rows.append(xyz)
                offsets.append(int(point[3]))
        if not rows:
            raise ValueError('empty_filtered_cloud')
        points = np.asarray(rows, dtype=float)
        points = (transform[:3,:3] @ points.T).T + transform[:3,3]
        stamps = np.asarray(offsets, dtype=np.int64) + snapshot.stamp_ns
        aligned, reference_pose = MapTracker._deskew_snapshot(points, stamps, snapshot.stamp_ns, samples, gap_sec, offset_sec)
        aligned.setflags(write=False)
        reference_pose.setflags(write=False)
        return CloudSample(snapshot.stamp_ns, aligned, reference_pose)

    def _consume_cloud(self, msg, deskew=False):
        odom = self._nearest_odom(self._stamp(msg))
        if odom is None:
            self._cloud_time_sync_errors += 1
            return
        try:
            tf = self._tf.lookup_transform(
                self.get_parameter('base_frame').value, msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
        except TransformException:
            self._cloud_tf_errors += 1
            return
        rows = []
        names = [field.name for field in msg.fields]
        if deskew and 'offset_time' not in names:
            self._publish_status('WAIT_QUERY', reason='point_timing_missing')
            return
        requested = ('x', 'y', 'z', 'offset_time') if deskew else ('x', 'y', 'z')
        offsets = []
        for p in point_cloud2.read_points(msg, field_names=requested, skip_nans=True):
            x, y, z = (float(p[0]), float(p[1]), float(p[2]))
            if 0.5 <= math.sqrt(x*x + y*y + z*z) <= 30.0:
                rows.append((x, y, z))
                if deskew: offsets.append(int(p[3]))
        if not rows:
            return
        points = np.asarray(rows, dtype=float)
        sensor_to_base = transform_matrix(tf.transform)
        points = (sensor_to_base[:3, :3] @ points.T).T + sensor_to_base[:3, 3]
        if deskew:
            try:
                points, reference_pose = self._deskew_points(points, np.asarray(offsets, dtype=np.int64) + self._stamp(msg), self._stamp(msg))
                odom = OdomSample(self._stamp(msg), reference_pose)
            except ValueError as exc:
                self._publish_status('WAIT_QUERY', reason='deskew:' + str(exc))
                return
        self._clouds.append(CloudSample(self._stamp(msg), points, odom.pose))

    def _write_pcd(self, points: np.ndarray, path: Path):
        points = points[:int(self.get_parameter('max_points').value)]
        with path.open('w', encoding='ascii') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
            f.write('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')

    def _record_reject(self, reason, *, reason_codes=None, **fields):
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
            reason_codes=list(reason_codes or []),
            decision='REJECT',
            consecutive_rejects=self._consecutive_rejects,
            **fields,
        )

    def _record_accept(self, **fields):
        self._consecutive_rejects = 0
        self._publish_status('TRACKING_OK', decision='ACCEPT', reason_codes=[],
                             consecutive_rejects=0, **fields)

    def _attempt_fields(self, ref, query_points):
        """Immutable per-attempt frame and input contract evidence.

        ``query`` is expressed in the reference cloud's base frame; the native
        matcher receives it with an initial T_map_base pose.  This data is
        emitted with every decision, without changing matching or rejection
        policy.
        """
        return {
            'attempt_id': self._attempt_id,
            'cloud_frame': self.get_parameter('base_frame').value,
            'cloud_source_frame_contract': 'sensor_cloud transformed to base_link',
            'map_crop_frame': self.get_parameter('map_frame').value,
            'seed_pose_frame': 'T_map_base_link',
            'odom_pose_frame': 'T_odom_base_link',
            'prediction_relation': 'T_map_base_link = T_map_odom * T_odom_base_link',
            'query_points': int(query_points),
            'cloud_time_sync_error_count': self._cloud_time_sync_errors,
            'cloud_tf_error_count': self._cloud_tf_errors,
            'reference_cloud_stamp_ns': ref.stamp_ns,
            'timestamp_ns': ref.stamp_ns,
            # A non-converged registration has no valid Hessian; retain the
            # keys as explicit null evidence rather than silently omitting it.
            'local_map_points': None,
            'registration_success': None,
            'backend_message': None,
            'hessian_lambda_min': None,
            'hessian_lambda_max': None,
            'hessian_condition_number': None,
        }

    @staticmethod
    def _pose_fields(prefix, matrix):
        translation, quaternion = matrix_pose(matrix)
        return {f'{prefix}_{key}': float(value) for key, value in zip(
            ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), (*translation, *quaternion))}

    def _debug_attempt_dir(self):
        if not bool(self.get_parameter('save_debug_cloud').value):
            return None
        root = Path(os.path.expanduser(str(self.get_parameter('debug_directory').value)))
        # This is a pending directory. It becomes a ring entry only after a
        # qualifying failure, so a later successful attempt cannot erase a
        # previously retained failure sample.
        path = root / f'.pending_{self._attempt_id:05d}'
        if path.exists():
            shutil.rmtree(path)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _retain_debug_artifact(self, pending):
        if not pending:
            return
        self._debug_failure_count += 1
        root = pending.parent
        limit = max(1, int(self.get_parameter('max_debug_failure_samples').value))
        target = root / f'attempt_{((self._debug_failure_count - 1) % limit) + 1:05d}'
        if target.exists():
            shutil.rmtree(target)
        pending.rename(target)

    @staticmethod
    def _write_debug_yaml(path, data):
        # JSON is valid YAML 1.2 and avoids a new runtime dependency.
        path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    def _track(self):
        if self._future is not None:
            return
        if not (self._localized and self._correction_valid):
            self._publish_status('WAIT_LOCALIZED', reason='manager_not_localized')
            return
        if not self._clouds:
            self._publish_status('WAIT_QUERY', reason='no_cloud')
            return
        ref = self._clouds[-1]
        if bool(self.get_parameter('shadow_mode').value) and (self.get_clock().now().nanoseconds - ref.stamp_ns)/1e9 > float(self.get_parameter('max_result_age_sec').value):
            self._publish_status('WAIT_QUERY', reason='cloud_expired')
            return
        cloud_snapshot = tuple(list(self._clouds)[-int(self.get_parameter('query_accumulate_clouds').value):])
        if len(cloud_snapshot) < max(1, int(self.get_parameter('query_accumulate_clouds').value)):
            self._publish_status('WAIT_QUERY', reason='query_window_incomplete')
            return
        self._attempt_id += 1
        fields = self._attempt_fields(ref, sum(len(cloud.points_base) for cloud in cloud_snapshot))
        try:
            map_odom_tf = self._tf.lookup_transform(
                self.get_parameter('map_frame').value, self.get_parameter('odom_frame').value,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
            map_odom = transform_matrix(map_odom_tf.transform)
        except (TransformException, ValueError):
            # This has no authority to increment reject/recovery state: a TF
            # lookup failure is an unavailable input, not a GICP decision.
            self._publish_status('WAIT_LOCALIZED', reason='map_odom_unavailable',
                                 reason_codes=['TF_ERROR'], decision='NOT_ATTEMPTED',
                                 **fields)
            return
        predicted = map_odom @ ref.odom_pose
        p, q = matrix_pose(predicted)
        try:
            cloud_tf = self._tf.lookup_transform(
                self.get_parameter('map_frame').value, self.get_parameter('odom_frame').value,
                rclpy.time.Time(nanoseconds=ref.stamp_ns), timeout=Duration(seconds=0.2))
            cloud_map_odom = transform_matrix(cloud_tf.transform)
            cloud_prediction = cloud_map_odom @ ref.odom_pose
            cp, cq = matrix_pose(cloud_prediction)
            relative = np.linalg.inv(map_odom) @ cloud_map_odom
            rp, rq = matrix_pose(relative)
            fields.update({
                'tf_cloud_time_available': True,
                **self._pose_fields('T_map_odom_cloud', cloud_map_odom),
                **self._pose_fields('T_map_base_prediction_cloud', cloud_prediction),
                'tf_latest_to_cloud_dx_m': float(rp[0]), 'tf_latest_to_cloud_dy_m': float(rp[1]),
                'tf_latest_to_cloud_dz_m': float(rp[2]),
                'tf_latest_to_cloud_qx': float(rq[0]), 'tf_latest_to_cloud_qy': float(rq[1]),
                'tf_latest_to_cloud_qz': float(rq[2]), 'tf_latest_to_cloud_qw': float(rq[3]),
                'prediction_latest_to_cloud_dx_m': float(cp[0]-p[0]),
                'prediction_latest_to_cloud_dy_m': float(cp[1]-p[1]),
                'prediction_latest_to_cloud_dz_m': float(cp[2]-p[2]),
            })
        except (TransformException, ValueError) as exc:
            fields.update({'tf_cloud_time_available': False, 'tf_cloud_time_error': str(exc)})
        fields.update({
            'tf_lookup_target_frame': self.get_parameter('map_frame').value,
            'tf_lookup_source_frame': self.get_parameter('odom_frame').value,
            'tf_lookup_time_policy': 'TIME_ZERO_LATEST',
            'tf_lookup_time_ns': 0,
            'tf_lookup_returned_stamp_ns': int(map_odom_tf.header.stamp.sec) * 1_000_000_000 + int(map_odom_tf.header.stamp.nanosec),
            'tf_lookup_age_vs_reference_cloud_sec': ((int(map_odom_tf.header.stamp.sec) * 1_000_000_000 + int(map_odom_tf.header.stamp.nanosec)) - ref.stamp_ns) / 1e9,
            'odom_pose_source_stamp_ns': ref.stamp_ns,
            'odom_pose_frame_contract': 'T_odom_base_link',
            'local_map_crop_center_x_m': float(p[0]),
            'local_map_crop_center_y_m': float(p[1]),
            'local_map_crop_center_z_m': float(p[2]),
            'local_map_crop_radius_xy_m': float(self.get_parameter('local_map_radius_xy_m').value),
            'local_map_crop_half_height_m': float(self.get_parameter('local_map_half_height_m').value),
            **self._pose_fields('T_map_base_prediction', predicted),
            **self._pose_fields('T_odom_base', ref.odom_pose),
            **self._pose_fields('T_map_odom', map_odom),
        })
        map_path = str(self.get_parameter('global_map').value)
        if not map_path or not os.path.isfile(map_path):
            self._record_reject('global_map_missing', reason_codes=['LOCAL_MAP_ERROR'], **fields)
            return
        debug_dir = self._debug_attempt_dir()
        fields.update({
            'native_initial_pose_x': float(p[0]), 'native_initial_pose_y': float(p[1]),
            'native_initial_pose_z': float(p[2]), 'native_initial_pose_qx': float(q[0]),
            'native_initial_pose_qy': float(q[1]), 'native_initial_pose_qz': float(q[2]),
            'native_initial_pose_qw': float(q[3]),
            'composition_to_prediction_translation_delta_m': 0.0,
            'composition_to_prediction_rotation_delta_deg': 0.0,
            'prediction_to_native_initial_translation_delta_m': 0.0,
            'prediction_to_native_initial_rotation_delta_deg': 0.0,
        })
        options = {
            'map': map_path, 'x': p[0], 'y': p[1], 'z': p[2],
            'threads': max(1,int(self.get_parameter('backend_threads').value)),
            'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3],
            'radius': float(self.get_parameter('local_map_radius_xy_m').value),
            'half-height': float(self.get_parameter('local_map_half_height_m').value),
        }
        if debug_dir: options['debug-crop'] = str(debug_dir / 'local_map_crop.pcd')
        identity = tuple(str(self.get_parameter(name).value) for name in ('global_map','map_id','map_version','map_hash'))
        self._cancel = threading.Event()
        self._context = (ref, predicted.copy(), dict(fields), debug_dir, self._odom_epoch,
                         file_identity(map_path), str(uuid.uuid4()), identity)
        self._future = self._worker.submit(self._run_tracking_backend, cloud_snapshot, tuple(options.items()),
            float(self.get_parameter('native_timeout_sec').value), self._cancel,
            str(self.get_parameter('map_hash').value), debug_dir,
            float(self.get_parameter('query_voxel_leaf_m').value), int(self.get_parameter('max_points').value),
            int(self.get_parameter('min_query_points').value))

    @staticmethod
    def _run_tracking_backend(clouds, options, timeout, cancelled, expected_hash, debug_dir, leaf, max_points, min_points):
        options = dict(options)
        map_path = options['map']
        if cancelled.is_set():
            raise RuntimeError('tracking_cancelled_before_query_assembly')
        inverse_ref = np.linalg.inv(clouds[-1].odom_pose)
        points = []
        for cloud in clouds:
            delta = inverse_ref @ cloud.odom_pose
            points.append((delta[:3,:3] @ cloud.points_base.T).T + delta[:3,3])
        points = MapTracker._voxel(np.concatenate(points, axis=0), leaf)[:max_points]
        if len(points) < min_points:
            raise RuntimeError('min_query_points')
        verified = bool(expected_hash and hash_file(map_path) == expected_hash)
        if expected_hash and not verified:
            raise RuntimeError('frozen_map_hash_mismatch')
        with tempfile.TemporaryDirectory(prefix='agt_map_tracker_') as work:
            scan = Path(work) / 'query.pcd'
            with scan.open('w', encoding='ascii') as stream:
                stream.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
                stream.write(f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
                for point in points:
                    stream.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')
            if debug_dir: shutil.copy2(scan, debug_dir / 'query_cloud.pcd')
            cmd = ['ros2','run','agt_global_relocalization_native','map_gicp_tracker','--scan',str(scan)]
            for key, value in options.items(): cmd.extend(['--' + key, str(value)])
            result = run_command(cmd, timeout, cancelled)
            line = next((line for line in reversed(result.stdout.splitlines()) if line.lstrip().startswith('{')), '')
            return json.loads(line), result, verified, len(points)

    def _poll_tracking(self):
        if self._future is None or not self._future.done():
            return
        future, context = self._future, self._context
        self._future = self._context = None
        ref, predicted, fields, debug_dir, epoch, map_stat, job_id, identity = context
        try:
            data, result, verified, point_count = future.result()
            self._map_hash_verified = verified
            if self._cancel.is_set() or epoch != self._odom_epoch:
                raise RuntimeError('tracking_odometry_epoch_expired')
            if file_identity(str(self.get_parameter('global_map').value)) != map_stat:
                raise RuntimeError('tracking_map_snapshot_changed')
            current_identity = tuple(str(self.get_parameter(name).value) for name in ('global_map','map_id','map_version','map_hash'))
            if current_identity != identity:
                raise RuntimeError('tracking_map_identity_changed')
            age = (self.get_clock().now().nanoseconds - ref.stamp_ns) / 1e9
            if not 0 <= age <= float(self.get_parameter('max_result_age_sec').value):
                raise RuntimeError('tracking_result_expired')
            fields.update({'job_id': job_id, 'odom_epoch': epoch, 'map_hash_verified': verified,
                           'reference_stamp_ns': ref.stamp_ns, 'query_points': point_count,
                           'map_id': identity[1], 'map_version': identity[2], 'map_hash': identity[3]})
            self._finish_tracking(ref, predicted, fields, debug_dir, data, result)
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            fields.update({'job_id': job_id, 'odom_epoch': epoch, 'reference_stamp_ns': ref.stamp_ns})
            self._record_reject('native_tracker:' + str(exc), reason_codes=['BACKEND_PROTOCOL_ERROR'], **fields)

    def _finish_tracking(self, ref, predicted, fields, debug_dir, data, result):
        if result.returncode != 0 and data.get('success'):
            raise RuntimeError('backend_exit_status_inconsistent_with_success')
        if not data.get('success'):
            fields.update({'native_returncode': int(result.returncode),
                           'native_stderr_tail': result.stderr[-512:],
                           'registration_success': False,
                           'backend_message': str(data.get('message', 'gicp_failed'))})
            reason_codes = ['GICP_NOT_CONVERGED'] if fields['backend_message'] == 'GICP did not converge' else ['LOCAL_MAP_ERROR']
            if debug_dir:
                self._write_debug_yaml(debug_dir / 'registration_result.yaml', {**fields, 'backend': data})
                self._retain_debug_artifact(debug_dir)
            self._record_reject(fields['backend_message'], reason_codes=reason_codes, **fields)
            return
        required = ('x','y','z','qx','qy','qz','qw','fitness','overlap')
        if not all(k in data and math.isfinite(float(data[k])) for k in required):
            self._record_reject('nonfinite_registration', reason_codes=['BACKEND_PROTOCOL_ERROR'], **fields)
            return
        fitness = float(data.get('fitness', math.inf))
        overlap = float(data.get('overlap', 0.0))
        hessian_condition = float(data.get('hessian_condition_number', math.inf))
        hessian_degenerate = bool(data.get('hessian_degenerate', False))
        fields.update({
            'fitness': fitness, 'overlap': overlap,
            'map_points': int(data.get('map_points', 0)),
            'local_map_points': int(data.get('map_points', 0)),
            'hessian_condition_number': hessian_condition,
            'hessian_degenerate': hessian_degenerate,
            'hessian_eigenvalues': data.get('hessian_eigenvalues', []),
            'hessian_lambda_min': min(data.get('hessian_eigenvalues', [math.nan])),
            'hessian_lambda_max': max(data.get('hessian_eigenvalues', [math.nan])),
            'registration_success': True,
            'backend_message': 'GICP converged',
        })
        if (bool(self.get_parameter('reject_degenerate_hessian').value)
                and (hessian_degenerate
                     or hessian_condition > float(
                         self.get_parameter('max_hessian_condition_number').value))):
            if debug_dir:
                shutil.rmtree(debug_dir)
            self._record_reject(
                'degenerate_geometry',
                reason_codes=['HESSIAN_BAD'], **fields,
            )
            return
        quality_codes = []
        if fitness > float(self.get_parameter('max_fitness').value):
            quality_codes.append('FITNESS_LOW')
        if overlap < float(self.get_parameter('min_overlap').value):
            quality_codes.append('OVERLAP_LOW')
        if quality_codes:
            if debug_dir:
                shutil.rmtree(debug_dir)
            self._record_reject('quality_gate', reason_codes=quality_codes, **fields)
            return
        measured = np.eye(4)
        measured[:3, :3] = quat_matrix((float(data['qx']), float(data['qy']), float(data['qz']), float(data['qw'])))
        measured[:3, 3] = (float(data['x']), float(data['y']), float(data['z']))
        translation_delta = measured[:3, 3] - predicted[:3, 3]
        translation_innovation = float(np.linalg.norm(translation_delta))
        def yaw_of(matrix):
            return math.atan2(matrix[1, 0], matrix[0, 0])
        signed_yaw_innovation = (yaw_of(measured) - yaw_of(predicted) + math.pi) % (2.0 * math.pi) - math.pi
        yaw_innovation = abs(signed_yaw_innovation)
        fields.update({
            'translation_innovation_m': translation_innovation,
            'translation_innovation_dx_m': float(translation_delta[0]),
            'translation_innovation_dy_m': float(translation_delta[1]),
            'translation_innovation_dz_m': float(translation_delta[2]),
            'yaw_innovation_deg': math.degrees(yaw_innovation),
            'yaw_innovation_signed_deg': math.degrees(signed_yaw_innovation),
            # Hessian diagnostics are collected even while hard rejection is
            # deliberately disabled for this acceptance phase.
            'diagnostic_flags': ['HESSIAN_BAD'] if (hessian_degenerate or hessian_condition > float(self.get_parameter('max_hessian_condition_number').value)) else [],
        })
        if (translation_innovation > float(self.get_parameter('max_translation_innovation_m').value)
                or yaw_innovation > math.radians(float(self.get_parameter('max_yaw_innovation_deg').value))):
            if debug_dir:
                self._write_debug_yaml(debug_dir / 'registration_result.yaml', {**fields, 'backend': data})
                self._retain_debug_artifact(debug_dir)
            self._record_reject('innovation_gate', reason_codes=['INNOVATION_HIGH'], **fields)
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
        fields['correction_applied'] = bool(self.get_parameter('apply_correction').value) and not bool(self.get_parameter('shadow_mode').value)
        if fields['correction_applied']:
            self._pose_pub.publish(pose_msg)
        elif debug_dir:
            # Successful attempts are not failure artifacts.
            shutil.rmtree(debug_dir)
        fields['num_inliers'] = int(data.get('num_inliers', 0))
        fields['normalized_hessian_condition_number'] = float(data.get('normalized_hessian_condition_number', 1.0e30))
        self._record_accept(**fields)

    @staticmethod
    def _interpolated_snapshot(samples, stamps, max_gap_sec):
        times = np.asarray([sample.stamp_ns for sample in samples], dtype=np.int64)
        if len(times) < 2:
            raise ValueError('odom_history_missing')
        index = np.searchsorted(times, stamps, side='right')
        index = np.clip(index, 1, len(times) - 1)
        ta, tb = times[index - 1], times[index]
        if np.any(stamps < ta) or np.any(stamps > tb):
            raise ValueError('point_time_not_bracketed')
        if np.any(tb - ta > max_gap_sec * 1e9):
            raise ValueError('odom_gap_exceeds_bound')
        alpha = ((stamps - ta) / (tb - ta)).astype(float)
        poses = samples
        translations = np.asarray([sample.pose[:3, 3] for sample in poses])
        quaternions = np.asarray([matrix_pose(sample.pose)[1] for sample in poses])
        qa, qb = quaternions[index - 1].copy(), quaternions[index].copy()
        dots = np.sum(qa * qb, axis=1)
        qb[dots < 0] *= -1
        dots = np.clip(np.abs(dots), 0.0, 1.0)
        angles = np.arccos(dots)
        near = dots > 0.9995
        denominators = np.sin(angles)
        denominators[near] = 1.0
        weights_a = np.sin((1.0 - alpha) * angles) / denominators
        weights_b = np.sin(alpha * angles) / denominators
        weights_a[near], weights_b[near] = 1.0 - alpha[near], alpha[near]
        q = qa * weights_a[:, None] + qb * weights_b[:, None]
        q /= np.linalg.norm(q, axis=1)[:, None]
        t = translations[index - 1] * (1.0-alpha[:,None]) + translations[index] * alpha[:,None]
        return t, q

    @staticmethod
    def _deskew_snapshot(points, stamps, reference_ns, samples, max_gap_sec, max_offset_sec):
        if np.any(stamps < reference_ns) or np.any(stamps - reference_ns > max_offset_sec * 1e9):
            raise ValueError('point_offset_outside_scan_bound')
        translation, quaternion = MapTracker._interpolated_snapshot(samples, stamps, max_gap_sec)
        ref_t, ref_q = MapTracker._interpolated_snapshot(samples, np.asarray([reference_ns], dtype=np.int64), max_gap_sec)
        # Quaternion rotation for a batch: v'=v+2w(qv x v)+2(qv x(qv x v)).
        cross = np.cross(quaternion[:, :3], points)
        odom_points = points + 2.0 * quaternion[:,3,None] * cross + 2.0 * np.cross(quaternion[:,:3],cross) + translation
        ref_rotation = quat_matrix(ref_q[0])
        aligned = (ref_rotation.T @ (odom_points - ref_t[0]).T).T
        ref_pose = np.eye(4); ref_pose[:3,:3] = ref_rotation; ref_pose[:3,3] = ref_t[0]
        return aligned, ref_pose

    def _deskew_points(self, points, stamps, reference_ns):
        return self._deskew_snapshot(points, stamps, reference_ns, tuple(self._odom),
            float(self.get_parameter('deskew_max_odom_gap_sec').value),
            float(self.get_parameter('max_point_offset_sec').value))

    def _publish_quality(self, state, fields):
        reference_ns = int(fields.get('reference_stamp_ns', fields.get('reference_cloud_stamp_ns', 0)))
        if reference_ns <= 0:
            return
        out = GlobalQuality()
        out.header.frame_id = str(self.get_parameter('map_frame').value)
        out.header.stamp = rclpy.time.Time(nanoseconds=reference_ns).to_msg()
        out.map_id, out.map_version, out.map_hash = (str(fields.get(name, self.get_parameter(name).value)) for name in ('map_id','map_version','map_hash'))
        out.job_id = str(fields.get('job_id', f'tracker-{self._attempt_id}'))
        out.odom_epoch = int(fields.get('odom_epoch', self._odom_epoch))
        out.residual = float(fields.get('fitness', 1.0e9))
        out.overlap = float(fields.get('overlap', 0.0))
        out.inliers = int(fields.get('num_inliers', 0))
        out.position_std_m = out.yaw_std_rad = 1.0e9
        out.translation_innovation_m = float(fields.get('translation_innovation_m', 1.0e9))
        out.yaw_innovation_rad = math.radians(float(fields.get('yaw_innovation_deg', 1.0e9)))
        out.quality = max(0.0, min(1.0, out.overlap)) * math.exp(-max(0.0, out.residual)) if math.isfinite(out.residual) else 0.0
        q = self._odom_quality
        now = self.get_clock().now().nanoseconds
        odom_good = (q is not None and q.valid and int(q.odom_epoch) == self._odom_epoch
                     and 0 <= (now - int(q.header.stamp.sec)*1_000_000_000 - int(q.header.stamp.nanosec)) / 1e9 <= 0.30
                     and 0 <= (now - self._odom_quality_rx_ns) / 1e9 <= 0.30)
        hessian_good = float(fields.get('normalized_hessian_condition_number', 1.0e30)) <= float(self.get_parameter('max_normalized_hessian_condition').value)
        out.valid = bool(state == 'TRACKING_OK' and bool(self.get_parameter('shadow_mode').value)
                         and bool(self.get_parameter('quality_calibrated').value) and odom_good
                         and out.inliers > 0 and all((out.map_id,out.map_version,out.map_hash))
                         and bool(fields.get('map_hash_verified', False)) and hessian_good
                         and all(math.isfinite(v) for v in (out.residual,out.overlap,out.translation_innovation_m,out.yaw_innovation_rad)))
        out.reason = str(fields.get('reason', state)) + ';empirical_score_not_probability;uncertainty_unavailable'
        self._quality_pub.publish(out)

    def destroy_node(self):
        self._cancel.set()
        self._preprocessor.shutdown(wait=True, cancel_futures=True)
        self._worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    @staticmethod
    def _voxel(points, leaf):
        if len(points) == 0 or leaf <= 0.0:
            return points
        keys = np.floor(points / leaf).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(indices)]

    def _publish_status(self, state, **fields):
        data = {'state': state, 'stamp_ns': self.get_clock().now().nanoseconds, **fields}
        self._publish_quality(state, fields)
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
