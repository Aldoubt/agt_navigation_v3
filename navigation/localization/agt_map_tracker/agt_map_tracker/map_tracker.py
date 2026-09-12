"""Low-rate local-map tracking without taking TF ownership."""

from __future__ import annotations

import json
import math
import os
import shutil
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
            'apply_correction': True,
            'save_debug_cloud': False,
            'debug_directory': '~/.ros/agt_map_tracker_debug',
            'max_debug_failure_samples': 20,
        }.items():
            self.declare_parameter(name, default)
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
        self._attempt_id += 1
        fields = self._attempt_fields(ref, len(query))
        if len(query) < int(self.get_parameter('min_query_points').value):
            self._record_reject('min_query_points', reason_codes=['MIN_QUERY_POINTS'], **fields)
            return
        try:
            map_odom_tf = self._tf.lookup_transform(
                self.get_parameter('map_frame').value, self.get_parameter('odom_frame').value,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
            map_odom = transform_matrix(map_odom_tf.transform)
        except TransformException:
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
        except TransformException as exc:
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
        with tempfile.TemporaryDirectory(prefix='agt_map_tracker_') as work:
            scan_path = Path(work) / 'query.pcd'
            self._write_pcd(query, scan_path)
            debug_dir = self._debug_attempt_dir()
            if debug_dir:
                shutil.copy2(scan_path, debug_dir / 'query_cloud.pcd')
                self._write_debug_yaml(debug_dir / 'initial_pose.yaml', fields)
            cmd = ['ros2', 'run', 'agt_global_relocalization_native', 'map_gicp_tracker',
                   '--map', map_path, '--scan', str(scan_path), '--x', str(p[0]), '--y', str(p[1]), '--z', str(p[2]),
                   '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                   '--radius', str(self.get_parameter('local_map_radius_xy_m').value),
                   '--half-height', str(self.get_parameter('local_map_half_height_m').value)]
            # Native initial pose is intentionally the exact matrix_pose result
            # above. Persist an independent serialized copy for audit; deltas
            # must remain zero unless this construction path changes.
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
            if debug_dir:
                cmd.extend(['--debug-crop', str(debug_dir / 'local_map_crop.pcd')])
            try:
                result = subprocess.run(cmd, check=False, capture_output=True, text=True,
                                        timeout=float(self.get_parameter('native_timeout_sec').value))
                # ``ros2 run`` appends its own failure line after the native
                # JSON result when the matcher returns non-zero.  Select the
                # last JSON object rather than treating that wrapper line as a
                # tracker protocol failure.  This preserves the same reject
                # outcome while making the reason evidence truthful.
                json_line = next((line for line in reversed(result.stdout.splitlines())
                                  if line.lstrip().startswith('{')), '')
                data = json.loads(json_line)
            except (OSError, subprocess.SubprocessError, ValueError, IndexError) as exc:
                # Preserve bounded backend evidence.  A malformed CLI response
                # is not evidence that the cropped map is bad, so keep this
                # distinct from a reported matcher/local-map failure.
                backend = locals().get('result')
                if backend is not None:
                    fields.update({
                        'native_returncode': int(backend.returncode),
                        'native_stdout_tail': backend.stdout[-512:],
                        'native_stderr_tail': backend.stderr[-512:],
                    })
                self._record_reject(f'native_tracker:{exc}',
                                    reason_codes=['BACKEND_PROTOCOL_ERROR'], **fields)
                return
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
        fields['correction_applied'] = bool(self.get_parameter('apply_correction').value)
        if fields['correction_applied']:
            self._pose_pub.publish(pose_msg)
        elif debug_dir:
            # Successful attempts are not failure artifacts.
            shutil.rmtree(debug_dir)
        self._record_accept(**fields)

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
