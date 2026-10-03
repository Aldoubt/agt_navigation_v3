from __future__ import annotations

import json
import math
import os
import signal
import shlex
import shutil
import statistics
import subprocess
import tempfile
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import time
from collections import deque
from pathlib import Path

import rclpy
from agt_batch_lio_adapter.extrinsics import (
    compose_transform as compose_mount_transform,
    load_lio_body_to_lidar,
    transform_msg_to_tuple,
)
from agt_robot_interfaces.msg import MapStatus
from agt_navigation_interfaces.msg import GlobalQuality, OdomQuality, RelocalizationCandidate
from .query_job import QueryJob, CloudSnapshot, asset_identity, file_identity, hash_file, interpolate_pose, run_command, PoseInterpolator
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
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
        p('cancel_topic', '/agt/relocalization/cancel')
        p('output_pose_topic', '/agt/relocalization/pose')
        p('publish_legacy_pose', True)
        p('status_topic', '/agt/global_relocalization/status')
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('query_frame', 'base_link')
        # Formal PGO-map contract: the stored map and formal poses are body
        # frame products, therefore the query must use mapping-era T_body_livox.
        # base_link remains an explicit compatibility mode only.
        p('relocalization_query_frame_mode', 'mapping_body')
        # Keep the BBS candidate pose and query cloud in the formal PGO body
        # contract by default.  base_link is an explicit deprecated fallback.
        p('bbs_query_frame_mode', 'mapping_body')
        p('mapping_body_livox_translation', [-0.011, -0.02329, 0.04412])
        p('mapping_body_livox_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        # Vehicle-mount conversion is derived from the same Batch-LIO runtime
        # config and robot_state_publisher TF used by the local odometry path.
        p('batch_lio_config_file', '')  # Legacy Batch-only argument.
        p('body_to_base_calibration_file', '')
        p('mount_lidar_frame', 'livox_frame')
        p('mount_base_frame', 'base_link')
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
        p('query_capture_dir', '')
        p('sdk_timeout_sec', 10.0)
        p('backend_threads', 2)
        p('require_nondegenerate_hessian', False)
        p('max_normalized_hessian_condition', 1.0e10)
        p('accumulate_clouds', 5)
        p('min_points', 2000)
        p('max_points', 250000)
        p('query_min_range_m', 0.5)
        p('query_max_range_m', 30.0)
        p('query_voxel_leaf_m', 0.25)
        p('require_stationary', True)
        p('moving_query_enabled', False)
        p('moving_query_calibrated', False)
        p('deskew_max_odom_gap_sec', 0.05)
        p('max_point_offset_sec', 0.20)
        p('odom_buffer_sec', 30.0)
        p('candidate_topic', '/agt/localization/candidate')
        p('odom_quality_topic', '/agt/odometry/quality')
        p('map_id', '')
        p('map_version', '')
        p('map_hash', '')
        p('require_candidate_ambiguity', False)
        p('min_candidate_ambiguity_margin', 0.0)
        p('max_candidate_age_sec', 20.0)
        p('local_odom_topic', '/agt/odometry/local')
        # Navigation uses LiDAR odometry for the stationary gate. An empty
        # override resolves to local_odom_topic; wheel data is diagnostic only.
        p('stationary_odom_topic', '')
        p('odom_freshness_sec', 0.50)
        p('stationary_linear_threshold_mps', 0.05)
        p('stationary_angular_threshold_rps', 0.08)
        p('stationary_filter_window_samples', 5)
        p('stationary_hard_linear_threshold_mps', 0.20)
        p('stationary_hard_angular_threshold_rps', 0.30)
        p('min_score', 0.50)
        p('max_fitness', 1.00)
        p('min_overlap', 0.20)
        p('best_position_std_m', 0.15)
        p('worst_position_std_m', 0.80)
        p('best_yaw_std_deg', 3.0)
        p('worst_yaw_std_deg', 15.0)
        p('auto_request', False)
        # Opt-in P2.15 diagnostic capture. Empty keeps normal runtime behavior.
        p('cloud_contract_capture_dir', '')

        self.clouds = deque(maxlen=max(1, int(self.get_parameter('accumulate_clouds').value)) + (20 if bool(self.get_parameter('moving_query_enabled').value) else 0))
        self.busy = False
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='agt_relocalization')
        self._future = None
        self._job = None
        self._cancel_event = threading.Event()
        self._request_epoch = 0
        self._last_submitted_reference_ns = 0
        self._odom_epoch = 1
        self._odom_history = deque()
        self._odom_quality = None
        self._odom_quality_rx_ns = 0
        self.auto_requested = False
        self.auto_timer = None
        self.latest_odom = None
        self.latest_odom_rx_ns = 0
        self.latest_motion = None
        motion_window = max(
            1, int(self.get_parameter('stationary_filter_window_samples').value))
        self.motion_samples = deque(maxlen=motion_window)
        self.pending_request = False
        self.active_map_status = None
        self.last_query_raw_points = 0
        self.last_query_cloud_contract = None
        self._base_link_compatibility_warned = False
        self._body_aligned_alias_warned = False
        self._body_to_base_cache = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.candidate_pub = self.create_publisher(RelocalizationCandidate, self.get_parameter('candidate_topic').value, 10)
        self.create_subscription(OdomQuality, self.get_parameter('odom_quality_topic').value, self._on_odom_quality, 10)
        self.create_timer(0.05, self._poll_job)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('output_pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.query_pub = self.create_publisher(PointCloud2, '/agt/relocalization/query_cloud', 10)
        self.coarse_cloud_pub = self.create_publisher(PointCloud2, '/agt/relocalization/coarse_aligned_cloud', 10)
        self.aligned_cloud_pub = self.create_publisher(PointCloud2, '/agt/relocalization/aligned_cloud', 10)
        self.coarse_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/agt/relocalization/coarse_pose', 10)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self.on_cloud, qos_profile_sensor_data)
        stationary_odom_topic = str(
            self.get_parameter('stationary_odom_topic').value).strip()
        if not stationary_odom_topic:
            stationary_odom_topic = str(self.get_parameter('local_odom_topic').value)
        self.stationary_odom_topic = stationary_odom_topic
        self.create_subscription(Odometry, stationary_odom_topic, self.on_odom, 50)
        self.create_subscription(Empty, self.get_parameter('request_topic').value, self.on_request, 10)
        self.create_subscription(Empty, self.get_parameter('cancel_topic').value, lambda _: self._cancel_job('cancel requested'), 10)

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
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0 or (self.clouds and stamp_ns <= int(self.clouds[-1].header.stamp.sec) * 1_000_000_000 + int(self.clouds[-1].header.stamp.nanosec)):
            return
        self.clouds.append(msg)
        if self.pending_request and not self.busy:
            self._try_start_request()
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
        self.pending_request = False
        self.auto_requested = False

    def on_map_status(self, msg):
        previous_generation = int(self.active_map_status.generation) if self.active_map_status else -1
        self.active_map_status = msg
        if (not msg.active) or int(msg.generation) != previous_generation:
            self._cancel_job('active map changed')
            self._last_submitted_reference_ns = 0
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
    def robust_motion(samples, required_samples):
        """Return median linear/angular motion once the filter window is full."""
        required = max(1, int(required_samples))
        if len(samples) < required:
            return None
        window = list(samples)[-required:]
        return (statistics.median(sample[0] for sample in window),
                statistics.median(sample[1] for sample in window))

    @staticmethod
    def has_complete_query(cloud_count, required_clouds):
        """A registration query is valid only after all configured frames arrive."""
        return int(cloud_count) >= max(1, int(required_clouds))

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

    def _on_odom_quality(self, msg):
        epoch = int(msg.odom_epoch)
        if epoch != self._odom_epoch:
            self._cancel_job('odometry epoch changed')
            self._odom_history.clear()
            self.clouds.clear()
            self._odom_epoch = epoch
            self._last_submitted_reference_ns = 0
        self._odom_quality = msg
        self._odom_quality_rx_ns = self.get_clock().now().nanoseconds

    def on_odom(self, msg):
        if (msg.header.frame_id != self.get_parameter('odom_frame').value
                or msg.child_frame_id != self.get_parameter('mount_base_frame').value):
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        values = [getattr(msg.pose.pose.position, k) for k in ('x', 'y', 'z')]
        values += [getattr(msg.pose.pose.orientation, k) for k in ('x', 'y', 'z', 'w')]
        if stamp_ns <= 0 or not all(math.isfinite(float(v)) for v in values) or sum(v*v for v in values[3:]) <= 1e-24:
            return
        if self._odom_history and stamp_ns == self._odom_history[-1][0]:
            return
        if self._odom_history and stamp_ns < self._odom_history[-1][0]:
            self._cancel_job('odometry source time reset')
            self._odom_history.clear()
            self.clouds.clear()
            self.motion_samples.clear()
            self.latest_odom = None
            self._odom_quality = None
        pose = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), values))
        self._odom_history.append((stamp_ns, pose))
        oldest = stamp_ns - int(float(self.get_parameter('odom_buffer_sec').value) * 1e9)
        while self._odom_history and self._odom_history[0][0] < oldest:
            self._odom_history.popleft()
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
            self.motion_samples.append(self.latest_motion)
        if bool(self.get_parameter('require_stationary').value):
            filtered = self.robust_motion(
                self.motion_samples,
                self.get_parameter('stationary_filter_window_samples').value)
            hard_moving = self.latest_motion is not None and (
                self.latest_motion[0] > float(
                    self.get_parameter('stationary_hard_linear_threshold_mps').value)
                or self.latest_motion[1] > float(
                    self.get_parameter('stationary_hard_angular_threshold_rps').value)
            )
            moving = filtered is None or (
                filtered[0] > float(self.get_parameter('stationary_linear_threshold_mps').value)
                or filtered[1] > float(self.get_parameter('stationary_angular_threshold_rps').value)
            )
            if first_odom or hard_moving or moving:
                self.clouds.clear()
        if self.pending_request and not self.busy:
            self._try_start_request()

    def status(self, state, detail='', **extra):
        m = String()
        payload = {'state': state, 'detail': detail}
        payload.update(extra)
        m.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(m)
        self.get_logger().info(m.data)

    def stationary_gate(self):
        if not bool(self.get_parameter('require_stationary').value):
            if not (bool(self.get_parameter('moving_query_enabled').value)
                    and bool(self.get_parameter('moving_query_calibrated').value)):
                return False, 'moving queries require enabled and calibrated deskew'
            quality = self._odom_quality
            now = self.get_clock().now().nanoseconds
            timeout = float(self.get_parameter('odom_freshness_sec').value)
            if (quality is None or not quality.valid or int(quality.odom_epoch) != self._odom_epoch
                    or not 0 <= (now - self._odom_quality_rx_ns) / 1e9 <= timeout
                    or not 0 <= (now - int(quality.header.stamp.sec) * 1_000_000_000
                        - int(quality.header.stamp.nanosec)) / 1e9 <= timeout):
                return False, 'moving query requires fresh valid odometry quality'
            return True, 'moving query with per-point deskew'
        if self.latest_odom is None or self.latest_odom_rx_ns <= 0:
            return False, 'no local odometry available for stationary gate'
        age = (self.get_clock().now().nanoseconds - self.latest_odom_rx_ns) / 1e9
        source_age = (self.get_clock().now().nanoseconds - int(self.latest_odom.header.stamp.sec) * 1_000_000_000 - int(self.latest_odom.header.stamp.nanosec)) / 1e9
        if not 0 <= age <= float(self.get_parameter('odom_freshness_sec').value) or not 0 <= source_age <= float(self.get_parameter('odom_freshness_sec').value):
            return False, f'local odometry stale: {age:.3f}s'
        window = max(
            1, int(self.get_parameter('stationary_filter_window_samples').value))
        filtered = self.robust_motion(self.motion_samples, window)
        if filtered is None:
            return False, (
                f'stationary odometry filter is not ready: '
                f'{len(self.motion_samples)}/{window} samples')
        if self.latest_motion is not None and (
                self.latest_motion[0] > float(
                    self.get_parameter('stationary_hard_linear_threshold_mps').value)
                or self.latest_motion[1] > float(
                    self.get_parameter('stationary_hard_angular_threshold_rps').value)):
            return False, (
                f'robot moving above hard gate: linear={self.latest_motion[0]:.3f}m/s '
                f'angular={self.latest_motion[1]:.3f}rad/s')
        linear, angular = filtered
        if linear > float(self.get_parameter('stationary_linear_threshold_mps').value):
            return False, f'robot moving (filtered): linear={linear:.3f}m/s'
        if angular > float(self.get_parameter('stationary_angular_threshold_rps').value):
            return False, f'robot rotating (filtered): angular={angular:.3f}rad/s'
        return True, (
            f'stationary source={self.stationary_odom_topic} '
            f'linear={linear:.3f}m/s angular={angular:.3f}rad/s')

    def _request_readiness(self):
        stationary, detail = self.stationary_gate()
        if not stationary:
            return False, 'WAIT_STATIONARY', detail
        required = max(1, int(self.get_parameter('accumulate_clouds').value))
        if not self.has_complete_query(len(self.clouds), required):
            return (False, 'COLLECTING',
                    f'collecting stationary PointCloud2 query: '
                    f'{len(self.clouds)}/{required} frames')
        return True, 'QUERY_READY', detail

    def _try_start_request(self):
        if self.busy or not self.pending_request:
            return False
        ready, _, _ = self._request_readiness()
        if not ready:
            return False

        self.pending_request = False
        try:
            job = self._freeze_job()
        except Exception as exc:
            self.status('REJECTED', str(exc))
            if 'waiting for complete timestamp-bracketed' in str(exc):
                self.pending_request = True
            else:
                self._rearm_auto_request(clear_clouds=True)
            return True
        self.busy = True
        self._job = job
        self._last_submitted_reference_ns = job.reference_ns
        self._cancel_event = threading.Event()
        self.status('BBS_SEARCHING', 'asynchronous immutable query submitted', job_id=job.job_id,
                    input_clouds=len(job.clouds), odom_epoch=job.odom_epoch, map_id=job.map_id,
                    map_version=job.map_version)
        self._future = self._executor.submit(self._prepare_and_execute, job, self._cancel_event)
        return True

    def on_request(self, _msg):
        if self.busy:
            if self._cancel_event.is_set(): self.pending_request = True
            self.status('BUSY', 'relocalization already running')
            return
        self.pending_request = True
        if self._try_start_request():
            return
        _, state, detail = self._request_readiness()
        self.status(
            state,
            detail,
            collected_clouds=len(self.clouds),
            required_clouds=max(1, int(self.get_parameter('accumulate_clouds').value)),
            stationary_odom_topic=self.stationary_odom_topic,
        )

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

    @staticmethod
    def _normalized_xyzw(values, name):
        if len(values) != 4:
            raise RuntimeError(f'{name} must contain exactly four xyzw values')
        qx, qy, qz, qw = (float(v) for v in values)
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if not all(math.isfinite(v) for v in (qx,qy,qz,qw)) or norm <= 1e-12:
            raise RuntimeError(f'{name} has zero quaternion')
        return qx/norm, qy/norm, qz/norm, qw/norm

    @staticmethod
    def _rotate_xyz(x, y, z, q):
        qx, qy, qz, qw = q
        tx = 2.0 * (qy*z - qz*y)
        ty = 2.0 * (qz*x - qx*z)
        tz = 2.0 * (qx*y - qy*x)
        return (x + qw*tx + (qy*tz - qz*ty),
                y + qw*ty + (qz*tx - qx*tz),
                z + qw*tz + (qx*ty - qy*tx))

    @staticmethod
    def compose_pose(first, second):
        """Compose T_ac = T_ab * T_bc; pose dictionaries use xyzw quaternions."""
        fq = GlobalRelocalization._normalized_xyzw(
            (first['qx'], first['qy'], first['qz'], first['qw']), 'first quaternion')
        sq = GlobalRelocalization._normalized_xyzw(
            (second['qx'], second['qy'], second['qz'], second['qw']), 'second quaternion')
        rx, ry, rz = GlobalRelocalization._rotate_xyz(second['x'], second['y'], second['z'], fq)
        ax, ay, az, aw = fq
        bx, by, bz, bw = sq
        q = (aw*bx + ax*bw + ay*bz - az*by,
             aw*by - ax*bz + ay*bw + az*bx,
             aw*bz + ax*by - ay*bx + az*bw,
             aw*bw - ax*bx - ay*by - az*bz)
        q = GlobalRelocalization._normalized_xyzw(q, 'composed quaternion')
        return {'x': first['x'] + rx, 'y': first['y'] + ry, 'z': first['z'] + rz,
                'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3]}

    @staticmethod
    def inverse_pose(pose):
        q = GlobalRelocalization._normalized_xyzw(
            (pose['qx'], pose['qy'], pose['qz'], pose['qw']), 'pose quaternion')
        iq = (-q[0], -q[1], -q[2], q[3])
        tx, ty, tz = GlobalRelocalization._rotate_xyz(-pose['x'], -pose['y'], -pose['z'], iq)
        return {'x': tx, 'y': ty, 'z': tz, 'qx': iq[0], 'qy': iq[1], 'qz': iq[2], 'qw': iq[3]}

    @staticmethod
    def normalize_query_frame_mode(mode):
        """Return the canonical query-frame mode without changing pose math."""
        mode = str(mode).strip()
        if mode == 'body_aligned':
            return 'mapping_body', 'body_aligned'
        if mode in ('mapping_body', 'base_link'):
            return mode, ''
        raise RuntimeError(
            'relocalization_query_frame_mode must be mapping_body or base_link '
            '(body_aligned is a deprecated alias)')

    def query_frame_contract(self):
        configured = str(self.get_parameter('relocalization_query_frame_mode').value).strip()
        mode, alias = self.normalize_query_frame_mode(configured)
        if alias and not self._body_aligned_alias_warned:
            self.get_logger().warning(
                'relocalization_query_frame_mode=body_aligned is deprecated; '
                'use mapping_body for formal PGO map packages')
            self._body_aligned_alias_warned = True
        if mode == 'base_link':
            if not self._base_link_compatibility_warned:
                self.get_logger().warning(
                    'relocalization_query_frame_mode=base_link is deprecated for '
                    'formal PGO map packages; migrate to mapping_body')
                self._base_link_compatibility_warned = True
            return mode, str(self.get_parameter('query_frame').value), None
        t = list(self.get_parameter('mapping_body_livox_translation').value)
        if len(t) != 3:
            raise RuntimeError('mapping_body_livox_translation must contain exactly three values')
        q = self._normalized_xyzw(
            list(self.get_parameter('mapping_body_livox_quaternion_xyzw').value),
            'mapping_body_livox_quaternion_xyzw')
        return mode, 'body', {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
                               'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3]}

    def body_to_base_pose(self):
        """Resolve T_body_base from canonical internal + physical mount sources."""
        if self._body_to_base_cache is not None:
            return dict(self._body_to_base_cache)

        config_path = os.path.expanduser(
            str(self.get_parameter('body_to_base_calibration_file').value).strip()
            or str(self.get_parameter('batch_lio_config_file').value).strip())
        if not config_path:
            raise RuntimeError(
                'body_to_base_calibration_file (or legacy batch_lio_config_file) '
                'is required to derive body->base_link')
        t_body_lidar, q_body_lidar = load_lio_body_to_lidar(config_path)

        lidar_frame = str(self.get_parameter('mount_lidar_frame').value).strip()
        base_frame = str(self.get_parameter('mount_base_frame').value).strip()
        if not lidar_frame or not base_frame:
            raise RuntimeError('mount_lidar_frame and mount_base_frame must not be empty')

        try:
            # target=lidar, source=base -> T_lidar_base from the calibrated
            # robot_description chain. Time(0) is intentional for static TF.
            tf = self.tf_buffer.lookup_transform(
                lidar_frame,
                base_frame,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'physical mount TF {lidar_frame} <- {base_frame} unavailable: {exc}'
            ) from exc

        t_lidar_base, q_lidar_base = transform_msg_to_tuple(tf.transform)
        t_body_base, q_body_base = compose_mount_transform(
            t_body_lidar, q_body_lidar, t_lidar_base, q_lidar_base)
        self._body_to_base_cache = {
            'x': t_body_base[0], 'y': t_body_base[1], 'z': t_body_base[2],
            'qx': q_body_base[0], 'qy': q_body_base[1],
            'qz': q_body_base[2], 'qw': q_body_base[3],
        }
        self.get_logger().info(
            f'Resolved relocalization body->{base_frame} from active LIO T_body_lidar + '
            f'robot_description {lidar_frame}<- {base_frame}: '
            f't=[{t_body_base[0]:.6f}, {t_body_base[1]:.6f}, {t_body_base[2]:.6f}] '
            f'q=[{q_body_base[0]:.9f}, {q_body_base[1]:.9f}, '
            f'{q_body_base[2]:.9f}, {q_body_base[3]:.9f}]'
        )
        return dict(self._body_to_base_cache)

    def query_pose_to_base_pose(self, query_pose, mode):
        """Convert native T_map_query to the ROS T_map_base boundary contract."""
        if mode == 'base_link':
            return dict(query_pose)
        if mode == 'mapping_body':
            return self.compose_pose(query_pose, self.body_to_base_pose())
        raise RuntimeError(f'unsupported query pose conversion mode: {mode!r}')

    def merged_points(self, clouds=None, odom_samples=None, moving=False):
        rows = []
        raw_rows = []
        transform_records = []
        capture_contract = bool(str(self.get_parameter('cloud_contract_capture_dir').value).strip())
        max_points = int(self.get_parameter('max_points').value)
        min_range = float(self.get_parameter('query_min_range_m').value)
        max_range = float(self.get_parameter('query_max_range_m').value)
        min_range_sq = min_range * min_range
        max_range_sq = max_range * max_range
        voxel_leaf = float(self.get_parameter('query_voxel_leaf_m').value)
        mode, query_frame, fixed_transform = self.query_frame_contract()
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))

        def finalize():
            self.last_query_raw_points = len(rows)
            if capture_contract:
                self.last_query_cloud_contract = {
                    'query_frame': query_frame,
                    'raw_rows': raw_rows,
                    'transformed_rows': list(rows),
                    'transforms': transform_records,
                }
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

        clouds = list(self.clouds) if clouds is None else list(clouds)
        odom_samples = list(self._odom_history) if odom_samples is None else list(odom_samples)
        query_from_base = self.inverse_pose(self.body_to_base_pose()) if moving and mode == 'mapping_body' else None
        ref_pose = None
        max_gap_ns = int(float(self.get_parameter('deskew_max_odom_gap_sec').value) * 1e9)
        if moving:
            ref_ns = int(clouds[-1].header.stamp.sec) * 1_000_000_000 + int(clouds[-1].header.stamp.nanosec)
            ref_pose = interpolate_pose(odom_samples, ref_ns, max_gap_ns, self._normalized_xyzw,
                                        self.compose_pose, self.inverse_pose)
            if query_from_base is not None:
                ref_pose = self.compose_pose(ref_pose, query_from_base)
            ref_pose = self.inverse_pose(ref_pose)
        for cloud in clouds:
            if not cloud.header.frame_id:
                raise RuntimeError('relocalization scan has empty frame_id')
            if fixed_transform is None:
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
            else:
                transform = None

            if capture_contract:
                if transform is None:
                    transform_records.append({
                        'source_frame': cloud.header.frame_id, 'target_frame': query_frame,
                        'stamp_ns': int(cloud.header.stamp.sec) * 1_000_000_000 + int(cloud.header.stamp.nanosec),
                        'source': 'mapping-era parameter T_body_livox',
                        'translation': [fixed_transform['x'], fixed_transform['y'], fixed_transform['z']],
                        'quaternion_xyzw': [fixed_transform['qx'], fixed_transform['qy'],
                                            fixed_transform['qz'], fixed_transform['qw']],
                    })
                else:
                    tr = transform.transform.translation
                    qr = transform.transform.rotation
                    transform_records.append({
                        'source_frame': cloud.header.frame_id,
                        'target_frame': query_frame,
                        'stamp_ns': int(cloud.header.stamp.sec) * 1_000_000_000 + int(cloud.header.stamp.nanosec),
                        'translation': [tr.x, tr.y, tr.z],
                        'quaternion_xyzw': [qr.x, qr.y, qr.z, qr.w],
                    })

            names = [f.name for f in cloud.fields]
            requested = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
            if moving:
                if 'offset_time' not in names:
                    raise RuntimeError('moving query requires Livox offset_time nanoseconds')
                requested += ('offset_time',)
            for pt in point_cloud2.read_points(cloud, field_names=requested, skip_nans=True):
                x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                intensity = float(pt[3]) if 'intensity' in names else 0.0
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    range_sq = x*x + y*y + z*z
                    if range_sq < min_range_sq or range_sq > max_range_sq:
                        continue
                    if capture_contract:
                        raw_rows.append((x, y, z, intensity))
                    if transform is None:
                        x, y, z = self._rotate_xyz(x, y, z, (
                            fixed_transform['qx'], fixed_transform['qy'],
                            fixed_transform['qz'], fixed_transform['qw']))
                        x += fixed_transform['x']; y += fixed_transform['y']; z += fixed_transform['z']
                    else:
                        x, y, z = self.transform_xyz(x, y, z, transform)
                    if moving:
                        offset_ns = int(pt[-1])
                        if offset_ns < 0 or offset_ns > float(self.get_parameter('max_point_offset_sec').value) * 1e9:
                            raise RuntimeError('point time offset outside calibrated scan bound')
                        point_ns = int(cloud.header.stamp.sec) * 1_000_000_000 + int(cloud.header.stamp.nanosec) + offset_ns
                        point_pose = interpolate_pose(odom_samples, point_ns, max_gap_ns,
                                                      self._normalized_xyzw, self.compose_pose, self.inverse_pose)
                        if query_from_base is not None:
                            point_pose = self.compose_pose(point_pose, query_from_base)
                        delta = self.compose_pose(ref_pose, point_pose)
                        x, y, z = self._rotate_xyz(x, y, z, [delta[k] for k in ('qx', 'qy', 'qz', 'qw')])
                        x += delta['x']; y += delta['y']; z += delta['z']
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
        map_id = str(self.get_parameter('map_id').value)
        map_version = str(self.get_parameter('map_version').value)
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

    def _cancel_job(self, reason):
        self._request_epoch += 1
        self._cancel_event.set()
        self.pending_request = False
        if self._job is not None:
            self.status('CANCELLED', reason, job_id=self._job.job_id)

    def _freeze_job(self):
        mode, frame, fixed_transform = self.query_frame_contract()
        bbs_mode, _ = self.normalize_query_frame_mode(str(self.get_parameter('bbs_query_frame_mode').value))
        if mode != bbs_mode:
            raise RuntimeError('query and BBS frame modes must match')
        clouds = tuple(self.clouds)
        moving = not bool(self.get_parameter('require_stationary').value)
        if moving:
            latest_ns = self._odom_history[-1][0] if self._odom_history else 0
            bound = int(float(self.get_parameter('max_point_offset_sec').value) * 1e9)
            clouds = tuple(c for c in clouds if int(c.header.stamp.sec) * 1_000_000_000
                           + int(c.header.stamp.nanosec) + bound <= latest_ns)
        count = max(1, int(self.get_parameter('accumulate_clouds').value))
        if len(clouds) < count:
            raise RuntimeError('waiting for complete timestamp-bracketed query')
        clouds = clouds[-count:]
        reference_ns = int(clouds[-1].header.stamp.sec) * 1_000_000_000 + int(clouds[-1].header.stamp.nanosec)
        if reference_ns <= self._last_submitted_reference_ns:
            raise RuntimeError('waiting for complete timestamp-bracketed query with new cloud source time')
        transforms = []
        for cloud in clouds:
            if not cloud.header.frame_id:
                raise RuntimeError('query cloud frame is empty')
            if fixed_transform is None:
                tf = self.tf_buffer.lookup_transform(frame,cloud.header.frame_id,Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
                t,q = transform_msg_to_tuple(tf.transform)
                transform = dict(zip(('x','y','z','qx','qy','qz','qw'),list(t)+list(q)))
            else:
                if cloud.header.frame_id.lstrip('/') != str(self.get_parameter('mount_lidar_frame').value).lstrip('/'):
                    raise RuntimeError('mapping-body query requires unmodified Livox sensor frame')
                transform = dict(fixed_transform)
            transforms.append(tuple(transform.items()))
        if moving and mode == 'mapping_body':
            path = str(self.get_parameter('body_to_base_calibration_file').value).strip() or str(self.get_parameter('batch_lio_config_file').value).strip()
            runtime_t,runtime_q = load_lio_body_to_lidar(os.path.expanduser(path))
            query_t = tuple(fixed_transform[k] for k in ('x','y','z'))
            query_q = tuple(fixed_transform[k] for k in ('qx','qy','qz','qw'))
            if (max(abs(a-b) for a,b in zip(runtime_t,query_t)) > 1.0e-8
                    or abs(sum(a*b for a,b in zip(runtime_q,query_q))) < 1.0-1.0e-8):
                raise RuntimeError('moving query mapping-body calibration differs from pose conversion body; explicit matching frozen-map calibration required')
        options = tuple((name,self.get_parameter(name).value) for name in (
            'max_points','min_points','query_min_range_m','query_max_range_m','query_voxel_leaf_m',
            'deskew_max_odom_gap_sec','max_point_offset_sec'))
        global_map, assets, map_id, version, generation = self.resolve_map_inputs()
        if moving and not all((map_id, version, str(self.get_parameter('map_hash').value).strip())):
            raise RuntimeError('moving query requires frozen map id/version/SHA256')
        if not global_map or not Path(global_map).is_file():
            raise RuntimeError('global map PCD is unavailable')
        if assets and not Path(assets).is_dir():
            raise RuntimeError('relocalization assets are unavailable')
        candidate = str(self.get_parameter('candidate_sdk_command').value).strip()
        candidate_db = Path(assets) / 'polar_context.db' if assets else None
        command = candidate if candidate and candidate_db and candidate_db.is_file() else str(self.get_parameter('sdk_command').value).strip()
        if not command:
            raise RuntimeError('relocalization backend command is empty')
        timeout = float(self.get_parameter('sdk_timeout_sec').value)
        base_from_body = self.inverse_pose(self.body_to_base_pose())
        values = {
            'global_map': global_map, 'timeout_sec': timeout,
            'backend_threads': max(1, int(self.get_parameter('backend_threads').value)),
            'assets_arg': f'--assets-dir {shlex.quote(assets)}' if assets else '',
            'local_map_radius_xy': float(self.get_parameter('backend_local_map_radius_xy').value),
            'local_map_half_height': float(self.get_parameter('backend_local_map_half_height').value),
            'min_local_map_points': int(self.get_parameter('backend_min_local_map_points').value),
            'bbs_query_frame_mode': bbs_mode,
            **{f'base_from_body_{k}': v for k, v in base_from_body.items()},
        }
        # The established CLI template uses tx/ty/tz rather than x/y/z.
        for axis in ('x', 'y', 'z'):
            values[f'base_from_body_t{axis}'] = base_from_body[axis]
        self._request_epoch += 1
        return QueryJob(
            str(uuid.uuid4()), self._request_epoch, self._odom_epoch,
            reference_ns,
            self.get_clock().now().nanoseconds, time.monotonic() + timeout,
            (), tuple(CloudSnapshot.freeze(cloud) for cloud in clouds), tuple(transforms), tuple((stamp, dict(pose)) for stamp,pose in self._odom_history), options, mode, frame, tuple(self.body_to_base_pose().items()), global_map, assets,
            map_id, version, generation, str(self.get_parameter('map_hash').value).strip(),
            file_identity(global_map), asset_identity(assets), command, timeout, tuple(values.items()),
            os.path.expanduser(str(self.get_parameter('work_dir').value)),
            str(self.get_parameter('query_capture_dir').value).strip(), moving)

    @staticmethod
    def _prepare_and_execute(job, cancelled):
        options = dict(job.query_options)
        rows = []
        min_range_sq, max_range_sq = float(options['query_min_range_m']) ** 2, float(options['query_max_range_m']) ** 2
        interpolation = PoseInterpolator(job.odom_samples, int(float(options['deskew_max_odom_gap_sec'])*1e9),
                                         GlobalRelocalization._normalized_xyzw)
        query_from_base = GlobalRelocalization.inverse_pose(dict(job.body_to_base)) if job.mode == 'mapping_body' else None
        reference = None
        if job.moving:
            reference = interpolation.at(job.reference_ns)
            if query_from_base is not None: reference = GlobalRelocalization.compose_pose(reference,query_from_base)
            reference = GlobalRelocalization.inverse_pose(reference)
        for cloud, transform_items in zip(job.clouds,job.transforms):
            if cancelled.is_set(): raise RuntimeError('query cancelled during assembly')
            cloud = cloud.message()
            transform = dict(transform_items)
            names = [field.name for field in cloud.fields]
            has_intensity = 'intensity' in names
            requested = ('x','y','z') + (('intensity',) if has_intensity else ())
            if job.moving:
                if 'offset_time' not in names: raise RuntimeError('moving query requires Livox per-point offset_time')
                requested += ('offset_time',)
            for i,point in enumerate(point_cloud2.read_points(cloud,field_names=requested,skip_nans=True)):
                if i % 2048 == 0:
                    if cancelled.is_set(): raise RuntimeError('query cancelled during deskew')
                    if time.monotonic() > job.deadline_monotonic: raise RuntimeError('query deadline exceeded during deskew')
                x,y,z = (float(point[k]) for k in range(3))
                intensity = float(point[3]) if has_intensity else 0.0
                if not all(math.isfinite(v) for v in (x,y,z,intensity)): continue
                distance_sq = x*x+y*y+z*z
                if not min_range_sq <= distance_sq <= max_range_sq: continue
                x,y,z = GlobalRelocalization._rotate_xyz(x,y,z,[transform[k] for k in ('qx','qy','qz','qw')])
                x += transform['x']; y += transform['y']; z += transform['z']
                if job.moving:
                    offset = int(point[-1])
                    if not 0 <= offset <= float(options['max_point_offset_sec'])*1e9:
                        raise RuntimeError('point offset exceeds calibrated scan bound')
                    point_ns = int(cloud.header.stamp.sec)*1_000_000_000+int(cloud.header.stamp.nanosec)+offset
                    pose = interpolation.at(point_ns)
                    if query_from_base is not None: pose = GlobalRelocalization.compose_pose(pose,query_from_base)
                    delta = GlobalRelocalization.compose_pose(reference,pose)
                    x,y,z = GlobalRelocalization._rotate_xyz(x,y,z,[delta[k] for k in ('qx','qy','qz','qw')])
                    x += delta['x']; y += delta['y']; z += delta['z']
                rows.append((x,y,z,intensity))
                if len(rows) >= int(options['max_points']): break
            if len(rows) >= int(options['max_points']): break
        leaf = float(options['query_voxel_leaf_m'])
        if leaf > 0:
            voxels = {}
            for row in rows:
                key = tuple(math.floor(row[k]/leaf) for k in range(3))
                if key not in voxels: voxels[key] = row
            rows = list(voxels.values())
        if len(rows) < int(options['min_points']): raise RuntimeError('query has insufficient points')
        prepared = replace(job,rows=tuple(rows),clouds=(),transforms=(),odom_samples=())
        result,actual_hash = GlobalRelocalization._execute_job(prepared,cancelled)
        return prepared,result,actual_hash

    @staticmethod
    def _execute_job(job, cancelled):
        # This worker has no node, publishers, live parameters or cloud deque.
        if cancelled.is_set():
            raise RuntimeError('query cancelled before backend start')
        if asset_identity(job.assets_path) != job.asset_stats:
            raise RuntimeError('relocalization assets changed before backend')
        actual_hash = hash_file(job.map_path, cancelled, job.deadline_monotonic)
        if job.expected_hash and actual_hash != job.expected_hash:
            raise RuntimeError('frozen map hash does not match configured SHA256')
        if file_identity(job.map_path) != job.map_stat:
            raise RuntimeError('map file changed while hashing')
        work = Path(job.work_dir)
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='query_', dir=work) as td:
            scan = Path(td) / 'query_scan.pcd'
            GlobalRelocalization.write_ascii_pcd(scan, job.rows)
            values = dict(job.command_values)
            values['scan_pcd'] = str(scan)
            command = shlex.split(job.command_template.format(**values))
            capture = None
            if job.capture_dir:
                root = Path(job.capture_dir).expanduser()
                root.mkdir(parents=True, exist_ok=True)
                capture = root / f'query_{job.job_id}'
                saved = capture.with_suffix('.pcd')
                shutil.copyfile(scan, saved)
                capture.with_suffix('.json').write_text(json.dumps({
                    'job_id': job.job_id, 'request_epoch': job.request_epoch,
                    'odom_epoch': job.odom_epoch, 'reference_ns': job.reference_ns,
                    'map_id': job.map_id, 'map_version': job.map_version,
                    'map_hash': actual_hash, 'moving_deskew': job.moving,
                    'query_frame': job.query_frame, 'point_count': len(job.rows),
                    'replay_command': [str(saved) if part == str(scan) else part for part in command],
                }, indent=2) + '\n', encoding='utf-8')
            remaining = min(job.timeout_sec, job.deadline_monotonic - time.monotonic())
            if remaining <= 0:
                raise RuntimeError('query deadline exceeded before backend')
            proc = run_command(command, remaining, cancelled)
            if capture:
                capture.with_suffix('.backend.json').write_text(json.dumps({
                    'returncode': proc.returncode, 'stdout': proc.stdout, 'stderr': proc.stderr,
                }, indent=2) + '\n', encoding='utf-8')
            if proc.returncode:
                raise RuntimeError(f'backend returned {proc.returncode}: {(proc.stderr or proc.stdout)[-1024:]}')
            line = next((line for line in reversed(proc.stdout.splitlines()) if line.lstrip().startswith('{')), '')
            result = json.loads(line)
        if file_identity(job.map_path) != job.map_stat or asset_identity(job.assets_path) != job.asset_stats:
            raise RuntimeError('map or relocalization assets changed during backend computation')
        return result, actual_hash

    def _poll_job(self):
        if self._future is None or not self._future.done():
            return
        future, job = self._future, self._job
        self._future = None
        self._job = None
        self.busy = False
        try:
            job, result, actual_hash = future.result()
            if self._cancel_event.is_set() or job.request_epoch != self._request_epoch:
                raise RuntimeError('cancelled or superseded query result')
            if job.odom_epoch != self._odom_epoch:
                raise RuntimeError('query odometry epoch expired')
            if time.monotonic() > job.deadline_monotonic:
                raise RuntimeError('query result arrived beyond its deadline')
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - job.reference_ns) / 1e9 > float(self.get_parameter('max_candidate_age_sec').value):
                raise RuntimeError('historical query result is too old')
            if now_ns < job.reference_ns or file_identity(job.map_path) != job.map_stat:
                raise RuntimeError('query time or map snapshot expired')
            current = self.resolve_map_inputs()
            if current != (job.map_path,job.assets_path,job.map_id,job.map_version,job.map_generation):
                raise RuntimeError('map inputs changed during relocalization')
            if asset_identity(job.assets_path) != job.asset_stats:
                raise RuntimeError('relocalization assets changed before publication')
            self.verify_map_snapshot(job.map_id, job.map_version, job.map_generation)
            self._publish_job_result(job, result, actual_hash)
        except Exception as exc:
            retry = self.pending_request
            self.status('REJECTED', str(exc), job_id=job.job_id)
            self._rearm_auto_request(clear_clouds=True)
            self.pending_request = retry

    def _publish_job_result(self, job, result, actual_hash):
        if not bool(result.get('success', False)):
            raise RuntimeError(str(result.get('message', 'backend failure')))
        required = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'score', 'fitness', 'overlap')
        if not all(math.isfinite(float(result[k])) for k in required):
            raise RuntimeError('backend returned nonfinite registration values')
        score, fitness, overlap = (float(result[k]) for k in ('score', 'fitness', 'overlap'))
        if (score < float(self.get_parameter('min_score').value)
                or fitness > float(self.get_parameter('max_fitness').value)
                or overlap < float(self.get_parameter('min_overlap').value)):
            raise RuntimeError('backend quality gate failed')
        condition = float(result.get('normalized_hessian_condition_number',1.0e30))
        if job.moving or bool(self.get_parameter('require_nondegenerate_hessian').value):
            if not math.isfinite(condition) or condition > float(self.get_parameter('max_normalized_hessian_condition').value):
                raise RuntimeError('normalized registration Hessian is unobservable')
        ambiguity_valid = bool(result.get('ambiguity_valid', False))
        margin = float(result.get('ambiguity_margin', 0.0))
        if not math.isfinite(margin):
            raise RuntimeError('backend returned nonfinite candidate ambiguity')
        if bool(self.get_parameter('require_candidate_ambiguity').value) and (
                not ambiguity_valid or margin < float(self.get_parameter('min_candidate_ambiguity_margin').value)):
            raise RuntimeError('spatially separated candidate ambiguity is unresolved')
        q = self._normalized_xyzw([result[k] for k in ('qx', 'qy', 'qz', 'qw')], 'backend quaternion')
        query_pose = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                              [float(result[k]) for k in ('x', 'y', 'z')] + list(q)))
        base_pose = query_pose if job.mode == 'base_link' else self.compose_pose(query_pose, dict(job.body_to_base))
        stamp = Time(nanoseconds=job.reference_ns).to_msg()
        self.query_pub.publish(self.cloud_message(job.rows, stamp, job.query_frame))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.get_parameter('map_frame').value)
        for axis in ('x', 'y', 'z'):
            setattr(msg.pose.pose.position, axis, base_pose[axis])
        for axis in ('x', 'y', 'z', 'w'):
            setattr(msg.pose.pose.orientation, axis, base_pose['q' + axis])
        # Compatibility covariance retains the V1 heuristic; it is explicitly
        # NOT a calibrated uncertainty or the GlobalQuality validity predicate.
        pos_std = self._lerp('worst_position_std_m', 'best_position_std_m', min(1.0, max(0.0, score)))
        yaw_std = math.radians(self._lerp('worst_yaw_std_deg', 'best_yaw_std_deg', min(1.0, max(0.0, score))))
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = pos_std * pos_std
        cov[21] = cov[28] = math.radians(10.0) ** 2
        cov[35] = yaw_std * yaw_std
        msg.pose.covariance = cov
        candidate = RelocalizationCandidate()
        candidate.job_id = job.job_id
        candidate.request_epoch = job.request_epoch
        candidate.odom_epoch = job.odom_epoch
        candidate.map_id, candidate.map_version, candidate.map_hash = job.map_id, job.map_version, actual_hash
        candidate.reference_stamp = stamp
        candidate.completed_stamp = self.get_clock().now().to_msg()
        candidate.pose = msg
        quality = GlobalQuality()
        quality.header = msg.header
        quality.map_id, quality.map_version, quality.map_hash = job.map_id, job.map_version, actual_hash
        quality.job_id, quality.odom_epoch = job.job_id, job.odom_epoch
        quality.valid = False  # A registration is a candidate until owner verification.
        quality.quality, quality.residual, quality.overlap = score, fitness, overlap
        quality.inliers = int(result.get('num_inliers', 0))
        quality.position_std_m, quality.yaw_std_rad = 1.0e9, 1.0e9
        quality.ambiguity_valid, quality.ambiguity_margin = ambiguity_valid, margin
        quality.reason = 'candidate_unverified;covariance_is_legacy_heuristic'
        candidate.quality = quality
        self.candidate_pub.publish(candidate)
        # Research owner consumes the typed observation. The legacy owner keeps
        # its stationary V1 pose interface and hard-anchor acceptance policy.
        if bool(self.get_parameter('publish_legacy_pose').value) and not job.moving:
            self.pose_pub.publish(msg)
        self.aligned_cloud_pub.publish(self.cloud_message(
            self.apply_pose(job.rows, [query_pose[k] for k in ('x', 'y', 'z')], q),
            stamp, msg.header.frame_id))
        coarse_keys = ('coarse_x', 'coarse_y', 'coarse_z', 'coarse_qx', 'coarse_qy', 'coarse_qz', 'coarse_qw')
        if all(k in result and math.isfinite(float(result[k])) for k in coarse_keys):
            cp = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [float(result[k]) for k in coarse_keys]))
            cb = cp if job.mode == 'base_link' else self.compose_pose(cp, dict(job.body_to_base))
            cm = PoseWithCovarianceStamped(); cm.header = msg.header
            for axis in ('x', 'y', 'z'): setattr(cm.pose.pose.position, axis, cb[axis])
            for axis in ('x', 'y', 'z', 'w'): setattr(cm.pose.pose.orientation, axis, cb['q' + axis])
            self.coarse_pose_pub.publish(cm)
            self.coarse_cloud_pub.publish(self.cloud_message(self.apply_pose(job.rows,
                [cp[k] for k in ('x', 'y', 'z')], [cp[k] for k in ('qx', 'qy', 'qz', 'qw')]), stamp, msg.header.frame_id))
        self.status('SUCCEEDED', 'historical global base candidate published',
                    job_id=job.job_id, odom_epoch=job.odom_epoch, reference_ns=job.reference_ns,
                    map_id=job.map_id, map_version=job.map_version, map_hash=actual_hash,
                    score=score, fitness=fitness, overlap=overlap, ambiguity_valid=ambiguity_valid,
                    ambiguity_margin=margin, normalized_hessian_condition=result.get('normalized_hessian_condition_number'),
                    position_std_m=pos_std, yaw_std_deg=math.degrees(yaw_std),
                    query_frame=job.query_frame, refined_pose_T_map_query=query_pose,
                    published_pose_T_map_base=base_pose)

    def run_once(self):
        """Compatibility entry: submit asynchronous work, never block spin."""
        self.pending_request = True
        return self._try_start_request()

    def destroy_node(self):
        self._cancel_event.set()
        self._executor.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    def _lerp(self, low_name, high_name, t):
        low = float(self.get_parameter(low_name).value)
        high = float(self.get_parameter(high_name).value)
        return low + (high - low) * t


def main(args=None):
    def _interrupt(_signum, _frame):
        raise KeyboardInterrupt

    node = None
    try:
        signal.signal(signal.SIGINT, _interrupt)
        signal.signal(signal.SIGTERM, _interrupt)
        # Let spin unwind before destroying subscriptions. rclpy's default
        # handler shuts the context down inside signal delivery, which can
        # race an in-flight PointCloud2 take/callback.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = GlobalRelocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
