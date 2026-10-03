from __future__ import annotations

import math
import json
import signal
import bisect
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from agt_robot_interfaces.msg import LocalizationMetrics, LocalizationStatus
from agt_navigation_interfaces.msg import GlobalQuality, OdomQuality, RelocalizationCandidate, RecoveryStatus


@dataclass(frozen=True)
class _Pose3:
    p: Tuple[float, float, float]
    q: Tuple[float, float, float, float]  # x y z w


def _q_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if not all(math.isfinite(v) for v in q) or n <= 1e-12:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def _q_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_rotate(q, v):
    q = _q_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    rq = _q_multiply(_q_multiply(q, vq), _q_conjugate(q))
    return rq[0], rq[1], rq[2]


def _inverse(t: _Pose3) -> _Pose3:
    q_inv = _q_conjugate(_q_normalize(t.q))
    p_inv = _q_rotate(q_inv, (-t.p[0], -t.p[1], -t.p[2]))
    return _Pose3(p_inv, q_inv)


def _compose(a: _Pose3, b: _Pose3) -> _Pose3:
    bp = _q_rotate(a.q, b.p)
    p = (a.p[0] + bp[0], a.p[1] + bp[1], a.p[2] + bp[2])
    q = _q_normalize(_q_multiply(a.q, b.q))
    return _Pose3(p, q)


def _slerp(a: _Pose3, b: _Pose3, alpha: float) -> _Pose3:
    """Interpolate SE(3) conservatively for the map->odom correction."""
    alpha = max(0.0, min(1.0, float(alpha)))
    qa = _q_normalize(a.q)
    qb = _q_normalize(b.q)
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-x for x in qb)
        dot = -dot
    if dot > 0.9995:
        q = _q_normalize(tuple(x + alpha * (y - x) for x, y in zip(qa, qb)))
    else:
        theta = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta = math.sin(theta)
        wa = math.sin((1.0 - alpha) * theta) / sin_theta
        wb = math.sin(alpha * theta) / sin_theta
        q = _q_normalize(tuple(wa * x + wb * y for x, y in zip(qa, qb)))
    p = tuple(x + alpha * (y - x) for x, y in zip(a.p, b.p))
    return _Pose3(p, q)


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _se3_interpolate(a: _Pose3, b: _Pose3, alpha: float) -> _Pose3:
    """a * Exp(alpha * Log(a^-1 * b)), with stable small-angle branches."""
    alpha = max(0.0, min(1.0, alpha))
    delta = _compose(_inverse(a), b)
    q = _q_normalize(delta.q)
    if q[3] < 0.0: q = tuple(-v for v in q)
    sine_half = math.sqrt(sum(v*v for v in q[:3]))
    theta = 2.0 * math.atan2(sine_half, q[3])
    phi = tuple(v * theta/sine_half for v in q[:3]) if sine_half > 1e-10 else tuple(2*v for v in q[:3])
    cross_t = _cross(phi,delta.p)
    cross2_t = _cross(phi,cross_t)
    inv_coeff = (1.0 - 0.5*theta/math.tan(0.5*theta))/(theta*theta) if theta > 1e-5 else 1.0/12.0 + theta*theta/720.0
    rho = tuple(t - 0.5*c + inv_coeff*c2 for t,c,c2 in zip(delta.p,cross_t,cross2_t))
    scaled_phi, scaled_rho = tuple(alpha*v for v in phi), tuple(alpha*v for v in rho)
    angle = alpha*theta
    a_coeff = (1.0-math.cos(angle))/(angle*angle) if angle > 1e-5 else 0.5-angle*angle/24.0
    b_coeff = (angle-math.sin(angle))/(angle**3) if angle > 1e-5 else 1.0/6.0-angle*angle/120.0
    cross_rho = _cross(scaled_phi,scaled_rho)
    cross2_rho = _cross(scaled_phi,cross_rho)
    p = tuple(r+a_coeff*c+b_coeff*c2 for r,c,c2 in zip(scaled_rho,cross_rho,cross2_rho))
    rotation_scale = math.sin(angle/2.0)/angle if angle > 1e-10 else 0.5
    rotation = tuple(v*rotation_scale for v in scaled_phi)+(math.cos(angle/2.0),)
    return _compose(a,_Pose3(p,rotation))


def _translation_delta(a: _Pose3, b: _Pose3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a.p, b.p)))


def _rotation_delta(a: _Pose3, b: _Pose3) -> float:
    qa, qb = _q_normalize(a.q), _q_normalize(b.q)
    return 2.0 * math.acos(min(1.0, abs(sum(x*y for x,y in zip(qa,qb)))))


def _yaw(q) -> float:
    x, y, z, w = _q_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _odom_pose(msg: Odometry) -> _Pose3:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return _Pose3((p.x, p.y, p.z), _q_normalize((q.x, q.y, q.z, q.w)))


def _global_pose(msg: PoseWithCovarianceStamped) -> _Pose3:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return _Pose3((p.x, p.y, p.z), _q_normalize((q.x, q.y, q.z, q.w)))


def _correction_delta(current: Optional[_Pose3], previous: Optional[_Pose3]):
    """Return translation/yaw change between two map->odom corrections."""
    if current is None or previous is None:
        return 0.0, 0.0
    return (
        _translation_delta(current, previous),
        abs(_angle_wrap(_yaw(current.q) - _yaw(previous.q))),
    )


def _tracking_innovation_exceeds(
    translation: float,
    yaw_delta: float,
    max_translation: float,
    max_yaw: float,
) -> bool:
    return translation > max_translation or yaw_delta > max_yaw


def _make_metrics_message(
    state: LocalizationState,
    tracking_health: str,
    reason: str,
    correction: Optional[_Pose3],
    correction_translation_delta: float,
    correction_yaw_delta: float,
    tracking_innovation_translation: float,
    tracking_innovation_yaw: float,
    global_position_std: float,
    global_yaw_std: float,
    stamp,
) -> LocalizationMetrics:
    """Build the machine-readable metrics message without changing localization."""
    msg = LocalizationMetrics()
    msg.state = state.value
    msg.tracking_health = tracking_health
    msg.reason = reason
    if correction is not None:
        msg.map_odom_x, msg.map_odom_y, msg.map_odom_z = correction.p
        msg.map_odom_yaw = _yaw(correction.q)
    msg.correction_translation_delta = float(correction_translation_delta)
    msg.correction_yaw_delta = float(correction_yaw_delta)
    msg.tracking_innovation_translation = float(tracking_innovation_translation)
    msg.tracking_innovation_yaw = float(tracking_innovation_yaw)
    msg.global_position_std = float(global_position_std)
    msg.global_yaw_std = float(global_yaw_std)
    if stamp is not None:
        msg.stamp = stamp
    return msg


class LocalizationState(str, Enum):
    """Internal state machine, kept richer than the frozen wire message."""

    BOOT = 'BOOT'
    WAIT_GLOBAL = 'WAIT_GLOBAL'
    LOCALIZED = 'LOCALIZED'
    TRACKING = 'TRACKING'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'
    RECOVERY_REQUESTED = 'RECOVERY_REQUESTED'
    RELOCALIZING = 'RELOCALIZING'


_WIRE_STATE_BY_INTERNAL = {
    # LocalizationStatus is a frozen compatibility interface. TRACKING uses
    # the existing LOCALIZED value on the wire; the complete state is exposed
    # on /agt/relocalization/status as JSON.
    LocalizationState.BOOT: LocalizationStatus.STATE_BOOT,
    LocalizationState.WAIT_GLOBAL: LocalizationStatus.STATE_WAIT_GLOBAL,
    LocalizationState.LOCALIZED: LocalizationStatus.STATE_LOCALIZED,
    LocalizationState.TRACKING: LocalizationStatus.STATE_LOCALIZED,
    LocalizationState.DEGRADED: LocalizationStatus.STATE_DEGRADED,
    LocalizationState.LOST: LocalizationStatus.STATE_LOST,
    LocalizationState.RECOVERY_REQUESTED: LocalizationStatus.STATE_RELOCALIZING,
    LocalizationState.RELOCALIZING: LocalizationStatus.STATE_RELOCALIZING,
}


def _wire_state(state: LocalizationState) -> int:
    return _WIRE_STATE_BY_INTERNAL[state]


def _local_odom_loss_recoverable(
    state: LocalizationState,
    reason: str,
    correction_available: bool,
) -> bool:
    """Allow a transient local-odom outage to recover without changing map->odom."""
    return (
        state == LocalizationState.LOST
        and reason.startswith('local_odom_lost:')
        and correction_available
    )


class _RecoveryStateMachine:
    """Recovery transitions and cooldown gate used by the ROS node."""

    def __init__(self, cooldown_sec: float) -> None:
        self.cooldown_sec = max(0.0, float(cooldown_sec))
        self.last_request_sec: Optional[float] = None
        self.request_count = 0
        self.state = LocalizationState.BOOT
        self.pending = False
        self.requested = False
        self.reason = 'boot'

    def cooldown_remaining(self, now_sec: float) -> float:
        if self.last_request_sec is None:
            return 0.0
        return max(0.0, self.cooldown_sec - (float(now_sec) - self.last_request_sec))

    def request(self, now_sec: float) -> bool:
        if self.cooldown_remaining(now_sec) > 0.0:
            return False
        self.last_request_sec = float(now_sec)
        self.request_count += 1
        return True

    def tracking_failure(self, reason: str, auto_request: bool) -> None:
        self.state = LocalizationState.LOST
        self.reason = reason
        self.pending = bool(auto_request)
        self.requested = False

    def try_request(self, now_sec: float) -> bool:
        if not self.pending or self.requested or not self.request(now_sec):
            return False
        self.state = LocalizationState.RECOVERY_REQUESTED
        self.reason = 'global_relocalization_requested'
        self.pending = False
        self.requested = True
        self.state = LocalizationState.RELOCALIZING
        self.reason = 'relocalization_requested'
        return True

    def global_pose_accepted(self) -> None:
        self.state = LocalizationState.LOCALIZED
        self.reason = 'global_pose_accepted'
        self.pending = False
        self.requested = False


class LocalizationManager(Node):
    """Own map->odom and turn a validated global base pose into a global correction.

    Global relocalization supplies T_map_base at time t. A time-near local
    odometry sample supplies T_odom_base at the same t. This node computes:

        T_map_odom = T_map_base * inverse(T_odom_base)

    It is intentionally the only node in this stack that publishes map->odom.
    """

    def __init__(self) -> None:
        super().__init__('agt_localization_manager')
        self.declare_parameter('local_odom_topic', '/agt/odometry/local')
        self.declare_parameter('global_pose_topic', '/agt/relocalization/pose')
        self.declare_parameter('tracking_pose_topic', '/agt/map_tracking/pose')
        self.declare_parameter('tracking_status_topic', '/agt/map_tracking/status')
        self.declare_parameter('status_topic', '/agt/localization/status')
        self.declare_parameter('relocalization_request_topic', '/agt/relocalization/request')
        self.declare_parameter('relocalization_cancel_topic', '/agt/relocalization/cancel')
        self.declare_parameter('relocalization_service', '/agt/localization/relocalize')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('local_odom_timeout_sec', 0.30)
        self.declare_parameter('local_odom_lost_sec', 1.00)
        self.declare_parameter('global_match_max_skew_sec', 0.10)
        self.declare_parameter('odom_buffer_sec', 30.0)
        self.declare_parameter('max_global_position_std_m', 1.00)
        self.declare_parameter('max_global_yaw_std_deg', 20.0)
        self.declare_parameter('accept_zero_covariance', False)
        self.declare_parameter('tf_publish_rate_hz', 30.0)
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_version', '')
        self.declare_parameter('debug_identity_map_odom', False)
        self.declare_parameter('correction_smoothing_enabled', True)
        self.declare_parameter('correction_tau_sec', 3.0)
        self.declare_parameter('max_correction_linear_rate_mps', 0.10)
        self.declare_parameter('max_correction_yaw_rate_degps', 2.0)
        self.declare_parameter('max_tracking_translation_innovation_m', 0.50)
        self.declare_parameter('max_tracking_yaw_innovation_deg', 5.0)
        self.declare_parameter('tracking_consecutive_accepts', 2)
        self.declare_parameter('tracking_consistency_translation_m', 0.20)
        self.declare_parameter('tracking_consistency_yaw_deg', 2.0)
        self.declare_parameter('tracking_recovery_auto_request', True)
        self.declare_parameter('recovery_cooldown_sec', 5.0)
        self.declare_parameter('metrics_topic', '/agt/localization/metrics')
        self.declare_parameter('debug_status_topic', '/agt/relocalization/status')
        self.declare_parameter('global_status_topic', '/agt/global_relocalization/status')
        self.declare_parameter('map_events_topic', '/agt/map/events')
        for name, value in {
            'research_recovery_enabled': False, 'research_recovery_calibrated': False,
            'candidate_topic': '/agt/localization/candidate', 'map_hash': '',
            'odom_quality_topic': '/agt/odometry/quality',
            'global_quality_topic': '/agt/localization/quality',
            'recovery_status_topic': '/agt/localization/recovery_status',
            'global_observation_topic': '/agt/localization/global_observation',
            'global_quality_timeout_sec': 3.0, 'tracking_status_timeout_sec': 3.0,
            'candidate_max_age_sec': 20.0, 'candidate_completion_timeout_sec': 1.0,
            'research_odom_interpolation_max_gap_sec': 0.05,
            'research_candidate_retry_sec': 2.0,
            'candidate_min_quality': 0.50, 'candidate_max_residual': 1.0,
            'candidate_min_overlap': 0.40, 'candidate_min_ambiguity_margin': 0.0,
            'recovery_consecutive_candidates': 2, 'recovery_blend_duration_sec': 5.0,
            'recovery_max_robot_displacement_m': 1.0, 'recovery_max_yaw_delta_deg': 10.0,
            'recovery_max_rotation_delta_deg': 10.0,
            'recovery_consistency_m': 0.20, 'recovery_consistency_yaw_deg': 2.0,
            'recovery_settle_translation_m': 0.02, 'recovery_settle_yaw_deg': 0.2,
            'recovery_verification_count': 2,
        }.items():
            self.declare_parameter(name, value)

        self._odom: Deque[Odometry] = deque()
        self._last_odom_rx_ns = 0
        self._odom_epoch = 1
        self._odom_quality = None
        self._odom_quality_rx_ns = 0
        self._global_valid = False
        self._last_global_accept_ns = 0
        self._last_candidate_stamp_ns = 0
        self._last_candidate_epoch = 0
        self._last_candidate_job = ''
        self._recovery_job_id = ''
        self._next_research_query_ns = 0
        self._last_research_query_odom_ns = 0
        self._last_candidate = None
        self._latest_observation = None
        self._last_observation_stamp_ns = 0
        self._last_tracking_status_rx_ns = 0
        self._last_tracking_status_source_ns = 0
        self._recovery_blending = False
        self._recovery_start = None
        self._recovery_start_ns = 0
        self._last_recovery_candidate = None
        self._recovery_candidate_count = 0
        self._recovery_verify_count = 0
        self._correction_current: Optional[_Pose3] = None
        self._correction_target: Optional[_Pose3] = None
        self._last_tracking_measurement: Optional[_Pose3] = None
        self._tracking_consistent_count = 0
        self._tracking_health = 'UNKNOWN'
        self._tracking_innovation_bad = False
        self._tracking_recovery_requested = False
        self._recovery_pending = False
        self._recovery = _RecoveryStateMachine(
            self.get_parameter('recovery_cooldown_sec').value)
        self._last_tick_ns = self.get_clock().now().nanoseconds
        self._last_global_std = (math.inf, math.inf)
        self._last_metrics_correction: Optional[_Pose3] = None
        self._tracking_innovation_translation = 0.0
        self._tracking_innovation_yaw = 0.0
        self._correction_translation_delta = 0.0
        self._correction_yaw_delta = 0.0
        self._state = LocalizationState.BOOT
        self._reason = 'boot'

        self._recovery_status_pub = self.create_publisher(RecoveryStatus, self.get_parameter('recovery_status_topic').value, 10)
        self._global_quality_pub = self.create_publisher(GlobalQuality, self.get_parameter('global_quality_topic').value, 10)
        self.create_subscription(RelocalizationCandidate, self.get_parameter('candidate_topic').value, self._on_global_candidate, 10)
        self.create_subscription(OdomQuality, self.get_parameter('odom_quality_topic').value, self._on_odom_quality, 10)
        self.create_subscription(GlobalQuality, self.get_parameter('global_observation_topic').value, self._on_quality_observation, 10)
        self.create_timer(0.2, self._publish_global_quality)
        self._tf = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(
            LocalizationStatus, self.get_parameter('status_topic').value, 10)
        self._metrics_pub = self.create_publisher(
            LocalizationMetrics, self.get_parameter('metrics_topic').value, 10)
        self._debug_status_pub = self.create_publisher(
            String, self.get_parameter('debug_status_topic').value, 10)
        self._cancel_pub = self.create_publisher(Empty, self.get_parameter('relocalization_cancel_topic').value, 10)
        self._request_pub = self.create_publisher(
            Empty, self.get_parameter('relocalization_request_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('local_odom_topic').value, self._on_odom, 100)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('global_pose_topic').value,
            self._on_global_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('tracking_pose_topic').value,
            self._on_tracking_pose,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('tracking_status_topic').value,
            self._on_tracking_status,
            20,
        )
        self._backend_debug_state = None
        self.create_subscription(String, self.get_parameter('global_status_topic').value,
                                 self._on_backend_status, 20)
        self.create_subscription(String, self.get_parameter('map_events_topic').value,
                                 self._on_map_event, 10)
        self.create_service(
            Trigger,
            self.get_parameter('relocalization_service').value,
            self._on_relocalize,
        )

        tf_rate = max(float(self.get_parameter('tf_publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / tf_rate, self._tick)
        self.create_timer(0.2, self._publish_status)
        self.create_timer(0.2, self._publish_metrics)
        self.get_logger().info(
            'Localization Manager started as the exclusive map->odom owner.')

    def _research_enabled(self):
        return bool(self.get_parameter('research_recovery_enabled').value)

    def _on_odom_quality(self, msg):
        now = self.get_clock().now().nanoseconds
        source = _stamp_ns(msg.header.stamp)
        timeout = float(self.get_parameter('local_odom_timeout_sec').value)
        if int(msg.odom_epoch) == 0 or not 0 <= (now - source) / 1e9 <= timeout:
            return
        if self._odom_quality is not None and source <= _stamp_ns(self._odom_quality.header.stamp):
            return
        epoch = int(msg.odom_epoch)
        if epoch != self._odom_epoch:
            self._invalidate_anchor('odometry_epoch_changed', clear=True)
            self._odom.clear()
            self._last_candidate_stamp_ns = 0
            self._last_candidate_epoch = 0
            self._last_candidate_job = ''
            self._odom_epoch = epoch
        self._odom_quality = msg
        self._odom_quality_rx_ns = self.get_clock().now().nanoseconds

    def _fresh_odom_quality(self):
        q = self._odom_quality
        now = self.get_clock().now().nanoseconds
        timeout = float(self.get_parameter('local_odom_timeout_sec').value)
        return (q is not None and q.valid and int(q.odom_epoch) == self._odom_epoch
                and 0 <= (now - _stamp_ns(q.header.stamp)) / 1e9 <= timeout
                and 0 <= (now - self._odom_quality_rx_ns) / 1e9 <= timeout)

    def _invalidate_anchor(self, reason, clear=False):
        self._global_valid = False
        self._recovery_blending = False
        self._recovery_verify_count = 0
        self._last_recovery_candidate = None
        self._recovery_candidate_count = 0
        self._latest_observation = None
        self._last_observation_stamp_ns = 0
        self._recovery_job_id = ''
        self._next_research_query_ns = 0
        self._last_research_query_odom_ns = 0
        self._correction_target = self._correction_current
        if clear or not self._research_enabled():
            self._correction_current = None
            self._correction_target = None
        self._state = LocalizationState.DEGRADED if self._correction_current else LocalizationState.LOST
        self._reason = reason

    def _candidate_reject(self, reason):
        self._reason = 'candidate_rejected:' + reason
        self.get_logger().warn(self._reason, throttle_duration_sec=1.0)

    def _on_global_candidate(self, candidate):
        if not self._research_enabled():
            return  # V1 keeps its existing pose/hard-anchor contract.
        if not bool(self.get_parameter('research_recovery_calibrated').value):
            self._candidate_reject('research_policy_uncalibrated')
            return
        now = self.get_clock().now().nanoseconds
        stamp = _stamp_ns(candidate.reference_stamp)
        completed = _stamp_ns(candidate.completed_stamp)
        expected = (str(self.get_parameter('map_id').value), str(self.get_parameter('map_version').value),
                    str(self.get_parameter('map_hash').value))
        if not all(expected) or (candidate.map_id, candidate.map_version, candidate.map_hash) != expected:
            self._candidate_reject('frozen_map_identity_or_hash_mismatch')
            return
        if int(candidate.odom_epoch) != self._odom_epoch or not self._fresh_odom_quality():
            self._candidate_reject('odom_epoch_or_quality_invalid')
            return
        if (not candidate.job_id or candidate.job_id == self._last_candidate_job
                or stamp <= self._last_candidate_stamp_ns
                or int(candidate.request_epoch) <= self._last_candidate_epoch):
            self._candidate_reject('duplicate_or_out_of_order_job')
            return
        if (stamp <= 0 or stamp != _stamp_ns(candidate.pose.header.stamp)
                or stamp > completed or completed > now
                or (now - stamp) / 1e9 > float(self.get_parameter('candidate_max_age_sec').value)
                or (now - completed) / 1e9 > float(self.get_parameter('candidate_completion_timeout_sec').value)):
            self._candidate_reject('candidate_timestamp_or_age_invalid')
            return
        quality = candidate.quality
        if (quality.header.frame_id != candidate.pose.header.frame_id
                or _stamp_ns(quality.header.stamp) != stamp):
            self._candidate_reject('quality_reference_frame_or_stamp_mismatch')
            return
        if (quality.map_id, quality.map_version, quality.map_hash, quality.job_id, int(quality.odom_epoch)) != (
                candidate.map_id, candidate.map_version, candidate.map_hash, candidate.job_id, self._odom_epoch):
            self._candidate_reject('quality_identity_mismatch')
            return
        scalars = [quality.quality, quality.residual, quality.overlap, quality.ambiguity_margin]
        if (not all(math.isfinite(v) for v in scalars) or quality.inliers == 0
                or not 0 <= quality.quality <= 1 or not 0 <= quality.overlap <= 1 or quality.residual < 0
                or quality.quality < float(self.get_parameter('candidate_min_quality').value)
                or quality.residual > float(self.get_parameter('candidate_max_residual').value)
                or quality.overlap < float(self.get_parameter('candidate_min_overlap').value)
                or not quality.ambiguity_valid
                or float(self.get_parameter('candidate_min_ambiguity_margin').value) <= 0
                or quality.ambiguity_margin < float(self.get_parameter('candidate_min_ambiguity_margin').value)):
            self._candidate_reject('raw_registration_or_ambiguity_gate')
            return
        if candidate.pose.header.frame_id != self.get_parameter('map_frame').value:
            self._candidate_reject('candidate_frame_invalid')
            return
        local = self._bracketed_odom_pose(stamp)
        if local is None or not self._odom:
            self._candidate_reject('historical_odometry_unavailable')
            return
        try:
            correction = _compose(_global_pose(candidate.pose), _inverse(local))
            if not all(math.isfinite(v) for v in correction.p + correction.q):
                raise ValueError('nonfinite correction')
        except ValueError as exc:
            self._candidate_reject(str(exc))
            return
        self._last_candidate_stamp_ns = stamp
        self._last_candidate_epoch = int(candidate.request_epoch)
        self._last_candidate_job = candidate.job_id
        self._last_candidate = candidate
        # Limit the change at the robot's CURRENT odom pose. Limiting only the
        # map->odom origin ignores the displacement induced by yaw at distance.
        current_odom = _odom_pose(self._odom[-1])
        new_base = _compose(correction, current_odom)
        if self._correction_current is not None:
            old_base = _compose(self._correction_current, current_odom)
            displacement = _translation_delta(old_base, new_base)
            yaw_delta = abs(_angle_wrap(_yaw(new_base.q) - _yaw(old_base.q)))
            if (displacement > float(self.get_parameter('recovery_max_robot_displacement_m').value)
                    or yaw_delta > math.radians(float(self.get_parameter('recovery_max_yaw_delta_deg').value))
                    or _rotation_delta(old_base,new_base) > math.radians(float(self.get_parameter('recovery_max_rotation_delta_deg').value))):
                self._invalidate_anchor('recovery_candidate_exceeds_trust_region')
                self._candidate_reject('recovery_candidate_exceeds_trust_region')
                return
        previous = self._last_recovery_candidate
        if previous is not None:
            prev_base = _compose(previous, current_odom)
            consistent = (_translation_delta(prev_base, new_base) <= float(self.get_parameter('recovery_consistency_m').value)
                          and _rotation_delta(prev_base,new_base)
                          <= math.radians(float(self.get_parameter('recovery_consistency_yaw_deg').value)))
            self._recovery_candidate_count = self._recovery_candidate_count + 1 if consistent else 1
        else:
            self._recovery_candidate_count = 1
        self._last_recovery_candidate = correction
        self._global_valid = False
        needed = max(2, int(self.get_parameter('recovery_consecutive_candidates').value))
        if self._recovery_candidate_count < needed:
            self._state = LocalizationState.DEGRADED
            self._reason = 'recovery_waiting_candidate_consistency'
            self._next_research_query_ns = now + int(max(0.1, float(self.get_parameter('research_candidate_retry_sec').value)) * 1e9)
            return
        if self._correction_current is None:
            # No map TF exists until independent consistent candidates agree.
            # Establish an unverified anchor for shadow matching; task/global
            # authority remains disabled until new observations verify it.
            self._correction_current = correction
            self._last_global_accept_ns = now
            self._recovery.global_pose_accepted()
        self._correction_target = correction
        self._next_research_query_ns = 0
        self._recovery_job_id = candidate.job_id
        self._recovery_start = self._correction_current
        self._recovery_start_ns = now
        self._recovery_blending = True
        self._recovery_verify_count = 0
        self._latest_observation = None
        self._state = LocalizationState.DEGRADED
        self._reason = 'recovery_blending_global_correction'
        self._recovery.pending = False
        self._recovery.requested = False

    def _request_next_research_candidate(self, now_ns):
        """Finish the candidate protocol after one initial user request.

        This schedules observation work only, with no task/motion permission
        and no anchor invalidation. The backend remains the single job owner.
        """
        candidate = self._last_candidate
        needed = max(2, int(self.get_parameter('recovery_consecutive_candidates').value))
        if (not self._research_enabled() or not bool(self.get_parameter('research_recovery_calibrated').value)
                or self._recovery_blending or candidate is None
                or not 0 < self._recovery_candidate_count < needed
                or self._next_research_query_ns <= 0 or now_ns < self._next_research_query_ns
                or not self._fresh_odom_quality() or not self._odom
                or self._local_age() > float(self.get_parameter('local_odom_timeout_sec').value)):
            return
        expected = tuple(str(self.get_parameter(name).value) for name in ('map_id','map_version','map_hash'))
        if ((candidate.map_id,candidate.map_version,candidate.map_hash) != expected
                or int(candidate.odom_epoch) != self._odom_epoch):
            return
        source_ns = _stamp_ns(self._odom[-1].header.stamp)
        if source_ns <= max(self._last_candidate_stamp_ns,self._last_research_query_odom_ns):
            return
        if self._backend_debug_state in {'BBS_SEARCHING','BBS_COARSE_FOUND','GICP_REFINING','BUSY','COLLECTING','WAIT_STATIONARY'}:
            return
        self._request_pub.publish(Empty())
        self._last_research_query_odom_ns = source_ns
        self._next_research_query_ns = now_ns + int(max(0.1, float(self.get_parameter('research_candidate_retry_sec').value)) * 1e9)
        self._reason = 'recovery_next_independent_query_requested'

    def _on_quality_observation(self, msg):
        # This is a read-only tracker observation. It never supplies correction
        # or turns TRACKING_OK into localization acceptance.
        if not self._research_enabled():
            return
        now = self.get_clock().now().nanoseconds
        expected = (str(self.get_parameter('map_id').value), str(self.get_parameter('map_version').value),
                    str(self.get_parameter('map_hash').value))
        if ((msg.map_id, msg.map_version, msg.map_hash) != expected
                or int(msg.odom_epoch) != self._odom_epoch or not msg.job_id
                or msg.header.frame_id != self.get_parameter('map_frame').value
                or _stamp_ns(msg.header.stamp) <= self._last_observation_stamp_ns
                or not 0 <= (now - _stamp_ns(msg.header.stamp)) / 1e9
                    <= float(self.get_parameter('global_quality_timeout_sec').value)):
            return
        self._last_observation_stamp_ns = _stamp_ns(msg.header.stamp)
        self._latest_observation = msg
        good = (msg.valid and self._fresh_odom_quality()
                and all(math.isfinite(v) for v in (msg.quality, msg.residual, msg.overlap,
                                                   msg.translation_innovation_m, msg.yaw_innovation_rad))
                and msg.quality >= float(self.get_parameter('candidate_min_quality').value)
                and msg.residual <= float(self.get_parameter('candidate_max_residual').value)
                and msg.overlap >= float(self.get_parameter('candidate_min_overlap').value)
                and msg.translation_innovation_m <= float(self.get_parameter('recovery_consistency_m').value)
                and msg.yaw_innovation_rad <= math.radians(float(self.get_parameter('recovery_consistency_yaw_deg').value)))
        if not good:
            self._recovery_verify_count = 0
            self._global_valid = False
            self._state = LocalizationState.DEGRADED
            self._reason = 'global_observation_degraded:' + msg.reason
            return
        if self._recovery_blending and self._odom and _stamp_ns(msg.header.stamp) >= self._recovery_start_ns:
            odom = _odom_pose(self._odom[-1])
            delta = _translation_delta(_compose(self._correction_current, odom), _compose(self._correction_target, odom))
            angle = _rotation_delta(self._correction_current,self._correction_target)
            if (delta <= float(self.get_parameter('recovery_settle_translation_m').value)
                    and angle <= math.radians(float(self.get_parameter('recovery_settle_yaw_deg').value))):
                self._recovery_verify_count += 1
                if self._recovery_verify_count >= max(2, int(self.get_parameter('recovery_verification_count').value)):
                    self._recovery_blending = False
                    self._global_valid = True
                    self._last_global_accept_ns = now
                    self._state = LocalizationState.TRACKING
                    self._reason = 'recovery_settled_and_latest_observations_verified'
            else:
                self._recovery_verify_count = 0
        elif self._global_valid:
            self._state = LocalizationState.TRACKING
            self._reason = 'latest_global_observation_verified'

    def _smooth_research_correction(self, now_ns, dt):
        if not self._recovery_blending or not self._odom or not self._fresh_odom_quality():
            return
        if not bool(self.get_parameter('research_recovery_calibrated').value):
            return
        duration = max(1.0e-3, float(self.get_parameter('recovery_blend_duration_sec').value))
        phase = max(0.0, min(1.0, (now_ns - self._recovery_start_ns) / (duration * 1e9)))
        desired = _se3_interpolate(self._recovery_start, self._correction_target, phase * phase * (3.0 - 2.0 * phase))
        odom = _odom_pose(self._odom[-1])
        current_base = _compose(self._correction_current, odom)
        max_move = max(0.0, float(self.get_parameter('max_correction_linear_rate_mps').value)) * dt
        max_angle = math.radians(max(0.0, float(self.get_parameter('max_correction_yaw_rate_degps').value))) * dt
        lo, hi = 0.0, 1.0
        for _ in range(24):
            alpha = (lo + hi) / 2.0
            proposed = _slerp(self._correction_current, desired, alpha)
            proposed_base = _compose(proposed, odom)
            move = _translation_delta(current_base, proposed_base)
            angle = _rotation_delta(current_base,proposed_base)
            if move <= max_move and angle <= max_angle:
                lo = alpha
            else:
                hi = alpha
        self._correction_current = _slerp(self._correction_current, desired, lo)

    def _publish_recovery_status(self):
        out = RecoveryStatus()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(self.get_parameter('map_frame').value)
        out.map_id, out.map_version = str(self.get_parameter('map_id').value), str(self.get_parameter('map_version').value)
        out.map_hash = str(self.get_parameter('map_hash').value)
        out.odom_epoch = self._odom_epoch
        out.job_id = self._recovery_job_id
        out.active = bool(self._research_enabled() and self._recovery_blending)
        out.remaining_translation_m = out.remaining_yaw_rad = 1.0e9
        if self._correction_current and self._correction_target and self._odom:
            odom = _odom_pose(self._odom[-1])
            out.remaining_translation_m = _translation_delta(_compose(self._correction_current, odom), _compose(self._correction_target, odom))
            out.remaining_yaw_rad = abs(_angle_wrap(_yaw(self._correction_current.q) - _yaw(self._correction_target.q)))
        observation = self._latest_observation
        fresh = (observation is not None and 0 <= (self.get_clock().now().nanoseconds - _stamp_ns(observation.header.stamp)) / 1e9
                 <= float(self.get_parameter('global_quality_timeout_sec').value))
        if observation is not None:
            out.verified_observation_stamp = observation.header.stamp
        out.complete = bool(self._research_enabled() and self._global_valid and not self._recovery_blending
                            and self._recovery_job_id and fresh and observation.valid and self._fresh_odom_quality()
                            and out.remaining_translation_m <= float(self.get_parameter('recovery_settle_translation_m').value)
                            and out.remaining_yaw_rad <= math.radians(float(self.get_parameter('recovery_settle_yaw_deg').value))
                            and self._correction_current is not None and self._correction_target is not None
                            and _rotation_delta(self._correction_current,self._correction_target)
                                <= math.radians(float(self.get_parameter('recovery_settle_yaw_deg').value)))
        out.reason = self._reason
        self._recovery_status_pub.publish(out)

    def _publish_global_quality(self):
        now = self.get_clock().now().nanoseconds
        out = GlobalQuality()
        observation = self._latest_observation
        if observation is not None:
            import copy
            out = copy.deepcopy(observation)
            out.job_id = self._recovery_job_id
        else:
            out.header.frame_id = str(self.get_parameter('map_frame').value)
            out.header.stamp = self.get_clock().now().to_msg()
            out.map_id, out.map_version = str(self.get_parameter('map_id').value), str(self.get_parameter('map_version').value)
            out.map_hash = str(self.get_parameter('map_hash').value)
            out.odom_epoch = self._odom_epoch
            out.position_std_m = out.yaw_std_rad = 1.0e9
        fresh = (observation is not None and 0 <= (now - _stamp_ns(observation.header.stamp)) / 1e9
                 <= float(self.get_parameter('global_quality_timeout_sec').value))
        out.valid = bool(self._research_enabled() and self._global_valid and fresh and observation.valid
                         and self._fresh_odom_quality()
                         and bool(self.get_parameter('research_recovery_calibrated').value))
        out.reason = self._reason if out.valid else 'unverified_or_expired_global_quality:' + self._reason
        self._global_quality_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.get_parameter('odom_frame').value:
            self.get_logger().warn(
                f'Ignoring local odom with frame {msg.header.frame_id!r}',
                throttle_duration_sec=2.0,
            )
            return
        if msg.child_frame_id != self.get_parameter('base_frame').value:
            self.get_logger().warn(
                f'Ignoring local odom child frame {msg.child_frame_id!r}',
                throttle_duration_sec=2.0,
            )
            return

        source_ns = _stamp_ns(msg.header.stamp)
        if source_ns <= 0:
            return
        try:
            pose = _odom_pose(msg)
            if not all(math.isfinite(v) for v in pose.p + pose.q):
                return
        except ValueError:
            return
        if self._odom and source_ns == _stamp_ns(self._odom[-1].header.stamp):
            return
        if self._odom and source_ns < _stamp_ns(self._odom[-1].header.stamp):
            self._invalidate_anchor('odometry_source_time_reset', clear=True)
            self._odom.clear()
            self._odom_quality = None
        self._odom.append(msg)
        self._last_odom_rx_ns = self.get_clock().now().nanoseconds
        newest_ns = _stamp_ns(msg.header.stamp)
        keep_ns = int(float(self.get_parameter('odom_buffer_sec').value) * 1e9)
        while self._odom and newest_ns - _stamp_ns(self._odom[0].header.stamp) > keep_ns:
            self._odom.popleft()

    def _nearest_odom(self, target_ns: int) -> Optional[Odometry]:
        if not self._odom:
            return None
        sample = min(self._odom, key=lambda m: abs(_stamp_ns(m.header.stamp) - target_ns))
        skew = abs(_stamp_ns(sample.header.stamp) - target_ns) / 1e9
        if skew > float(self.get_parameter('global_match_max_skew_sec').value):
            return None
        return sample

    def _bracketed_odom_pose(self, target_ns):
        """Use the same reference time as deskew, without nearest/extrapolation."""
        samples = list(self._odom)
        stamps = [_stamp_ns(msg.header.stamp) for msg in samples]
        index = bisect.bisect_left(stamps, target_ns)
        if index < len(samples) and stamps[index] == target_ns:
            return _odom_pose(samples[index])
        if index == 0 or index == len(samples):
            return None
        a, b = stamps[index - 1], stamps[index]
        if b <= a or (b - a) / 1e9 > float(self.get_parameter('research_odom_interpolation_max_gap_sec').value):
            return None
        return _slerp(_odom_pose(samples[index - 1]), _odom_pose(samples[index]), (target_ns - a) / (b - a))

    def _validate_global(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != self.get_parameter('map_frame').value:
            return False, math.inf, math.inf, 'global_pose_wrong_frame'
        target_ns = _stamp_ns(msg.header.stamp)
        if target_ns <= 0:
            return False, math.inf, math.inf, 'global_pose_zero_stamp'

        cov = msg.pose.covariance
        pose_values = tuple(getattr(msg.pose.pose.position, k) for k in ('x', 'y', 'z')) + tuple(getattr(msg.pose.pose.orientation, k) for k in ('x', 'y', 'z', 'w'))
        if not all(math.isfinite(float(v)) for v in tuple(cov) + pose_values) or any(float(cov[k]) < 0.0 for k in (0, 7, 14, 35)):
            return False, math.inf, math.inf, 'global_pose_nonfinite_or_negative_covariance'
        pos_var = max(float(cov[0]), float(cov[7]), float(cov[14]), 0.0)
        yaw_var = max(float(cov[35]), 0.0)
        all_zero = all(abs(float(v)) < 1e-12 for v in cov)
        if all_zero and not bool(self.get_parameter('accept_zero_covariance').value):
            return False, math.inf, math.inf, 'global_pose_missing_covariance'

        pos_std = math.sqrt(pos_var)
        yaw_std_deg = math.degrees(math.sqrt(yaw_var))
        if pos_std > float(self.get_parameter('max_global_position_std_m').value):
            return False, pos_std, yaw_std_deg, 'global_position_uncertainty_too_large'
        if yaw_std_deg > float(self.get_parameter('max_global_yaw_std_deg').value):
            return False, pos_std, yaw_std_deg, 'global_yaw_uncertainty_too_large'
        return True, pos_std, yaw_std_deg, 'accepted'

    def _on_global_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._research_enabled():
            return  # Avoid double application/bypass of the typed candidate protocol.
        ok, pos_std, yaw_std, reason = self._validate_global(msg)
        self._last_global_std = (pos_std, yaw_std)
        if not ok:
            self._reason = reason
            self.get_logger().warn(f'Rejected global pose: {reason}')
            return

        local = self._nearest_odom(_stamp_ns(msg.header.stamp))
        if local is None:
            self._reason = 'no_time_aligned_local_odom'
            self.get_logger().warn(
                'Rejected global pose: no local odometry sample within configured time skew.')
            return

        try:
            map_base = _global_pose(msg)
            odom_base = _odom_pose(local)
            correction = _compose(map_base, _inverse(odom_base))
        except ValueError as exc:
            self._reason = f'invalid_pose:{exc}'
            self.get_logger().error(self._reason)
            return

        # A global result is a hard anchor.  It is intentionally distinct from
        # the low-rate tracker, which only changes the target correction.
        self._correction_current = correction
        self._correction_target = correction
        self._global_valid = True
        self._last_global_accept_ns = self.get_clock().now().nanoseconds
        self._tracking_recovery_requested = False
        self._tracking_health = 'UNKNOWN'
        self._last_tracking_measurement = None
        self._tracking_consistent_count = 0
        self._tracking_innovation_bad = False
        self._recovery.global_pose_accepted()
        self._state = self._recovery.state
        self._reason = self._recovery.reason
        self._recovery_pending = False
        self.get_logger().info(
            f'Global correction accepted: position_std={pos_std:.3f}m, '
            f'yaw_std={yaw_std:.2f}deg')

    def _on_tracking_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Accept a local-map measurement without taking ownership of TF."""
        if self._research_enabled():
            return  # Read-only observation topic supplies research verification.
        if self._correction_current is None:
            self._reason = 'tracking_ignored_without_global_anchor'
            return
        if self._state in {
            LocalizationState.LOST,
            LocalizationState.RECOVERY_REQUESTED,
            LocalizationState.RELOCALIZING,
        }:
            self._reason = 'tracking_ignored_while_recovery_active'
            return
        ok, pos_std, yaw_std, reason = self._validate_global(msg)
        if not ok:
            self._reason = f'tracking_rejected:{reason}'
            return
        local = self._nearest_odom(_stamp_ns(msg.header.stamp))
        if local is None:
            self._reason = 'tracking_rejected:no_time_aligned_local_odom'
            return
        try:
            measurement = _compose(_global_pose(msg), _inverse(_odom_pose(local)))
            predicted_base = _compose(self._correction_current, _odom_pose(local))
            measured_base = _global_pose(msg)
            translation = _translation_delta(predicted_base, measured_base)
            yaw_delta = abs(_angle_wrap(_yaw(measured_base.q) - _yaw(predicted_base.q)))
            self._tracking_innovation_translation = translation
            self._tracking_innovation_yaw = yaw_delta
        except ValueError as exc:
            self._reason = f'tracking_rejected:invalid_pose:{exc}'
            return

        max_translation = float(self.get_parameter('max_tracking_translation_innovation_m').value)
        max_yaw = math.radians(float(self.get_parameter('max_tracking_yaw_innovation_deg').value))
        if _tracking_innovation_exceeds(translation, yaw_delta, max_translation, max_yaw):
            self._tracking_consistent_count = 0
            self._last_tracking_measurement = None
            self._tracking_health = 'DEGRADED'
            self._tracking_innovation_bad = True
            self._state = LocalizationState.DEGRADED
            self._reason = 'tracking_suspect:innovation_gate'
            self.get_logger().warn(
                f'Rejected map tracking innovation: translation={translation:.3f}m '
                f'yaw={math.degrees(yaw_delta):.2f}deg', throttle_duration_sec=2.0)
            return

        if self._last_tracking_measurement is not None:
            d = _translation_delta(measurement, self._last_tracking_measurement)
            dy = abs(_angle_wrap(_yaw(measurement.q) - _yaw(self._last_tracking_measurement.q)))
            if (d <= float(self.get_parameter('tracking_consistency_translation_m').value)
                    and dy <= math.radians(float(self.get_parameter('tracking_consistency_yaw_deg').value))):
                self._tracking_consistent_count += 1
            else:
                self._tracking_consistent_count = 1
        else:
            self._tracking_consistent_count = 1
        self._last_tracking_measurement = measurement

        needed = max(1, int(self.get_parameter('tracking_consecutive_accepts').value))
        if self._tracking_consistent_count < needed:
            self._reason = 'tracking_waiting_consistency'
            return
        self._correction_target = measurement
        self._tracking_health = 'TRACKING_OK'
        self._tracking_innovation_bad = False
        self._reason = 'map_tracking_target_updated'
        self._state = LocalizationState.TRACKING

    def _on_tracking_status(self, msg: String) -> None:
        """Consume tracker health without giving the tracker TF ownership."""
        try:
            payload = json.loads(msg.data)
            state = str(payload.get('state', '')).strip()
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not state:
            return
        source_ns = int(payload.get('stamp_ns', 0))
        now_ns = self.get_clock().now().nanoseconds
        if source_ns and (source_ns <= self._last_tracking_status_source_ns or not 0 <= (now_ns-source_ns)/1e9 <= float(self.get_parameter('tracking_status_timeout_sec').value)):
            return
        if self._research_enabled():
            return
        self._last_tracking_status_rx_ns = now_ns
        self._last_tracking_status_source_ns = source_ns
        if state == 'TRACKING_OK':
            # A backend status cannot override this owner's innovation or
            # consecutive-consistency rejection for the accompanying pose.
            if self._tracking_innovation_bad or self._tracking_consistent_count < max(1, int(self.get_parameter('tracking_consecutive_accepts').value)):
                return
            self._tracking_health = state
            self._tracking_recovery_requested = False
            self._tracking_innovation_bad = False
            if (self._correction_current is not None and self._state not in {
                    LocalizationState.LOST,
                    LocalizationState.RECOVERY_REQUESTED,
                    LocalizationState.RELOCALIZING,
            }):
                self._state = LocalizationState.TRACKING
                self._reason = 'map_tracking_ok'
            return
        self._tracking_health = state
        if state == 'RECOVERY_REQUIRED':
            self._mark_tracking_lost('map_tracking_recovery_required')
        elif state == 'DEGRADED':
            if self._state not in {
                LocalizationState.LOST,
                LocalizationState.RECOVERY_REQUESTED,
                LocalizationState.RELOCALIZING,
            }:
                self._state = LocalizationState.DEGRADED
                self._reason = 'map_tracking_degraded'
        elif state == 'HOLD':
            self._reason = 'map_tracking_hold'

    def _on_backend_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            state = str(payload.get('state', '')).strip()
            detail = str(payload.get('detail', '')).strip()
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not state:
            return

        self._backend_debug_state = state
        if self._correction_current is None and state in {
                'WAIT_MANUAL_INITIAL_POSE', 'MANUAL_SEED_REJECTED', 'MANUAL_GICP_REJECTED'}:
            self._state = LocalizationState.WAIT_GLOBAL
            self._reason = 'waiting_manual_initial_pose' if state == 'WAIT_MANUAL_INITIAL_POSE' else state.lower()
            return
        if self._correction_current is None and state == 'MANUAL_GICP_REFINING':
            self._state = LocalizationState.RELOCALIZING
            self._reason = 'manual_gicp_refining'
            return
        if state in {
                'WAIT_STATIONARY', 'COLLECTING', 'QUERY_READY',
                'BBS_SEARCHING', 'BBS_COARSE_FOUND', 'GICP_REFINING'}:
            if self._correction_current is None:
                self._state = LocalizationState.RELOCALIZING
                self._reason = f'global_relocalization:{state.lower()}'
            return

        if state in {'FAILED', 'REJECTED'} and self._correction_current is None:
            # A terminal backend failure must not leave the public state stuck
            # at RELOCALIZING forever.  WAIT_GLOBAL remains fail-closed while
            # making it explicit that a fresh request is required.
            self._state = LocalizationState.WAIT_GLOBAL
            suffix = detail.replace('\n', ' ')[:160] if detail else 'unspecified'
            self._reason = f'global_relocalization_{state.lower()}:{suffix}'
            self._recovery_pending = False
            return

    def _on_map_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if event.get('type') != 'MAP_ACTIVATED':
            return
        event_id = str(event.get('map_id', '')).strip()
        event_version = str(event.get('map_version', event.get('version', ''))).strip()
        configured = (str(self.get_parameter('map_id').value).strip(),
                      str(self.get_parameter('map_version').value).strip())
        if event_id and event_version and configured != (event_id, event_version):
            self._invalidate_anchor('active_map_changed', clear=True)
            self._state = LocalizationState.RELOCALIZING
            self._reason = f'active_map_changed:{event_id}/{event_version}'
            self.get_logger().warn(
                f'Active map changed to {event_id}/{event_version}; waiting for relocalization.')

    def _on_relocalize(self, request, response):
        del request
        self._cancel_pub.publish(Empty())
        self._invalidate_anchor('manual_relocalization_requested')
        self._state = LocalizationState.RECOVERY_REQUESTED
        self._reason = 'manual_relocalization_requested'
        self._request_pub.publish(Empty())
        self._state = LocalizationState.RELOCALIZING
        self._reason = 'relocalization_requested'
        response.success = True
        response.message = 'global correction invalidated and relocalization requested'
        return response

    def _local_age(self) -> float:
        if self._last_odom_rx_ns <= 0:
            return math.inf
        now = self.get_clock().now().nanoseconds
        if not self._odom:
            return math.inf
        source_age = (now - _stamp_ns(self._odom[-1].header.stamp)) / 1e9
        if source_age < -0.05:
            return math.inf
        return max(0.0, source_age, (now - self._last_odom_rx_ns) / 1e9)

    def _mark_tracking_lost(self, reason: str) -> None:
        """Move through LOST and, if enabled, schedule global recovery."""
        # The previous correction must not remain advertised as valid while
        # the tracker has declared the map alignment lost.
        self._invalidate_anchor(reason)
        self._recovery.tracking_failure(
            reason,
            bool(self.get_parameter('tracking_recovery_auto_request').value),
        )
        self._state = self._recovery.state
        self._reason = self._recovery.reason
        self._recovery_pending = self._recovery.pending
        self._try_request_recovery()

    def _try_request_recovery(self) -> bool:
        if not self._recovery.pending or self._recovery.requested:
            return False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not self._recovery.try_request(now_sec):
            remaining = self._recovery.cooldown_remaining(now_sec)
            self._reason = f'recovery_cooldown:{remaining:.2f}s'
            return False

        # RECOVERY_REQUESTED is an intentional transition even though the
        # request is published immediately and the steady state becomes
        # RELOCALIZING in the same callback.
        self._state = self._recovery.state
        self._reason = 'global_relocalization_requested'
        self._tracking_recovery_requested = self._recovery.requested
        self._recovery_pending = self._recovery.pending
        self._request_pub.publish(Empty())
        self._state = LocalizationState.RELOCALIZING
        self._reason = 'relocalization_requested'
        self.get_logger().warn(
            'Map tracker requested recovery; global relocalization was triggered.')
        return True

    def _update_state(self) -> None:
        self._try_request_recovery()
        if self._research_enabled():
            if self._local_age() > float(self.get_parameter('local_odom_timeout_sec').value) or not self._fresh_odom_quality():
                self._global_valid = False
                self._state = LocalizationState.LOST
                self._reason = 'research_local_odom_unusable'
                return
            if self._latest_observation is None or (self.get_clock().now().nanoseconds - _stamp_ns(self._latest_observation.header.stamp)) / 1e9 > float(self.get_parameter('global_quality_timeout_sec').value):
                self._global_valid = False
                self._state = LocalizationState.DEGRADED if self._correction_current else LocalizationState.WAIT_GLOBAL
                self._reason = 'latest_global_observation_unavailable'
            elif not self._global_valid:
                self._state = LocalizationState.DEGRADED if self._correction_current else LocalizationState.WAIT_GLOBAL
            return
        if self._last_tracking_status_rx_ns and (self.get_clock().now().nanoseconds - self._last_tracking_status_rx_ns) / 1e9 > float(self.get_parameter('tracking_status_timeout_sec').value):
            self._tracking_health = 'DEGRADED'
            self._reason = 'map_tracking_status_expired'
        if self._state in {
            LocalizationState.RECOVERY_REQUESTED,
            LocalizationState.RELOCALIZING,
        }:
            return
        if self._state == LocalizationState.LOST and not _local_odom_loss_recoverable(
                self._state, self._reason, self._correction_current is not None):
            return

        age = self._local_age()
        timeout = float(self.get_parameter('local_odom_timeout_sec').value)
        lost = float(self.get_parameter('local_odom_lost_sec').value)

        if self._last_odom_rx_ns <= 0:
            self._state = LocalizationState.WAIT_GLOBAL
            self._reason = 'waiting_local_odom'
            return
        if self._correction_current is None:
            self._state = LocalizationState.WAIT_GLOBAL
            # Keep the backend's terminal result visible until a new request
            # changes state. Otherwise this periodic tick hides REJECTED before
            # the field launcher can stop waiting and report the real error.
            if not self._reason.startswith((
                    'global_relocalization_failed:',
                    'global_relocalization_rejected:')):
                self._reason = ('waiting_manual_initial_pose' if self._backend_debug_state in {
                    'WAIT_MANUAL_INITIAL_POSE', 'MANUAL_SEED_REJECTED', 'MANUAL_GICP_REJECTED'}
                    else 'waiting_global_pose')
            return
        if age > lost:
            self._state = LocalizationState.LOST
            self._reason = f'local_odom_lost:{age:.2f}s'
        elif age > timeout:
            self._state = LocalizationState.DEGRADED
            self._reason = f'local_odom_stale:{age:.2f}s'
        elif self._tracking_health in {'DEGRADED', 'RECOVERY_REQUIRED'}:
            self._state = LocalizationState.DEGRADED
            if self._tracking_health == 'RECOVERY_REQUIRED':
                self._reason = 'map_tracking_recovery_required'
            else:
                self._reason = 'map_tracking_degraded'
        else:
            self._state = (
                LocalizationState.TRACKING
                if self._tracking_health == 'TRACKING_OK'
                else LocalizationState.LOCALIZED)
            if self._reason.startswith('local_odom_'):
                self._reason = 'tracking'

    def _publish_tf(self) -> None:
        if self._correction_current is None and not bool(self.get_parameter('debug_identity_map_odom').value):
            return
        if self._correction_current is None:
            # Visualization-only fallback. It never changes the localization
            # state and is intentionally disabled in the formal configuration.
            p, q = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        elif self._research_enabled() and self._local_age() <= float(self.get_parameter('local_odom_timeout_sec').value) and self._fresh_odom_quality():
            # Last correction is continuity metadata; global_valid remains false.
            pass
        elif self._state not in (
            LocalizationState.LOCALIZED,
            LocalizationState.TRACKING,
            LocalizationState.DEGRADED,
        ):
            # Stop refreshing a dynamic TF once localization is LOST. The TF
            # buffer will naturally expire instead of making a stale transform
            # look permanently valid to Nav2.
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id = self.get_parameter('odom_frame').value
        if self._correction_current is not None:
            p = self._correction_current.p
            q = self._correction_current.q
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf.sendTransform(t)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = max(0.0, min(1.0, (now_ns - self._last_tick_ns) / 1e9))
        self._last_tick_ns = now_ns
        if self._research_enabled():
            self._request_next_research_candidate(now_ns)
            self._smooth_research_correction(now_ns, dt)
        elif self._correction_current is not None and self._correction_target is not None:
            if bool(self.get_parameter('correction_smoothing_enabled').value):
                tau = max(1.0e-3, float(self.get_parameter('correction_tau_sec').value))
                alpha = 1.0 - math.exp(-dt / tau)
                max_step = float(self.get_parameter('max_correction_linear_rate_mps').value) * dt
                distance = _translation_delta(self._correction_current, self._correction_target)
                if distance > 1.0e-9:
                    alpha = min(alpha, max_step / distance)
                yaw_distance = abs(_angle_wrap(_yaw(self._correction_target.q) - _yaw(self._correction_current.q)))
                max_yaw_step = math.radians(float(
                    self.get_parameter('max_correction_yaw_rate_degps').value)) * dt
                if yaw_distance > 1.0e-9:
                    alpha = min(alpha, max_yaw_step / yaw_distance)
                self._correction_current = _slerp(self._correction_current, self._correction_target, alpha)
            else:
                self._correction_current = self._correction_target
        self._update_state()
        self._publish_tf()
        self._publish_recovery_status()

    def _publish_status(self) -> None:
        self._update_state()
        age = self._local_age()
        out = LocalizationStatus()
        out.stamp = self.get_clock().now().to_msg()
        out.state = _wire_state(self._state)
        out.local_odom_fresh = math.isfinite(age) and age <= float(
            self.get_parameter('local_odom_timeout_sec').value)
        out.global_correction_valid = self._global_valid if self._research_enabled() else self._correction_current is not None
        out.local_odom_age_sec = float(age if math.isfinite(age) else 1.0e9)
        # A global correction is an anchor, not a periodic sensor reading. Its
        # usefulness does not expire merely because a new global pose is absent.
        out.global_correction_age_sec = ((self.get_clock().now().nanoseconds-self._last_global_accept_ns)/1e9 if self._last_global_accept_ns else 1.0e9) if self._research_enabled() else (0.0 if self._correction_current is not None else 1.0e9)
        out.global_position_std_m = float(
            self._last_global_std[0] if math.isfinite(self._last_global_std[0]) else 1.0e9)
        out.global_yaw_std_deg = float(
            self._last_global_std[1] if math.isfinite(self._last_global_std[1]) else 1.0e9)
        out.map_id = str(self.get_parameter('map_id').value)
        out.map_version = str(self.get_parameter('map_version').value)
        out.reason = self._reason
        self._status_pub.publish(out)
        debug = String()
        debug.data = self._debug_state()
        self._debug_status_pub.publish(debug)

    def _publish_metrics(self) -> None:
        self._update_state()
        if self._correction_current is not None:
            (self._correction_translation_delta,
             self._correction_yaw_delta) = _correction_delta(
                 self._correction_current, self._last_metrics_correction)
            self._last_metrics_correction = self._correction_current
        else:
            self._correction_translation_delta = 0.0
            self._correction_yaw_delta = 0.0

        # Keep diagnostics finite for bag/CSV consumers; 1e9 means unknown,
        # matching the existing LocalizationStatus convention.
        global_position_std = (
            self._last_global_std[0]
            if math.isfinite(self._last_global_std[0]) else 1.0e9)
        global_yaw_std = (
            math.radians(self._last_global_std[1])
            if math.isfinite(self._last_global_std[1]) else 1.0e9)
        metrics = _make_metrics_message(
            self._state,
            self._tracking_health,
            self._reason,
            self._correction_current,
            self._correction_translation_delta,
            self._correction_yaw_delta,
            self._tracking_innovation_translation,
            self._tracking_innovation_yaw,
            global_position_std,
            global_yaw_std,
            self.get_clock().now().to_msg(),
        )
        self._metrics_pub.publish(metrics)

    def _debug_state(self) -> str:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        return json.dumps({
            'state': self._state.value,
            'wire_state': int(_wire_state(self._state)),
            'reason': self._reason,
            'tracking_health': self._tracking_health,
            'backend_state': self._backend_debug_state,
            'recovery_pending': self._recovery.pending,
            'recovery_request_count': self._recovery.request_count,
            'recovery_cooldown_remaining_sec': round(
                self._recovery.cooldown_remaining(now_sec), 3),
        }, sort_keys=True)


def main(args=None) -> None:
    def _interrupt(_signum, _frame):
        raise KeyboardInterrupt

    node = None
    try:
        signal.signal(signal.SIGINT, _interrupt)
        signal.signal(signal.SIGTERM, _interrupt)
        # Unwind the executor before destroying its active subscriptions.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = LocalizationManager()
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
