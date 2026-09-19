"""Read-only Localization v1 shadow manager.

This node computes a candidate map->odom correction but never publishes TF,
requests global relocalization, or changes legacy runtime behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from agt_localization_core.correction_gate import (
    accepts_tracking_observation,
    smooth_correction,
    tracking_innovation,
)
from agt_localization_core.odom_buffer import OdomBuffer, OdomSample
from agt_localization_core.pose_math import Pose3, correction_delta, map_to_odom, translation_delta, wrap_to_pi, yaw
from agt_localization_core.state_machine import LocalizationState as CoreState, RecoveryStateMachine
from agt_robot_interfaces.msg import LocalizationMetrics, LocalizationStatus

from .diagnostics_publisher import ShadowDiagnosticsPublisher
from .legacy_bridge import parse_tracker_status


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _pose_from_odometry(message: Odometry) -> Pose3:
    pose = message.pose.pose
    return Pose3(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def _pose_from_covariance(message: PoseWithCovarianceStamped) -> Pose3:
    pose = message.pose.pose
    return Pose3(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


@dataclass(frozen=True)
class ShadowConfig:
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    base_frame: str = 'base_link'
    odom_buffer_sec: float = 30.0
    global_match_max_skew_sec: float = 0.10
    local_odom_timeout_sec: float = 0.30
    local_odom_lost_sec: float = 1.00
    correction_smoothing_enabled: bool = True
    correction_tau_sec: float = 3.0
    max_correction_linear_rate_mps: float = 0.10
    max_correction_yaw_rate_degps: float = 2.0
    max_tracking_translation_innovation_m: float = 0.50
    max_tracking_yaw_innovation_deg: float = 5.0
    tracking_consecutive_accepts: int = 2
    tracking_consistency_translation_m: float = 0.20
    tracking_consistency_yaw_deg: float = 2.0
    tracking_recovery_auto_request: bool = True
    recovery_cooldown_sec: float = 5.0
    tracking_enabled: bool = True


@dataclass(frozen=True)
class ShadowSnapshot:
    state: CoreState
    correction: Optional[Pose3]
    source: str
    reason: str
    decision: str
    innovation_translation: float
    innovation_yaw: float
    has_local_odom: bool
    tracking_enabled: bool
    recovery_requested: bool
    map_frame: str
    odom_frame: str


class ShadowLocalizationModel:
    """Core-backed state model used by the shadow node and unit tests."""

    def __init__(self, config: ShadowConfig) -> None:
        self.config = config
        self.odom = OdomBuffer(config.odom_buffer_sec)
        self.recovery = RecoveryStateMachine(config.recovery_cooldown_sec)
        self.state = CoreState.BOOT
        self.correction: Optional[Pose3] = None
        self.target: Optional[Pose3] = None
        self.last_odom_rx_ns = 0
        self.last_tick_ns = 0
        self.last_tracking_measurement: Optional[Pose3] = None
        self.tracking_consistent_count = 0
        self.tracking_health = 'UNKNOWN'
        self.source = 'bootstrap'
        self.reason = 'boot'
        self.decision = 'NONE'
        self.innovation_translation = 0.0
        self.innovation_yaw = 0.0
        self.recovery_requested = False

    def observe_odom(self, stamp_ns: int, pose: Pose3, receipt_ns: int) -> None:
        self.odom.append(OdomSample(stamp_ns, pose))
        self.last_odom_rx_ns = receipt_ns

    def _nearest_odom(self, stamp_ns: int) -> Optional[OdomSample]:
        return self.odom.nearest(stamp_ns, self.config.global_match_max_skew_sec)

    def accept_global(self, stamp_ns: int, map_to_base: Pose3) -> bool:
        local = self._nearest_odom(stamp_ns)
        self.source = 'global_relocalization'
        if local is None:
            self.reason = 'no_time_aligned_local_odom'
            self.decision = 'REJECTED'
            return False
        self.correction = map_to_odom(map_to_base, local.pose)
        self.target = self.correction
        self.last_tracking_measurement = None
        self.tracking_consistent_count = 0
        self.tracking_health = 'UNKNOWN'
        self.recovery.global_pose_accepted()
        self.state = self.recovery.state
        self.reason = self.recovery.reason
        self.decision = 'ACCEPTED'
        self.recovery_requested = False
        return True

    def accept_tracking(self, stamp_ns: int, measured_map_to_base: Pose3) -> bool:
        self.source = 'local_tracker'
        if not self.config.tracking_enabled:
            self.reason = 'tracking_disabled'
            self.decision = 'IGNORED'
            return False
        if self.correction is None:
            self.reason = 'tracking_ignored_without_global_anchor'
            self.decision = 'IGNORED'
            return False
        if self.state in {CoreState.LOST, CoreState.RECOVERY_REQUESTED, CoreState.RELOCALIZING}:
            self.reason = 'tracking_ignored_while_recovery_active'
            self.decision = 'IGNORED'
            return False
        local = self._nearest_odom(stamp_ns)
        if local is None:
            self.reason = 'tracking_rejected:no_time_aligned_local_odom'
            self.decision = 'REJECTED'
            return False
        measurement = map_to_odom(measured_map_to_base, local.pose)
        innovation = tracking_innovation(self.correction, local.pose, measured_map_to_base)
        self.innovation_translation = innovation.translation_m
        self.innovation_yaw = innovation.yaw_rad
        if not accepts_tracking_observation(
            innovation,
            self.config.max_tracking_translation_innovation_m,
            math.radians(self.config.max_tracking_yaw_innovation_deg),
        ):
            self.tracking_consistent_count = 0
            self.last_tracking_measurement = None
            self.tracking_health = 'DEGRADED'
            self.state = CoreState.DEGRADED
            self.reason = 'tracking_suspect:innovation_gate'
            self.decision = 'REJECTED'
            return False
        if self.last_tracking_measurement is None:
            self.tracking_consistent_count = 1
        else:
            distance = translation_delta(measurement, self.last_tracking_measurement)
            yaw_distance = abs(wrap_to_pi(
                yaw(measurement.quaternion) - yaw(self.last_tracking_measurement.quaternion)))
            if (distance <= self.config.tracking_consistency_translation_m
                    and yaw_distance <= math.radians(self.config.tracking_consistency_yaw_deg)):
                self.tracking_consistent_count += 1
            else:
                self.tracking_consistent_count = 1
        self.last_tracking_measurement = measurement
        if self.tracking_consistent_count < max(1, self.config.tracking_consecutive_accepts):
            self.reason = 'tracking_waiting_consistency'
            self.decision = 'PENDING'
            return False
        self.target = measurement
        self.tracking_health = 'TRACKING_OK'
        self.state = CoreState.TRACKING
        self.reason = 'map_tracking_target_updated'
        self.decision = 'ACCEPTED'
        return True

    def observe_tracking_status(self, state: str, now_sec: float) -> None:
        if not state:
            return
        self.tracking_health = state
        self.source = 'local_tracker_status'
        if state == 'TRACKING_OK' and self.correction is not None:
            self.state = CoreState.TRACKING
            self.reason = 'map_tracking_ok'
            return
        if state == 'DEGRADED' and self.state not in {CoreState.LOST, CoreState.RELOCALIZING}:
            self.state = CoreState.DEGRADED
            self.reason = 'map_tracking_degraded'
            return
        if state == 'RECOVERY_REQUIRED':
            self.correction = None
            self.target = None
            self.recovery.tracking_failure(
                'map_tracking_recovery_required', self.config.tracking_recovery_auto_request)
            self.state = self.recovery.state
            self.reason = self.recovery.reason
            self.recovery_requested = self.recovery.try_request(now_sec)
            if self.recovery_requested:
                self.state = self.recovery.state
                self.reason = self.recovery.reason
            self.decision = 'WOULD_REQUEST_RECOVERY' if self.recovery_requested else 'RECOVERY_HELD'

    def tick(self, now_ns: int) -> None:
        dt = 0.0 if self.last_tick_ns == 0 else max(0.0, min(1.0, (now_ns - self.last_tick_ns) / 1.0e9))
        self.last_tick_ns = now_ns
        if self.correction is not None and self.target is not None:
            self.correction = smooth_correction(
                self.correction, self.target, dt,
                smoothing_enabled=self.config.correction_smoothing_enabled,
                tau_sec=self.config.correction_tau_sec,
                max_linear_rate_mps=self.config.max_correction_linear_rate_mps,
                max_yaw_rate_radps=math.radians(self.config.max_correction_yaw_rate_degps),
            )
        if self.state in {CoreState.LOST, CoreState.RECOVERY_REQUESTED, CoreState.RELOCALIZING}:
            return
        if self.last_odom_rx_ns <= 0:
            self.state = CoreState.WAIT_GLOBAL
            self.reason = 'waiting_local_odom'
            return
        age = max(0.0, (now_ns - self.last_odom_rx_ns) / 1.0e9)
        if self.correction is None:
            self.state = CoreState.WAIT_GLOBAL
            self.reason = 'waiting_global_pose'
        elif age > self.config.local_odom_lost_sec:
            self.state = CoreState.LOST
            self.reason = f'local_odom_lost:{age:.2f}s'
        elif age > self.config.local_odom_timeout_sec:
            self.state = CoreState.DEGRADED
            self.reason = f'local_odom_stale:{age:.2f}s'
        elif self.tracking_health in {'DEGRADED', 'RECOVERY_REQUIRED'}:
            self.state = CoreState.DEGRADED
            self.reason = 'map_tracking_degraded'
        elif self.tracking_health == 'TRACKING_OK':
            self.state = CoreState.TRACKING
        else:
            self.state = CoreState.LOCALIZED

    def snapshot(self) -> ShadowSnapshot:
        return ShadowSnapshot(
            state=self.state,
            correction=self.correction,
            source=self.source,
            reason=self.reason,
            decision=self.decision,
            innovation_translation=self.innovation_translation,
            innovation_yaw=self.innovation_yaw,
            has_local_odom=len(self.odom) > 0,
            tracking_enabled=self.config.tracking_enabled,
            recovery_requested=self.recovery_requested,
            map_frame=self.config.map_frame,
            odom_frame=self.config.odom_frame,
        )


class ShadowManagerNode(Node):
    """Read-only ROS wrapper around :class:`ShadowLocalizationModel`."""

    def __init__(self) -> None:
        super().__init__('agt_localization_shadow_manager')
        self._declare_parameters()
        self.model = ShadowLocalizationModel(self._config_from_parameters())
        self._legacy_status: Optional[LocalizationStatus] = None
        self._legacy_metrics: Optional[LocalizationMetrics] = None
        self._diagnostics = ShadowDiagnosticsPublisher(self)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self._on_odom, 100)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('global_pose_topic').value, self._on_global, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('tracking_pose_topic').value, self._on_tracking, 10)
        self.create_subscription(String, self.get_parameter('tracking_status_topic').value, self._on_tracking_status, 20)
        self.create_subscription(LocalizationStatus, self.get_parameter('legacy_status_topic').value, self._on_legacy_status, 10)
        self.create_subscription(LocalizationMetrics, self.get_parameter('legacy_metrics_topic').value, self._on_legacy_metrics, 10)
        rate = max(0.1, float(self.get_parameter('diagnostic_publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            'Localization v1 shadow manager started: diagnostics only; no TF broadcaster and no recovery request publisher.')

    def _declare_parameters(self) -> None:
        defaults = ShadowConfig()
        for name, value in {
            'local_odom_topic': '/agt/odometry/local',
            'global_pose_topic': '/agt/relocalization/pose',
            'tracking_pose_topic': '/agt/map_tracking/pose',
            'tracking_status_topic': '/agt/map_tracking/status',
            'legacy_status_topic': '/agt/localization/status',
            'legacy_metrics_topic': '/agt/localization/metrics',
            'shadow_state_topic': '/agt/localization/v1/shadow/state',
            'shadow_map_odom_topic': '/agt/localization/v1/shadow/map_odom',
            'shadow_diagnostics_topic': '/agt/localization/v1/shadow/diagnostics',
            'map_frame': defaults.map_frame,
            'odom_frame': defaults.odom_frame,
            'base_frame': defaults.base_frame,
            'odom_buffer_sec': defaults.odom_buffer_sec,
            'global_match_max_skew_sec': defaults.global_match_max_skew_sec,
            'local_odom_timeout_sec': defaults.local_odom_timeout_sec,
            'local_odom_lost_sec': defaults.local_odom_lost_sec,
            'correction_smoothing_enabled': defaults.correction_smoothing_enabled,
            'correction_tau_sec': defaults.correction_tau_sec,
            'max_correction_linear_rate_mps': defaults.max_correction_linear_rate_mps,
            'max_correction_yaw_rate_degps': defaults.max_correction_yaw_rate_degps,
            'max_tracking_translation_innovation_m': defaults.max_tracking_translation_innovation_m,
            'max_tracking_yaw_innovation_deg': defaults.max_tracking_yaw_innovation_deg,
            'tracking_consecutive_accepts': defaults.tracking_consecutive_accepts,
            'tracking_consistency_translation_m': defaults.tracking_consistency_translation_m,
            'tracking_consistency_yaw_deg': defaults.tracking_consistency_yaw_deg,
            'tracking_recovery_auto_request': defaults.tracking_recovery_auto_request,
            'recovery_cooldown_sec': defaults.recovery_cooldown_sec,
            'tracking_enabled': defaults.tracking_enabled,
            'max_global_position_std_m': 1.0,
            'max_global_yaw_std_deg': 20.0,
            'accept_zero_covariance': False,
            'diagnostic_publish_rate_hz': 5.0,
            'map_id': '',
            'map_version': '',
        }.items():
            self.declare_parameter(name, value)

    def _config_from_parameters(self) -> ShadowConfig:
        values = {name: self.get_parameter(name).value for name in ShadowConfig.__dataclass_fields__}
        return ShadowConfig(**values)

    def _on_odom(self, message: Odometry) -> None:
        if message.header.frame_id != self.model.config.odom_frame:
            self.get_logger().warn('Ignoring shadow local odom with unexpected parent frame', throttle_duration_sec=2.0)
            return
        if message.child_frame_id != self.model.config.base_frame:
            self.get_logger().warn('Ignoring shadow local odom with unexpected child frame', throttle_duration_sec=2.0)
            return
        self.model.observe_odom(_stamp_ns(message.header.stamp), _pose_from_odometry(message), self.get_clock().now().nanoseconds)

    def _validate_global(self, message: PoseWithCovarianceStamped) -> Optional[str]:
        if message.header.frame_id != self.model.config.map_frame:
            return 'global_pose_wrong_frame'
        if _stamp_ns(message.header.stamp) <= 0:
            return 'global_pose_zero_stamp'
        covariance = message.pose.covariance
        all_zero = all(abs(float(value)) < 1.0e-12 for value in covariance)
        if all_zero and not bool(self.get_parameter('accept_zero_covariance').value):
            return 'global_pose_missing_covariance'
        position_std = math.sqrt(max(float(covariance[0]), float(covariance[7]), float(covariance[14]), 0.0))
        yaw_std_deg = math.degrees(math.sqrt(max(float(covariance[35]), 0.0)))
        if position_std > float(self.get_parameter('max_global_position_std_m').value):
            return 'global_position_uncertainty_too_large'
        if yaw_std_deg > float(self.get_parameter('max_global_yaw_std_deg').value):
            return 'global_yaw_uncertainty_too_large'
        return None

    def _on_global(self, message: PoseWithCovarianceStamped) -> None:
        invalid_reason = self._validate_global(message)
        if invalid_reason:
            self.model.source = 'global_relocalization'
            self.model.reason = invalid_reason
            self.model.decision = 'REJECTED'
            return
        self.model.accept_global(_stamp_ns(message.header.stamp), _pose_from_covariance(message))

    def _on_tracking(self, message: PoseWithCovarianceStamped) -> None:
        invalid_reason = self._validate_global(message)
        if invalid_reason:
            self.model.source = 'local_tracker'
            self.model.reason = f'tracking_rejected:{invalid_reason}'
            self.model.decision = 'REJECTED'
            return
        self.model.accept_tracking(_stamp_ns(message.header.stamp), _pose_from_covariance(message))

    def _on_tracking_status(self, message: String) -> None:
        payload = parse_tracker_status(message.data)
        self.model.observe_tracking_status(str(payload.get('state', '')).strip(), self.get_clock().now().nanoseconds / 1.0e9)

    def _on_legacy_status(self, message: LocalizationStatus) -> None:
        self._legacy_status = message

    def _on_legacy_metrics(self, message: LocalizationMetrics) -> None:
        self._legacy_metrics = message

    def _on_tick(self) -> None:
        now = self.get_clock().now()
        self.model.tick(now.nanoseconds)
        self._diagnostics.publish(
            self.model.snapshot(), now.to_msg(), self._legacy_status, self._legacy_metrics,
            str(self.get_parameter('map_id').value), str(self.get_parameter('map_version').value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShadowManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
