"""Publish shadow-only diagnostics without touching the TF graph."""

from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from agt_localization_core.pose_math import Pose3, wrap_to_pi, yaw
from agt_localization_interfaces.msg import LocalizationState
from agt_robot_interfaces.msg import LocalizationMetrics, LocalizationStatus

from .legacy_bridge import legacy_state_name, public_v1_state


class ShadowDiagnosticsPublisher:
    """Publish ordinary messages; this class never creates a TF broadcaster."""

    def __init__(self, node) -> None:
        self._node = node
        self._state_pub = node.create_publisher(
            LocalizationState, node.get_parameter('shadow_state_topic').value, 10)
        self._map_odom_pub = node.create_publisher(
            TransformStamped, node.get_parameter('shadow_map_odom_topic').value, 10)
        self._diagnostics_pub = node.create_publisher(
            String, node.get_parameter('shadow_diagnostics_topic').value, 10)

    @staticmethod
    def _legacy_delta(correction: Optional[Pose3], metrics: Optional[LocalizationMetrics]):
        if correction is None or metrics is None:
            return None, None
        translation = math.sqrt(
            (correction.position[0] - metrics.map_odom_x) ** 2
            + (correction.position[1] - metrics.map_odom_y) ** 2
            + (correction.position[2] - metrics.map_odom_z) ** 2)
        yaw_delta = abs(wrap_to_pi(yaw(correction.quaternion) - metrics.map_odom_yaw))
        return translation, yaw_delta

    def publish(self, snapshot, stamp, legacy_status: Optional[LocalizationStatus],
                legacy_metrics: Optional[LocalizationMetrics], map_id: str, map_version: str) -> None:
        state = LocalizationState()
        state.stamp = stamp
        state.state = public_v1_state(snapshot.state, snapshot.has_local_odom)
        state.source = snapshot.source
        state.reason = snapshot.reason
        state.has_global_correction = snapshot.correction is not None
        state.tracking_enabled = snapshot.tracking_enabled
        state.recovery_requested = snapshot.recovery_requested
        state.map_id = map_id
        state.map_version = map_version
        self._state_pub.publish(state)

        if snapshot.correction is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = snapshot.map_frame
            transform.child_frame_id = snapshot.odom_frame
            transform.transform.translation.x = snapshot.correction.position[0]
            transform.transform.translation.y = snapshot.correction.position[1]
            transform.transform.translation.z = snapshot.correction.position[2]
            transform.transform.rotation.x = snapshot.correction.quaternion[0]
            transform.transform.rotation.y = snapshot.correction.quaternion[1]
            transform.transform.rotation.z = snapshot.correction.quaternion[2]
            transform.transform.rotation.w = snapshot.correction.quaternion[3]
            # This is a normal namespaced topic, deliberately not /tf.
            self._map_odom_pub.publish(transform)

        translation_delta, yaw_delta = self._legacy_delta(snapshot.correction, legacy_metrics)
        diagnostics = {
            'legacy_state': legacy_state_name(legacy_status) if legacy_status else 'UNAVAILABLE',
            'legacy_reason': legacy_status.reason if legacy_status else '',
            'v1_public_state': state.state,
            'v1_core_state': snapshot.state.value,
            'source': snapshot.source,
            'accept_reject_reason': snapshot.reason,
            'correction_decision': snapshot.decision,
            'shadow_has_correction': snapshot.correction is not None,
            'shadow_map_odom_x': snapshot.correction.position[0] if snapshot.correction else None,
            'shadow_map_odom_y': snapshot.correction.position[1] if snapshot.correction else None,
            'shadow_map_odom_z': snapshot.correction.position[2] if snapshot.correction else None,
            'shadow_map_odom_yaw': yaw(snapshot.correction.quaternion) if snapshot.correction else None,
            'legacy_shadow_translation_delta_m': translation_delta,
            'legacy_shadow_yaw_delta_rad': yaw_delta,
            'tracking_innovation_translation_m': snapshot.innovation_translation,
            'tracking_innovation_yaw_rad': snapshot.innovation_yaw,
            'recovery_would_request': snapshot.recovery_requested,
            'legacy_metrics_available': legacy_metrics is not None,
        }
        message = String()
        message.data = json.dumps(diagnostics, sort_keys=True)
        self._diagnostics_pub.publish(message)
