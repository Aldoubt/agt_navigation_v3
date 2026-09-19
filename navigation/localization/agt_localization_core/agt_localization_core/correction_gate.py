"""Pure tracking innovation and correction smoothing logic."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .pose_math import Pose3, compose, interpolate_se3, translation_delta, wrap_to_pi, yaw


@dataclass(frozen=True)
class Innovation:
    translation_m: float
    yaw_rad: float


def tracking_innovation(current_map_to_odom: Pose3, odom_to_base: Pose3, measured_map_to_base: Pose3) -> Innovation:
    """Measure the legacy tracker innovation in the map/base frame."""
    predicted_map_to_base = compose(current_map_to_odom, odom_to_base)
    return Innovation(
        translation_m=translation_delta(predicted_map_to_base, measured_map_to_base),
        yaw_rad=abs(wrap_to_pi(yaw(measured_map_to_base.quaternion) - yaw(predicted_map_to_base.quaternion))),
    )


def exceeds_innovation_gate(innovation: Innovation, max_translation_m: float, max_yaw_rad: float) -> bool:
    return innovation.translation_m > max_translation_m or innovation.yaw_rad > max_yaw_rad


def accepts_tracking_observation(innovation: Innovation, max_translation_m: float, max_yaw_rad: float) -> bool:
    return not exceeds_innovation_gate(innovation, max_translation_m, max_yaw_rad)


def smooth_correction(
    current: Pose3,
    target: Pose3,
    dt_sec: float,
    *,
    smoothing_enabled: bool,
    tau_sec: float,
    max_linear_rate_mps: float,
    max_yaw_rate_radps: float,
) -> Pose3:
    """Mirror the legacy manager's correction tick, including both rate limits."""
    if not smoothing_enabled:
        return target
    dt = max(0.0, min(1.0, float(dt_sec)))
    tau = max(1.0e-3, float(tau_sec))
    alpha = 1.0 - math.exp(-dt / tau)
    distance = translation_delta(current, target)
    max_step = float(max_linear_rate_mps) * dt
    if distance > 1.0e-9:
        alpha = min(alpha, max_step / distance)
    yaw_distance = abs(wrap_to_pi(yaw(target.quaternion) - yaw(current.quaternion)))
    max_yaw_step = float(max_yaw_rate_radps) * dt
    if yaw_distance > 1.0e-9:
        alpha = min(alpha, max_yaw_step / yaw_distance)
    return interpolate_se3(current, target, alpha)
