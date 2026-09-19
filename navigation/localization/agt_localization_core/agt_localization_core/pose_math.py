"""Pure SE(3) helpers mirrored from the legacy localization manager.

This module intentionally has no ROS message or node dependency.  ``Pose3``
uses the same quaternion order as the legacy manager: ``(x, y, z, w)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose3:
    position: Vector3
    quaternion: Quaternion


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError('zero quaternion')
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    ax, ay, az, aw = left
    bx, by, bz, bw = right
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    q = normalize_quaternion(quaternion)
    rotated = quaternion_multiply(
        quaternion_multiply(q, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(q),
    )
    return rotated[0], rotated[1], rotated[2]


def inverse(pose: Pose3) -> Pose3:
    quaternion = quaternion_conjugate(normalize_quaternion(pose.quaternion))
    position = rotate_vector(quaternion, tuple(-value for value in pose.position))
    return Pose3(position, quaternion)


def compose(left: Pose3, right: Pose3) -> Pose3:
    right_position = rotate_vector(left.quaternion, right.position)
    position = tuple(left.position[index] + right_position[index] for index in range(3))
    quaternion = normalize_quaternion(quaternion_multiply(left.quaternion, right.quaternion))
    return Pose3(position, quaternion)


def map_to_odom(map_to_base: Pose3, odom_to_base: Pose3) -> Pose3:
    """Return the frozen correction contract: T_map_odom = T_map_base * inv(T_odom_base)."""
    return compose(map_to_base, inverse(odom_to_base))


def yaw(quaternion: Quaternion) -> float:
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def translation_delta(left: Pose3, right: Pose3) -> float:
    return math.sqrt(sum((left.position[index] - right.position[index]) ** 2 for index in range(3)))


def correction_delta(current: Optional[Pose3], previous: Optional[Pose3]) -> Tuple[float, float]:
    if current is None or previous is None:
        return 0.0, 0.0
    return translation_delta(current, previous), abs(wrap_to_pi(yaw(current.quaternion) - yaw(previous.quaternion)))


def interpolate_se3(start: Pose3, end: Pose3, alpha: float) -> Pose3:
    """Legacy-equivalent linear translation plus shortest-path quaternion SLERP."""
    alpha = max(0.0, min(1.0, float(alpha)))
    qa = normalize_quaternion(start.quaternion)
    qb = normalize_quaternion(end.quaternion)
    dot = sum(left * right for left, right in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-value for value in qb)  # type: ignore[assignment]
        dot = -dot
    if dot > 0.9995:
        quaternion = normalize_quaternion(
            tuple(left + alpha * (right - left) for left, right in zip(qa, qb)))
    else:
        theta = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta = math.sin(theta)
        start_weight = math.sin((1.0 - alpha) * theta) / sin_theta
        end_weight = math.sin(alpha * theta) / sin_theta
        quaternion = normalize_quaternion(
            tuple(start_weight * left + end_weight * right for left, right in zip(qa, qb)))
    position = tuple(
        start.position[index] + alpha * (end.position[index] - start.position[index])
        for index in range(3)
    )
    return Pose3(position, quaternion)
