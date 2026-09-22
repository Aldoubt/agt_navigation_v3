"""ROS-free, timestamp-aware base-frame twist estimation for navigation."""

from __future__ import annotations

import math
from agt_batch_lio_adapter.extrinsics import q_conj, q_mul, q_norm, rotate


def pose_delta_twist(previous_p, previous_q, current_p, current_q, dt):
    """Differentiate T_odom_base; express both velocities in base axes."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('pose delta dt must be positive')
    linear = rotate(q_conj(current_q), tuple(
        (current_p[i] - previous_p[i]) / dt for i in range(3)))
    delta = q_norm(q_mul(q_conj(previous_q), current_q))
    if delta[3] < 0.0:  # q and -q describe the same rotation.
        delta = tuple(-value for value in delta)
    sine = math.sqrt(sum(value * value for value in delta[:3]))
    if sine < 1.0e-10:
        angular = (0.0, 0.0, 0.0)
    else:
        angle = 2.0 * math.atan2(sine, max(0.0, delta[3]))
        angular = tuple(value * angle / (sine * dt) for value in delta[:3])
    return linear, angular


def limit_vector(vector, deadband, limit):
    values = tuple(0.0 if abs(value) < deadband else value for value in vector)
    norm = math.sqrt(sum(value * value for value in values))
    if norm > limit and norm > 0.0:
        values = tuple(value * limit / norm for value in values)
    return values


class PoseTwistEstimator:
    """Keep only accepted poses; never differentiate a clock reset or long gap.

    This bounds twist, not position: it is not a pose-quality or slip detector.
    The offset between IMU and base is already included in each base pose.
    """

    def __init__(self, min_dt=0.01, max_dt=0.50, linear_deadband=0.01,
                 angular_deadband=0.01, max_linear=2.0, max_angular=3.0):
        values = (min_dt, max_dt, linear_deadband, angular_deadband,
                  max_linear, max_angular)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('twist limits must be finite')
        if not (0.0 < min_dt <= max_dt and min(linear_deadband, angular_deadband) >= 0.0
                and min(max_linear, max_angular) > 0.0):
            raise ValueError('invalid twist interval or limits')
        self.min_dt, self.max_dt = min_dt, max_dt
        self.linear_deadband, self.angular_deadband = linear_deadband, angular_deadband
        self.max_linear, self.max_angular = max_linear, max_angular
        self.previous = None

    def update(self, position, quaternion, stamp_ns):
        position = tuple(position)
        quaternion = tuple(quaternion)
        if (stamp_ns <= 0 or len(position) != 3 or len(quaternion) != 4
                or not all(math.isfinite(value) for value in position + quaternion)):
            raise ValueError('pose and timestamp must be finite and valid')
        quaternion = q_norm(quaternion)
        zero = (0.0, 0.0, 0.0)
        result = (zero, zero)
        if self.previous is not None:
            previous_p, previous_q, previous_stamp = self.previous
            if stamp_ns <= previous_stamp:
                raise ValueError('non-increasing odometry timestamp')
            dt = (stamp_ns - previous_stamp) / 1.0e9
            if self.min_dt <= dt <= self.max_dt:
                linear, angular = pose_delta_twist(
                    previous_p, previous_q, position, quaternion, dt)
                result = (
                    limit_vector(linear, self.linear_deadband, self.max_linear),
                    limit_vector(angular, self.angular_deadband, self.max_angular),
                )
        self.previous = (position, quaternion, stamp_ns)
        return result
