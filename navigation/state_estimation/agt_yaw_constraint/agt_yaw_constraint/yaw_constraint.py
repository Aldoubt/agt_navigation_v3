"""Observation-only wheel/LIO yaw diagnostics with explicit sync accounting."""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import rclpy
from agt_robot_interfaces.msg import YawConstraint
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from an xyzw quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def odom_yaw(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


def planar_speed(msg: Odometry) -> float:
    twist = msg.twist.twist
    return math.hypot(float(twist.linear.x), float(twist.linear.y))


def _stamp_ns(msg: Odometry) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def frame_mismatch(lio_odom: Odometry, wheel_odom: Odometry) -> bool:
    """Return whether the two odometry frame pairs differ."""
    return (
        lio_odom.header.frame_id != wheel_odom.header.frame_id
        or lio_odom.child_frame_id != wheel_odom.child_frame_id
    )


def build_yaw_constraint(lio_odom: Odometry, wheel_odom: Odometry) -> YawConstraint:
    """Build a matched observation without speed or jump filtering."""
    lio_stamp_ns = _stamp_ns(lio_odom)
    wheel_stamp_ns = _stamp_ns(wheel_odom)
    out = YawConstraint()
    out.stamp = wheel_odom.header.stamp
    if out.stamp.sec == 0 and out.stamp.nanosec == 0:
        out.stamp = lio_odom.header.stamp
    out.lio_stamp = lio_odom.header.stamp
    out.wheel_stamp = wheel_odom.header.stamp
    out.time_offset_sec = (wheel_stamp_ns - lio_stamp_ns) / 1e9
    out.lio_yaw = odom_yaw(lio_odom)
    out.wheel_yaw = odom_yaw(wheel_odom)
    out.delta_yaw = wrap_to_pi(out.wheel_yaw - out.lio_yaw)
    out.yaw_rate_lio = float(lio_odom.twist.twist.angular.z)
    out.yaw_rate_wheel = float(wheel_odom.twist.twist.angular.z)
    return out


def yaw_jump_rejected(
    observation: YawConstraint,
    max_delta_yaw_jump_deg: float,
) -> bool:
    """Return true when the absolute yaw difference exceeds the jump gate."""
    return abs(math.degrees(float(observation.delta_yaw))) > max(
        0.0, float(max_delta_yaw_jump_deg)
    )


def compute_yaw_constraint(
    lio_odom: Odometry,
    wheel_odom: Odometry,
    min_speed_threshold: float,
) -> Optional[YawConstraint]:
    """Compute one normal observation; return None while wheel speed is low."""
    if planar_speed(wheel_odom) < max(0.0, float(min_speed_threshold)):
        return None
    return build_yaw_constraint(lio_odom, wheel_odom)


class YawConstraintNode(Node):
    """Publish matched yaw diagnostics; this node never publishes TF/corrections."""

    def __init__(self) -> None:
        super().__init__('agt_yaw_constraint')
        self.declare_parameter('local_odom_topic', '/agt/odometry/local')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('output_topic', '/agt/localization/yaw_constraint')
        self.declare_parameter(
            'debug_output_topic', '/agt/localization/yaw_constraint_debug')
        self.declare_parameter('min_speed_threshold', 0.05)
        self.declare_parameter('sync_max_dt_sec', 0.2)
        self.declare_parameter('max_delta_yaw_jump_deg', 45.0)
        self.declare_parameter('stats_log_period_sec', 10.0)

        self._lio_queue: deque[Odometry] = deque()
        self._wheel_queue: deque[Odometry] = deque()
        self._max_queue_size = 200
        self._matched_count = 0
        self._dropped_lio_count = 0
        self._dropped_wheel_count = 0
        self._yaw_jump_rejected_count = 0
        self._time_offset_sum_sec = 0.0
        self._time_offset_count = 0
        self._first_frames = {'lio': None, 'wheel': None}
        self._frame_warnings = set()

        self._publisher = self.create_publisher(
            YawConstraint,
            self.get_parameter('output_topic').value,
            10,
        )
        self._debug_publisher = self.create_publisher(
            YawConstraint,
            self.get_parameter('debug_output_topic').value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('local_odom_topic').value,
            self._on_lio,
            50,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('wheel_odom_topic').value,
            self._on_wheel,
            50,
        )
        self._stats_timer = self.create_timer(
            float(self.get_parameter('stats_log_period_sec').value),
            self._log_stats,
        )

    def _on_lio(self, msg: Odometry) -> None:
        self._log_input_frame('lio', msg)
        self._lio_queue.append(msg)
        self._trim_queue('lio')
        self._try_match()

    def _on_wheel(self, msg: Odometry) -> None:
        self._log_input_frame('wheel', msg)
        self._wheel_queue.append(msg)
        self._trim_queue('wheel')
        self._try_match()

    def _log_input_frame(self, source: str, msg: Odometry) -> None:
        signature = (msg.header.frame_id, msg.child_frame_id)
        first = self._first_frames[source]
        if first is None:
            self._first_frames[source] = signature
            self.get_logger().info(
                f'{source} odom: header.frame_id={signature[0]!r}, '
                f'child_frame_id={signature[1]!r}')
        elif first != signature:
            self.get_logger().warning(
                f'{source} odom frame changed: '
                f'{first[0]}->{first[1]} to {signature[0]}->{signature[1]}')

    def _trim_queue(self, source: str) -> None:
        queue = self._lio_queue if source == 'lio' else self._wheel_queue
        while len(queue) > self._max_queue_size:
            queue.popleft()
            if source == 'lio':
                self._dropped_lio_count += 1
            else:
                self._dropped_wheel_count += 1

    def _try_match(self) -> None:
        max_dt = float(self.get_parameter('sync_max_dt_sec').value)
        while self._lio_queue and self._wheel_queue:
            lio = self._lio_queue[0]
            lio_ns = _stamp_ns(lio)
            # Discard samples that are definitely too old before selecting a
            # pair. This matters when wheel odom is much faster than LIO.
            if _stamp_ns(self._wheel_queue[0]) < lio_ns - int(max_dt * 1e9):
                self._wheel_queue.popleft()
                self._dropped_wheel_count += 1
                continue
            if _stamp_ns(self._lio_queue[0]) > _stamp_ns(self._wheel_queue[-1]) + int(max_dt * 1e9):
                self._lio_queue.popleft()
                self._dropped_lio_count += 1
                continue

            wheel_index, wheel = min(
                enumerate(self._wheel_queue),
                key=lambda item: abs(_stamp_ns(item[1]) - lio_ns),
            )
            wheel_ns = _stamp_ns(wheel)
            offset_sec = (wheel_ns - lio_ns) / 1e9
            if abs(offset_sec) > max_dt:
                # There is no pair inside the window. Drop the older side and
                # wait for a future sample on the other side.
                if lio_ns < wheel_ns:
                    self._lio_queue.popleft()
                    self._dropped_lio_count += 1
                else:
                    self._wheel_queue.popleft()
                    self._dropped_wheel_count += 1
                continue

            self._lio_queue.popleft()
            del self._wheel_queue[wheel_index]
            self._matched_count += 1
            self._time_offset_sum_sec += offset_sec
            self._time_offset_count += 1
            self._publish_match(lio, wheel)

    def _publish_match(self, lio: Odometry, wheel: Odometry) -> None:
        if frame_mismatch(lio, wheel):
            signature = (
                lio.header.frame_id,
                lio.child_frame_id,
                wheel.header.frame_id,
                wheel.child_frame_id,
            )
            if signature not in self._frame_warnings:
                self._frame_warnings.add(signature)
                self.get_logger().warning(
                    'Frame mismatch: '
                    f'wheel {wheel.header.frame_id}->{wheel.child_frame_id}, '
                    f'LIO {lio.header.frame_id}->{lio.child_frame_id}; '
                    'frames are not modified')

        observation = build_yaw_constraint(lio, wheel)
        # Debug receives every time-matched pair, including low speed and jump
        # rejected observations, so filtering is observable rather than silent.
        self._debug_publisher.publish(observation)

        if planar_speed(wheel) < max(
            0.0, float(self.get_parameter('min_speed_threshold').value)
        ):
            return
        if yaw_jump_rejected(
            observation,
            self.get_parameter('max_delta_yaw_jump_deg').value,
        ):
            self._yaw_jump_rejected_count += 1
            return
        self._publisher.publish(observation)

    def _log_stats(self) -> None:
        mean_offset = (
            self._time_offset_sum_sec / self._time_offset_count
            if self._time_offset_count
            else float('nan')
        )
        self.get_logger().info(
            'yaw constraint sync stats: '
            f'matched={self._matched_count}, '
            f'drop_lio={self._dropped_lio_count}, '
            f'drop_wheel={self._dropped_wheel_count}, '
            f'yaw_jump_rejected={self._yaw_jump_rejected_count}, '
            f'mean_time_offset={mean_offset:.6f}s')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YawConstraintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
