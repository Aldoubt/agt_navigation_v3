from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CommandGuard(Node):
    """50 Hz velocity limiter/watchdog between Nav2 and the Bunker CAN driver.

    The Bunker driver remains the hardware owner. This node only constrains and
    refreshes Twist commands; it does not publish wheel odometry or TF.
    """

    def __init__(self) -> None:
        super().__init__('agt_bunker_command_guard')
        self.declare_parameter('input_topic', '/cmd_vel_smoothed')
        self.declare_parameter('output_topic', '/mux/cmd_vel')
        self.declare_parameter('output_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('max_linear_x', 0.60)
        self.declare_parameter('max_angular_z', 0.80)
        self.declare_parameter('max_linear_accel', 0.60)
        self.declare_parameter('max_linear_decel', 1.20)
        self.declare_parameter('max_angular_accel', 1.20)
        self.declare_parameter('publish_rate_topic', '/agt/control/cmd_output_rate_hz')

        self._target = Twist()
        self._output = Twist()
        self._last_rx_ns = 0
        self._last_tick_ns = self.get_clock().now().nanoseconds
        self._rate_window_start_ns = self._last_tick_ns
        self._rate_count = 0

        self._pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self._rate_pub = self.create_publisher(
            Float32, self.get_parameter('publish_rate_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._on_cmd, 20)

        rate = max(float(self.get_parameter('output_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Command guard {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value} at {rate:.1f} Hz')

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-abs(limit), min(abs(limit), value))

    @staticmethod
    def _approach(current: float, target: float, up_step: float, down_step: float) -> float:
        delta = target - current
        if abs(delta) < 1e-12:
            return target
        same_direction = current == 0.0 or target == 0.0 or math.copysign(1.0, current) == math.copysign(1.0, target)
        speeding_up = same_direction and abs(target) > abs(current)
        step = up_step if speeding_up else down_step
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def _on_cmd(self, msg: Twist) -> None:
        self._target = msg
        self._last_rx_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = max((now_ns - self._last_tick_ns) / 1e9, 1e-4)
        self._last_tick_ns = now_ns

        timeout = float(self.get_parameter('command_timeout_sec').value)
        stale = self._last_rx_ns <= 0 or (now_ns - self._last_rx_ns) / 1e9 > timeout

        if stale:
            # A stale upstream command is a fault condition: send zero immediately.
            self._output = Twist()
        else:
            target_v = self._clamp(
                float(self._target.linear.x), self.get_parameter('max_linear_x').value)
            target_w = self._clamp(
                float(self._target.angular.z), self.get_parameter('max_angular_z').value)

            accel = float(self.get_parameter('max_linear_accel').value) * dt
            decel = float(self.get_parameter('max_linear_decel').value) * dt
            angular = float(self.get_parameter('max_angular_accel').value) * dt

            self._output.linear.x = self._approach(
                self._output.linear.x, target_v, accel, decel)
            self._output.angular.z = self._approach(
                self._output.angular.z, target_w, angular, angular)
            self._output.linear.y = 0.0
            self._output.linear.z = 0.0
            self._output.angular.x = 0.0
            self._output.angular.y = 0.0

        self._pub.publish(self._output)
        self._rate_count += 1
        elapsed = (now_ns - self._rate_window_start_ns) / 1e9
        if elapsed >= 1.0:
            msg = Float32()
            msg.data = float(self._rate_count / elapsed)
            self._rate_pub.publish(msg)
            self._rate_window_start_ns = now_ns
            self._rate_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        for _ in range(3):
            node._pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
