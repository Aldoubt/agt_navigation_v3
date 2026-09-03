from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelGuard(Node):
    """Clamp, slew-limit and refresh velocity commands at a fixed rate.

    Nav2 publishes to /cmd_vel. This guard is the only software path that is
    expected to publish to the Bunker driver's /mux/cmd_vel input.
    """

    def __init__(self) -> None:
        super().__init__('agt_cmd_vel_guard')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/mux/cmd_vel')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('max_linear_x', 0.55)
        self.declare_parameter('max_reverse_x', 0.20)
        self.declare_parameter('max_angular_z', 0.65)
        self.declare_parameter('max_linear_accel', 0.45)
        self.declare_parameter('max_linear_decel', 0.55)
        self.declare_parameter('max_angular_accel', 0.80)

        self.target = Twist()
        self.output = Twist()
        self.last_rx_ns = 0

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.create_subscription(Twist, input_topic, self._on_cmd, 10)
        self.pub = self.create_publisher(Twist, output_topic, 10)

        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'cmd_vel guard: {input_topic} -> {output_topic} @ {rate:.1f} Hz'
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.target = msg
        self.last_rx_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def _slew(current: float, target: float, rise: float, fall: float, dt: float) -> float:
        delta = target - current
        if math.isclose(delta, 0.0, abs_tol=1e-9):
            return target
        limit = rise * dt if delta > 0.0 else fall * dt
        return current + max(-limit, min(limit, delta))

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        timeout = float(self.get_parameter('command_timeout_sec').value)
        stale = self.last_rx_ns <= 0 or (now_ns - self.last_rx_ns) / 1e9 > timeout

        target_x = 0.0 if stale else float(self.target.linear.x)
        target_w = 0.0 if stale else float(self.target.angular.z)

        target_x = self._clamp(
            target_x,
            -float(self.get_parameter('max_reverse_x').value),
            float(self.get_parameter('max_linear_x').value),
        )
        target_w = self._clamp(
            target_w,
            -float(self.get_parameter('max_angular_z').value),
            float(self.get_parameter('max_angular_z').value),
        )

        accel_x = float(self.get_parameter('max_linear_accel').value)
        decel_x = float(self.get_parameter('max_linear_decel').value)
        accel_w = float(self.get_parameter('max_angular_accel').value)

        # If reducing absolute linear speed, use the stronger deceleration limit.
        x_limit = decel_x if abs(target_x) < abs(self.output.linear.x) else accel_x
        self.output.linear.x = self._slew(
            self.output.linear.x, target_x, x_limit, x_limit, self.dt
        )
        self.output.angular.z = self._slew(
            self.output.angular.z, target_w, accel_w, accel_w, self.dt
        )

        # Tracked base is non-holonomic. Explicitly zero unused axes.
        self.output.linear.y = 0.0
        self.output.linear.z = 0.0
        self.output.angular.x = 0.0
        self.output.angular.y = 0.0
        self.pub.publish(self.output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
