from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FastLioAdapter(Node):
    """Expose a selected FAST-LIO2 odometry stream as AGT canonical local odometry.

    The adapter deliberately does not create or rewrite TF. It verifies the
    configured frame contract and republishes the original odometry message.
    Frame conversion belongs in a measured/calibrated integration layer, not in
    a silent topic relay.
    """

    def __init__(self) -> None:
        super().__init__('agt_fastlio_adapter')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/agt/odometry/local')
        self.declare_parameter('expected_odom_frame', 'odom')
        self.declare_parameter('expected_base_frame', 'base_link')
        self.declare_parameter('max_input_age_sec', 0.20)
        self.declare_parameter('reject_zero_stamp', True)

        self._accepted = 0
        self._rejected = 0
        self._last_warn_ns = 0

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(Odometry, output_topic, 20)
        self.create_subscription(Odometry, input_topic, self._on_odom, 50)
        self.get_logger().info(
            f'FAST-LIO adapter: {input_topic} -> {output_topic}; '
            f'expected frames {self.get_parameter("expected_odom_frame").value} -> '
            f'{self.get_parameter("expected_base_frame").value}')

    @staticmethod
    def _stamp_ns(msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _warn_throttled(self, text: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns >= 2_000_000_000:
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def _on_odom(self, msg: Odometry) -> None:
        expected_odom = str(self.get_parameter('expected_odom_frame').value)
        expected_base = str(self.get_parameter('expected_base_frame').value)

        if msg.header.frame_id != expected_odom or msg.child_frame_id != expected_base:
            self._rejected += 1
            self._warn_throttled(
                'Rejecting FAST-LIO odometry because frame contract does not match: '
                f'got {msg.header.frame_id!r}->{msg.child_frame_id!r}, expected '
                f'{expected_odom!r}->{expected_base!r}. Fix the FAST-LIO/URDF adapter; '
                'do not silently relabel frames.')
            return

        stamp_ns = self._stamp_ns(msg)
        if stamp_ns == 0 and bool(self.get_parameter('reject_zero_stamp').value):
            self._rejected += 1
            self._warn_throttled('Rejecting zero-stamped FAST-LIO odometry.')
            return

        if stamp_ns > 0:
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            max_age = float(self.get_parameter('max_input_age_sec').value)
            # Negative age can happen under simulated time/clock transitions;
            # reject only clearly stale positive ages here.
            if math.isfinite(age) and age > max_age:
                self._rejected += 1
                self._warn_throttled(
                    f'Rejecting stale FAST-LIO odometry: age={age:.3f}s > {max_age:.3f}s')
                return

        self._pub.publish(msg)
        self._accepted += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
