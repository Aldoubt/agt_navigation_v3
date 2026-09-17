from __future__ import annotations

import math
from copy import deepcopy

import rclpy
from agt_batch_lio_adapter.extrinsics import (
    compose_transform,
    load_batch_lio_body_to_lidar,
    transform_msg_to_tuple,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .frame_conversion import body_vector_to_base, compose_body_to_base_pose


class FastLioAdapter(Node):
    """Expose a selected FAST-LIO2 odometry stream as AGT canonical local odometry.

    The legacy pass-through mode only verifies and republishes the input.  The
    explicit ``convert_body_to_base`` mode composes a calibrated
    ``T_body_base_link`` and is the navigation integration boundary for the
    upstream FAST-LIO2 ``odom -> body`` stream.
    """

    def __init__(self) -> None:
        super().__init__('agt_fastlio_adapter')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/agt/odometry/local')
        self.declare_parameter('expected_odom_frame', 'odom')
        self.declare_parameter('expected_base_frame', 'base_link')
        self.declare_parameter('output_odom_frame', 'odom')
        self.declare_parameter('output_base_frame', 'base_link')
        self.declare_parameter('convert_body_to_base', False)
        self.declare_parameter('body_to_base_calibration_file', '')
        self.declare_parameter('mount_lidar_frame', 'livox_frame')
        self.declare_parameter('mount_base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.20)
        self.declare_parameter('max_input_age_sec', 0.20)
        self.declare_parameter('reject_zero_stamp', True)

        self._accepted = 0
        self._rejected = 0
        self._last_warn_ns = 0
        self._body_to_base_cache = None

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(Odometry, output_topic, 20)
        self._tf = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Odometry, input_topic, self._on_odom, 50)
        self.get_logger().info(
            f'FAST-LIO adapter: {input_topic} -> {output_topic}; '
            f'expected frames {self.get_parameter("expected_odom_frame").value} -> '
            f'{self.get_parameter("expected_base_frame").value}; '
            f'body-to-base conversion={bool(self.get_parameter("convert_body_to_base").value)}')

    @staticmethod
    def _stamp_ns(msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _warn_throttled(self, text: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns >= 2_000_000_000:
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def _resolve_body_to_base(self):
        """Use the same frozen calibration composition as global relocalization."""
        if self._body_to_base_cache is not None:
            return self._body_to_base_cache
        config_path = str(self.get_parameter('body_to_base_calibration_file').value).strip()
        if not config_path:
            raise RuntimeError('body_to_base_calibration_file is required when convert_body_to_base=true')
        t_body_lidar, q_body_lidar = load_batch_lio_body_to_lidar(config_path)
        lidar_frame = str(self.get_parameter('mount_lidar_frame').value).strip()
        base_frame = str(self.get_parameter('mount_base_frame').value).strip()
        try:
            tf = self._tf_buffer.lookup_transform(
                lidar_frame, base_frame, Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'physical mount TF {lidar_frame} <- {base_frame} unavailable: {exc}') from exc
        t_lidar_base, q_lidar_base = transform_msg_to_tuple(tf.transform)
        self._body_to_base_cache = compose_transform(
            t_body_lidar, q_body_lidar, t_lidar_base, q_lidar_base)
        self.get_logger().info(
            'Resolved FAST-LIO body->base_link from frozen body/lidar calibration '
            f'and robot_description {lidar_frame}<-{base_frame}.')
        return self._body_to_base_cache

    def _publish_odom_tf(self, msg: Odometry) -> None:
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = str(self.get_parameter('output_odom_frame').value)
        tf.child_frame_id = str(self.get_parameter('output_base_frame').value)
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)

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

        convert = bool(self.get_parameter('convert_body_to_base').value)
        out = msg
        if convert:
            try:
                t_body_base, q_body_base = self._resolve_body_to_base()
            except RuntimeError as exc:
                self._rejected += 1
                self._warn_throttled(str(exc))
                return
            out = deepcopy(msg)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            translation, orientation = compose_body_to_base_pose(
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), t_body_base, q_body_base)
            out.header.frame_id = str(self.get_parameter('output_odom_frame').value)
            out.child_frame_id = str(self.get_parameter('output_base_frame').value)
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = translation
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = orientation
            # FAST-LIO2 currently publishes zero twist.  Rotate non-zero values
            # if a future upstream version supplies body-frame velocities.
            linear = body_vector_to_base(
                (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z), q_body_base)
            angular = body_vector_to_base(
                (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z), q_body_base)
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = linear
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular

        self._pub.publish(out)
        if convert:
            # FAST-LIO2 still owns odom->body. This explicit transformed edge
            # is the sole navigation odom->base_link publisher in this mode.
            self._publish_odom_tf(out)
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
