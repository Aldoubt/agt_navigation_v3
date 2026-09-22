from __future__ import annotations

import math
from collections import deque
from copy import deepcopy

import rclpy
from agt_batch_lio_adapter.diagnostics import classify_adapter_state
from agt_batch_lio_adapter.extrinsics import (
    compose_transform, load_lio_body_to_lidar, q_norm, transform_msg_to_tuple,
)
from agt_robot_interfaces.msg import AdapterStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .frame_conversion import body_twist_to_base, compose_body_to_base_pose
from .pose_twist import PoseTwistEstimator


class FastLioAdapter(Node):
    """Canonical local odometry, TF and readiness from one FAST-LIO2 frontend.

    Navigation uses calibrated base poses and derives a coherent base-frame
    twist, including angular velocity omitted by the current upstream node.
    Legacy pass-through remains available with convert_body_to_base=false.
    Never restamp an old pose or synthesize motion to satisfy readiness checks.
    """

    def __init__(self) -> None:
        super().__init__('agt_fastlio_adapter')
        defaults = [
            ('input_topic', '/Odometry'), ('output_topic', '/agt/odometry/local'),
            ('expected_odom_frame', 'odom'), ('expected_base_frame', 'base_link'),
            ('output_odom_frame', 'odom'), ('output_base_frame', 'base_link'),
            ('tf_child_frame', 'base_footprint'), ('convert_body_to_base', False),
            ('body_to_base_calibration_file', ''),
            ('mount_lidar_frame', 'livox_frame'), ('mount_base_frame', 'base_link'),
            ('tf_timeout_sec', 0.20),  # Kept for launch compatibility; static reads are nonblocking.
            ('max_input_age_sec', 0.20), ('max_future_input_sec', 0.05),
            ('reject_zero_stamp', True), ('derive_twist_from_pose', False),
            ('twist_min_dt_sec', 0.01), ('twist_max_dt_sec', 0.50),
            ('twist_linear_deadband_mps', 0.01), ('twist_angular_deadband_rps', 0.01),
            ('twist_max_linear_mps', 2.0), ('twist_max_angular_rps', 3.0),
            ('adapter_status_topic', '/agt/odometry/adapter_status'),
            ('diagnostic_publish_rate_hz', 5.0), ('stale_pose_timeout_sec', 1.0),
            ('publish_rate_window_sec', 1.0), ('debug_path_topic', ''),
            ('debug_path_max_poses', 2000),
        ]
        for name, value in defaults:
            self.declare_parameter(name, value)
        self._accepted = 0
        self._rejected = 0
        self._last_warn_ns = 0
        self._last_input_stamp_ns = 0
        self._last_valid_rx_ns = 0
        self._last_output_stamp_ns = 0
        self._publish_times = deque()
        self._body_to_base_cache = None
        self._base_to_tf_child_cache = None
        self._internal_extrinsic = None
        if self._value('convert_body_to_base'):
            config = str(self._value('body_to_base_calibration_file')).strip()
            if not config:
                raise ValueError('body_to_base_calibration_file is required for navigation')
            self._internal_extrinsic = load_lio_body_to_lidar(config)
        if (float(self._value('max_input_age_sec')) <= 0.0
                or float(self._value('max_future_input_sec')) < 0.0
                or float(self._value('diagnostic_publish_rate_hz')) <= 0.0):
            raise ValueError('invalid odometry freshness or diagnostic rate configuration')
        self._twist = PoseTwistEstimator(
            min_dt=float(self._value('twist_min_dt_sec')),
            max_dt=float(self._value('twist_max_dt_sec')),
            linear_deadband=float(self._value('twist_linear_deadband_mps')),
            angular_deadband=float(self._value('twist_angular_deadband_rps')),
            max_linear=float(self._value('twist_max_linear_mps')),
            max_angular=float(self._value('twist_max_angular_rps')),
        )
        self._pub = self.create_publisher(Odometry, self._value('output_topic'), 20)
        self._tf = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._status_pub = self.create_publisher(
            AdapterStatus, self._value('adapter_status_topic'), 10)
        self.create_timer(1.0 / float(self._value('diagnostic_publish_rate_hz')),
                          self._publish_adapter_status)
        self._path = deque(maxlen=max(1, int(self._value('debug_path_max_poses'))))
        path_topic = str(self._value('debug_path_topic')).strip()
        self._path_pub = self.create_publisher(Path, path_topic, 1) if path_topic else None
        self.create_subscription(Odometry, self._value('input_topic'), self._on_odom, 50)
        self.get_logger().info(
            f'FAST-LIO adapter: {self._value("input_topic")} -> '
            f'{self._value("output_topic")}; expected '
            f'{self._value("expected_odom_frame")}->{self._value("expected_base_frame")}; '
            f'base conversion={self._value("convert_body_to_base")}; '
            f'pose-derived twist={self._value("derive_twist_from_pose")}')

    def _value(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _stamp_ns(msg: Odometry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _reject(self, text):
        self._rejected += 1
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns >= 2_000_000_000:
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def _static_transform(self, target, source):
        # Do not block a single-threaded executor waiting for its own TF callback.
        try:
            return transform_msg_to_tuple(
                self._tf_buffer.lookup_transform(target, source, Time()).transform)
        except TransformException as exc:
            raise RuntimeError(f'static mount TF {target} <- {source} unavailable: {exc}') from exc

    def _resolve_body_to_base(self):
        if self._body_to_base_cache is None:
            t_body_lidar, q_body_lidar = self._internal_extrinsic
            t_lidar_base, q_lidar_base = self._static_transform(
                str(self._value('mount_lidar_frame')), str(self._value('mount_base_frame')))
            self._body_to_base_cache = compose_transform(
                t_body_lidar, q_body_lidar, t_lidar_base, q_lidar_base)
            self.get_logger().info(
                'Resolved FAST-LIO body->base_link from active frontend r_il/t_il '
                'and robot_description; navigation owns odom->base_footprint only.')
        return self._body_to_base_cache

    def _make_odom_tf(self, msg):
        if self._base_to_tf_child_cache is None:
            self._base_to_tf_child_cache = self._static_transform(
                str(self._value('output_base_frame')), str(self._value('tf_child_frame')))
        pose = msg.pose.pose
        translation, orientation = compose_transform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            *self._base_to_tf_child_cache)
        transform = TransformStamped()
        transform.header = deepcopy(msg.header)
        transform.child_frame_id = str(self._value('tf_child_frame'))
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = translation
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = orientation
        return transform

    def _on_odom(self, msg: Odometry) -> None:
        stamp_ns = self._stamp_ns(msg)
        if stamp_ns > 0:
            self._last_input_stamp_ns = stamp_ns
        if (msg.header.frame_id != str(self._value('expected_odom_frame'))
                or msg.child_frame_id != str(self._value('expected_base_frame'))):
            self._reject('Rejecting FAST-LIO odometry: input frame contract does not match')
            return
        if stamp_ns <= 0 and bool(self._value('reject_zero_stamp')):
            self._reject('Rejecting zero-stamped FAST-LIO odometry')
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
        if stamp_ns > 0 and (age > float(self._value('max_input_age_sec'))
                             or age < -float(self._value('max_future_input_sec'))):
            self._reject(f'Rejecting stale/future FAST-LIO odometry: age={age:.3f}s')
            return
        if self._last_output_stamp_ns > 0 and stamp_ns <= self._last_output_stamp_ns:
            self._reject('Rejecting non-increasing FAST-LIO timestamp; reset the stack on clock reset')
            return
        out = deepcopy(msg)
        pose = msg.pose.pose
        position = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        try:
            if not all(math.isfinite(value) for value in position + quaternion):
                raise ValueError('non-finite FAST-LIO pose')
            quaternion = q_norm(quaternion)
            convert = bool(self._value('convert_body_to_base'))
            if convert:
                translation, rotation = self._resolve_body_to_base()
                position, quaternion = compose_body_to_base_pose(
                    position, quaternion, translation, rotation)
                out.header.frame_id = str(self._value('output_odom_frame'))
                out.child_frame_id = str(self._value('output_base_frame'))
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = quaternion
            # Resolve both static mount edges before publishing either odometry or TF.
            transform = self._make_odom_tf(out) if convert else None
            if stamp_ns > 0 and (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9 > float(self._value('max_input_age_sec')):
                raise ValueError('odometry became stale while resolving its frame conversion')
            if bool(self._value('derive_twist_from_pose')):
                # Differentiate the already converted base pose, so its lever arm
                # and angular velocity are coherent. No assumption of zero gyro.
                linear, angular = self._twist.update(position, quaternion, stamp_ns)
            elif convert:
                twist = msg.twist.twist
                linear, angular = body_twist_to_base(
                    (twist.linear.x, twist.linear.y, twist.linear.z),
                    (twist.angular.x, twist.angular.y, twist.angular.z), translation, rotation)
            else:
                twist = msg.twist.twist
                linear = (twist.linear.x, twist.linear.y, twist.linear.z)
                angular = (twist.angular.x, twist.angular.y, twist.angular.z)
            if not all(math.isfinite(value) for value in linear + angular):
                raise ValueError('non-finite FAST-LIO twist')
        except (RuntimeError, ValueError, TypeError) as exc:
            self._reject(str(exc))
            return
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = linear
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular
        if transform is not None:
            self._tf.sendTransform(transform)
        self._pub.publish(out)
        now_ns = self.get_clock().now().nanoseconds
        self._accepted += 1
        self._last_valid_rx_ns = now_ns
        self._last_output_stamp_ns = stamp_ns
        self._publish_times.append(now_ns)
        if self._path_pub is not None:
            item = PoseStamped()
            item.header, item.pose = deepcopy(out.header), deepcopy(out.pose.pose)
            self._path.append(item)
            path = Path()
            path.header, path.poses = deepcopy(out.header), list(self._path)
            self._path_pub.publish(path)

    def _publish_adapter_status(self):
        now_ns = self.get_clock().now().nanoseconds
        age = max(0.0, (now_ns - self._last_input_stamp_ns) / 1.0e9) if self._last_input_stamp_ns else None
        valid_age = max(0.0, (now_ns - self._last_valid_rx_ns) / 1.0e9) if self._last_valid_rx_ns else None
        window = max(0.1, float(self._value('publish_rate_window_sec')))
        while self._publish_times and now_ns - self._publish_times[0] > window * 1.0e9:
            self._publish_times.popleft()
        msg = AdapterStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = int(classify_adapter_state(
            age, valid_age, float(self._value('max_input_age_sec')),
            float(self._value('stale_pose_timeout_sec'))))
        msg.input_age_sec = age if age is not None else -1.0
        msg.last_valid_pose_age_sec = valid_age if valid_age is not None else -1.0
        msg.accepted_count, msg.rejected_count = self._accepted, self._rejected
        msg.publish_rate = len(self._publish_times) / window
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioAdapter()
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
