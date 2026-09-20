from __future__ import annotations

import math
import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from agt_robot_interfaces.msg import AdapterStatus

from agt_batch_lio_adapter.diagnostics import AdapterState, classify_adapter_state

from agt_batch_lio_adapter.extrinsics import (
    compose_transform,
    load_batch_lio_body_to_lidar,
    transform_msg_to_tuple,
)


def q_norm(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    x, y, z, w = q_norm(q)
    return (-x, -y, -z, w)


def rotate(q, v):
    rq = q_mul(q_mul(q_norm(q), (v[0], v[1], v[2], 0.0)), q_conj(q))
    return rq[0], rq[1], rq[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def pose_delta_twist(previous_p, previous_q, current_p, current_q, dt):
    """Estimate child-frame linear/angular velocity from two parent-frame poses."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('pose delta dt must be positive')

    dp_parent = tuple((current_p[i] - previous_p[i]) / dt for i in range(3))
    v_child = rotate(q_conj(current_q), dp_parent)

    dq = q_norm(q_mul(q_conj(previous_q), current_q))
    if dq[3] < 0.0:
        dq = tuple(-v for v in dq)
    w = max(-1.0, min(1.0, dq[3]))
    angle = 2.0 * math.acos(w)
    sin_half = math.sqrt(max(0.0, 1.0 - w * w))
    if sin_half < 1e-9 or angle < 1e-9:
        omega_child = (0.0, 0.0, 0.0)
    else:
        scale = angle / (sin_half * dt)
        omega_child = (dq[0] * scale, dq[1] * scale, dq[2] * scale)
    return v_child, omega_child


def clamp_vector_norm(v, limit):
    norm = math.sqrt(sum(x * x for x in v))
    if norm <= limit or norm <= 1e-12:
        return v
    scale = limit / norm
    return tuple(x * scale for x in v)


class BatchLioAdapter(Node):
    """Convert Batch-LIO camera_init->body odometry into AGT local odometry.

    The parent-frame conversion is a semantic alias: Batch-LIO's camera_init is
    the local inertial origin and AGT names that local origin `odom`.

    The child-frame conversion no longer owns a second vehicle-mount
    calibration. In the normal field path it derives T_body_base from:

      1. Batch-LIO's exact runtime mapping.extrinsic_R/T (T_body_lidar), and
      2. robot_state_publisher's calibrated lidar<-base_link static TF.

    This keeps the MID360 internal extrinsic and the physical chassis mount in
    their respective sources of truth while eliminating duplicated body-to-base
    constants. Odometry remains odom->base_link; TF uses odom->base_footprint
    so the dynamic rotation center is on the ground plane.
    """

    def __init__(self):
        super().__init__('agt_batch_lio_adapter')
        self.declare_parameter('input_topic', '/aft_mapped_to_init')
        self.declare_parameter('output_topic', '/agt/odometry/local')
        self.declare_parameter('source_parent_frame', 'camera_init')
        self.declare_parameter('source_child_frame', 'body')
        self.declare_parameter('output_parent_frame', 'odom')
        self.declare_parameter('output_child_frame', 'base_link')
        self.declare_parameter('tf_child_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('batch_lio_config_file', '')
        self.declare_parameter('max_input_age_sec', 0.20)
        self.declare_parameter('tf_timeout_sec', 0.20)
        self.declare_parameter('adapter_status_topic', '/agt/odometry/adapter_status')
        self.declare_parameter('diagnostic_publish_rate_hz', 5.0)
        self.declare_parameter('stale_pose_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_window_sec', 1.0)

        # Legacy emergency override only. The normal field path keeps this
        # false and derives T_body_base from Batch-LIO config + chassis TF.
        self.declare_parameter('use_configured_extrinsic', False)
        self.declare_parameter('body_to_base_translation', [])
        self.declare_parameter('body_to_base_quaternion_xyzw', [])

        self.declare_parameter('allow_parent_alias', False)
        self.declare_parameter('debug_path_topic', '/agt/debug/local_path')
        self.declare_parameter('derive_twist_from_pose', True)
        self.declare_parameter('twist_min_dt_sec', 0.01)
        self.declare_parameter('twist_max_dt_sec', 0.50)
        self.declare_parameter('twist_linear_deadband_mps', 0.01)
        self.declare_parameter('twist_angular_deadband_rps', 0.01)
        self.declare_parameter('twist_max_linear_mps', 2.0)
        self.declare_parameter('twist_max_angular_rps', 3.0)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._body_to_base_cache = None
        self._base_to_tf_child_cache = None
        self._extrinsic_error_logged = False
        self._body_to_lidar = None
        if not bool(self.get_parameter('use_configured_extrinsic').value):
            config_path = os.path.expanduser(
                str(self.get_parameter('batch_lio_config_file').value).strip())
            if config_path:
                try:
                    self._body_to_lidar = load_batch_lio_body_to_lidar(config_path)
                except Exception as exc:
                    self.get_logger().error(
                        f'Cannot read Batch-LIO internal extrinsic from {config_path!r}: {exc}')
            else:
                self.get_logger().error(
                    'batch_lio_config_file is required when use_configured_extrinsic=false')

        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 50)
        self.status_pub = self.create_publisher(
            AdapterStatus,
            str(self.get_parameter('adapter_status_topic').value),
            10,
        )
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('debug_path_topic').value), 10)
        self.path = Path()
        self.path.header.frame_id = str(self.get_parameter('output_parent_frame').value)
        self.previous_base_pose = None
        self.previous_base_stamp_ns = 0
        self._last_input_header_stamp_ns = 0
        self._last_input_rx_ns = 0
        self._last_valid_pose_rx_ns = 0
        self._accepted_count = 0
        self._rejected_count = 0
        self._publish_times_ns = deque()
        self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value), self.on_odom, 100)
        diagnostic_rate = max(
            float(self.get_parameter('diagnostic_publish_rate_hz').value), 0.1)
        self.create_timer(1.0 / diagnostic_rate, self._publish_adapter_status)

        source = (
            'explicit legacy body_to_base override'
            if bool(self.get_parameter('use_configured_extrinsic').value)
            else 'Batch-LIO internal extrinsic + robot_description lidar/base TF'
        )
        self.get_logger().info(
            'Batch-LIO adapter started. odom->camera_init is an explicit identity '
            f'local-origin alias; body->base_link source: {source}.')

    def _publish_parent_alias(self):
        src_parent = str(self.get_parameter('source_parent_frame').value)
        out_parent = str(self.get_parameter('output_parent_frame').value)
        if src_parent == out_parent:
            return
        if not bool(self.get_parameter('allow_parent_alias').value):
            self.get_logger().warning(
                f'parent alias disabled: TF {out_parent}->{src_parent} will not be published')
            return

        alias = TransformStamped()
        alias.header.stamp = self.get_clock().now().to_msg()
        alias.header.frame_id = out_parent
        alias.child_frame_id = src_parent
        alias.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(alias)
        self.get_logger().info(
            f'Published local-origin identity alias TF: {out_parent} -> {src_parent}')

    def _resolve_body_to_base(self, out_child):
        if self._body_to_base_cache is not None:
            return self._body_to_base_cache

        if bool(self.get_parameter('use_configured_extrinsic').value):
            t = list(self.get_parameter('body_to_base_translation').value)
            q = list(self.get_parameter('body_to_base_quaternion_xyzw').value)
            if len(t) != 3 or len(q) != 4:
                raise RuntimeError(
                    'legacy configured body->base_link override requires 3+4 values')
            resolved = (
                (float(t[0]), float(t[1]), float(t[2])),
                q_norm((float(q[0]), float(q[1]), float(q[2]), float(q[3]))),
            )
            self._body_to_base_cache = resolved
            return resolved

        if self._body_to_lidar is None:
            raise RuntimeError(
                'Batch-LIO internal extrinsic unavailable; check batch_lio_config_file')

        lidar_frame = str(self.get_parameter('lidar_frame').value).strip()
        if not lidar_frame:
            raise RuntimeError('lidar_frame must not be empty')

        try:
            # target=lidar, source=base gives T_lidar_base from the physical
            # robot_description chain. Time(0) is correct for a static mount and
            # avoids exact-time /tf_static replay ordering issues.
            tf = self.buffer.lookup_transform(
                lidar_frame,
                out_child,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'physical mount TF {lidar_frame} <- {out_child} unavailable: {exc}') from exc

        t_body_lidar, q_body_lidar = self._body_to_lidar
        t_lidar_base, q_lidar_base = transform_msg_to_tuple(tf.transform)
        resolved = compose_transform(
            t_body_lidar, q_body_lidar, t_lidar_base, q_lidar_base)
        self._body_to_base_cache = resolved
        self.get_logger().info(
            f'Resolved body->{out_child} from Batch-LIO T_body_lidar + '
            f'robot_description {lidar_frame}<- {out_child}: '
            f't=[{resolved[0][0]:.6f}, {resolved[0][1]:.6f}, {resolved[0][2]:.6f}] '
            f'q=[{resolved[1][0]:.9f}, {resolved[1][1]:.9f}, '
            f'{resolved[1][2]:.9f}, {resolved[1][3]:.9f}]'
        )
        return resolved

    def _resolve_base_to_tf_child(self, base_frame, tf_child):
        if base_frame == tf_child:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        if self._base_to_tf_child_cache is not None:
            return self._base_to_tf_child_cache
        try:
            transform = self.buffer.lookup_transform(
                base_frame, tf_child, Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'rotation-center TF {base_frame} <- {tf_child} unavailable: {exc}') from exc
        self._base_to_tf_child_cache = transform_msg_to_tuple(transform.transform)
        return self._base_to_tf_child_cache

    def on_odom(self, msg: Odometry):
        now_ns = self.get_clock().now().nanoseconds
        src_parent = str(self.get_parameter('source_parent_frame').value)
        src_child = str(self.get_parameter('source_child_frame').value)
        out_parent = str(self.get_parameter('output_parent_frame').value)
        out_child = str(self.get_parameter('output_child_frame').value)
        tf_child = str(self.get_parameter('tf_child_frame').value)
        self._last_input_rx_ns = now_ns
        if msg.header.frame_id != src_parent or msg.child_frame_id != src_child:
            self._rejected_count += 1
            self.get_logger().warning(
                f'drop Batch-LIO odom frames {msg.header.frame_id}->{msg.child_frame_id}; '
                f'expected {src_parent}->{src_child}')
            return

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds <= 0:
            self._rejected_count += 1
            return
        self._last_input_header_stamp_ns = stamp.nanoseconds
        age = (now_ns - stamp.nanoseconds) / 1e9
        if age > float(self.get_parameter('max_input_age_sec').value):
            self._rejected_count += 1
            return

        p_cb = msg.pose.pose.position
        q_cb = msg.pose.pose.orientation
        q_camera_body = q_norm((q_cb.x, q_cb.y, q_cb.z, q_cb.w))

        try:
            t_body_base, q_body_base = self._resolve_body_to_base(out_child)
            t_base_tf_child, q_base_tf_child = self._resolve_base_to_tf_child(
                out_child, tf_child)
            self._extrinsic_error_logged = False
        except Exception as exc:
            self._rejected_count += 1
            if not self._extrinsic_error_logged:
                self.get_logger().error(str(exc))
                self._extrinsic_error_logged = True
            return

        offset_camera = rotate(q_camera_body, t_body_base)
        p_camera_base = (
            p_cb.x + offset_camera[0],
            p_cb.y + offset_camera[1],
            p_cb.z + offset_camera[2],
        )
        q_camera_base = q_norm(q_mul(q_camera_body, q_body_base))

        current_pose = (p_camera_base, q_camera_base)
        v_base = (0.0, 0.0, 0.0)
        w_base = (0.0, 0.0, 0.0)
        if bool(self.get_parameter('derive_twist_from_pose').value):
            if self.previous_base_pose is not None and self.previous_base_stamp_ns > 0:
                dt = (stamp.nanoseconds - self.previous_base_stamp_ns) / 1e9
                min_dt = float(self.get_parameter('twist_min_dt_sec').value)
                max_dt = float(self.get_parameter('twist_max_dt_sec').value)
                if min_dt <= dt <= max_dt:
                    v_base, w_base = pose_delta_twist(
                        self.previous_base_pose[0], self.previous_base_pose[1],
                        current_pose[0], current_pose[1], dt)
                    linear_deadband = float(
                        self.get_parameter('twist_linear_deadband_mps').value)
                    angular_deadband = float(
                        self.get_parameter('twist_angular_deadband_rps').value)
                    v_base = tuple(
                        0.0 if abs(v) < linear_deadband else v for v in v_base)
                    w_base = tuple(
                        0.0 if abs(v) < angular_deadband else v for v in w_base)
                    v_base = clamp_vector_norm(
                        v_base, float(self.get_parameter('twist_max_linear_mps').value))
                    w_base = clamp_vector_norm(
                        w_base, float(self.get_parameter('twist_max_angular_rps').value))
            self.previous_base_pose = current_pose
            self.previous_base_stamp_ns = stamp.nanoseconds
        else:
            tw = msg.twist.twist
            v_body = (tw.linear.x, tw.linear.y, tw.linear.z)
            w_body = (tw.angular.x, tw.angular.y, tw.angular.z)
            wxr = cross(w_body, t_body_base)
            v_base_origin_body = (
                v_body[0] + wxr[0],
                v_body[1] + wxr[1],
                v_body[2] + wxr[2],
            )
            q_base_body = q_conj(q_body_base)
            v_base = rotate(q_base_body, v_base_origin_body)
            w_base = rotate(q_base_body, w_body)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = out_parent
        out.child_frame_id = out_child
        out.pose = msg.pose
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p_camera_base
        out.pose.pose.orientation.x = q_camera_base[0]
        out.pose.pose.orientation.y = q_camera_base[1]
        out.pose.pose.orientation.z = q_camera_base[2]
        out.pose.pose.orientation.w = q_camera_base[3]
        out.twist = msg.twist
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = v_base
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = w_base
        self.pub.publish(out)

        tf = TransformStamped()
        tf.header.stamp = out.header.stamp
        tf.header.frame_id = out_parent
        tf.child_frame_id = tf_child
        p_camera_tf_child, q_camera_tf_child = compose_transform(
            p_camera_base, q_camera_base, t_base_tf_child, q_base_tf_child)
        tf.transform.translation.x = p_camera_tf_child[0]
        tf.transform.translation.y = p_camera_tf_child[1]
        tf.transform.translation.z = p_camera_tf_child[2]
        tf.transform.rotation.x = q_camera_tf_child[0]
        tf.transform.rotation.y = q_camera_tf_child[1]
        tf.transform.rotation.z = q_camera_tf_child[2]
        tf.transform.rotation.w = q_camera_tf_child[3]
        self.tf_broadcaster.sendTransform(tf)
        self._accepted_count += 1
        self._last_valid_pose_rx_ns = now_ns
        self._publish_times_ns.append(now_ns)

        self.path.header.stamp = out.header.stamp
        pose = PoseStamped()
        pose.header = out.header
        pose.pose = out.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 20000:
            self.path.poses = self.path.poses[-20000:]
        self.path_pub.publish(self.path)

    def _publish_adapter_status(self):
        """Publish diagnostics only; never synthesize odometry or TF."""
        now_ns = self.get_clock().now().nanoseconds
        input_age = None
        if self._last_input_header_stamp_ns > 0:
            input_age = max(0.0, (now_ns - self._last_input_header_stamp_ns) / 1e9)
        last_valid_age = None
        if self._last_valid_pose_rx_ns > 0:
            last_valid_age = max(0.0, (now_ns - self._last_valid_pose_rx_ns) / 1e9)
        window_ns = int(max(
            float(self.get_parameter('publish_rate_window_sec').value), 0.1) * 1e9)
        while self._publish_times_ns and now_ns - self._publish_times_ns[0] > window_ns:
            self._publish_times_ns.popleft()
        publish_rate = len(self._publish_times_ns) / (window_ns / 1e9)
        state = classify_adapter_state(
            input_age,
            last_valid_age,
            float(self.get_parameter('max_input_age_sec').value),
            float(self.get_parameter('stale_pose_timeout_sec').value),
        )
        out = AdapterStatus()
        out.stamp = self.get_clock().now().to_msg()
        out.state = int(state)
        out.input_age_sec = -1.0 if input_age is None else float(input_age)
        out.last_valid_pose_age_sec = -1.0 if last_valid_age is None else float(last_valid_age)
        out.accepted_count = self._accepted_count
        out.rejected_count = self._rejected_count
        out.publish_rate = float(publish_rate)
        self.status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BatchLioAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
