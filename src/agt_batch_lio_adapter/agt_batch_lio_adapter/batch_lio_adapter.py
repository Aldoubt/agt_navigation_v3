from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


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
    # nav_msgs/Odometry twist is expressed in child frame. Rotate the base-origin
    # translational velocity from odom into the current base frame.
    v_child = rotate(q_conj(current_q), dp_parent)

    # q_prev^-1 * q_curr is the shortest incremental body rotation. Convert it
    # to an angle-axis rate instead of differentiating Euler angles across wrap.
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
    """Convert Batch-LIO camera_init->body odometry into AGT odom->base_link.

    The parent-frame conversion is an explicit semantic alias: Batch-LIO's
    camera_init is the local inertial origin and AGT names that local origin
    `odom`. The child-frame conversion is geometric and normally uses the
    versioned calibrated body<-base_link extrinsic directly. TF lookup remains
    available as a fallback/debug path.
    """

    def __init__(self):
        super().__init__('agt_batch_lio_adapter')
        self.declare_parameter('input_topic', '/aft_mapped_to_init')
        self.declare_parameter('output_topic', '/agt/odometry/local')
        self.declare_parameter('source_parent_frame', 'camera_init')
        self.declare_parameter('source_child_frame', 'body')
        self.declare_parameter('output_parent_frame', 'odom')
        self.declare_parameter('output_child_frame', 'base_link')
        self.declare_parameter('max_input_age_sec', 0.20)
        self.declare_parameter('tf_timeout_sec', 0.20)
        self.declare_parameter('use_configured_extrinsic', True)
        self.declare_parameter(
            'body_to_base_translation', [-0.16403417, 0.02439982, -0.49511119])
        self.declare_parameter(
            'body_to_base_quaternion_xyzw',
            [0.000477000, -0.100267018, -0.001592000, 0.994959177])
        self.declare_parameter('allow_parent_alias', True)
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
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_parent_alias()
        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 50)
        self.path_pub = self.create_publisher(Path, str(self.get_parameter('debug_path_topic').value), 10)
        self.path = Path()
        self.path.header.frame_id = str(self.get_parameter('output_parent_frame').value)
        self.previous_base_pose = None
        self.previous_base_stamp_ns = 0
        self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value), self.on_odom, 100)
        self.get_logger().info(
            'Batch-LIO adapter started. odom->camera_init is an explicit identity local-origin alias; '
            'body->base_link uses the configured calibrated extrinsic by default.')

    def _publish_parent_alias(self):
        src_parent = str(self.get_parameter('source_parent_frame').value)
        out_parent = str(self.get_parameter('output_parent_frame').value)
        if src_parent == out_parent:
            return
        if not bool(self.get_parameter('allow_parent_alias').value):
            self.get_logger().warning(
                f'parent alias disabled: TF {out_parent}->{src_parent} will not be published')
            return

        # Batch-LIO's camera_init and AGT's odom are two names for the same
        # local inertial origin. Publish that semantic alias as a real static TF
        # so downstream consumers see a single-parent tree:
        #   map -> odom -> camera_init -> body -> base_link
        # Publishing odom->base_link here would give base_link two TF parents
        # because Batch-LIO already owns camera_init->body and the launch owns
        # body->base_link.
        alias = TransformStamped()
        alias.header.stamp = self.get_clock().now().to_msg()
        alias.header.frame_id = out_parent
        alias.child_frame_id = src_parent
        alias.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(alias)
        self.get_logger().info(
            f'Published local-origin identity alias TF: {out_parent} -> {src_parent}')

    def on_odom(self, msg: Odometry):
        src_parent = str(self.get_parameter('source_parent_frame').value)
        src_child = str(self.get_parameter('source_child_frame').value)
        out_parent = str(self.get_parameter('output_parent_frame').value)
        out_child = str(self.get_parameter('output_child_frame').value)
        if msg.header.frame_id != src_parent or msg.child_frame_id != src_child:
            self.get_logger().warning(
                f'drop Batch-LIO odom frames {msg.header.frame_id}->{msg.child_frame_id}; '
                f'expected {src_parent}->{src_child}')
            return
        if src_parent != out_parent and not bool(self.get_parameter('allow_parent_alias').value):
            self.get_logger().error('parent alias disabled; cannot expose camera_init as odom')
            return

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds <= 0:
            return
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        if age > float(self.get_parameter('max_input_age_sec').value):
            return

        p_cb = msg.pose.pose.position
        q_cb = msg.pose.pose.orientation
        q_camera_body = q_norm((q_cb.x, q_cb.y, q_cb.z, q_cb.w))
        if bool(self.get_parameter('use_configured_extrinsic').value):
            t = list(self.get_parameter('body_to_base_translation').value)
            q = list(self.get_parameter('body_to_base_quaternion_xyzw').value)
            if len(t) != 3 or len(q) != 4:
                self.get_logger().error('configured body->base_link extrinsic must be 3+4 values')
                return
            t_body_base = (float(t[0]), float(t[1]), float(t[2]))
            q_body_base = q_norm((float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        else:
            try:
                # The fallback is a static extrinsic; Time(0) avoids exact-time
                # extrapolation failures during rosbag /clock replay.
                tf = self.buffer.lookup_transform(
                    src_child, out_child, Time(),
                    timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
            except TransformException as exc:
                self.get_logger().warning(f'body<-base_link TF unavailable: {exc}')
                return
            t_body_base = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            )
            qr = tf.transform.rotation
            q_body_base = q_norm((qr.x, qr.y, qr.z, qr.w))

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
                    linear_deadband = float(self.get_parameter('twist_linear_deadband_mps').value)
                    angular_deadband = float(self.get_parameter('twist_angular_deadband_rps').value)
                    v_base = tuple(0.0 if abs(v) < linear_deadband else v for v in v_base)
                    w_base = tuple(0.0 if abs(v) < angular_deadband else v for v in w_base)
                    v_base = clamp_vector_norm(
                        v_base, float(self.get_parameter('twist_max_linear_mps').value))
                    w_base = clamp_vector_norm(
                        w_base, float(self.get_parameter('twist_max_angular_rps').value))
            self.previous_base_pose = current_pose
            self.previous_base_stamp_ns = stamp.nanoseconds
        else:
            # Fallback for LIO implementations that publish a trustworthy body
            # twist. Functionhx/Batch-LIO currently publishes zeros here, which
            # is why pose-delta derivation is enabled for the AGT navigation path.
            tw = msg.twist.twist
            v_body = (tw.linear.x, tw.linear.y, tw.linear.z)
            w_body = (tw.angular.x, tw.angular.y, tw.angular.z)
            wxr = cross(w_body, t_body_base)
            v_base_origin_body = (
                v_body[0] + wxr[0], v_body[1] + wxr[1], v_body[2] + wxr[2])
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
        self.path.header.stamp = out.header.stamp
        pose = PoseStamped()
        pose.header = out.header
        pose.pose = out.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 20000:
            self.path.poses = self.path.poses[-20000:]
        self.path_pub.publish(self.path)


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
