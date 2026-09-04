from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


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


class BatchLioAdapter(Node):
    """Convert Batch-LIO camera_init->body odometry into AGT odom->base_link.

    The parent-frame conversion is an explicit semantic alias: Batch-LIO's
    camera_init is the local inertial origin and AGT names that local origin
    `odom`. The child-frame conversion is geometric and uses the measured
    static TF body<-base_link.
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
        self.declare_parameter('allow_parent_alias', True)

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 50)
        self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value), self.on_odom, 100)
        self.get_logger().info(
            'Batch-LIO adapter started. camera_init->odom is an explicit local-origin alias; '
            'body->base_link is resolved from TF.')

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

        try:
            tf = self.buffer.lookup_transform(
                src_child, out_child, stamp,
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)))
        except TransformException as exc:
            self.get_logger().warning(f'body<-base_link TF unavailable: {exc}')
            return

        p_cb = msg.pose.pose.position
        q_cb = msg.pose.pose.orientation
        q_camera_body = q_norm((q_cb.x, q_cb.y, q_cb.z, q_cb.w))
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

        # Odometry twist is conventionally expressed in child frame. Move the
        # linear velocity from body origin to base origin, then rotate body->base.
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
