"""Expose Gazebo truth odometry through the same contract as local LIO odometry."""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimOdometryAdapter(Node):
    """The sole simulation owner of ``odom -> base_link``.

    Gazebo stays private behind ``/sim/ground_truth_odom``.  This node republishes
    its pose and twist with the production local-odometry topic/frame contract;
    it deliberately never publishes ``map -> odom``.
    """

    def __init__(self):
        super().__init__('agt_sim_odometry_adapter')
        self.declare_parameter('input_topic', '/sim/ground_truth_odom')
        self.declare_parameter('output_topic', '/agt/odometry/local')
        self.declare_parameter('output_parent_frame', 'odom')
        self.declare_parameter('output_child_frame', 'base_link')
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 50)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._on_odom, 50)

    def _on_odom(self, incoming: Odometry) -> None:
        outgoing = copy.deepcopy(incoming)
        # Gazebo's simulation clock is carried by the input message. A zero
        # stamp only occurs before Gazebo time begins; use this node's sim clock
        # in that narrow case rather than wall time.
        if outgoing.header.stamp.sec == 0 and outgoing.header.stamp.nanosec == 0:
            outgoing.header.stamp = self.get_clock().now().to_msg()
        outgoing.header.frame_id = self.get_parameter('output_parent_frame').value
        outgoing.child_frame_id = self.get_parameter('output_child_frame').value
        self.publisher.publish(outgoing)

        transform = TransformStamped()
        transform.header = outgoing.header
        transform.child_frame_id = outgoing.child_frame_id
        transform.transform.translation.x = outgoing.pose.pose.position.x
        transform.transform.translation.y = outgoing.pose.pose.position.y
        transform.transform.translation.z = outgoing.pose.pose.position.z
        transform.transform.rotation = outgoing.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = SimOdometryAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
