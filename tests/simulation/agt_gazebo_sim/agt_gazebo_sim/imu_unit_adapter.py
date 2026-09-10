import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuUnitAdapter(Node):
    """Convert Gazebo SI acceleration into the g-units expected by pinned LIO configs."""

    def __init__(self):
        super().__init__('agt_sim_imu_unit_adapter')
        self.declare_parameter('input_topic', '/sim/imu/data')
        self.declare_parameter('output_topic', '/agt/sensors/imu/data')
        self.declare_parameter('gravity_mps2', 9.80665)
        self.declare_parameter('output_frame', 'imu_link')
        self.scale = 1.0 / float(self.get_parameter('gravity_mps2').value)
        self.pub = self.create_publisher(Imu, str(self.get_parameter('output_topic').value), 200)
        self.create_subscription(Imu, str(self.get_parameter('input_topic').value), self.on_imu, 200)
        self.get_logger().info(
            f"Gazebo IMU adapter: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}, acceleration scale={self.scale:.8f}")

    def on_imu(self, msg: Imu):
        out = copy.deepcopy(msg)
        out.header.frame_id = str(self.get_parameter('output_frame').value)
        out.linear_acceleration.x *= self.scale
        out.linear_acceleration.y *= self.scale
        out.linear_acceleration.z *= self.scale
        scale2 = self.scale * self.scale
        if out.linear_acceleration_covariance[0] >= 0.0:
            out.linear_acceleration_covariance = [
                value * scale2 for value in out.linear_acceleration_covariance]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuUnitAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
