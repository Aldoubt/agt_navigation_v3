import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class MappingDrive(Node):
    """Closed-loop low-speed mapping route using Gazebo truth only as a test driver."""

    def __init__(self):
        super().__init__('agt_sim_mapping_drive')
        self.declare_parameter('odom_topic', '/sim/ground_truth_odom')
        self.declare_parameter('cmd_topic', '/mux/cmd_vel')
        self.declare_parameter('initial_hold_sec', 6.0)
        self.declare_parameter('linear_speed', 0.45)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('position_tolerance', 0.30)
        self.declare_parameter('heading_tolerance', 0.18)
        self.declare_parameter('route', [
            -3.5, -2.5,
             3.5, -2.5,
             3.5,  2.5,
            -3.5,  2.5,
            -3.5, -2.5,
             0.0,  0.0,
        ])
        flat = list(self.get_parameter('route').value)
        if len(flat) < 4 or len(flat) % 2:
            raise RuntimeError('route must contain x,y pairs')
        self.route = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]
        self.index = 0
        self.pose = None
        self.started_wall = time.monotonic()
        self.finished = False
        self.pub = self.create_publisher(Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 20)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Mapping route has {len(self.route)} waypoints')

    def on_odom(self, msg):
        self.pose = msg

    def stop(self):
        self.pub.publish(Twist())

    def tick(self):
        if self.finished:
            self.stop()
            return
        if self.pose is None:
            return
        if time.monotonic() - self.started_wall < float(self.get_parameter('initial_hold_sec').value):
            self.stop()
            return

        p = self.pose.pose.pose.position
        yaw = yaw_from_quaternion(self.pose.pose.pose.orientation)
        tx, ty = self.route[self.index]
        dx, dy = tx - p.x, ty - p.y
        distance = math.hypot(dx, dy)
        tolerance = float(self.get_parameter('position_tolerance').value)
        if distance <= tolerance:
            self.get_logger().info(
                f'waypoint {self.index + 1}/{len(self.route)} reached at ({p.x:.2f},{p.y:.2f})')
            self.index += 1
            self.stop()
            if self.index >= len(self.route):
                self.finished = True
                self.get_logger().info('MAPPING_ROUTE_COMPLETE')
            return

        target_yaw = math.atan2(dy, dx)
        error = wrap_angle(target_yaw - yaw)
        cmd = Twist()
        heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        if abs(error) > heading_tolerance:
            limit = float(self.get_parameter('angular_speed').value)
            cmd.angular.z = max(-limit, min(limit, 1.5 * error))
        else:
            cmd.linear.x = min(float(self.get_parameter('linear_speed').value), distance)
            limit = float(self.get_parameter('angular_speed').value)
            cmd.angular.z = max(-limit, min(limit, 1.2 * error))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = MappingDrive()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
