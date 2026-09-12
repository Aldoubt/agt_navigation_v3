import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class NavGoalProbe(Node):
    def __init__(self):
        super().__init__('agt_sim_nav_goal_probe')
        self.declare_parameter('goal_x', 2.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('truth_topic', '/sim/ground_truth_odom')
        self.declare_parameter('max_truth_error_m', 0.45)
        self.truth = None
        self.create_subscription(
            Odometry, str(self.get_parameter('truth_topic').value), self.on_truth, 20)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def on_truth(self, msg):
        self.truth = msg

    def run(self):
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('NavigateToPose action server not available')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('NavigateToPose goal rejected')
        self.get_logger().info('NAV_GOAL_ACCEPTED')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError('NavigateToPose result timeout')
        if int(wrapped.status) != 4:
            raise RuntimeError(f'NavigateToPose did not succeed, status={wrapped.status}')
        for _ in range(20):
            if self.truth is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.truth is None:
            raise RuntimeError('no Gazebo truth odometry for acceptance check')
        p = self.truth.pose.pose.position
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        error = math.hypot(p.x - gx, p.y - gy)
        self.get_logger().info(
            f'NAV_GOAL_SUCCEEDED truth=({p.x:.3f},{p.y:.3f}) target=({gx:.3f},{gy:.3f}) error={error:.3f}m')
        if error > float(self.get_parameter('max_truth_error_m').value):
            raise RuntimeError(f'ground-truth goal error too large: {error:.3f}m')
        self.get_logger().info('NAV_GOAL_ACCEPTANCE_PASS')


def main():
    rclpy.init()
    node = NavGoalProbe()
    code = 0
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)
