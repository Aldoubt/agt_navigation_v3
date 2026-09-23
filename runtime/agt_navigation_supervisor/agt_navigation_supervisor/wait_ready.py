"""CLI gate for the composite V4 health contract."""

import argparse
import time

import rclpy
from agt_navigation_interfaces.msg import NavigationHealth
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--samples', type=int, default=3)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Node('agt_navigation_health_gate')
    received = []
    node.create_subscription(NavigationHealth, '/navigation/health',
                             received.append, QoSProfile(
                                 depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE))
    consecutive = 0
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            for health in received:
                if health.status == NavigationHealth.READY:
                    consecutive += 1
                    if consecutive >= args.samples:
                        print(f'NAV_READY map={health.map_id}/{health.map_version} '
                              f'robot={health.robot_profile}')
                        return
                else:
                    consecutive = 0
                    if health.status == NavigationHealth.ERROR:
                        parser.exit(2, f'{health.state}: {health.last_error_message}\n')
            received.clear()
        parser.exit(2, 'Navigation Health did not reach READY before timeout\n')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
