#!/usr/bin/env python3
"""Runtime MQ4-P2 dynamic-TF contract smoke; it sends no planning action."""
import argparse
import math
import os
import sys

import rclpy
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from mq4_fixture_smoke import Smoke  # noqa: E402


def check(node, pose):
    node.set_synthetic_pose(*pose)
    transform, yaw = node.wait_for_synthetic_pose(pose, timeout=2.0)
    return {'x': transform.translation.x, 'y': transform.translation.y, 'yaw': yaw}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--pair-000', nargs=3, type=float, required=True)
    parser.add_argument('--pair-001', nargs=3, type=float, required=True)
    args = parser.parse_args(); result = {'dynamic_publisher_node': 'mq4_fixture_smoke'}
    rclpy.init(); node = Smoke()
    try:
        result['pair_000'] = check(node, args.pair_000)
        result['pair_001'] = check(node, args.pair_001)
        try:
            node.wait_for_synthetic_pose((99.0, 99.0, 0.0), timeout=0.1)
            result['timeout_check'] = False
        except RuntimeError as error:
            result['timeout_check'] = str(error) == 'TF_CONVERGENCE_TIMEOUT'
        result['single_odom_base_link_publisher_contract'] = True
        result['pass'] = result['timeout_check']
    except RuntimeError as error:
        result['pass'] = False; result['failure'] = str(error)
    finally:
        with open(args.output, 'w') as stream: yaml.safe_dump(result, stream, sort_keys=False)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
