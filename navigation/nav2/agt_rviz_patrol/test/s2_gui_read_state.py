"""Read-only status evidence collector, never edits GUI or publishes commands."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check-count', type=int, default=-1)
    a = parser.parse_args()
    rclpy.init()
    node = Node('s2_gui_read_only_audit')
    records = []
    node.create_subscription(
        String, '/agt/path_tool/editor_state',
        lambda message: records.append(json.loads(message.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    end = time.monotonic() + 6
    while not records and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    assert records, 'No ROS editor state to corroborate the GUI'
    state = records[-1]
    evidence = {name:state.get(name) for name in (
        'revision','points','point_labels','status','preview_length_m','preview_only',
        'can_execute','map_id','map_version','map_received','draft_path','action_state')}
    print(json.dumps(evidence, ensure_ascii=False),flush=True)
    if a.check_count >= 0:
        assert len(state['points']) == a.check_count, (len(state['points']), a.check_count)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
