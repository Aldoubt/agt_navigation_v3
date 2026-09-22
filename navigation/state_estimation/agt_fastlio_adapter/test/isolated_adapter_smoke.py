"""Explicit opt-in synthetic message test. Never start a frontend or robot driver.

AGT_RUN_ISOLATED_FASTLIO_TEST=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 \
  python3 isolated_adapter_smoke.py

All data and TF topics are additionally remapped under /test/fastlio_adapter.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import time


def main():
    if (os.environ.get('AGT_RUN_ISOLATED_FASTLIO_TEST') != '1'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or os.environ.get('ROS_DOMAIN_ID', '0') == '0'):
        raise SystemExit('Refusing test: explicit opt-in, nonzero isolated domain and localhost-only are required')

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster
    from agt_robot_interfaces.msg import AdapterStatus
    from agt_fastlio_adapter.fastlio_adapter import FastLioAdapter

    with tempfile.TemporaryDirectory(prefix='agt_fastlio_adapter_test_') as directory:
        calibration = Path(directory) / 'fastlio.yaml'
        calibration.write_text('r_il: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n'
                               't_il: [-0.011, -0.02329, 0.04412]\n')
        rclpy.init(args=[
            '--ros-args',
            '-r', '/tf:=/test/fastlio_adapter/tf',
            '-r', '/tf_static:=/test/fastlio_adapter/tf_static',
            '-p', 'use_sim_time:=false',
            '-p', 'input_topic:=/test/fastlio_adapter/input',
            '-p', 'output_topic:=/test/fastlio_adapter/output',
            '-p', 'adapter_status_topic:=/test/fastlio_adapter/status',
            '-p', 'expected_base_frame:=body',
            '-p', 'convert_body_to_base:=true',
            '-p', 'derive_twist_from_pose:=true',
            '-p', f'body_to_base_calibration_file:={calibration}',
        ])
        adapter = FastLioAdapter()
        fixture = Node('fastlio_adapter_isolated_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(adapter)
        executor.add_node(fixture)
        outputs, statuses, transforms = [], [], []
        publisher = fixture.create_publisher(Odometry, '/test/fastlio_adapter/input', 10)
        fixture.create_subscription(Odometry, '/test/fastlio_adapter/output', outputs.append, 20)
        fixture.create_subscription(AdapterStatus, '/test/fastlio_adapter/status', statuses.append, 20)
        fixture.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 50)
        broadcaster = StaticTransformBroadcaster(fixture)
        static = []
        for parent, child, x, z in [
            ('base_footprint', 'base_link', 0.0, 0.2),
            ('base_link', 'livox_frame', 0.25, 0.8),
        ]:
            message = TransformStamped()
            message.header.stamp = fixture.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = parent, child
            message.transform.translation.x, message.transform.translation.z = x, z
            message.transform.rotation.w = 1.0
            static.append(message)
        broadcaster.sendTransform(static)
        sent_stamps = set()

        def publish(counter, age):
            message = Odometry()
            stamp = fixture.get_clock().now().nanoseconds - int(age * 1e9)
            message.header.stamp = Time(nanoseconds=stamp).to_msg()
            message.header.frame_id, message.child_frame_id = 'odom', 'body'
            message.pose.pose.position.x = counter * .02
            message.pose.pose.orientation.z = math.sin(counter * .02 / 2)
            message.pose.pose.orientation.w = math.cos(counter * .02 / 2)
            sent_stamps.add(stamp)
            publisher.publish(message)

        try:
            deadline, next_publish, count = time.monotonic() + 8.0, 0.0, 0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    publish(count, .02)
                    count += 1
                    next_publish = now + .1
                executor.spin_once(timeout_sec=.02)
                if len(outputs) >= 6 and any(s.state == 0 for s in statuses) and len(transforms) >= 6:
                    break
            # Drain any already sent fresh messages before measuring rejection.
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            assert len(outputs) >= 6, 'no canonical odometry after static TF became available'
            assert len(transforms) == len(outputs), 'odometry and navigation TF must be paired'
            assert any(abs(m.twist.twist.angular.z) > .05 for m in outputs), 'missing angular velocity'
            assert any(s.state == 0 and s.publish_rate > 0 for s in statuses), 'no FRESH diagnostic'
            for message in outputs:
                stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                assert stamp in sent_stamps, 'input timestamp was changed'
                assert (message.header.frame_id, message.child_frame_id) == ('odom', 'base_link')
            assert {(t.header.frame_id, t.child_frame_id) for t in transforms} == {('odom', 'base_footprint')}
            accepted = len(outputs)
            deadline, next_publish = time.monotonic() + 1.5, 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    publish(count, .6)
                    count += 1
                    next_publish = now + .1
                executor.spin_once(timeout_sec=.02)
            assert len(outputs) == accepted and len(transforms) == accepted, 'stale input escaped gate'
            assert statuses[-1].state == 2 and statuses[-1].publish_rate == 0.0
            assert statuses[-1].rejected_count > 0
            print(json.dumps({
                'result': 'PASS', 'domain_id': os.environ['ROS_DOMAIN_ID'],
                'accepted_odometry': accepted, 'paired_tf': len(transforms),
                'angular_velocity_nonzero': True, 'original_stamps_preserved': True,
                'stale_input_rejected': statuses[-1].rejected_count,
                'final_adapter_state': statuses[-1].state,
                'data_and_tf_topics_under_test_namespace': True,
                'no_frontend_hardware_or_command_publisher_started': True,
            }, ensure_ascii=False))
        finally:
            executor.remove_node(adapter)
            executor.remove_node(fixture)
            adapter.destroy_node()
            fixture.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
