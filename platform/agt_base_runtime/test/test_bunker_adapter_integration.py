"""Fake-driver contract test; every topic is isolated under /p3 and no driver is launched."""

import os
import signal
import statistics
import subprocess
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from bunker_msgs.msg import BunkerRCState, BunkerStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402


EXECUTABLE = os.environ.get('BUNKER_ADAPTER_EXECUTABLE', '')
skip_without_adapter = pytest.mark.skipif(
    not EXECUTABLE or not Path(EXECUTABLE).is_file(),
    reason='needs a built BUNKER_ADAPTER_EXECUTABLE')


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@skip_without_adapter
def test_bunker_adapter_fake_driver_command_odom_status_and_shutdown():
    rclpy.init()
    fake = rclpy.create_node('p3_fake_bunker_driver')
    received_commands = []
    received_odom = []
    received_state = []
    received_health = []
    command_times = []
    lock = threading.Lock()

    def on_command(msg):
        with lock:
            received_commands.append(msg)
            command_times.append(time.monotonic())

    fake.create_subscription(
        Twist, '/p3/mux/cmd_vel', on_command, 20)
    fake.create_subscription(
        Odometry, '/p3/agt/base/odom',
        lambda msg: (lock.acquire(), received_odom.append(msg), lock.release()), 20)
    fake.create_subscription(
        DiagnosticArray, '/p3/agt/base/state',
        lambda msg: (lock.acquire(), received_state.append(msg), lock.release()), 10)
    fake.create_subscription(
        DiagnosticArray, '/p3/agt/base/health',
        lambda msg: (lock.acquire(), received_health.append(msg), lock.release()), 10)
    command_pub = fake.create_publisher(Twist, '/p3/base/cmd_vel', 20)
    odom_pub = fake.create_publisher(Odometry, '/p3/wheel/odom', 20)
    status_pub = fake.create_publisher(BunkerStatus, '/p3/bunker_status', 10)
    rc_pub = fake.create_publisher(BunkerRCState, '/p3/bunker_rc_state', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    args = [
        EXECUTABLE, '--ros-args',
        '-p', 'input_topic:=/p3/base/cmd_vel',
        '-p', 'output_topic:=/p3/mux/cmd_vel',
        '-p', 'odom_input_topic:=/p3/wheel/odom',
        '-p', 'odom_output_topic:=/p3/agt/base/odom',
        '-p', 'driver_status_topic:=/p3/bunker_status',
        '-p', 'remote_status_topic:=/p3/bunker_rc_state',
        '-p', 'state_topic:=/p3/agt/base/state',
        '-p', 'health_topic:=/p3/agt/base/health',
        '-p', 'command_timeout_sec:=0.15',
    ]
    process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert wait_for(lambda: fake.count_subscribers('/p3/base/cmd_vel') == 1), 'adapter command input not discovered'
        assert wait_for(lambda: fake.count_publishers('/p3/mux/cmd_vel') == 1), 'fake driver output not discovered'
        assert wait_for(lambda: fake.count_subscribers('/p3/wheel/odom') == 1), 'adapter odometry input not discovered'
        assert wait_for(lambda: fake.count_subscribers('/p3/bunker_status') == 1), 'adapter status input not discovered'
        assert wait_for(lambda: fake.count_subscribers('/p3/bunker_rc_state') == 1), 'adapter remote input not discovered'
        mux_owners = fake.get_publishers_info_by_topic('/p3/mux/cmd_vel')
        assert len(mux_owners) == 1
        assert not fake.get_publishers_info_by_topic('/tf')
        assert not fake.get_publishers_info_by_topic('/tf_static')

        request = Twist()
        request.linear.x = 0.3
        request.linear.y = 2.0
        request.angular.x = 3.0
        request.angular.z = -0.2
        command_pub.publish(request)
        assert wait_for(
            lambda: any(abs(msg.linear.x - 0.3) < 1.0e-9 for msg in received_commands)), (
                'fake driver received no translated command')
        assert wait_for(
            lambda: sum(abs(msg.linear.x - 0.3) < 1.0e-9 for msg in received_commands) >= 4,
            timeout=1.0), 'adapter did not refresh the active command at 50 Hz'
        with lock:
            active_times = [stamp for stamp, msg in zip(command_times, received_commands)
                            if abs(msg.linear.x - 0.3) < 1.0e-9]
        median_period = statistics.median(
            right - left for left, right in zip(active_times, active_times[1:]))
        assert 0.015 <= median_period <= 0.030, f'unexpected adapter period: {median_period:.6f}s'
        with lock:
            command = received_commands[-1]
        assert abs(command.linear.x - 0.3) < 1.0e-9
        assert command.linear.y == 0.0 and command.linear.z == 0.0
        assert command.angular.x == 0.0 and command.angular.y == 0.0
        assert abs(command.angular.z + 0.2) < 1.0e-9

        odom = Odometry()
        odom.header.stamp.sec = 12
        odom.header.stamp.nanosec = 34
        odom.header.frame_id = 'wheel_odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 1.5
        odom.twist.twist.linear.x = 0.4
        odom_pub.publish(odom)
        status = BunkerStatus()
        status.vehicle_state = 2
        status.control_mode = 3
        status.error_code = 0
        status_pub.publish(status)
        rc = BunkerRCState()
        rc.swa = 1
        rc.swb = 2
        rc_pub.publish(rc)
        def state_has_driver_and_remote_values():
            with lock:
                snapshots = list(received_state)
            for snapshot in snapshots:
                values = {entry.key: entry.value for entry in snapshot.status[0].values}
                if (values.get('kinematics') == 'skid_steer' and
                    values.get('driver_vehicle_state_code') == '2' and
                    values.get('remote_switch_a') == '1'):
                    return True
            return False

        assert wait_for(lambda: bool(received_odom)), 'canonical odometry did not arrive'
        assert wait_for(state_has_driver_and_remote_values), 'state diagnostics did not normalize fake driver feedback'
        assert wait_for(lambda: bool(received_health)), 'health diagnostics did not arrive'
        with lock:
            normalized = received_odom[-1]
        assert normalized.header.stamp.sec == 12
        assert normalized.header.stamp.nanosec == 34
        assert normalized.header.frame_id == 'wheel_odom'
        assert normalized.child_frame_id == 'base_link'
        assert abs(normalized.pose.pose.position.x - 1.5) < 1.0e-9
        assert abs(normalized.twist.twist.linear.x - 0.4) < 1.0e-9

        # The adapter's command watchdog replaces an expired command with zero.
        assert wait_for(
            lambda: bool(received_commands) and
            received_commands[-1].linear.x == 0.0 and received_commands[-1].angular.z == 0.0,
            timeout=2.0), 'stale canonical command was not stopped'

        # Refresh a nonzero command and confirm graceful shutdown emits zeros.
        command_pub.publish(request)
        assert wait_for(
            lambda: received_commands and abs(received_commands[-1].linear.x - 0.3) < 1.0e-9,
            timeout=2.0), 'fake driver did not receive refreshed command'
        with lock:
            before_shutdown = len(received_commands)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=5.0) == 0
        assert wait_for(
            lambda: len(received_commands) >= before_shutdown + 3 and
            all(msg.linear.x == 0.0 and msg.angular.z == 0.0
                for msg in received_commands[-3:]),
            timeout=2.0), 'adapter shutdown did not publish three zeros'
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        executor.shutdown(timeout_sec=1.0)
        fake.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
