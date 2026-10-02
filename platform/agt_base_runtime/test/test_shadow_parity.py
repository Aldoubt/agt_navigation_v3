"""Run the old Python guard and C++ guard together on isolated shadow topics."""

import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from agt_robot_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

NAV_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(NAV_ROOT / 'bringup' / 'agt_base_control'))
from agt_base_control.cmd_vel_guard import CmdVelGuard  # noqa: E402


class PythonGuardShadow(CmdVelGuard):
    OVERRIDES = {}

    def declare_parameter(self, name, value=None, *args, **kwargs):
        return super().declare_parameter(name, self.OVERRIDES.get(name, value), *args, **kwargs)


class Probe(Node):
    def __init__(self, ns):
        super().__init__('p2_guard_shadow_probe_' + ns)
        self.command = self.create_publisher(Twist, f'/{ns}/command', 20)
        self.manual = self.create_publisher(Twist, f'/{ns}/manual', 20)
        self.mode = self.create_publisher(String, f'/{ns}/mode', 10)
        self.localization = self.create_publisher(LocalizationStatus, f'/{ns}/localization', 20)
        self.permission = self.create_publisher(Bool, f'/{ns}/permission', 20)
        self.python_samples = []
        self.cpp_samples = []
        self.cpp_states = []
        self.create_subscription(
            Twist, f'/{ns}/python_shadow',
            lambda msg: self.python_samples.append(
                (time.monotonic(), float(msg.linear.x), float(msg.angular.z))), 100)
        self.create_subscription(
            Twist, f'/{ns}/cpp_shadow',
            lambda msg: self.cpp_samples.append(
                (time.monotonic(), float(msg.linear.x), float(msg.angular.z))), 100)
        self.create_subscription(
            String, f'/{ns}/cpp_state', lambda msg: self.cpp_states.append(
                (time.monotonic(), msg.data)), 100)

    @staticmethod
    def status(state=LocalizationStatus.STATE_LOCALIZED, local_fresh=True, correction=True):
        msg = LocalizationStatus()
        msg.stamp = rclpy.clock.Clock().now().to_msg()
        msg.state = int(state)
        msg.local_odom_fresh = local_fresh
        msg.global_correction_valid = correction
        msg.reason = 'p2_shadow_parity'
        return msg

    @staticmethod
    def twist(linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        return msg


def _ros_parameter(name, value):
    return ['-p', f'{name}:={value}']


def test_python_cpp_shadow_sample_parity():
    executable = os.environ.get('MOTION_GUARD_CPP_EXECUTABLE', '')
    assert executable and Path(executable).is_file(), 'C++ motion guard binary was not built'
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = str(200 + (os.getpid() % 30))
    if not rclpy.ok():
        rclpy.init()

    ns = 'p2shadow_' + uuid.uuid4().hex[:8]
    topics = {
        'input_topic': f'/{ns}/command',
        'manual_input_topic': f'/{ns}/manual',
        'control_mode_topic': f'/{ns}/mode',
        'output_topic': f'/{ns}/python_shadow',
        'localization_status_topic': f'/{ns}/localization',
        'payload_drive_permission_topic': f'/{ns}/permission',
        'payload_hold_topic': f'/{ns}/python_hold',
        'state_topic': f'/{ns}/python_state',
        'require_localization_status': True,
        'localization_status_timeout_sec': 0.75,
        'allow_degraded_localization': False,
        'require_payload_drive_permission': True,
        'payload_drive_permission_timeout_sec': 0.5,
        'command_timeout_sec': 0.25,
        'publish_rate_hz': 50.0,
        'max_linear_x': 0.55,
        'max_reverse_x': 0.20,
        'max_angular_z': 0.65,
        'max_linear_accel': 0.45,
        'max_linear_decel': 0.80,
        'max_angular_accel': 0.80,
    }
    PythonGuardShadow.OVERRIDES = topics
    python_guard = PythonGuardShadow()
    probe = Probe(ns)
    executor = SingleThreadedExecutor()
    executor.add_node(python_guard)
    executor.add_node(probe)

    cpp_output = f'/{ns}/cpp_shadow'
    cpp_state = f'/{ns}/cpp_state'
    cpp_hold = f'/{ns}/cpp_hold'
    cpp_topics = dict(topics)
    cpp_topics['output_topic'] = cpp_output
    cpp_topics['payload_hold_topic'] = cpp_hold
    cpp_topics['state_topic'] = cpp_state
    cpp_args = [executable, '--ros-args', '-r', '__node:=agt_cmd_vel_guard_cpp']
    for key, value in cpp_topics.items():
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        cpp_args += _ros_parameter(key, value)
    cpp_env = dict(os.environ)
    log_path = Path('/tmp') / f'{ns}_cpp_guard.log'
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(cpp_args, env=cpp_env, stdout=log_stream, stderr=subprocess.STDOUT)

    def spin_for(duration, state=LocalizationStatus.STATE_LOCALIZED,
                 local_fresh=True, correction=True, command=None, permission=True):
        end = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                probe.localization.publish(Probe.status(state, local_fresh, correction))
                if permission is not None:
                    probe.permission.publish(Bool(data=permission))
                if command is not None:
                    probe.command.publish(command)
                next_publish = now + 0.02
            executor.spin_once(timeout_sec=0.002)

    def latest_cpp_state():
        return probe.cpp_states[-1][1] if probe.cpp_states else None

    def compare_window(start, end, tolerance=0.035, measure_rate=False):
        python = [s for s in probe.python_samples if start <= s[0] <= end]
        cpp = [s for s in probe.cpp_samples if start <= s[0] <= end]
        assert len(python) >= 12 and len(cpp) >= 12, (len(python), len(cpp), log_path.read_text())
        nearest_pairs = []
        for p in python:
            c = min(cpp, key=lambda sample: abs(sample[0] - p[0]))
            if abs(c[0] - p[0]) <= 0.025:
                nearest_pairs.append((p, c))
        assert len(nearest_pairs) >= 10, f'only {len(nearest_pairs)} aligned shadow samples'
        max_linear = max(abs(p[1] - c[1]) for p, c in nearest_pairs)
        max_angular = max(abs(p[2] - c[2]) for p, c in nearest_pairs)
        median_period = None
        if measure_rate:
            periods = [b[0] - a[0] for a, b in zip(cpp, cpp[1:])]
            median_period = sorted(periods)[len(periods) // 2]
            assert 0.015 <= median_period <= 0.025, (
                f'C++ output period {median_period * 1000:.2f} ms is outside the 50 Hz target')
        assert max_linear <= tolerance, f'linear parity error {max_linear:.6f} > {tolerance}'
        assert max_angular <= tolerance, f'angular parity error {max_angular:.6f} > {tolerance}'
        return len(nearest_pairs), max_linear, max_angular, median_period

    try:
        assert process.poll() is None
        # Discovery, valid localization and allowed payload state; no command => READY.
        spin_for(0.55, command=None, permission=True)
        assert latest_cpp_state() == 'READY', (latest_cpp_state(), log_path.read_text())

        active_start = time.monotonic()
        spin_for(0.65, command=Probe.twist(0.40, 0.30), permission=True)
        active_end = time.monotonic()
        assert latest_cpp_state() == 'ACTIVE'
        active = compare_window(active_start + 0.10, active_end - 0.05, measure_rate=True)

        # No new command for >0.25 s must stop both shadow outputs.
        stale_start = time.monotonic()
        spin_for(0.80, command=None, permission=True)
        stale_end = time.monotonic()
        assert latest_cpp_state() == 'STALE_COMMAND'
        stale = compare_window(stale_start + 0.32, stale_end - 0.05)

        # Lost localization is a hard stop and a separate diagnostic state.
        localization_start = time.monotonic()
        spin_for(0.30, state=LocalizationStatus.STATE_LOST,
                 local_fresh=False, correction=False,
                 command=Probe.twist(0.40, 0.30), permission=True)
        localization_end = time.monotonic()
        assert latest_cpp_state() == 'LOCALIZATION_BLOCKED'
        localization = compare_window(localization_start + 0.05, localization_end - 0.05)

        # Valid state but invalid local odometry health gets its own fail-closed state.
        health_start = time.monotonic()
        spin_for(0.30, state=LocalizationStatus.STATE_LOCALIZED,
                 local_fresh=False, correction=True,
                 command=Probe.twist(0.40, 0.30), permission=True)
        health_end = time.monotonic()
        assert latest_cpp_state() == 'HEALTH_BLOCKED'
        health = compare_window(health_start + 0.05, health_end - 0.05)

        # Recover only after fresh localization and a fresh command.
        recovery_start = time.monotonic()
        spin_for(0.18, command=None, permission=True)
        assert latest_cpp_state() == 'READY'
        spin_for(0.45, command=Probe.twist(-0.20, -0.15), permission=True)
        recovery_end = time.monotonic()
        assert latest_cpp_state() == 'ACTIVE'
        recovery = compare_window(recovery_start + 0.25, recovery_end - 0.05)

        # Fresh payload denial hard-stops immediately; missing permission also blocks.
        spin_for(0.12, state=LocalizationStatus.STATE_LOCALIZED,
                 command=Probe.twist(0.25, 0.10), permission=False)
        assert latest_cpp_state() == 'HEALTH_BLOCKED'
        spin_for(0.12, command=Probe.twist(0.25, 0.10), permission=None)
        assert latest_cpp_state() == 'HEALTH_BLOCKED'

        spin_for(0.12, command=Probe.twist(0.25, 0.10), permission=True)
        assert latest_cpp_state() == 'ACTIVE'
        spin_for(0.62, command=Probe.twist(0.25, 0.10), permission=None)
        assert latest_cpp_state() == 'HEALTH_BLOCKED'  # permission became stale

        windows = [active, stale, localization, health, recovery]
        cpp_period_ms = statistics.median(
            window[3] for window in windows if window[3] is not None) * 1000.0
        print(
            'SHADOW_PARITY PASS '
            f'aligned_samples={sum(w[0] for w in windows)} '
            f'max_linear_error={max(w[1] for w in windows):.6f} '
            f'max_angular_error={max(w[2] for w in windows):.6f} '
            f'cpp_median_period_ms={cpp_period_ms:.2f} '
            'states=READY,ACTIVE,STALE_COMMAND,LOCALIZATION_BLOCKED,HEALTH_BLOCKED',
            flush=True,
        )

        shutdown_start = len(probe.cpp_samples)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5.0)
        drain_until = time.monotonic() + 0.25
        while time.monotonic() < drain_until:
            executor.spin_once(timeout_sec=0.005)
        shutdown_samples = probe.cpp_samples[shutdown_start:]
        shutdown_zeros = [s for s in shutdown_samples if abs(s[1]) < 1e-9 and abs(s[2]) < 1e-9]
        assert len(shutdown_zeros) >= 3, (
            f'C++ shutdown published only {len(shutdown_zeros)} zero samples: {shutdown_samples}')
        assert all(abs(s[1]) < 1e-9 and abs(s[2]) < 1e-9 for s in shutdown_samples[-3:]), (
            f'last C++ shutdown outputs were not zero: {shutdown_samples[-3:]}')
        assert process.returncode == 0, log_path.read_text()
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=2.0)
        log_stream.close()
        executor.remove_node(probe)
        executor.remove_node(python_guard)
        probe.destroy_node()
        python_guard.destroy_node()
        PythonGuardShadow.OVERRIDES = {}
        rclpy.try_shutdown()
