"""rclpy test: agt_cmd_vel_guard payload (arm) drive-permission interlock.

Runs the real CmdVelGuard node in-process with unique topics. No hardware.
"""
import time
import uuid

import pytest
import rclpy
from agt_base_control.cmd_vel_guard import CmdVelGuard
from agt_robot_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

MOVING = 0.05


class Probe(Node):
    def __init__(self, ns):
        super().__init__('payload_interlock_probe_' + ns)
        self.cmd = self.create_publisher(Twist, f'/{ns}/cmd', 20)
        self.manual = self.create_publisher(Twist, f'/{ns}/manual', 20)
        self.mode = self.create_publisher(String, f'/{ns}/mode', 10)
        self.status = self.create_publisher(LocalizationStatus, f'/{ns}/status', 20)
        self.perm = self.create_publisher(Bool, f'/{ns}/perm', 20)
        self.samples = []
        self.holds = []
        self.create_subscription(Bool, f"/{ns}/hold", lambda m: self.holds.append(
            (time.monotonic(), m.data)), 100)
        self.create_subscription(Twist, f'/{ns}/out', lambda m: self.samples.append(
            (time.monotonic(), m.linear.x, m.angular.z)), 100)

    def localized(self):
        msg = LocalizationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = int(LocalizationStatus.STATE_LOCALIZED)
        msg.local_odom_fresh = True
        msg.global_correction_valid = True
        return msg


def make(ns, **extra):
    overrides = {
        'input_topic': f'/{ns}/cmd', 'manual_input_topic': f'/{ns}/manual',
        'control_mode_topic': f'/{ns}/mode', 'output_topic': f'/{ns}/out',
        'localization_status_topic': f'/{ns}/status',
        'payload_drive_permission_topic': f'/{ns}/perm',
        'payload_hold_topic': f'/{ns}/hold',
        'require_payload_drive_permission': True,
        'max_linear_accel': 10.0, 'max_linear_decel': 10.0, 'max_angular_accel': 10.0,
    }
    overrides.update(extra)
    return overrides


class GuardUnderTest(CmdVelGuard):
    OVERRIDES = {}

    def declare_parameter(self, name, value=None, *a, **k):  # noqa: D401
        return super().declare_parameter(name, self.OVERRIDES.get(name, value), *a, **k)


def spin(executor, probe, seconds, perm=None, cmd=None, manual=None):
    end = time.monotonic() + seconds
    nxt = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= nxt:
            probe.status.publish(probe.localized())
            if perm is not None:
                probe.perm.publish(Bool(data=perm))
            if cmd is not None:
                probe.cmd.publish(cmd)
            if manual is not None:
                probe.manual.publish(manual)
            nxt = now + 0.02
        executor.spin_once(timeout_sec=0.005)


def twist(x):
    msg = Twist()
    msg.linear.x = float(x)
    return msg


def max_speed(probe, since):
    vals = [abs(x) for t, x, _ in probe.samples if t >= since]
    return max(vals) if vals else None


@pytest.fixture()
def env():
    rclpy.init()
    ns = 'g' + uuid.uuid4().hex[:8]
    executor = SingleThreadedExecutor()
    probe = Probe(ns)
    executor.add_node(probe)
    created = []

    def build(**extra):
        GuardUnderTest.OVERRIDES = make(ns, **extra)
        guard = GuardUnderTest()
        executor.add_node(guard)
        created.append(guard)
        spin(executor, probe, 0.3)  # discovery
        return guard

    yield executor, probe, build
    for node in created:
        node.destroy_node()
    probe.destroy_node()
    rclpy.try_shutdown()


def test_never_received_permission_blocks_navigation(env):
    executor, probe, build = env
    build()
    t0 = time.monotonic()
    spin(executor, probe, 0.6, perm=None, cmd=twist(0.3))
    assert max_speed(probe, t0) == 0.0


def test_true_permission_allows_and_false_hard_stops(env):
    executor, probe, build = env
    build()
    spin(executor, probe, 0.6, perm=True, cmd=twist(0.3))
    assert max_speed(probe, time.monotonic() - 0.2) > MOVING
    t1 = time.monotonic()
    spin(executor, probe, 0.1, perm=False, cmd=twist(0.3))
    t_check = time.monotonic()
    spin(executor, probe, 0.5, perm=False, cmd=twist(0.3))
    assert max_speed(probe, t_check) == 0.0, 'False permission must hard-stop'
    assert t_check - t1 < 0.2


def test_stale_permission_blocks(env):
    executor, probe, build = env
    build(payload_drive_permission_timeout_sec=0.3)
    spin(executor, probe, 0.5, perm=True, cmd=twist(0.3))
    assert max_speed(probe, time.monotonic() - 0.2) > MOVING
    spin(executor, probe, 0.5, perm=None, cmd=twist(0.3))  # permission goes silent
    t_check = time.monotonic()
    spin(executor, probe, 0.4, perm=None, cmd=twist(0.3))
    assert max_speed(probe, t_check) == 0.0


def test_manual_hmi_is_also_blocked(env):
    executor, probe, build = env
    build()
    probe.mode.publish(String(data='manual'))
    spin(executor, probe, 0.2)
    t0 = time.monotonic()
    spin(executor, probe, 0.6, perm=False, manual=twist(0.3))
    assert max_speed(probe, t0) == 0.0
    spin(executor, probe, 0.6, perm=True, manual=twist(0.3))
    assert max_speed(probe, time.monotonic() - 0.2) > MOVING


def test_reopen_does_not_replay_cached_command(env):
    executor, probe, build = env
    build()
    spin(executor, probe, 0.5, perm=False, cmd=twist(0.3))  # intent produced while blocked
    t0 = time.monotonic()
    spin(executor, probe, 0.5, perm=True, cmd=None)  # permission back, no fresh cmd
    assert max_speed(probe, t0) == 0.0


def test_gate_disabled_keeps_bunker_behaviour(env):
    executor, probe, build = env
    build(require_payload_drive_permission=False)
    spin(executor, probe, 0.6, perm=None, cmd=twist(0.3))
    assert max_speed(probe, time.monotonic() - 0.2) > MOVING


def holds_since(probe, since):
    return [v for t, v in probe.holds if t >= since]


def test_payload_hold_acknowledges_deny_only_with_zero_output(env):
    """R3.1: the arm starts only after the guard confirms deny + zero output."""
    executor, probe, build = env
    build()
    spin(executor, probe, 0.6, perm=True, cmd=twist(0.3))
    t0 = time.monotonic() - 0.2
    assert max_speed(probe, t0) > MOVING
    assert holds_since(probe, t0) and not any(holds_since(probe, t0)),         'permission True: no hold'
    spin(executor, probe, 0.15, perm=False, cmd=twist(0.3))
    t1 = time.monotonic()
    spin(executor, probe, 0.5, perm=False, cmd=twist(0.3))
    vals = holds_since(probe, t1)
    assert vals and all(vals), 'deny received and output zero -> hold True'
    assert max_speed(probe, t1) == 0.0


def test_payload_hold_false_when_interlock_disabled(env):
    executor, probe, build = env
    build(require_payload_drive_permission=False)
    t0 = time.monotonic()
    spin(executor, probe, 0.5, perm=False, cmd=twist(0.3))
    vals = holds_since(probe, t0)
    assert vals and not any(vals), 'disabled interlock never claims a hold'


def test_payload_hold_not_claimed_for_missing_or_stale_permission(env):
    """R3.1b: missing/stale permission stops the base but is NOT an ack of a deny."""
    executor, probe, build = env
    build()
    t0 = time.monotonic()
    spin(executor, probe, 0.5, perm=None, cmd=twist(0.3))
    vals = holds_since(probe, t0)
    assert vals and not any(vals), "never-received permission is not a hold"
    assert max_speed(probe, t0) == 0.0
    spin(executor, probe, 0.3, perm=False, cmd=twist(0.3))
    assert all(holds_since(probe, time.monotonic() - 0.1))
    t1 = time.monotonic() + 0.7        # > payload_drive_permission_timeout_sec 0.5
    spin(executor, probe, 1.0, perm=None, cmd=twist(0.3))
    vals = holds_since(probe, t1)
    assert vals and not any(vals), "stale permission is not a hold"
