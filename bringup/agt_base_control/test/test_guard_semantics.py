"""Deterministic baseline tests for the existing Python motion guard.

These tests call the current Python node callbacks/tick directly and only
publish to isolated ``/p2_python_baseline_*`` topics. They never connect to a
base-driver command topic.
"""

import uuid

import pytest
import rclpy
from agt_base_control.cmd_vel_guard import CmdVelGuard
from agt_robot_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


class GuardUnderTest(CmdVelGuard):
    OVERRIDES = {}

    def declare_parameter(self, name, value=None, *args, **kwargs):
        return super().declare_parameter(name, self.OVERRIDES.get(name, value), *args, **kwargs)


@pytest.fixture()
def guard():
    if not rclpy.ok():
        rclpy.init()
    ns = 'p2_python_baseline_' + uuid.uuid4().hex[:10]
    GuardUnderTest.OVERRIDES = {
        'input_topic': f'/{ns}/command',
        'manual_input_topic': f'/{ns}/manual',
        'control_mode_topic': f'/{ns}/mode',
        'output_topic': f'/{ns}/output',
        'localization_status_topic': f'/{ns}/localization',
        'payload_drive_permission_topic': f'/{ns}/payload_permission',
        'payload_hold_topic': f'/{ns}/payload_hold',
        'require_localization_status': True,
        'require_payload_drive_permission': False,
    }
    node = GuardUnderTest()
    yield node
    node.destroy_node()
    GuardUnderTest.OVERRIDES = {}
    rclpy.try_shutdown()


def localized_status(state=LocalizationStatus.STATE_LOCALIZED, fresh=True, correction=True):
    msg = LocalizationStatus()
    msg.state = int(state)
    msg.local_odom_fresh = fresh
    msg.global_correction_valid = correction
    msg.reason = 'p2_python_baseline'
    return msg


def command(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def tick_after(guard, dt_sec=0.02):
    now_ns = guard.get_clock().now().nanoseconds
    guard.last_tick_ns = now_ns - int(dt_sec * 1e9)
    guard._tick_inner()


def test_startup_without_status_or_command_is_zero(guard):
    tick_after(guard)
    assert guard.output.linear.x == 0.0
    assert guard.output.angular.z == 0.0
    assert guard.last_rx_ns == 0


def test_linear_angular_clamps_and_unused_axes(guard):
    guard.set_parameters([
        Parameter('max_linear_accel', value=100.0),
        Parameter('max_linear_decel', value=100.0),
        Parameter('max_angular_accel', value=100.0),
    ])
    guard._on_localization_status(localized_status())
    guard._on_cmd(command(10.0, 10.0))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(0.55)
    assert guard.output.angular.z == pytest.approx(0.65)

    guard._on_cmd(command(-10.0, -10.0))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(-0.20)
    assert guard.output.angular.z == pytest.approx(-0.65)
    assert guard.output.linear.y == 0.0
    assert guard.output.linear.z == 0.0
    assert guard.output.angular.x == 0.0
    assert guard.output.angular.y == 0.0


def test_positive_negative_slew_and_deceleration_limits(guard):
    guard._on_localization_status(localized_status())
    guard._on_cmd(command(0.5, 0.5))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(0.045, abs=2e-5)
    assert guard.output.angular.z == pytest.approx(0.08, abs=2e-5)

    # Reversing direction uses the same positive-acceleration limit while the
    # target magnitude is larger than the current magnitude.
    guard._on_cmd(command(-0.5, -0.5))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(0.0, abs=2e-5)
    assert guard.output.angular.z == pytest.approx(0.0, abs=2e-5)
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(-0.045, abs=2e-5)
    assert guard.output.angular.z == pytest.approx(-0.08, abs=2e-5)

    # Reducing a positive linear target uses the distinct deceleration limit.
    guard.output.linear.x = 0.4
    guard.output.angular.z = 0.2
    guard._on_cmd(command(0.0, 0.0))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(0.32, abs=2e-5)
    assert guard.output.angular.z == pytest.approx(0.12, abs=2e-5)


def test_command_freshness_uses_reception_time_and_stales_to_zero(guard):
    guard._on_localization_status(localized_status())
    guard._on_cmd(command(0.3, 0.0))
    received_ns = guard.last_rx_ns
    assert 0 <= guard.get_clock().now().nanoseconds - received_ns < 50_000_000
    tick_after(guard)
    assert guard.output.linear.x > 0.0

    guard.last_rx_ns -= int(0.26 * 1e9)
    tick_after(guard)
    assert guard.output.linear.x == 0.0
    assert guard.output.angular.z == 0.0


def test_localization_health_loss_stops_and_requires_fresh_recovery_command(guard):
    guard._on_localization_status(localized_status())
    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x > 0.0

    guard._on_localization_status(localized_status(
        LocalizationStatus.STATE_LOST, fresh=False, correction=False))
    assert guard.output.linear.x == 0.0
    assert guard.last_rx_ns == 0
    guard._on_cmd(command(0.3, 0.2))  # rejected while localization is unsafe
    guard._on_localization_status(localized_status())
    tick_after(guard)
    assert guard.output.linear.x == 0.0

    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x > 0.0


def test_payload_health_missing_false_stale_and_recovery_are_fail_closed(guard):
    guard.set_parameters([Parameter('require_payload_drive_permission', value=True)])
    guard._on_localization_status(localized_status())
    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x == 0.0  # no initial permission

    guard._on_payload_permission(Bool(data=True))
    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x > 0.0

    guard._on_payload_permission(Bool(data=False))
    assert guard.output.linear.x == 0.0  # fresh denial hard-stops immediately
    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x == 0.0

    guard._on_payload_permission(Bool(data=True))
    tick_after(guard)
    assert guard.output.linear.x == 0.0  # blocked command was not cached
    guard._on_cmd(command(0.3, 0.2))
    tick_after(guard)
    assert guard.output.linear.x > 0.0

    guard.last_payload_rx_ns -= int(0.51 * 1e9)
    tick_after(guard)
    assert guard.output.linear.x == 0.0


def test_fresh_zero_command_slews_down_instead_of_being_discarded(guard):
    guard._on_localization_status(localized_status())
    guard.output.linear.x = 0.3
    guard.output.angular.z = 0.2
    guard._on_cmd(command(0.0, 0.0))
    tick_after(guard, 0.1)
    assert guard.output.linear.x == pytest.approx(0.22, abs=2e-5)
    assert guard.output.angular.z == pytest.approx(0.12, abs=2e-5)
