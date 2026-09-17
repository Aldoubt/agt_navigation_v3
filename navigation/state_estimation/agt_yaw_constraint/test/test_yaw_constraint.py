import math

import pytest
from nav_msgs.msg import Odometry

from agt_yaw_constraint.yaw_constraint import (
    build_yaw_constraint,
    compute_yaw_constraint,
    frame_mismatch,
    yaw_jump_rejected,
)


def make_odom(
    yaw,
    speed,
    yaw_rate,
    stamp_sec=1,
    stamp_nanosec=0,
    frame_id='odom',
    child_frame_id='base_link',
):
    msg = Odometry()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    msg.twist.twist.linear.x = speed
    msg.twist.twist.angular.z = yaw_rate
    return msg


def test_two_odom_inputs_compute_yaw_difference_and_rates():
    lio = make_odom(math.radians(10.0), 1.0, 0.12)
    wheel = make_odom(math.radians(30.0), 1.0, 0.20)

    result = compute_yaw_constraint(lio, wheel, min_speed_threshold=0.05)

    assert result is not None
    assert result.lio_yaw == pytest.approx(math.radians(10.0))
    assert result.wheel_yaw == pytest.approx(math.radians(30.0))
    assert result.delta_yaw == pytest.approx(math.radians(20.0))
    assert result.yaw_rate_lio == pytest.approx(0.12)
    assert result.yaw_rate_wheel == pytest.approx(0.20)


def test_time_stamps_and_signed_time_offset_are_reported():
    lio = make_odom(0.0, 1.0, 0.0, stamp_sec=10, stamp_nanosec=100_000_000)
    wheel = make_odom(0.0, 1.0, 0.0, stamp_sec=10, stamp_nanosec=250_000_000)

    result = build_yaw_constraint(lio, wheel)

    assert result.lio_stamp == lio.header.stamp
    assert result.wheel_stamp == wheel.header.stamp
    assert result.time_offset_sec == pytest.approx(0.15)


def test_yaw_difference_wraps_179_and_minus_179_degrees():
    lio = make_odom(math.radians(-179.0), 1.0, 0.0)
    wheel = make_odom(math.radians(179.0), 1.0, 0.0)

    result = compute_yaw_constraint(lio, wheel, min_speed_threshold=0.05)

    assert result is not None
    assert result.delta_yaw == pytest.approx(math.radians(-2.0))


def test_low_speed_filters_yaw_observation():
    lio = make_odom(0.2, 0.01, 0.4)
    wheel = make_odom(0.8, 0.02, 0.5)

    assert compute_yaw_constraint(lio, wheel, min_speed_threshold=0.05) is None


def test_yaw_jump_over_45_degrees_is_rejected():
    lio = make_odom(0.0, 1.0, 0.0)
    wheel = make_odom(math.radians(46.0), 1.0, 0.0)
    result = build_yaw_constraint(lio, wheel)

    assert yaw_jump_rejected(result, 45.0)
    assert not yaw_jump_rejected(result, 50.0)


def test_frame_mismatch_is_detected_without_modifying_frames():
    lio = make_odom(0.0, 1.0, 0.0, frame_id='odom', child_frame_id='base_link')
    wheel = make_odom(
        0.0,
        1.0,
        0.0,
        frame_id='wheel_odom',
        child_frame_id='base_footprint',
    )

    assert frame_mismatch(lio, wheel)
    assert lio.header.frame_id == 'odom'
    assert wheel.header.frame_id == 'wheel_odom'
