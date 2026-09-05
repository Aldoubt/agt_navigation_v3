import math

import pytest
from nav_msgs.msg import Odometry

from agt_navigation_runtime.mission_runtime import odom_planar_motion_rate


def make_odom(stamp_sec: int, stamp_nanosec: int, x: float, y: float, yaw: float):
    msg = Odometry()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    return msg


def test_pose_delta_detects_motion_even_when_twist_is_zero():
    previous = make_odom(1, 0, 0.0, 0.0, 0.0)
    current = make_odom(1, 200_000_000, 0.10, 0.0, 0.04)

    # Odometry twist remains zero, matching the problematic Batch-LIO behavior.
    assert current.twist.twist.linear.x == 0.0
    assert current.twist.twist.angular.z == 0.0

    linear, angular = odom_planar_motion_rate(previous, current, 1.0)
    assert linear == pytest.approx(0.5, abs=1e-6)
    assert angular == pytest.approx(0.2, abs=1e-6)


def test_pose_delta_accepts_small_stationary_jitter():
    previous = make_odom(2, 0, 0.0, 0.0, 0.0)
    current = make_odom(2, 200_000_000, 0.004, -0.002, 0.006)
    linear, angular = odom_planar_motion_rate(previous, current, 1.0)
    assert linear < 0.04
    assert angular < 0.06


def test_pose_delta_rejects_invalid_time_interval():
    previous = make_odom(3, 0, 0.0, 0.0, 0.0)
    same_time = make_odom(3, 0, 1.0, 0.0, 0.0)
    too_late = make_odom(5, 0, 1.0, 0.0, 0.0)
    assert odom_planar_motion_rate(previous, same_time, 1.0) is None
    assert odom_planar_motion_rate(previous, too_late, 1.0) is None
