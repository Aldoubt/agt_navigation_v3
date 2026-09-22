"""Exercise production callbacks with message types and fake I/O; no ROS graph."""
from collections import deque
import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import TransformException

from agt_fastlio_adapter.fastlio_adapter import FastLioAdapter
from agt_fastlio_adapter.pose_twist import PoseTwistEstimator


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    sendTransform = publish


class StaticBuffer:
    missing = False

    def lookup_transform(self, target, source, stamp):
        if self.missing and source == 'base_footprint':
            raise TransformException('fixture static transform unavailable')
        result = TransformStamped()
        result.transform.rotation.w = 1.0
        if source == 'base_link':
            result.transform.translation.x = -0.3
            result.transform.translation.z = -0.8
        else:
            result.transform.translation.z = -0.2
        return result


class Harness(FastLioAdapter):
    """Deliberately does not call Node.__init__, so no DDS participant is created."""
    def __init__(self):
        self.now_ns = 10_000_000_000
        self.parameters = {
            'expected_odom_frame': 'odom', 'expected_base_frame': 'body',
            'output_odom_frame': 'odom', 'output_base_frame': 'base_link',
            'tf_child_frame': 'base_footprint', 'mount_lidar_frame': 'livox_frame',
            'mount_base_frame': 'base_link', 'max_input_age_sec': .2,
            'max_future_input_sec': .05, 'reject_zero_stamp': True,
            'convert_body_to_base': True, 'derive_twist_from_pose': True,
            'publish_rate_window_sec': 1.0, 'stale_pose_timeout_sec': 1.0,
        }
        self._accepted = self._rejected = self._last_warn_ns = 0
        self._last_input_stamp_ns = self._last_valid_rx_ns = self._last_output_stamp_ns = 0
        self._publish_times = deque()
        self._body_to_base_cache = self._base_to_tf_child_cache = None
        self._internal_extrinsic = ((-.011, -.02329, .04412), (0., 0., 0., 1.))
        self._twist = PoseTwistEstimator()
        self._tf_buffer = StaticBuffer()
        self._pub, self._tf, self._status_pub = Sink(), Sink(), Sink()
        self._path_pub = None

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.now_ns))

    def get_logger(self):
        return SimpleNamespace(info=lambda text: None, warn=lambda text: None)


def odom(adapter, age=.05, x=0., angle=0.):
    msg = Odometry()
    msg.header.stamp = Time(nanoseconds=adapter.now_ns - int(age*1e9)).to_msg()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'body'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.orientation.z = math.sin(angle/2)
    msg.pose.pose.orientation.w = math.cos(angle/2)
    return msg


def test_accepted_pose_has_paired_navigation_tf_and_nonzero_rotation_velocity():
    adapter = Harness()
    first = odom(adapter)
    adapter._on_odom(first)
    adapter.now_ns += 100_000_000
    second = odom(adapter, angle=.02)
    adapter._on_odom(second)
    assert adapter._accepted == 2 and adapter._rejected == 0
    assert len(adapter._pub.messages) == len(adapter._tf.messages) == 2
    out, transform = adapter._pub.messages[-1], adapter._tf.messages[-1]
    assert out.header.frame_id == 'odom' and out.child_frame_id == 'base_link'
    assert transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint'
    assert out.header.stamp == second.header.stamp == transform.header.stamp
    assert out.twist.twist.angular.z == pytest.approx(.2)
    assert second.twist.twist.angular.z == 0.0  # Input was not mutated.
    adapter._publish_adapter_status()
    status = adapter._status_pub.messages[-1]
    assert status.state == 0 and status.accepted_count == 2 and status.publish_rate == 2.0


def test_missing_tf_does_not_publish_partial_odometry_or_advance_history():
    adapter = Harness()
    adapter._tf_buffer.missing = True
    adapter._on_odom(odom(adapter))
    assert not adapter._pub.messages and not adapter._tf.messages
    assert adapter._accepted == 0 and adapter._rejected == 1
    assert adapter._twist.previous is None


def test_half_second_old_output_is_rejected_and_diagnostics_go_stale():
    adapter = Harness()
    adapter._on_odom(odom(adapter))
    adapter.now_ns += 100_000_000
    adapter._on_odom(odom(adapter, age=.6))
    adapter._publish_adapter_status()
    assert adapter._status_pub.messages[-1].state == 1
    assert adapter._accepted == 1 and adapter._rejected == 1
    adapter.now_ns += 1_100_000_000
    adapter._publish_adapter_status()
    assert adapter._status_pub.messages[-1].state == 2
    assert adapter._status_pub.messages[-1].publish_rate == 0.0
    assert len(adapter._pub.messages) == 1


@pytest.mark.parametrize('kind', ['zero_stamp', 'future', 'wrong_frame', 'nan', 'zero_quaternion'])
def test_invalid_input_cannot_reach_navigation(kind):
    adapter = Harness()
    msg = odom(adapter)
    if kind == 'zero_stamp':
        msg.header.stamp = Time(nanoseconds=0).to_msg()
    elif kind == 'future':
        msg = odom(adapter, age=-.1)
    elif kind == 'wrong_frame':
        msg.header.frame_id = 'map'
    elif kind == 'nan':
        msg.pose.pose.position.x = float('nan')
    else:
        msg.pose.pose.orientation.w = 0.0
    adapter._on_odom(msg)
    assert adapter._accepted == 0 and adapter._rejected == 1
    assert not adapter._pub.messages and not adapter._tf.messages


def test_duplicate_timestamp_is_not_republished():
    adapter = Harness()
    msg = odom(adapter)
    adapter._on_odom(msg)
    adapter._on_odom(msg)
    assert adapter._accepted == 1 and adapter._rejected == 1


def test_no_input_status_is_explicit_and_does_not_synthesize_odometry():
    adapter = Harness()
    adapter._publish_adapter_status()
    status = adapter._status_pub.messages[-1]
    assert status.state == 3 and status.publish_rate == 0.0
    assert not adapter._pub.messages and not adapter._tf.messages
