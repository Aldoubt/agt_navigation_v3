"""Opt-in isolated ROS Action tests; fake Nav2 only, no drivers or motion."""
import os
import threading
import time

import pytest
import rclpy
from agt_navigation_interfaces.action import FollowRoute, NavigateTo
from agt_navigation_interfaces.msg import NavigationHealth
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agt_navigation_capability.capability import NavigationCapability


pytestmark = pytest.mark.skipif(
    os.environ.get('AGT_RUN_ISOLATED_ROS_TESTS') != '1',
    reason='opt in with an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')


def wait(predicate, seconds=5):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('timed out waiting for isolated ROS action')


class FakeNav2(Node):
    def __init__(self):
        super().__init__('health_watchdog_fake_nav2')
        self.started = threading.Event()
        self.canceled = threading.Event()
        self.release = threading.Event()
        self.closed = threading.Event()
        self.cancel_calls = 0
        group = ReentrantCallbackGroup()
        self.nav_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', self._navigate,
            cancel_callback=self._cancel, callback_group=group)
        self.path_server = ActionServer(
            self, FollowPath, '/follow_path', self._follow,
            cancel_callback=self._cancel, callback_group=group)
        self.path_service = self.create_service(
            IsPathValid, '/is_path_valid', self._validate, callback_group=group)

    @staticmethod
    def _validate(_request, response):
        response.is_valid = True
        return response

    def _cancel(self, _goal):
        self.cancel_calls += 1
        return CancelResponse.ACCEPT

    def _run(self, handle, result_type):
        self.started.set()
        until = time.monotonic() + 7.0
        while time.monotonic() < until and not self.closed.is_set():
            if handle.is_cancel_requested:
                self.canceled.set()
                handle.canceled()
                return result_type()
            if self.release.is_set():
                handle.succeed()
                return result_type()
            time.sleep(0.01)
        if handle.is_active:
            handle.abort()
        return result_type()

    def _navigate(self, handle):
        return self._run(handle, NavigateToPose.Result)

    def _follow(self, handle):
        return self._run(handle, FollowPath.Result)


def ready_health():
    msg = NavigationHealth()
    msg.status = NavigationHealth.READY
    msg.state = 'NAV_READY'
    msg.robot_profile = 'bunker_v1'
    msg.platform_ready = True
    msg.lidar_alive = True
    msg.imu_alive = True
    msg.odom_alive = True
    msg.localized = True
    msg.nav2_active = True
    return msg


@pytest.fixture
def rig():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0')
    rclpy.init()
    cap = NavigationCapability()
    fake = FakeNav2()
    probe = Node('health_watchdog_probe')
    health_pub = probe.create_publisher(
        NavigationHealth, '/navigation/health',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    nav = ActionClient(probe, NavigateTo, '/navigation/navigate_to')
    route = ActionClient(probe, FollowRoute, '/navigation/follow_route')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (cap, fake, probe):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        yield cap, fake, health_pub, nav, route
    finally:
        # Let outstanding fake actions finish before destroying ROS entities,
        # including when an assertion itself failed (red-test run).
        fake.release.set()
        until = time.monotonic() + 4.0
        while cap.running and time.monotonic() < until:
            time.sleep(0.02)
        fake.closed.set()
        executor.shutdown(timeout_sec=6)
        for node in (cap, fake, probe):
            node.destroy_node()
        rclpy.try_shutdown()
        spinner.join(timeout=3)


def start(rig, kind):
    cap, fake, health_pub, nav, route = rig
    client = nav if kind == 'navigate' else route
    assert client.wait_for_server(timeout_sec=5)
    deadline = time.monotonic() + 3.0
    while not cap._health_ok() and time.monotonic() < deadline:
        health_pub.publish(ready_health())
        time.sleep(0.05)
    assert cap._health_ok()
    if kind == 'navigate':
        request = NavigateTo.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.pose.orientation.w = 1.0
    else:
        request = FollowRoute.Goal()
        request.path.header.frame_id = 'map'
        for x in (0.0, 1.0):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            request.path.poses.append(pose)
    sent = client.send_goal_async(request)
    wait(sent.done)
    handle = sent.result()
    assert handle.accepted
    wait(fake.started.is_set)
    health_pub.publish(ready_health())
    wait(lambda: cap._health_ok())
    return handle


@pytest.mark.parametrize('kind', ['navigate', 'follow'])
def test_missing_health_cancels_inflight_nav2_and_aborts_capability(rig, kind):
    cap, fake, _, _, _ = rig
    handle = start(rig, kind)
    wait(fake.canceled.is_set, 3.0)
    result_future = handle.get_result_async()
    wait(result_future.done)
    result = result_future.result().result
    assert not result.success
    assert result.error_code == 'HEALTH_DEGRADED'
    assert 'health_update_stale' in result.message
    assert cap.health_lost
    assert fake.cancel_calls == 1


def test_reported_degradation_cancels_and_does_not_unlatch_on_recovery(rig):
    cap, fake, pub, _, _ = rig
    handle = start(rig, 'navigate')
    degraded = ready_health()
    degraded.status = NavigationHealth.DEGRADED
    degraded.state = 'DEGRADED'
    degraded.last_error_code = 'LOCALIZATION_LOST'
    pub.publish(degraded)
    wait(fake.canceled.is_set, 1.5)
    pub.publish(ready_health())
    future = handle.get_result_async()
    wait(future.done)
    result = future.result().result
    assert not result.success and result.error_code == 'HEALTH_DEGRADED'
    assert 'LOCALIZATION_LOST' in result.message
    assert cap.health_lost
    assert fake.cancel_calls == 1


def test_fresh_health_does_not_cancel_inflight_nav2(rig):
    cap, fake, pub, _, _ = rig
    handle = start(rig, 'navigate')
    until = time.monotonic() + 2.0
    while time.monotonic() < until:
        pub.publish(ready_health())
        time.sleep(0.1)
    assert cap._health_ok() and not cap.health_lost
    assert not fake.canceled.is_set()
    fake.release.set()
    result_future = handle.get_result_async()
    wait(result_future.done)
    assert result_future.result().result.success
    assert fake.cancel_calls == 0
