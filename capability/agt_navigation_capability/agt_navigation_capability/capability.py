"""/navigation/navigate_to action; Nav2 completion is only one success gate."""

import time
import threading

import rclpy
from action_msgs.msg import GoalStatus
from agt_navigation_interfaces.action import FollowRoute, NavigateTo
from agt_navigation_interfaces.msg import NavigationHealth
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .policy import health_allows_motion, navigation_succeeded


class NavigationCapability(Node):
    def __init__(self):
        super().__init__('agt_navigation_capability')
        for name, default in (('robot_profile', 'bunker_v1'), ('map_id', ''),
                              ('map_version', ''), ('nav2_server_timeout_sec', 3.0)):
            self.declare_parameter(name, default)
        self.health = None
        self.health_received = 0.0
        self.running = False
        self._running_lock = threading.Lock()
        self.nav_goal = None
        self.health_lost = False
        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                       callback_group=group)
        self.follow_client = ActionClient(self, FollowPath, '/follow_path',
                                          callback_group=group)
        self.path_validation = self.create_client(IsPathValid, '/is_path_valid',
                                                  callback_group=group)
        self.action = ActionServer(
            self, NavigateTo, '/navigation/navigate_to',
            execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=self._cancel,
            callback_group=group)
        self.route_action = ActionServer(
            self, FollowRoute, '/navigation/follow_route',
            execute_callback=self._execute_follow,
            goal_callback=self._goal, cancel_callback=self._cancel,
            callback_group=group)
        self.create_subscription(
            NavigationHealth, '/navigation/health', self._on_health,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        self.active_pub = self.create_publisher(Bool, '/navigation/goal_active', 10)

    def _health_ok(self):
        value = lambda name: str(self.get_parameter(name).value)
        return health_allows_motion(
            self.health, time.monotonic() - self.health_received,
            value('robot_profile'), value('map_id'), value('map_version'))

    def _on_health(self, msg):
        self.health = msg
        self.health_received = time.monotonic()
        if self.running and not self._health_ok():
            self.health_lost = True
            if self.nav_goal is not None:
                self.nav_goal.cancel_goal_async()

    @staticmethod
    def _goal(_request):
        # Accept so a NOT_READY request receives a structured action result.
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _result(self, goal, code, message, canceled=False, result_type=NavigateTo.Result):
        result = result_type()
        result.success = False
        result.error_code = code
        result.message = message
        if canceled:
            goal.canceled()
        else:
            goal.abort()
        return result

    def _claim(self):
        with self._running_lock:
            if self.running:
                return False
            self.running = True
        self.health_lost = False
        self.active_pub.publish(Bool(data=True))
        return True

    def _release(self):
        self.nav_goal = None
        with self._running_lock:
            self.running = False
        self.active_pub.publish(Bool(data=False))

    async def _execute(self, goal):
        if not self._health_ok() or self.health.status != NavigationHealth.READY:
            return self._result(goal, 'NOT_READY', 'navigation health is not READY')
        if not self._claim():
            return self._result(goal, 'BUSY', 'another navigation goal is active')
        try:
            if not self.nav_client.wait_for_server(
                    timeout_sec=float(self.get_parameter('nav2_server_timeout_sec').value)):
                return self._result(goal, 'NAV2_UNAVAILABLE', 'Nav2 action server unavailable')
            request = NavigateToPose.Goal()
            request.pose = goal.request.pose
            nav_future = self.nav_client.send_goal_async(request)
            nav_goal = await nav_future
            if not nav_goal.accepted:
                return self._result(goal, 'NAV2_REJECTED', 'Nav2 rejected goal')
            self.nav_goal = nav_goal
            if self.health_lost or goal.is_cancel_requested:
                nav_goal.cancel_goal_async()
            wrapped = await nav_goal.get_result_async()
            if goal.is_cancel_requested:
                return self._result(goal, 'CANCELED', 'navigation canceled', canceled=True)
            if self.health_lost or not self._health_ok():
                return self._result(goal, 'HEALTH_DEGRADED', 'health degraded during navigation')
            if not navigation_succeeded(wrapped.status, self._health_ok()):
                return self._result(goal, 'NAV2_FAILED',
                                    f'Nav2 terminal status {wrapped.status}')
            result = NavigateTo.Result()
            result.success = True
            result.error_code = ''
            result.message = 'waypoint reached'
            goal.succeed()
            return result
        except Exception as exc:
            self.get_logger().error(f'navigation action failed: {exc}')
            return self._result(goal, 'NAVIGATION_ERROR', str(exc))
        finally:
            self._release()

    async def _execute_follow(self, goal):
        result_type = FollowRoute.Result
        path = goal.request.path
        if path.header.frame_id != 'map' or len(path.poses) < 2:
            return self._result(goal, 'INVALID_ROUTE', 'map-frame path with two poses required',
                                result_type=result_type)
        if not self._health_ok() or self.health.status != NavigationHealth.READY:
            return self._result(goal, 'NOT_READY', 'navigation health is not READY',
                                result_type=result_type)
        if not self._claim():
            return self._result(goal, 'BUSY', 'another navigation goal is active',
                                result_type=result_type)
        try:
            if not self.path_validation.wait_for_service(timeout_sec=2.0):
                return self._result(goal, 'PATH_VALIDATION_UNAVAILABLE',
                                    'Nav2 path validation unavailable', result_type=result_type)
            check = IsPathValid.Request()
            check.path = path
            valid = await self.path_validation.call_async(check)
            if not valid.is_valid:
                return self._result(goal, 'PATH_COLLISION',
                                    f'invalid path poses {list(valid.invalid_pose_indices)[:8]}',
                                    result_type=result_type)
            if not self._health_ok() or self.health_lost:
                return self._result(goal, 'HEALTH_DEGRADED', 'health lost during validation',
                                    result_type=result_type)
            if not self.follow_client.wait_for_server(timeout_sec=3.0):
                return self._result(goal, 'NAV2_UNAVAILABLE', 'Nav2 FollowPath unavailable',
                                    result_type=result_type)
            request = FollowPath.Goal()
            request.path = path
            request.controller_id = 'FollowPath'
            request.goal_checker_id = 'general_goal_checker'
            nav_goal = await self.follow_client.send_goal_async(request)
            if not nav_goal.accepted:
                return self._result(goal, 'NAV2_REJECTED', 'Nav2 rejected path',
                                    result_type=result_type)
            self.nav_goal = nav_goal
            if self.health_lost or goal.is_cancel_requested:
                nav_goal.cancel_goal_async()
            wrapped = await nav_goal.get_result_async()
            if goal.is_cancel_requested:
                return self._result(goal, 'CANCELED', 'route canceled', canceled=True,
                                    result_type=result_type)
            if self.health_lost or not self._health_ok():
                return self._result(goal, 'HEALTH_DEGRADED', 'health degraded during route',
                                    result_type=result_type)
            if not navigation_succeeded(wrapped.status, self._health_ok()):
                return self._result(goal, 'NAV2_FAILED', f'Nav2 status {wrapped.status}',
                                    result_type=result_type)
            result = result_type()
            result.success = True
            result.message = 'drawn route reached'
            goal.succeed()
            return result
        except Exception as exc:
            self.get_logger().error(f'follow route failed: {exc}')
            return self._result(goal, 'NAVIGATION_ERROR', str(exc), result_type=result_type)
        finally:
            self._release()


def main():
    rclpy.init()
    node = NavigationCapability()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
