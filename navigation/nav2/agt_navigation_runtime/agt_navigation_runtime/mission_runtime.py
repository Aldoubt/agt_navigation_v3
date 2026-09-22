from __future__ import annotations

import threading
import time
import json
import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from agt_robot_interfaces.action import ExecuteInspectionMission
from agt_robot_interfaces.msg import MissionStatus
from camera_gimbal_interfaces.action import AcquireView

from .mission_schema import load_mission
from .record_writer import RecordWriter
from .ros_wait import RosWaiter
from .action_execution import PendingAction, execute_action, cancel_and_confirm


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def odom_planar_motion_rate(previous: Odometry, current: Odometry, max_dt_sec: float):
    """Return pose-derived planar linear/yaw rates, or None for an invalid interval."""
    prev_ns = Time.from_msg(previous.header.stamp).nanoseconds
    curr_ns = Time.from_msg(current.header.stamp).nanoseconds
    dt = (curr_ns - prev_ns) / 1e9
    if dt <= 1e-4 or dt > max_dt_sec:
        return None

    p0 = previous.pose.pose.position
    p1 = current.pose.pose.position
    linear = math.hypot(p1.x - p0.x, p1.y - p0.y) / dt
    yaw0 = quaternion_to_yaw(previous.pose.pose.orientation)
    yaw1 = quaternion_to_yaw(current.pose.pose.orientation)
    angular = abs(wrap_angle(yaw1 - yaw0)) / dt
    return linear, angular


class MissionRuntime(Node):
    def __init__(self, **node_options):
        super().__init__('mission_runtime', **node_options)
        defaults = {
            'global_frame': 'map', 'base_frame': 'base_link',
            'mission_action': '/agt/mission/execute',
            'nav_action_timeout_sec': 300.0, 'camera_action_timeout_sec': 30.0,
            'goal_response_timeout_sec': 5.0, 'cancel_confirm_timeout_sec': 5.0,
            'runtime_poll_interval_sec': 0.05,
            'nav_action': '/navigate_to_pose',
            'acquire_view_action': '/camera_gimbal/acquire_view',
            'navsat_topic': '/ins/navsatfix',
            'local_odom_topic': '/agt/odometry/local',
            'hmi_task_request_topic': '/agt/task/request',
            'hmi_task_status_topic': '/agt/task/status',
            'record_root': '~/.ros/agt_inspection_records',
            'nav_server_timeout_sec': 10.0, 'camera_server_timeout_sec': 10.0,
            'tf_lookup_timeout_sec': 0.25, 'rtk_max_age_sec': 1.0,
            'default_point_settle_sec': 1.0,
            'stationary_linear_threshold_mps': 0.03,
            'stationary_angular_threshold_rps': 0.05,
            'stationary_pose_linear_threshold_mps': 0.04,
            'stationary_pose_angular_threshold_rps': 0.06,
            'stationary_pose_max_dt_sec': 1.0,
            'stationary_hold_sec': 0.8,
            'stationary_timeout_sec': 8.0,
            'odom_freshness_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        for name in ('nav_action_timeout_sec', 'camera_action_timeout_sec',
                     'goal_response_timeout_sec', 'cancel_confirm_timeout_sec',
                     'runtime_poll_interval_sec', 'nav_server_timeout_sec',
                     'camera_server_timeout_sec', 'stationary_timeout_sec',
                     'stationary_hold_sec', 'odom_freshness_sec'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self._control_group = ReentrantCallbackGroup()
        self._admission_lock = threading.RLock()
        self._odom_lock = threading.RLock()
        self._mission_reserved = False
        self._terminal_committing = False
        self._shutdown_requested = False
        self._fault_reason = ''
        self._fault_operations = []
        self._active_operation = None
        self._writer = None
        self._completed_points = 0
        self._current_point_id = ''
        self._current_view_tag = ''
        self._navigation_was_sent = False
        self._stationary_since_ns = None
        self._last_recorded_status = None
        self._parent_cancel_future = None
        self._waiter = RosWaiter(self, self._control_group)
        self.mission_action = self.resolve_topic_name(str(self.get_parameter('mission_action').value))
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.record_root = self.get_parameter('record_root').value
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rtk_samples = deque(maxlen=300)
        self.latest_local_odom = None
        self.latest_local_odom_rx_ns = 0
        self.previous_motion_odom = None
        self.latest_pose_linear_mps = None
        self.latest_pose_angular_rps = None
        self.latest_pose_motion_rx_ns = 0
        self.pending_hmi_mission = ''
        self.paused = False
        self.cancel_requested = False
        self.active_goal_handle = None
        self.active_nav_goal = None
        self.active_camera_goal = None

        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value, callback_group=self._control_group)
        self.camera_client = ActionClient(self, AcquireView, self.get_parameter('acquire_view_action').value, callback_group=self._control_group)
        self.loopback_client = ActionClient(self, ExecuteInspectionMission, self.mission_action, callback_group=self._control_group)
        self._parent_cancel_client = self.create_client(
            CancelGoal, self.mission_action + '/_action/cancel_goal', callback_group=self._control_group)
        self.server = ActionServer(
            self, ExecuteInspectionMission, self.mission_action,
            execute_callback=self.execute_mission,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._control_group,
        )
        self.status_pub = self.create_publisher(MissionStatus, '/agt/mission/status', 10)
        self.hmi_status_pub = self.create_publisher(String, self.get_parameter('hmi_task_status_topic').value, 10)
        self.create_subscription(String, self.get_parameter('hmi_task_request_topic').value, self.on_hmi_task_request, 10)
        self.create_subscription(NavSatFix, self.get_parameter('navsat_topic').value, self.on_navsat, 20)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self.on_local_odom, 50)
        self.create_service(Trigger, '/agt/task/start', self.on_hmi_start, callback_group=self._control_group)
        self.create_service(Trigger, '/agt/task/pause', self.on_hmi_pause, callback_group=self._control_group)
        self.create_service(Trigger, '/agt/task/cancel', self.on_hmi_cancel, callback_group=self._control_group)
        self.create_service(Trigger, '/agt/mission/reset_fault', self.on_reset_fault,
                            callback_group=self._control_group)
        self.add_on_set_parameters_callback(self._validate_safety_parameters)

    def _validate_safety_parameters(self, parameters):
        positive = {
            'nav_action_timeout_sec', 'camera_action_timeout_sec',
            'goal_response_timeout_sec', 'cancel_confirm_timeout_sec',
            'runtime_poll_interval_sec', 'nav_server_timeout_sec', 'camera_server_timeout_sec',
            'stationary_timeout_sec', 'stationary_hold_sec', 'odom_freshness_sec',
            'stationary_pose_max_dt_sec', 'tf_lookup_timeout_sec', 'rtk_max_age_sec',
        }
        nonnegative = {
            'stationary_linear_threshold_mps', 'stationary_angular_threshold_rps',
            'stationary_pose_linear_threshold_mps', 'stationary_pose_angular_threshold_rps',
            'default_point_settle_sec',
        }
        for parameter in parameters:
            if parameter.name in positive | nonnegative:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f'{parameter.name} must be numeric')
                if not math.isfinite(value) or value < 0 or (parameter.name in positive and value == 0):
                    return SetParametersResult(successful=False, reason=f'{parameter.name} has an invalid safety/timing value')
            if self._mission_reserved and parameter.name in positive | nonnegative | {'use_sim_time'}:
                return SetParametersResult(successful=False, reason='safety/timing parameters are frozen during an active mission')
        return SetParametersResult(successful=True)

    def goal_callback(self, request):
        with self._admission_lock:
            if (self._mission_reserved or self._fault_reason or self._shutdown_requested
                    or bool(request.resume)):
                return GoalResponse.REJECT
            self._mission_reserved = True
            self._terminal_committing = False
            return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        with self._admission_lock:
            if self._terminal_committing:
                return CancelResponse.REJECT
            self.cancel_requested = True
        self.cancel_active_subgoal()
        return CancelResponse.ACCEPT

    def cancel_active_subgoal(self):
        op = self._active_operation
        if op is not None:
            op.request_cancel()

    def on_hmi_task_request(self, msg):
        self.pending_hmi_mission = msg.data.strip()
        self.publish_text_status('TASK_LOADED', self.pending_hmi_mission)

    def on_hmi_start(self, _request, response):
        with self._admission_lock:
            if self._fault_reason:
                response.success, response.message = False, 'FAULT LATCHED: ' + self._fault_reason
                return response
            if self._mission_reserved or self.active_goal_handle is not None:
                response.success, response.message = False, 'A mission is already active'
                return response
        if not self.pending_hmi_mission:
            response.success, response.message = False, 'No task file has been handed off'
            return response
        if not self.loopback_client.server_is_ready():
            response.success, response.message = False, 'Mission action server unavailable'
            return response
        goal = ExecuteInspectionMission.Goal()
        goal.mission_file, goal.resume = self.pending_hmi_mission, False
        self.loopback_client.send_goal_async(goal)
        response.success, response.message = True, 'mission submitted; monitor task status/result'
        return response

    def on_hmi_pause(self, _request, response):
        if not self._mission_reserved:
            response.success, response.message = False, 'No active mission'
            return response
        self.paused = not self.paused
        response.success = True
        response.message = ('pause requested at next safe stage boundary; not an emergency stop'
                            if self.paused else 'resumed')
        return response

    def on_hmi_cancel(self, _request, response):
        with self._admission_lock:
            if self._terminal_committing:
                response.success, response.message = False, 'mission terminal result is already being committed'
                return response
        if self.active_goal_handle is None:
            response.success, response.message = True, 'No executing mission'
            return response
        self.cancel_requested = True
        self.cancel_active_subgoal()
        self._request_parent_cancel()
        response.success, response.message = True, 'cancel requested; terminal/stop confirmation pending'
        return response

    def on_navsat(self, msg):
        self.rtk_samples.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))

    def on_local_odom(self, msg):
        with self._odom_lock:
            now_ns = time.monotonic_ns()
            if self.previous_motion_odom is not None:
                rates = odom_planar_motion_rate(
                    self.previous_motion_odom,
                    msg,
                    float(self.get_parameter('stationary_pose_max_dt_sec').value),
                )
                if rates is None:
                    self.latest_pose_linear_mps = None
                    self.latest_pose_angular_rps = None
                    self.latest_pose_motion_rx_ns = 0
                else:
                    self.latest_pose_linear_mps, self.latest_pose_angular_rps = rates
                    self.latest_pose_motion_rx_ns = now_ns
            self.previous_motion_odom = msg
            self.latest_local_odom = msg
            self.latest_local_odom_rx_ns = now_ns
            stationary, _ = self._motion_is_stationary(now_ns)
            if stationary:
                if self._stationary_since_ns is None:
                    self._stationary_since_ns = now_ns
            else:
                self._stationary_since_ns = None

    def publish_status(self, state, mission_id='', point_id='', index=0, count=0,
                       detail='', error_code=0, goal_handle=None):
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state, msg.mission_id, msg.point_id = state, mission_id, point_id
        msg.point_index, msg.point_count = index, count
        msg.detail, msg.error_code = detail, error_code
        self.status_pub.publish(msg)
        text = String()
        text.data = json.dumps({
            'state': int(state), 'mission_id': mission_id, 'point_id': point_id,
            'point_index': index, 'point_count': count, 'detail': detail,
            'error_code': error_code, 'terminal': state in {MissionStatus.COMPLETED,
                MissionStatus.CANCELED, MissionStatus.ERROR}}, ensure_ascii=False)
        self.hmi_status_pub.publish(text)
        key = (int(state), point_id, index, detail, int(error_code))
        if self._writer is not None and key != self._last_recorded_status:
            try:
                self._writer.event('mission_state', state=int(state), point_id=point_id,
                                   point_index=index, detail=detail, error_code=int(error_code))
            except OSError as exc:
                self.get_logger().error(f'state journal write failed: {exc}')
            self._last_recorded_status = key
        if goal_handle is not None:
            feedback = ExecuteInspectionMission.Feedback()
            feedback.status = msg
            goal_handle.publish_feedback(feedback)

    def publish_text_status(self, state, detail=''):
        msg = String()
        msg.data = json.dumps({'state': state, 'detail': detail}, ensure_ascii=False)
        self.hmi_status_pub.publish(msg)

    async def wait_while_paused(self, goal_handle, mission, point, index):
        while self.paused and self.context.ok() and not self._interrupted(goal_handle):
            self.publish_status(MissionStatus.PAUSED, mission.mission_id, point.id, index,
                                len(mission.points), 'paused at stage boundary', goal_handle=goal_handle)
            await self._waiter.sleep(0.05)
        return not self._interrupted(goal_handle)

    async def wait_until_stationary(self, *, ignore_cancel=False):
        timeout = float(self.get_parameter('stationary_timeout_sec').value)
        hold = float(self.get_parameter('stationary_hold_sec').value)
        start = time.monotonic()
        stable_since = None
        while self.context.ok() and not self._shutdown_requested:
            if self.cancel_requested and not ignore_cancel:
                return False, 'stationary check canceled'
            now = time.monotonic()
            if now - start >= timeout:
                return False, 'base stationary state was not observed before timeout'
            stationary, detail = self._motion_is_stationary(time.monotonic_ns())
            if stationary:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= hold:
                    return True, detail
            else:
                stable_since = None
            await self._waiter.sleep(min(0.05, max(0.001, timeout - (now - start))))
        return False, 'runtime shutdown before stationary confirmation'

    async def execute_mission(self, goal_handle):
        self.active_goal_handle = goal_handle
        self.cancel_requested = bool(goal_handle.is_cancel_requested)
        self.paused = False
        self._parent_cancel_future = None
        self._completed_points = 0
        self._current_point_id = self._current_view_tag = ''
        self._navigation_was_sent = False
        self._last_recorded_status = None
        mission = None
        try:
            mission = load_mission(goal_handle.request.mission_file)
            self._writer = RecordWriter(self.record_root, mission)
            self.pending_hmi_mission = mission.source_file
            for index, point in enumerate(mission.points):
                self._current_point_id, self._current_view_tag = point.id, ''
                if not await self.wait_while_paused(goal_handle, mission, point, index):
                    return await self.finish_canceled(goal_handle, mission)
                self.publish_status(MissionStatus.NAVIGATING, mission.mission_id, point.id, index,
                                    len(mission.points), 'sending bounded Nav2 goal', goal_handle=goal_handle)
                nav = await self.navigate(point)
                if self._interrupted(goal_handle) or nav.state == 'canceled':
                    return await self.finish_canceled(goal_handle, mission)
                if not nav.success:
                    code = 1200 if nav.state == 'timeout' else 1000
                    return await self._finish_with_stop(goal_handle, mission, 'failed', code, nav.message)

                self.publish_status(MissionStatus.STABILIZING, mission.mission_id, point.id, index,
                                    len(mission.points), 'waiting for measured base stop', goal_handle=goal_handle)
                stopped, detail = await self.wait_until_stationary()
                if self._interrupted(goal_handle):
                    return await self.finish_canceled(goal_handle, mission)
                if not stopped:
                    return await self._finish_with_stop(goal_handle, mission, 'failed', 1100, detail)
                settle = (point.settle_time if point.settle_time >= 0 else
                          float(self.get_parameter('default_point_settle_sec').value))
                if settle > 0:
                    self.publish_status(MissionStatus.STABILIZING, mission.mission_id, point.id, index,
                                        len(mission.points), f'{detail}; extra settle {settle:.2f}s',
                                        goal_handle=goal_handle)
                    if not await self._sleep_interruptibly(settle, goal_handle):
                        return await self.finish_canceled(goal_handle, mission)

                for view in point.views:
                    self._current_view_tag = view.tag
                    if not await self.wait_while_paused(goal_handle, mission, point, index):
                        return await self.finish_canceled(goal_handle, mission)
                    self.publish_status(MissionStatus.CAPTURING, mission.mission_id, point.id, index,
                                        len(mission.points), view.tag, goal_handle=goal_handle)
                    capture = await self.capture(view)
                    # Persist failed/canceled required views BEFORE returning the parent result.
                    try:
                        if capture.get('image_path'):
                            capture['image_path'] = self._writer.adopt_image(
                                capture['image_path'], point.id, view.tag)
                        elif capture['success'] and view.save_image:
                            raise FileNotFoundError('camera reported success without a saved image')
                    except (OSError, ValueError) as exc:
                        capture.update(success=False, error_code=1400,
                                       message=f'image archive failed: {exc}')
                    record = self.make_record(mission, point, view, capture)
                    record.update(capture_status=capture.get('state', 'failed'),
                                  camera_error_message=capture.get('message', ''),
                                  action_termination_confirmed=capture.get('termination_confirmed', True))
                    self._writer.append(record)
                    self._writer.event('capture_recorded', point_id=point.id, view_tag=view.tag,
                                       success=capture['success'], error_code=capture['error_code'],
                                       message=capture.get('message', ''))
                    if self._interrupted(goal_handle) or capture.get('state') == 'canceled':
                        return await self.finish_canceled(goal_handle, mission)
                    if self._fault_reason or (not capture['success'] and view.required):
                        return await self._finish_with_stop(
                            goal_handle, mission, 'failed', int(capture['error_code']),
                            f'view {view.tag} failed: {capture.get("message", "")}')
                self._completed_points += 1
            if self._interrupted(goal_handle):
                return await self.finish_canceled(goal_handle, mission)
            with self._admission_lock:
                canceled_at_commit = self._interrupted(goal_handle)
                if not canceled_at_commit:
                    self._terminal_committing = True
            if canceled_at_commit:
                return await self.finish_canceled(goal_handle, mission)
            return self._finish(goal_handle, mission, 'completed', 0, str(self._writer.directory))
        except Exception as exc:
            self.get_logger().error(f'mission failed: {exc!r}')
            await self._stop_active_operation()
            return await self._finish_with_stop(goal_handle, mission, 'failed', 900, str(exc))
        finally:
            self._active_operation = None
            self.active_nav_goal = self.active_camera_goal = None
            self.active_goal_handle = None
            self._writer = None
            self.paused = False
            with self._admission_lock:
                self._mission_reserved = False

    async def finish_canceled(self, goal_handle, mission):
        await self._stop_active_operation()
        if not goal_handle.is_cancel_requested:
            self._request_parent_cancel()
            deadline = time.monotonic() + self._cancel_timeout()
            while (not goal_handle.is_cancel_requested and self.context.ok()
                   and not self._shutdown_requested and time.monotonic() < deadline):
                await self._waiter.sleep(0.02)
        if not goal_handle.is_cancel_requested:
            return await self._finish_with_stop(
                goal_handle, mission, 'failed', 1302,
                'child cancellation requested, but parent Action cancel state was not confirmed')
        return await self._finish_with_stop(goal_handle, mission, 'canceled', 400, 'mission canceled')

    async def navigate(self, point):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = self.get_clock().now().to_msg(), point.frame_id
        pose.pose.position.x, pose.pose.position.y = point.x, point.y
        qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return await self._run_action(self.nav_client, goal, 'nav')

    async def capture(self, view):
        goal = AcquireView.Goal()
        goal.heading, goal.roll, goal.pitch = view.heading, view.roll, view.pitch
        goal.capture_name, goal.tag = view.tag, view.tag
        goal.tolerance, goal.timeout = view.tolerance, view.timeout
        goal.stable_samples = view.stable_samples
        goal.settle_time, goal.image_timeout = view.settle_time, view.image_timeout
        goal.save_image = view.save_image
        outcome = await self._run_action(self.camera_client, goal, 'camera')
        r = outcome.wrapped_result.result if outcome.wrapped_result is not None else None
        data = {
            'success': bool(outcome.success and r is not None and r.success),
            'state': outcome.state, 'termination_confirmed': outcome.termination_confirmed,
            'error_code': int(r.error_code) if r is not None else 202,
            'message': r.message if r is not None else outcome.message,
            'image_path': r.image_path if r is not None else '',
            'image_stamp': r.image_stamp if r is not None else None,
            'gimbal_heading': float(r.actual_heading) if r is not None else None,
            'gimbal_roll': float(r.actual_roll) if r is not None else None,
            'gimbal_pitch': float(r.actual_pitch) if r is not None else None,
        }
        if outcome.state == 'timeout':
            data.update(error_code=1201, message=outcome.message)
        elif outcome.state == 'canceled':
            data.update(error_code=400, message=outcome.message)
        elif not data['success'] and data['error_code'] == 0:
            data.update(error_code=202, message=outcome.message)
        if not outcome.termination_confirmed:
            data.update(success=False, error_code=1300,
                        message='Action termination unconfirmed; new missions are locked')
        stamp = data.get('image_stamp')
        if stamp is not None and (stamp.sec < 0 or stamp.nanosec >= 1_000_000_000
                                  or (stamp.sec == 0 and stamp.nanosec == 0)):
            data['image_stamp'] = None
            if data['success']:
                data.update(success=False, error_code=1400,
                            message='camera reported success with an invalid image timestamp')
        if not data['success'] and data['state'] == 'succeeded':
            data['state'] = 'failed'
        return data

    def _interrupted(self, goal_handle=None):
        return (self.cancel_requested or self._shutdown_requested or not self.context.ok()
                or (goal_handle is not None and goal_handle.is_cancel_requested))

    def _cancel_timeout(self):
        return float(self.get_parameter('cancel_confirm_timeout_sec').value)

    async def _sleep_interruptibly(self, seconds, goal_handle=None):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._interrupted(goal_handle):
                return False
            await self._waiter.sleep(min(0.05, max(0.001, deadline - time.monotonic())))
        return not self._interrupted(goal_handle)

    def _motion_is_stationary(self, now_ns):
        with self._odom_lock:
            odom = self.latest_local_odom
            freshness = float(self.get_parameter('odom_freshness_sec').value)
            if (odom is None or not self.latest_local_odom_rx_ns
                    or (now_ns - self.latest_local_odom_rx_ns) / 1e9 > freshness):
                return False, 'local odometry is stale/missing'
            if (self.latest_pose_linear_mps is None or self.latest_pose_angular_rps is None
                    or not self.latest_pose_motion_rx_ns
                    or (now_ns - self.latest_pose_motion_rx_ns) / 1e9 > freshness):
                return False, 'pose-derived motion is stale/missing'
            t = odom.twist.twist
            linear = math.sqrt(t.linear.x ** 2 + t.linear.y ** 2 + t.linear.z ** 2)
            angular = math.sqrt(t.angular.x ** 2 + t.angular.y ** 2 + t.angular.z ** 2)
            pose_linear, pose_angular = self.latest_pose_linear_mps, self.latest_pose_angular_rps
            good = (
                linear <= float(self.get_parameter('stationary_linear_threshold_mps').value)
                and angular <= float(self.get_parameter('stationary_angular_threshold_rps').value)
                and pose_linear <= float(self.get_parameter('stationary_pose_linear_threshold_mps').value)
                and pose_angular <= float(self.get_parameter('stationary_pose_angular_threshold_rps').value))
            return good, (f'stationary={good}; twist={linear:.3f}m/s,{angular:.3f}rad/s; '
                          f'pose_delta={pose_linear:.3f}m/s,{pose_angular:.3f}rad/s')

    def _request_parent_cancel(self):
        handle = self.active_goal_handle
        if handle is None or handle.is_cancel_requested or self._parent_cancel_future is not None:
            return
        try:
            request = CancelGoal.Request()
            request.goal_info.goal_id = handle.goal_id
            self._parent_cancel_future = self._parent_cancel_client.call_async(request)
        except Exception as exc:
            self.get_logger().error(f'parent cancel request failed: {exc!r}')

    async def _run_action(self, client, goal, kind):
        writer = self._writer
        def event(name, **values):
            if writer is not None:
                writer.event(name, **values)
        op = PendingAction(kind, event)
        self._active_operation = op
        try:
            outcome = await execute_action(
                client, goal, op, sleep=self._waiter.sleep,
                is_cancel_requested=lambda: self._interrupted(self.active_goal_handle),
                total_timeout=float(self.get_parameter(kind + '_action_timeout_sec').value),
                server_timeout=float(self.get_parameter(kind + '_server_timeout_sec').value),
                goal_response_timeout=float(self.get_parameter('goal_response_timeout_sec').value),
                cancel_timeout=self._cancel_timeout(),
                poll_interval=float(self.get_parameter('runtime_poll_interval_sec').value))
            if kind == 'nav' and op.send_requested:
                self._navigation_was_sent = True
            if not outcome.termination_confirmed:
                self._latch_fault(f'{kind}: {outcome.message}; termination not confirmed', op)
            if writer is not None:
                writer.event('action_finished', state=outcome.state,
                             message=outcome.message, **op.snapshot())
            return outcome
        finally:
            # Keep unresolved handles in the fault latch for late-acceptance cancellation.
            if kind == 'nav' and op.send_requested:
                self._navigation_was_sent = True
            if op.termination_confirmed():
                self._active_operation = None

    async def _stop_active_operation(self):
        op = self._active_operation
        if op is None:
            return True
        if op.label == 'nav' and op.send_requested:
            self._navigation_was_sent = True
        confirmed = await cancel_and_confirm(
            op, sleep=self._waiter.sleep, timeout=self._cancel_timeout(),
            poll_interval=float(self.get_parameter('runtime_poll_interval_sec').value))
        if not confirmed:
            self._latch_fault('active Action could not be confirmed terminal', op)
        else:
            self._active_operation = None
        return confirmed

    def _latch_fault(self, reason, op=None):
        with self._admission_lock:
            self._fault_reason = reason
            if op is not None and op not in self._fault_operations:
                self._fault_operations.append(op)
        self.get_logger().error('MISSION FAULT LATCHED: ' + reason)

    def on_reset_fault(self, _request, response):
        with self._admission_lock:
            if self._mission_reserved:
                response.success, response.message = False, 'mission is still executing'
                return response
            if not self._fault_reason:
                response.success, response.message = True, 'no latched fault'
                return response
            if any(not op.termination_confirmed() for op in self._fault_operations):
                response.success, response.message = False, 'child Action termination remains unconfirmed'
                return response
            now_ns = time.monotonic_ns()
            stopped, detail = self._motion_is_stationary(now_ns)
            held = (self._stationary_since_ns is not None and
                    (now_ns - self._stationary_since_ns) / 1e9 >=
                    float(self.get_parameter('stationary_hold_sec').value))
            if not stopped or not held:
                response.success, response.message = False, 'fresh stationary observation required: ' + detail
                return response
            self._fault_reason = ''
            self._fault_operations.clear()
        self.publish_status(MissionStatus.IDLE, detail='fault reset after terminal Action and stationary observation')
        response.success, response.message = True, 'software fault reset; operator safety checks remain required'
        return response

    async def _finish_with_stop(self, handle, mission, status, code, message):
        confirmed = not self._fault_reason
        base_stopped = None
        if confirmed and self._navigation_was_sent and self.context.ok() and not self._shutdown_requested:
            self.publish_status(MissionStatus.STABILIZING,
                mission.mission_id if mission else '', self._current_point_id,
                self._completed_points, len(mission.points) if mission else 0,
                'ending mission: confirm measured base stop after child Action termination',
                goal_handle=handle)
            base_stopped, stop_detail = await self.wait_until_stationary(ignore_cancel=True)
            if not base_stopped:
                self._latch_fault('base stop could not be observed: ' + stop_detail)
                code, message = 1301, 'BASE STOP UNCONFIRMED: ' + stop_detail
        if self._fault_reason:
            status = 'failed'
            if code != 1301:
                code, message = 1300, 'ACTION TERMINATION UNCONFIRMED: ' + self._fault_reason
        return self._finish(handle, mission, status, code, message,
                            child_actions_terminal=confirmed, base_stationary=base_stopped)

    def _finish(self, handle, mission, status, code, message, **details):
        with self._admission_lock:
            self._terminal_committing = True
        result = ExecuteInspectionMission.Result()
        result.mission_id = mission.mission_id if mission else ''
        result.completed_points = self._completed_points
        result.success, result.error_code, result.message = status == 'completed', int(code), str(message)
        details.update(point_id=self._current_point_id, view_tag=self._current_view_tag,
                       fault_latched=bool(self._fault_reason))
        if self._writer is not None:
            try:
                self._writer.finalize(status, completed_points=self._completed_points,
                                      error_code=code, message=message, **details)
                if self._writer.last_event_error:
                    self.get_logger().warning('terminal manifest saved, event append failed: ' +
                                              self._writer.last_event_error)
            except Exception as exc:
                status, code, message = 'failed', 1401, f'terminal record could not be saved: {exc}'
                result.success, result.error_code, result.message = False, code, message
                self.get_logger().error(message)
        state = {'completed': MissionStatus.COMPLETED,
                 'canceled': MissionStatus.CANCELED}.get(status, MissionStatus.ERROR)
        self.publish_status(state, result.mission_id, self._current_point_id,
                            self._completed_points, len(mission.points) if mission else 0,
                            str(message), int(code))
        if handle.is_active:
            if status == 'completed':
                handle.succeed()
            elif status == 'canceled' and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        return result

    def prepare_shutdown(self):
        self._shutdown_requested = True
        self.cancel_requested = True
        self.cancel_active_subgoal()
        for op in self._fault_operations:
            op.request_cancel()
        if self._writer is not None:
            try:
                self._writer.finalize('interrupted', completed_points=self._completed_points,
                    error_code=1402, message='runtime shutdown; physical/Action stop must be verified',
                    fault_latched=bool(self._fault_reason))
            except Exception as exc:
                self.get_logger().error(f'shutdown record failed: {exc!r}')
        self._waiter.close()

    def nearest_rtk(self, stamp_ns):
        if not self.rtk_samples:
            return None, None
        sample_ns, msg = min(self.rtk_samples, key=lambda s: abs(s[0] - stamp_ns))
        age_sec = abs(sample_ns - stamp_ns) / 1e9
        if age_sec > float(self.get_parameter('rtk_max_age_sec').value):
            return None, age_sec
        return msg, age_sec

    def pose_at(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=float(self.get_parameter('tf_lookup_timeout_sec').value)))
            return tf.transform
        except TransformException as exc:
            self.get_logger().warning(f'pose lookup failed at image time: {exc}')
            return None

    def make_record(self, mission, point, view, capture):
        stamp = capture.get('image_stamp')
        stamp_ns = Time.from_msg(stamp).nanoseconds if stamp is not None else self.get_clock().now().nanoseconds
        pose = self.pose_at(stamp) if stamp is not None else None
        rtk, rtk_age = self.nearest_rtk(stamp_ns)
        return {
            'mission_id': mission.mission_id, 'map_id': mission.map_id, 'point_id': point.id,
            'view_tag': view.tag, 'image_path': capture.get('image_path', ''),
            'image_sec': int(stamp.sec) if stamp else 0, 'image_nanosec': int(stamp.nanosec) if stamp else 0,
            'pose_valid': pose is not None,
            'x': pose.translation.x if pose else None, 'y': pose.translation.y if pose else None,
            'z': pose.translation.z if pose else None,
            'qx': pose.rotation.x if pose else None, 'qy': pose.rotation.y if pose else None,
            'qz': pose.rotation.z if pose else None, 'qw': pose.rotation.w if pose else None,
            'rtk_valid': rtk is not None and int(rtk.status.status) >= 0,
            'rtk_age_sec': rtk_age,
            'latitude': rtk.latitude if rtk else None, 'longitude': rtk.longitude if rtk else None,
            'altitude': rtk.altitude if rtk else None, 'navsat_status': int(rtk.status.status) if rtk else -99,
            'gimbal_heading': capture.get('gimbal_heading'), 'gimbal_roll': capture.get('gimbal_roll'),
            'gimbal_pitch': capture.get('gimbal_pitch'), 'camera_error_code': int(capture.get('error_code', 0)),
        }


def main(args=None):
    rclpy.init(args=args)
    node = MissionRuntime()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.prepare_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
