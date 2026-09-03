from __future__ import annotations

import asyncio
import json
import math
from collections import deque

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from agt_robot_interfaces.action import ExecuteInspectionMission
from agt_robot_interfaces.msg import MissionStatus
from camera_gimbal_interfaces.action import AcquireView

from .mission_schema import load_mission
from .record_writer import RecordWriter


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MissionRuntime(Node):
    def __init__(self):
        super().__init__('mission_runtime')
        defaults = {
            'global_frame': 'map', 'base_frame': 'base_link',
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
            'stationary_hold_sec': 0.8,
            'stationary_timeout_sec': 8.0,
            'odom_freshness_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.record_root = self.get_parameter('record_root').value
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rtk_samples = deque(maxlen=300)
        self.latest_local_odom = None
        self.latest_local_odom_rx_ns = 0
        self.pending_hmi_mission = ''
        self.paused = False
        self.cancel_requested = False
        self.active_goal_handle = None
        self.active_nav_goal = None
        self.active_camera_goal = None

        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value)
        self.camera_client = ActionClient(self, AcquireView, self.get_parameter('acquire_view_action').value)
        self.loopback_client = ActionClient(self, ExecuteInspectionMission, '/agt/mission/execute')
        self.server = ActionServer(
            self, ExecuteInspectionMission, '/agt/mission/execute',
            execute_callback=self.execute_mission,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.status_pub = self.create_publisher(MissionStatus, '/agt/mission/status', 10)
        self.hmi_status_pub = self.create_publisher(String, self.get_parameter('hmi_task_status_topic').value, 10)
        self.create_subscription(String, self.get_parameter('hmi_task_request_topic').value, self.on_hmi_task_request, 10)
        self.create_subscription(NavSatFix, self.get_parameter('navsat_topic').value, self.on_navsat, 20)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self.on_local_odom, 50)
        self.create_service(Trigger, '/agt/task/start', self.on_hmi_start)
        self.create_service(Trigger, '/agt/task/pause', self.on_hmi_pause)
        self.create_service(Trigger, '/agt/task/cancel', self.on_hmi_cancel)

    def goal_callback(self, _goal):
        return GoalResponse.REJECT if self.active_goal_handle is not None else GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.cancel_requested = True
        self.cancel_active_subgoal()
        return CancelResponse.ACCEPT

    def cancel_active_subgoal(self):
        for handle in (self.active_nav_goal, self.active_camera_goal):
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f'failed to forward cancel: {exc}')

    def on_hmi_task_request(self, msg):
        self.pending_hmi_mission = msg.data.strip()
        self.publish_text_status('TASK_LOADED', self.pending_hmi_mission)

    def on_hmi_start(self, _request, response):
        if not self.pending_hmi_mission:
            response.success, response.message = False, 'No task file has been handed off'
            return response
        if self.active_goal_handle is not None:
            response.success, response.message = False, 'A mission is already active'
            return response
        if not self.loopback_client.wait_for_server(timeout_sec=0.2):
            response.success, response.message = False, 'Mission action server unavailable'
            return response
        goal = ExecuteInspectionMission.Goal()
        goal.mission_file = self.pending_hmi_mission
        goal.resume = False
        self.loopback_client.send_goal_async(goal)
        response.success, response.message = True, 'mission accepted for execution'
        return response

    def on_hmi_pause(self, _request, response):
        self.paused = not self.paused
        response.success = True
        response.message = 'paused' if self.paused else 'resumed'
        return response

    def on_hmi_cancel(self, _request, response):
        self.cancel_requested = True
        self.cancel_active_subgoal()
        response.success, response.message = True, 'cancel requested'
        return response

    def on_navsat(self, msg):
        self.rtk_samples.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))

    def on_local_odom(self, msg):
        self.latest_local_odom = msg
        self.latest_local_odom_rx_ns = self.get_clock().now().nanoseconds

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
            'error_code': error_code}, ensure_ascii=False)
        self.hmi_status_pub.publish(text)
        if goal_handle is not None:
            feedback = ExecuteInspectionMission.Feedback()
            feedback.status = msg
            goal_handle.publish_feedback(feedback)

    def publish_text_status(self, state, detail=''):
        msg = String()
        msg.data = json.dumps({'state': state, 'detail': detail}, ensure_ascii=False)
        self.hmi_status_pub.publish(msg)

    async def wait_while_paused(self, goal_handle, mission, point, index):
        while self.paused and rclpy.ok() and not self.cancel_requested:
            self.publish_status(MissionStatus.PAUSED, mission.mission_id, point.id, index,
                                len(mission.points), 'paused by operator', goal_handle=goal_handle)
            await asyncio.sleep(0.2)

    async def wait_until_stationary(self):
        linear_limit = float(self.get_parameter('stationary_linear_threshold_mps').value)
        angular_limit = float(self.get_parameter('stationary_angular_threshold_rps').value)
        hold_sec = float(self.get_parameter('stationary_hold_sec').value)
        timeout_sec = float(self.get_parameter('stationary_timeout_sec').value)
        freshness_sec = float(self.get_parameter('odom_freshness_sec').value)
        start_ns = self.get_clock().now().nanoseconds
        stable_since_ns = None

        while rclpy.ok() and not self.cancel_requested:
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - start_ns) / 1e9 > timeout_sec:
                return False, 'base did not become stationary before timeout'

            odom = self.latest_local_odom
            rx_age = (now_ns - self.latest_local_odom_rx_ns) / 1e9 if self.latest_local_odom_rx_ns else float('inf')
            if odom is None or rx_age > freshness_sec:
                stable_since_ns = None
                await asyncio.sleep(0.05)
                continue

            t = odom.twist.twist
            linear = math.sqrt(t.linear.x * t.linear.x + t.linear.y * t.linear.y + t.linear.z * t.linear.z)
            angular = math.sqrt(t.angular.x * t.angular.x + t.angular.y * t.angular.y + t.angular.z * t.angular.z)
            if linear <= linear_limit and angular <= angular_limit:
                if stable_since_ns is None:
                    stable_since_ns = now_ns
                elif (now_ns - stable_since_ns) / 1e9 >= hold_sec:
                    return True, f'stationary linear={linear:.3f}m/s angular={angular:.3f}rad/s'
            else:
                stable_since_ns = None
            await asyncio.sleep(0.05)
        return False, 'stationary check canceled'

    async def execute_mission(self, goal_handle):
        self.active_goal_handle = goal_handle
        self.cancel_requested = False
        result = ExecuteInspectionMission.Result()
        completed = 0
        try:
            mission = load_mission(goal_handle.request.mission_file)
            writer = RecordWriter(self.record_root, mission)
            self.pending_hmi_mission = mission.source_file
            for index, point in enumerate(mission.points):
                await self.wait_while_paused(goal_handle, mission, point, index)
                if self.cancel_requested or goal_handle.is_cancel_requested:
                    return self.finish_canceled(goal_handle, result, mission.mission_id, completed)

                self.publish_status(MissionStatus.NAVIGATING, mission.mission_id, point.id, index,
                                    len(mission.points), 'sending Nav2 goal', goal_handle=goal_handle)
                nav_ok, nav_error = await self.navigate(point)
                if self.cancel_requested or goal_handle.is_cancel_requested:
                    return self.finish_canceled(goal_handle, result, mission.mission_id, completed)
                if not nav_ok:
                    goal_handle.abort()
                    result.success, result.mission_id = False, mission.mission_id
                    result.completed_points, result.error_code, result.message = completed, 1000, nav_error
                    return result

                self.publish_status(MissionStatus.STABILIZING, mission.mission_id, point.id, index,
                                    len(mission.points), 'waiting for measured base stop', goal_handle=goal_handle)
                stopped, stop_detail = await self.wait_until_stationary()
                if not stopped:
                    goal_handle.abort()
                    result.success, result.mission_id = False, mission.mission_id
                    result.completed_points, result.error_code, result.message = completed, 1100, stop_detail
                    return result

                settle = point.settle_time if point.settle_time >= 0 else float(self.get_parameter('default_point_settle_sec').value)
                if settle > 0:
                    self.publish_status(MissionStatus.STABILIZING, mission.mission_id, point.id, index,
                                        len(mission.points), f'{stop_detail}; extra settle {settle:.2f}s', goal_handle=goal_handle)
                    await asyncio.sleep(settle)

                for view in point.views:
                    await self.wait_while_paused(goal_handle, mission, point, index)
                    if self.cancel_requested or goal_handle.is_cancel_requested:
                        return self.finish_canceled(goal_handle, result, mission.mission_id, completed)
                    self.publish_status(MissionStatus.CAPTURING, mission.mission_id, point.id, index,
                                        len(mission.points), view.tag, goal_handle=goal_handle)
                    capture = await self.capture(view)
                    if not capture['success'] and view.required:
                        goal_handle.abort()
                        result.success, result.mission_id = False, mission.mission_id
                        result.completed_points = completed
                        result.error_code = int(capture['error_code'])
                        result.message = f'required view {view.tag} failed: {capture["message"]}'
                        return result
                    capture['image_path'] = writer.adopt_image(capture.get('image_path', ''), point.id, view.tag)
                    writer.append(self.make_record(mission, point, view, capture))
                completed += 1

            goal_handle.succeed()
            result.success, result.mission_id = True, mission.mission_id
            result.completed_points, result.error_code, result.message = completed, 0, str(writer.directory)
            self.publish_status(MissionStatus.COMPLETED, mission.mission_id, '', completed,
                                len(mission.points), str(writer.directory), goal_handle=goal_handle)
            return result
        except Exception as exc:
            self.get_logger().error(f'mission failed: {exc!r}')
            if goal_handle.is_active:
                goal_handle.abort()
            result.success, result.completed_points, result.error_code, result.message = False, completed, 900, str(exc)
            return result
        finally:
            self.active_nav_goal = None
            self.active_camera_goal = None
            self.active_goal_handle = None

    def finish_canceled(self, goal_handle, result, mission_id, completed):
        goal_handle.canceled()
        result.success, result.mission_id = False, mission_id
        result.completed_points, result.error_code, result.message = completed, 400, 'mission canceled'
        return result

    async def navigate(self, point):
        timeout = float(self.get_parameter('nav_server_timeout_sec').value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            return False, 'Nav2 NavigateToPose action unavailable'
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = self.get_clock().now().to_msg(), point.frame_id
        pose.pose.position.x, pose.pose.position.y = point.x, point.y
        qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.active_nav_goal = await self.nav_client.send_goal_async(goal)
        if not self.active_nav_goal.accepted:
            self.active_nav_goal = None
            return False, 'Nav2 goal rejected'
        wrapped = await self.active_nav_goal.get_result_async()
        self.active_nav_goal = None
        return wrapped.status == 4, f'Nav2 finished with status={wrapped.status}'

    async def capture(self, view):
        timeout = float(self.get_parameter('camera_server_timeout_sec').value)
        if not self.camera_client.wait_for_server(timeout_sec=timeout):
            return {'success': False, 'error_code': 200, 'message': 'AcquireView unavailable'}
        goal = AcquireView.Goal()
        goal.heading, goal.roll, goal.pitch = view.heading, view.roll, view.pitch
        goal.capture_name, goal.tag = view.tag, view.tag
        goal.tolerance, goal.timeout = view.tolerance, view.timeout
        goal.stable_samples = view.stable_samples
        goal.settle_time, goal.image_timeout = view.settle_time, view.image_timeout
        goal.save_image = view.save_image
        self.active_camera_goal = await self.camera_client.send_goal_async(goal)
        if not self.active_camera_goal.accepted:
            self.active_camera_goal = None
            return {'success': False, 'error_code': 201, 'message': 'AcquireView rejected'}
        wrapped = await self.active_camera_goal.get_result_async()
        self.active_camera_goal = None
        r = wrapped.result
        return {
            'success': bool(r.success), 'error_code': int(r.error_code), 'message': r.message,
            'image_path': r.image_path, 'image_stamp': r.image_stamp,
            'gimbal_heading': float(r.actual_heading), 'gimbal_roll': float(r.actual_roll),
            'gimbal_pitch': float(r.actual_pitch),
        }

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
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
