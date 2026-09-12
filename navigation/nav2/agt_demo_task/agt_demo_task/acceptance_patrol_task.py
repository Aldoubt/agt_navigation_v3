"""Acceptance mission: navigate -> measured stop -> capture -> next -> return home."""

from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from agt_robot_interfaces.srv import CaptureCamera


def _quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AcceptancePatrolTask(Node):
    def __init__(self):
        super().__init__('acceptance_patrol_task')
        p = self.declare_parameter
        p('waypoint_file', '')
        p('navigate_action', '/navigate_to_pose')
        p('capture_service', '/capability/camera/capture')
        p('odom_topic', '/agt/odometry/local')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('status_topic', '/agt/acceptance/status')
        p('start_service', '/agt/acceptance/start')
        p('cancel_service', '/agt/acceptance/cancel')
        p('output_dir', '~/.ros/agt_acceptance')
        p('linear_stop_threshold_mps', 0.03)
        p('angular_stop_threshold_radps', 0.03)
        p('stop_hold_sec', 1.0)
        p('capture_required', True)
        p('tf_timeout_sec', 0.30)

        self._action = ActionClient(
            self, NavigateToPose, str(self.get_parameter('navigate_action').value))
        self._capture = self.create_client(
            CaptureCamera, str(self.get_parameter('capture_service').value))
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_service(
            Trigger, str(self.get_parameter('start_service').value), self._on_start)
        self.create_service(
            Trigger, str(self.get_parameter('cancel_service').value), self._on_cancel)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 50)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._waypoints, self._configured_home = self._load_waypoints()
        self._latest_odom = None
        self._state = 'IDLE'
        self._index = 0
        self._active_goal = None
        self._home_pose = None
        self._stop_since_ns = None
        self._log_path = None
        self.create_timer(0.1, self._tick)
        self._publish('IDLE', f'loaded {len(self._waypoints)} acceptance waypoints')

    def _load_waypoints(self):
        path = Path(os.path.expanduser(str(self.get_parameter('waypoint_file').value)))
        if not path.is_file():
            raise RuntimeError(f'acceptance waypoint_file does not exist: {path}')
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        return list(data.get('waypoints') or []), data.get('home')

    def _publish(self, state: str, detail: str, **extra):
        self._state = state
        payload = {'state': state, 'detail': detail, 'waypoint_index': self._index}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)
        self.get_logger().info(msg.data)
        self._log(payload)

    def _log(self, payload):
        if self._log_path is None:
            return
        row = dict(payload)
        row['time'] = self.get_clock().now().nanoseconds / 1e9
        with self._log_path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')

    def _new_log(self):
        root = Path(os.path.expanduser(str(self.get_parameter('output_dir').value)))
        root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = root / f'acceptance_mission_{stamp}.jsonl'

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _pose_from_dict(self, item) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = str(item.get('frame_id', self.get_parameter('map_frame').value))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(item['x'])
        msg.pose.position.y = float(item['y'])
        msg.pose.position.z = float(item.get('z', 0.0))
        qx, qy, qz, qw = _quat_from_yaw(float(item.get('yaw_rad', 0.0)))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _current_home_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
            pose = PoseStamped()
            pose.header.frame_id = str(self.get_parameter('map_frame').value)
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose
        except TransformException as exc:
            if self._configured_home:
                self.get_logger().warn(
                    f'could not capture live home pose ({exc}); using configured home')
                return self._pose_from_dict(self._configured_home)
            raise RuntimeError(f'cannot resolve mission home pose: {exc}') from exc

    def _on_start(self, _request, response):
        if self._state not in ('IDLE', 'COMPLETE', 'FAILED', 'CANCELED'):
            response.success = False
            response.message = f'mission already active: {self._state}'
            return response
        if not self._waypoints:
            response.success = False
            response.message = 'no acceptance waypoints configured'
            return response
        if not self._action.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = 'NavigateToPose action unavailable'
            return response
        try:
            self._home_pose = self._current_home_pose()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response

        self._new_log()
        self._index = 0
        self._active_goal = None
        self._stop_since_ns = None
        self._publish('STARTING', 'acceptance mission started', log=str(self._log_path))
        self._send_current_waypoint()
        response.success = True
        response.message = f'acceptance mission started; log={self._log_path}'
        return response

    def _on_cancel(self, _request, response):
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
        self._publish('CANCELED', 'operator canceled acceptance mission')
        response.success = True
        response.message = 'cancel requested'
        return response

    def _send_pose(self, pose: PoseStamped, label: str):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self._action.send_goal_async(goal)
        future.add_done_callback(lambda f, target=label: self._on_goal_response(f, target))
        self._publish(
            'NAVIGATING',
            f'navigating to {label}',
            target=label,
            goal_x=float(pose.pose.position.x),
            goal_y=float(pose.pose.position.y),
            goal_yaw_rad=_yaw_from_quat(pose.pose.orientation),
        )

    def _send_current_waypoint(self):
        if self._index >= len(self._waypoints):
            self._send_pose(self._home_pose, 'RETURN_HOME')
            return
        item = self._waypoints[self._index]
        self._send_pose(self._pose_from_dict(item), str(item.get('id', f'wp_{self._index}')))

    def _on_goal_response(self, future, label):
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(f'goal send failed for {label}: {exc}')
            return
        if not handle.accepted:
            self._fail(f'goal rejected for {label}')
            return
        self._active_goal = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, target=label: self._on_goal_result(f, target))

    def _on_goal_result(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._fail(f'goal result failed for {label}: {exc}')
            return
        self._active_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Nav2 {label} finished with status={result.status}')
            return
        if label == 'RETURN_HOME':
            self._publish('COMPLETE', 'all waypoints captured and robot returned home')
            return

        self._stop_since_ns = None
        self._publish('WAIT_STOP', f'Nav2 reached {label}; waiting for measured stop', target=label)

    def _tick(self):
        if self._state != 'WAIT_STOP' or self._latest_odom is None:
            return
        twist = self._latest_odom.twist.twist
        linear = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        angular = math.sqrt(twist.angular.x**2 + twist.angular.y**2 + twist.angular.z**2)
        if (linear <= float(self.get_parameter('linear_stop_threshold_mps').value)
                and angular <= float(self.get_parameter('angular_stop_threshold_radps').value)):
            now = self.get_clock().now().nanoseconds
            if self._stop_since_ns is None:
                self._stop_since_ns = now
                return
            held = (now - self._stop_since_ns) / 1e9
            if held >= float(self.get_parameter('stop_hold_sec').value):
                self._stop_since_ns = None
                self._on_measured_stop(linear, angular)
        else:
            self._stop_since_ns = None

    def _on_measured_stop(self, linear, angular):
        item = self._waypoints[self._index]
        label = str(item.get('id', f'wp_{self._index}'))
        final_pose = {}
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
            final_pose = {
                'final_x': float(tf.transform.translation.x),
                'final_y': float(tf.transform.translation.y),
                'final_z': float(tf.transform.translation.z),
                'final_yaw_rad': _yaw_from_quat(tf.transform.rotation),
            }
        except TransformException as exc:
            final_pose = {'final_pose_error': str(exc)}
        self._publish(
            'STOPPED',
            f'measured stop confirmed at {label}',
            target=label,
            linear_speed_mps=linear,
            angular_speed_radps=angular,
            **final_pose,
        )
        if not bool(item.get('capture', True)):
            self._advance()
            return
        if not self._capture.service_is_ready():
            if bool(self.get_parameter('capture_required').value):
                self._fail('camera capture service unavailable')
            else:
                self._advance()
            return

        self._publish('CAPTURING', f'capturing at {label}', target=label)
        future = self._capture.call_async(CaptureCamera.Request())
        future.add_done_callback(lambda f, target=label: self._on_capture(f, target))

    def _on_capture(self, future, label):
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f'camera capture failed at {label}: {exc}')
            return
        if not response.success:
            self._fail(f'camera capture rejected at {label}: {response.message}')
            return
        self._publish(
            'CAPTURED',
            f'capture complete at {label}',
            target=label,
            image_path=response.image_path,
        )
        self._advance()

    def _advance(self):
        self._index += 1
        self._send_current_waypoint()

    def _fail(self, detail):
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
            self._active_goal = None
        self._publish('FAILED', detail)


def main(args=None):
    rclpy.init(args=args)
    node = AcceptancePatrolTask()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
