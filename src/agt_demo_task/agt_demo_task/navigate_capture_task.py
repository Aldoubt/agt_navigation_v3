"""Observe RViz NavigateToPose completion and capture the current C1 frame."""

from __future__ import annotations

from collections import deque

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from agt_robot_interfaces.srv import CaptureCamera


RUNNING_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def goal_key(status: GoalStatus) -> str:
    return bytes(status.goal_info.goal_id.uuid).hex()


def status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'NAVIGATING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return names.get(status, 'UNKNOWN')


class NavigateCaptureTask(Node):
    """Bridge action status from Nav2 to the stable AGT camera service."""

    def __init__(self):
        super().__init__('navigate_capture_task')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('capture_service', '/capability/camera/capture')
        self.declare_parameter('status_topic', '/agt/demo_task/status')
        self.declare_parameter('status_marker_topic', '/agt/demo_task/status_marker')
        self.declare_parameter('base_frame', 'base_link')

        action_name = str(self.get_parameter('navigate_action').value)
        self._observed_goals: set[str] = set()
        self._completed_goals: set[str] = set()
        self._last_state: dict[str, int] = {}
        self._capture_queue: deque[str] = deque()
        self._capture_in_flight: str | None = None
        self._capture_client = self.create_client(
            CaptureCamera, str(self.get_parameter('capture_service').value))

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), latched_qos)
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('status_marker_topic').value), latched_qos)
        self.create_subscription(
            GoalStatusArray, f'{action_name}/_action/status', self._on_status, 10)
        self.create_timer(0.25, self._drain_capture_queue)
        self._publish_status('READY: set a Nav2 goal in RViz')

    def _publish_status(self, text: str, *, success: bool | None = None) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)

        marker = Marker()
        marker.header.frame_id = str(self.get_parameter('base_frame').value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'agt_demo_task_status'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 1.5
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if success is True:
            marker.color.g = 1.0
        elif success is False:
            marker.color.r = 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.0
        marker.text = text
        self._marker_pub.publish(marker)
        self.get_logger().info(text)

    def _on_status(self, status_array: GoalStatusArray) -> None:
        for status in status_array.status_list:
            key = goal_key(status)
            state = int(status.status)
            if state in RUNNING_STATES:
                self._observed_goals.add(key)
            if self._last_state.get(key) != state:
                self._last_state[key] = state
                if key in self._observed_goals:
                    self._publish_status(f'NAV2 {status_name(state)}: {key[:8]}')

            if state == GoalStatus.STATUS_SUCCEEDED and key in self._observed_goals:
                if key not in self._completed_goals:
                    self._completed_goals.add(key)
                    self._capture_queue.append(key)
                    self._publish_status(f'NAV2 SUCCEEDED: {key[:8]}; camera capture queued')
            elif state in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                if key in self._observed_goals:
                    self._completed_goals.add(key)
                    self._publish_status(f'NAV2 {status_name(state)}: {key[:8]}; no capture', success=False)

    def _drain_capture_queue(self) -> None:
        if self._capture_in_flight is not None or not self._capture_queue:
            return
        if not self._capture_client.service_is_ready():
            self._publish_status('NAV2 complete; waiting for camera capture service', success=False)
            return

        key = self._capture_queue.popleft()
        self._capture_in_flight = key
        self._publish_status(f'CAPTURING: {key[:8]}')
        future = self._capture_client.call_async(CaptureCamera.Request())
        future.add_done_callback(lambda result, goal_key=key: self._on_capture_result(goal_key, result))

    def _on_capture_result(self, key: str, future) -> None:
        self._capture_in_flight = None
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status(f'CAPTURE FAILED: {key[:8]}: {exc}', success=False)
            return
        if response.success:
            self._publish_status(f'CAPTURED: {response.image_path}', success=True)
        else:
            self._publish_status(f'CAPTURE FAILED: {response.message}', success=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateCaptureTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
