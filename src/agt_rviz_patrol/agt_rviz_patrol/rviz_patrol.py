from __future__ import annotations

import math
from pathlib import Path
from datetime import datetime

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

from agt_robot_interfaces.action import ExecuteInspectionMission


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RvizPatrol(Node):
    def __init__(self):
        super().__init__('rviz_patrol')
        defaults = {
            'goal_topic': '/goal_pose',
            'global_frame': 'map',
            'base_frame': 'base_link',
            'mission_action': '/agt/mission/execute',
            'map_id': 'demo_map',
            'preset_file': '',
            'mission_dir': '~/.ros/agt_rviz_patrol',
            'home_settle_time': 0.5,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.queue = []
        self.active_goal = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, ExecuteInspectionMission, self.get_parameter('mission_action').value)
        self.marker_pub = self.create_publisher(MarkerArray, '/agt/rviz_patrol/markers', 10)
        self.status_pub = self.create_publisher(String, '/agt/rviz_patrol/status', 10)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self.on_goal, 10)
        self.create_service(Trigger, '/agt/rviz_patrol/clear', self.on_clear)
        self.create_service(Trigger, '/agt/rviz_patrol/start', self.on_start)
        self.create_service(Trigger, '/agt/rviz_patrol/cancel', self.on_cancel)
        self.publish_status('READY')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().error(
                f'RViz goal frame must be {self.global_frame!r} in demo V1, got {msg.header.frame_id!r}')
            return
        self.queue.append(msg)
        self.publish_markers()
        self.publish_status(f'QUEUED {len(self.queue)} waypoint(s)')

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, pose in enumerate(self.queue):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'agt_rviz_patrol'
            marker.id = i + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x, marker.scale.y, marker.scale.z = 0.8, 0.12, 0.12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 0.2, 0.9
            arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def on_clear(self, _req, response):
        if self.active_goal is not None:
            response.success, response.message = False, 'mission active; cancel first'
            return response
        self.queue.clear()
        self.publish_markers()
        self.publish_status('QUEUE CLEARED')
        response.success, response.message = True, 'waypoint queue cleared'
        return response

    def on_cancel(self, _req, response):
        if self.active_goal is None:
            response.success, response.message = True, 'no active mission'
            return response
        self.active_goal.cancel_goal_async()
        response.success, response.message = True, 'cancel requested'
        self.publish_status('CANCEL REQUESTED')
        return response

    def current_home(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(), timeout=Duration(seconds=0.5))
        except TransformException as exc:
            raise RuntimeError(f'cannot capture home pose: {exc}') from exc
        t = tf.transform.translation
        return {
            'x': float(t.x), 'y': float(t.y),
            'yaw': yaw_from_quaternion(tf.transform.rotation),
            'frame_id': self.global_frame,
        }

    def load_views(self):
        preset = str(self.get_parameter('preset_file').value).strip()
        if not preset:
            raise RuntimeError('preset_file parameter is required')
        data = yaml.safe_load(Path(preset).expanduser().read_text(encoding='utf-8')) or {}
        views = data.get('views') or []
        if len(views) != 3:
            raise RuntimeError('demo V1 camera preset must contain exactly 3 views')
        return data

    def build_mission_file(self):
        home = self.current_home()
        preset = self.load_views()
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        mission_id = f'rviz_patrol_{stamp}'
        points = []
        for i, pose in enumerate(self.queue, start=1):
            points.append({
                'id': f'P{i:03d}',
                'pose': {
                    'x': float(pose.pose.position.x),
                    'y': float(pose.pose.position.y),
                    'yaw': yaw_from_quaternion(pose.pose.orientation),
                    'frame_id': self.global_frame,
                },
                'settle_time': float(preset.get('base_settle_time', 1.5)),
                'views': preset['views'],
            })
        points.append({
            'id': 'RETURN_HOME',
            'pose': home,
            'settle_time': float(self.get_parameter('home_settle_time').value),
            'views': [],
        })
        mission = {
            'version': 1,
            'mission_id': mission_id,
            'map_id': str(self.get_parameter('map_id').value),
            'points': points,
        }
        root = Path(str(self.get_parameter('mission_dir').value)).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        path = root / f'{mission_id}.yaml'
        path.write_text(yaml.safe_dump(mission, sort_keys=False, allow_unicode=True), encoding='utf-8')
        return path

    def on_start(self, _req, response):
        if self.active_goal is not None:
            response.success, response.message = False, 'mission already active'
            return response
        if not self.queue:
            response.success, response.message = False, 'no RViz waypoints queued'
            return response
        if not self.client.wait_for_server(timeout_sec=0.5):
            response.success, response.message = False, 'mission runtime action unavailable'
            return response
        try:
            mission_file = self.build_mission_file()
        except Exception as exc:
            response.success, response.message = False, str(exc)
            return response
        goal = ExecuteInspectionMission.Goal()
        goal.mission_file = str(mission_file)
        goal.resume = False
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        response.success, response.message = True, f'mission submitted: {mission_file}'
        self.publish_status(f'SUBMITTED {mission_file}')
        return response

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_status('MISSION REJECTED')
            return
        self.active_goal = handle
        self.queue.clear()
        self.publish_markers()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)
        self.publish_status('MISSION RUNNING')

    def on_result(self, future):
        wrapped = future.result()
        result = wrapped.result
        self.publish_status(
            f'MISSION DONE success={bool(result.success)} completed={int(result.completed_points)} '
            f'message={result.message}')
        self.active_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = RvizPatrol()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
