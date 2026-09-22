"""RViz route drawing plus live LIO and wheel-odometry trail visualization."""

from __future__ import annotations

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def interpolate_polyline(points, spacing: float):
    """Return ``(x, y, tangent_yaw)`` samples including every endpoint."""
    if len(points) < 2:
        return []
    spacing = max(float(spacing), 0.01)
    result = []
    for segment_index, (start, end) in enumerate(zip(points, points[1:])):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        if length < 1.0e-6:
            continue
        yaw = math.atan2(dy, dx)
        steps = max(1, int(math.ceil(length / spacing)))
        first = 0 if segment_index == 0 or not result else 1
        for step in range(first, steps + 1):
            ratio = step / steps
            result.append((
                float(start[0]) + ratio * dx,
                float(start[1]) + ratio * dy,
                yaw,
            ))
    return result


def align_relative_pose(wheel_pose, wheel_origin, map_origin):
    """Rigidly align a wheel-odom pose to the initial map-frame robot pose."""
    wx, wy, wyaw = wheel_pose
    w0x, w0y, w0yaw = wheel_origin
    mx, my, myaw = map_origin
    heading_offset = myaw - w0yaw
    cos_h = math.cos(heading_offset)
    sin_h = math.sin(heading_offset)
    dx = wx - w0x
    dy = wy - w0y
    return (
        mx + cos_h * dx - sin_h * dy,
        my + sin_h * dx + cos_h * dy,
        wyaw + heading_offset,
    )


class RvizPathTool(Node):
    def __init__(self):
        super().__init__('agt_rviz_path_tool')
        defaults = {
            'global_frame': 'map',
            'base_frame': 'base_link',
            'click_topic': '/clicked_point',
            'lio_odom_topic': '/agt/odometry/local',
            'wheel_odom_topic': '/wheel/odom',
            'follow_path_action': '/follow_path',
            'path_validation_service': '/is_path_valid',
            'controller_id': 'FollowPath',
            'goal_checker_id': 'general_goal_checker',
            # Match the 5 cm costmap resolution so footprint validation does
            # not jump across unchecked cells between route poses.
            'route_spacing': 0.05,
            'trail_sample_distance': 0.05,
            'max_trail_points': 20000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.route_points = []
        self.lio_points = []
        self.wheel_points = []
        self.wheel_origin = None
        self.map_origin = None
        self.active_goal = None
        self.start_pending = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.route_pub = self.create_publisher(Path, '/agt/path_tool/route', latched)
        self.lio_pub = self.create_publisher(Path, '/agt/path_tool/lio_path', latched)
        self.wheel_pub = self.create_publisher(Path, '/agt/path_tool/wheel_path', latched)
        self.status_pub = self.create_publisher(String, '/agt/path_tool/status', latched)

        self.create_subscription(
            PointStamped, str(self.get_parameter('click_topic').value), self.on_click, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('lio_odom_topic').value), self.on_lio, 20)
        self.create_subscription(
            Odometry, str(self.get_parameter('wheel_odom_topic').value), self.on_wheel, 50)
        self.create_service(Trigger, '/agt/path_tool/start', self.on_start)
        self.create_service(Trigger, '/agt/path_tool/clear_route', self.on_clear_route)
        self.create_service(Trigger, '/agt/path_tool/clear_trails', self.on_clear_trails)
        self.create_service(Trigger, '/agt/path_tool/cancel', self.on_cancel)

        self.follow_client = ActionClient(
            self, FollowPath, str(self.get_parameter('follow_path_action').value))
        self.validation_client = self.create_client(
            IsPathValid, str(self.get_parameter('path_validation_service').value))
        self.publish_status('READY: use RViz Publish Point to add route vertices')
        self.publish_all_paths()

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        t = transform.transform.translation
        return (
            float(t.x), float(t.y),
            yaw_from_quaternion(transform.transform.rotation),
        )

    def make_path(self, points):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = now
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation = quaternion_from_yaw(float(yaw))
            path.poses.append(pose)
        return path

    def route_path(self, prepend_robot=False):
        vertices = list(self.route_points)
        if prepend_robot:
            robot = self.current_map_pose()
            if robot is None:
                raise RuntimeError(f'cannot read TF {self.global_frame}->{self.base_frame}')
            if not vertices or math.hypot(
                    vertices[0][0] - robot[0], vertices[0][1] - robot[1]) > 0.02:
                vertices.insert(0, robot[:2])
        samples = interpolate_polyline(
            vertices, float(self.get_parameter('route_spacing').value))
        return self.make_path(samples)

    def publish_all_paths(self):
        self.route_pub.publish(self.route_path())
        self.lio_pub.publish(self.make_path(self.lio_points))
        self.wheel_pub.publish(self.make_path(self.wheel_points))

    def append_trail(self, points, pose):
        sample_distance = float(self.get_parameter('trail_sample_distance').value)
        if points and math.hypot(
                pose[0] - points[-1][0], pose[1] - points[-1][1]) < sample_distance:
            return False
        points.append(pose)
        max_points = int(self.get_parameter('max_trail_points').value)
        if len(points) > max_points:
            del points[:len(points) - max_points]
        return True

    def on_lio(self, _msg):
        pose = self.current_map_pose()
        if pose is not None and self.append_trail(self.lio_points, pose):
            self.lio_pub.publish(self.make_path(self.lio_points))

    def on_wheel(self, msg):
        p = msg.pose.pose.position
        wheel_pose = (
            float(p.x), float(p.y), yaw_from_quaternion(msg.pose.pose.orientation))
        if self.wheel_origin is None:
            map_pose = self.current_map_pose()
            if map_pose is None:
                return
            self.wheel_origin = wheel_pose
            self.map_origin = map_pose
        aligned = align_relative_pose(wheel_pose, self.wheel_origin, self.map_origin)
        if self.append_trail(self.wheel_points, aligned):
            self.wheel_pub.publish(self.make_path(self.wheel_points))

    def on_click(self, msg):
        frame = msg.header.frame_id or self.global_frame
        if frame != self.global_frame:
            self.publish_status(
                f'REJECTED POINT: expected frame {self.global_frame}, got {frame}')
            return
        self.route_points.append((float(msg.point.x), float(msg.point.y)))
        self.route_pub.publish(self.route_path())
        self.publish_status(f'ROUTE: {len(self.route_points)} clicked vertices')

    def on_start(self, _request, response):
        if self.active_goal is not None or self.start_pending:
            response.success, response.message = False, 'route already active or pending'
            return response
        if len(self.route_points) < 1:
            response.success, response.message = False, 'click at least one route point in RViz'
            return response
        try:
            path = self.route_path(prepend_robot=True)
        except RuntimeError as exc:
            response.success, response.message = False, str(exc)
            return response
        if len(path.poses) < 2:
            response.success, response.message = False, 'route is too short'
            return response
        if not self.validation_client.wait_for_service(timeout_sec=0.5):
            response.success, response.message = False, 'Nav2 /is_path_valid service unavailable'
            return response

        self.start_pending = True
        request = IsPathValid.Request()
        request.path = path
        future = self.validation_client.call_async(request)
        future.add_done_callback(lambda done: self.on_validation(done, path))
        response.success = True
        response.message = f'validating {len(path.poses)} route poses before motion'
        self.publish_status(f'VALIDATING ROUTE: {len(path.poses)} poses')
        return response

    def on_validation(self, future, path):
        try:
            result = future.result()
        except Exception as exc:
            self.start_pending = False
            self.publish_status(f'ROUTE VALIDATION ERROR: {exc}')
            return
        if not result.is_valid:
            self.start_pending = False
            indices = list(result.invalid_pose_indices)
            summary = ','.join(str(index) for index in indices[:8])
            if len(indices) > 8:
                summary += ',...'
            self.publish_status(
                f'ROUTE REJECTED: footprint collision at path indices [{summary}]')
            return
        if not self.follow_client.wait_for_server(timeout_sec=0.5):
            self.start_pending = False
            self.publish_status('ROUTE ERROR: Nav2 /follow_path action unavailable')
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(self.get_parameter('controller_id').value)
        goal.goal_checker_id = str(self.get_parameter('goal_checker_id').value)
        send_future = self.follow_client.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)
        self.publish_status('ROUTE VALID: submitting to Nav2 FollowPath')

    def on_goal_response(self, future):
        self.start_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.publish_status(f'ROUTE SUBMIT ERROR: {exc}')
            return
        if not handle.accepted:
            self.publish_status('ROUTE REJECTED BY CONTROLLER')
            return
        self.active_goal = handle
        handle.get_result_async().add_done_callback(self.on_result)
        self.publish_status('ROUTE RUNNING')

    def on_feedback(self, feedback):
        # Keep the latched status quiet; RViz paths show progress continuously.
        _ = feedback

    def on_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.publish_status(f'ROUTE RESULT ERROR: {exc}')
            self.active_goal = None
            return
        labels = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        self.publish_status(f'ROUTE {labels.get(status, f"STATUS_{status}")}')
        self.active_goal = None

    def on_clear_route(self, _request, response):
        if self.active_goal is not None or self.start_pending:
            response.success, response.message = False, 'cancel the active route first'
            return response
        self.route_points.clear()
        self.route_pub.publish(self.route_path())
        self.publish_status('ROUTE CLEARED')
        response.success, response.message = True, 'drawn route cleared'
        return response

    def on_clear_trails(self, _request, response):
        self.lio_points.clear()
        self.wheel_points.clear()
        self.wheel_origin = None
        self.map_origin = None
        self.lio_pub.publish(self.make_path([]))
        self.wheel_pub.publish(self.make_path([]))
        self.publish_status('LIO AND WHEEL TRAILS CLEARED')
        response.success, response.message = True, 'trajectory trails cleared'
        return response

    def on_cancel(self, _request, response):
        if self.active_goal is None:
            response.success, response.message = True, 'no active route'
            return response
        self.active_goal.cancel_goal_async()
        self.publish_status('ROUTE CANCEL REQUESTED')
        response.success, response.message = True, 'cancel requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RvizPathTool()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
