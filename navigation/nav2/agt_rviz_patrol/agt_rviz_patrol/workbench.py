"""Revisioned RViz workbench. Motion remains behind Navigation Capability."""
import copy
import json
import math
import time
import zlib

from action_msgs.msg import GoalStatus
from agt_navigation_interfaces.action import FollowRoute
from agt_navigation_interfaces.msg import NavigationHealth
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import IsPathValid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .path_tool import RvizPathTool, interpolate_polyline
from .route_editor import RouteEditor


class RouteWorkbench(RvizPathTool):
    def __init__(self):
        self.editor = RouteEditor()
        self.preview = None
        self.validating = False
        self.validation_token = 0
        self.cancel_pending = False
        self.health = None
        self.health_received = 0.0
        self.map_key = None
        self.progress = 0
        self.paused_pose = None
        self.lio_dirty = self.wheel_dirty = False
        self.status_text = 'INITIALIZING'
        super().__init__()
        for key, value in [('max_start_connection', 1.0), ('preview_ttl', 30.0),
                           ('map_id', ''), ('map_version', '')]:
            self.declare_parameter(key, value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.editor_pub = self.create_publisher(String, '/agt/path_tool/editor_state', latched)
        self.marker_pub = self.create_publisher(Marker, '/agt/path_tool/vertices', latched)
        self.create_subscription(String, '/agt/path_tool/edit', self.on_edit, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(NavigationHealth, '/navigation/health', self.on_health, latched)
        self.validator = self.create_client(IsPathValid, '/is_path_valid')
        for name, callback in [('preview', self.on_preview), ('undo', self.on_undo), ('redo', self.on_redo)]:
            self.create_service(Trigger, '/agt/path_tool/'+name, callback)
        self.create_timer(0.5, self.tick)
        self.publish_status('DRAFT: drag a stroke or edit waypoints; preview before execution')
        self.publish_draft()

    def offline(self):
        return bool(self.get_parameter('preview_only').value)

    def busy(self):
        return self.active_goal is not None or self.start_pending

    def locked(self):
        return self.busy() or self.paused_path is not None

    def publish_status(self, text):
        self.status_text = text
        super().publish_status(text)
        if hasattr(self, 'editor_pub'):
            self.publish_state()

    def health_ok(self):
        h = self.health
        return bool(h and time.monotonic()-self.health_received <= 1.5
                    and h.status == NavigationHealth.READY
                    and all((h.platform_ready, h.lidar_alive, h.imu_alive,
                             h.odom_alive, h.localized, h.nav2_active)))

    def publish_state(self):
        h = self.health
        data = dict(revision=self.editor.revision, points=self.editor.points, map_received=self.map_key is not None,
                    preview_length_m=self.preview.get("length_m") if self.preview else None,
                    locked=self.locked(), preview_only=self.offline(), status=self.status_text,
                    validating=self.validating, can_execute=bool(self.preview and not self.validating
                    and not self.busy() and not self.offline() and self.health_ok()),
                    map_id=h.map_id if h else self.get_parameter('map_id').value,
                    map_version=h.map_version if h else self.get_parameter('map_version').value,
                    health=h.state if h else 'UNAVAILABLE', localized=bool(h and h.localized),
                    resume_preview=bool(self.preview and self.preview['resume']),
                    progress_index=self.progress, distance_remaining=getattr(self, 'feedback_remaining', None))
        self.editor_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def invalidate(self):
        self.preview = None
        self.validating = False
        self.validation_token += 1

    def on_map(self, msg):
        info = msg.info
        key = (msg.header.frame_id, info.width, info.height, info.resolution,
               str(info.origin), zlib.crc32(msg.data.tobytes()))
        if key != self.map_key:
            self.map_key = key
            self.invalidate()
            self.publish_status('MAP UPDATED: preview required')

    def on_health(self, msg):
        old = self.health
        self.health = msg
        self.health_received = time.monotonic()
        if (old and (old.map_id, old.map_version) != (msg.map_id, msg.map_version)) or not self.health_ok():
            self.invalidate()

    def publish_draft(self):
        self.route_points = self.editor.points[:]
        self.route_pub.publish(self.route_path())
        if not hasattr(self, 'marker_pub'):
            return
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'route_vertices'; marker.id = 0; marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.18
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
        marker.points = [Point(x=x, y=y, z=0.04) for x,y in self.editor.points]
        self.marker_pub.publish(marker)
        self.publish_state()

    def edited(self):
        self.invalidate()
        self.publish_draft()
        self.publish_status(f'DRAFT revision {self.editor.revision}: preview required')

    def on_edit(self, msg):
        try:
            if self.locked():
                raise ValueError('route locked: cancel active/paused route before editing')
            if len(msg.data) > 256000:
                raise ValueError('edit message too large')
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('points'), list):
                raise ValueError('edit requires an object containing a point array')
            if type(data.get('revision')) is not int:
                raise ValueError('integer revision required')
            if data.get('frame') != self.global_frame:
                raise ValueError('edit requires map frame')
            self.editor.replace(data['points'], data['revision'])
            self.edited()
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.publish_status('EDIT REJECTED: '+str(exc))

    def on_click(self, msg):
        # Compatibility input still goes through the same bounds/revision/lock policy.
        if msg.header.frame_id != self.global_frame:
            self.publish_status('EDIT REJECTED: map frame required')
            return
        payload = dict(frame=self.global_frame, revision=self.editor.revision,
                       points=self.editor.points+[(msg.point.x, msg.point.y)])
        self.on_edit(String(data=json.dumps(payload)))

    def history_edit(self, response, operation):
        try:
            if self.locked():
                raise ValueError('cancel active/paused route before editing')
            operation()
            self.edited()
            response.success, response.message = True, self.status_text
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_undo(self, _, response):
        return self.history_edit(response, self.editor.undo)

    def on_redo(self, _, response):
        return self.history_edit(response, self.editor.redo)

    def on_clear_route(self, _, response):
        return self.history_edit(response, lambda: self.editor.replace([], self.editor.revision))

    def robot_pose(self):
        pose = self.current_map_pose()
        if pose is None or not all(math.isfinite(v) for v in pose):
            raise ValueError('map -> base_link unavailable')
        if not self.offline():
            from rclpy.time import Time
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
            stamp = Time.from_msg(tf.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp == 0 or not -0.1 <= age <= 0.5:
                raise ValueError('robot TF stale: fresh localization required')
        return pose

    def build_preview(self, points, robot):
        if len(points) < 1:
            raise ValueError('draw a route first')
        distance = math.dist(robot[:2], points[0])
        if distance > float(self.get_parameter('max_start_connection').value):
            raise ValueError('start connection exceeds limit; draw near robot or plan a connector')
        vertices = [robot[:2]] + list(points)
        spacing = float(self.get_parameter('route_spacing').value)
        if not math.isfinite(spacing) or not 0.01 <= spacing <= 0.05:
            raise ValueError('route spacing must be finite and between 0.01 and 0.05 m')
        samples = interpolate_polyline(vertices, spacing)
        if not 2 <= len(samples) <= 10000:
            raise ValueError('path too short or too many samples')
        return self.make_path(samples)

    def on_preview(self, _, response):
        try:
            if self.busy():
                raise ValueError('route active or pending')
            if self.map_key is None:
                raise ValueError('map not received')
            if not self.offline() and not self.health_ok():
                raise ValueError('navigation health not READY')
            robot = self.robot_pose()
            if self.paused_path is not None:
                if self.paused_pose is None or math.dist(robot[:2], self.paused_pose[:2]) > 0.15:
                    raise ValueError('robot moved after pause; cancel and redraw instead of ambiguous resume')
                if any(abs(i-self.progress) > 20 and
                       math.hypot(p.pose.position.x-robot[0], p.pose.position.y-robot[1]) < 0.2
                       for i, p in enumerate(self.paused_path.poses)):
                    raise ValueError('ambiguous crossing/loop at pause: cancel and redraw')
                points = [(p.pose.position.x,p.pose.position.y)
                          for p in self.paused_path.poses[self.progress:]]
            else:
                points = self.editor.points
            path = self.build_preview(points, robot)
            self.invalidate()
            snapshot = dict(path=path, revision=self.editor.revision, map_key=self.map_key,
                            robot=robot, time=time.monotonic(), resume=self.paused_path is not None,
                            length_m=sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                                   b.pose.position.y-a.pose.position.y)
                                         for a,b in zip(path.poses,path.poses[1:])))
            self.route_pub.publish(path)
            if self.offline():
                self.preview = snapshot
                self.publish_status('OFFLINE GEOMETRY PREVIEW: no collision validation; execution disabled')
            else:
                if not self.validator.service_is_ready():
                    raise ValueError('IsPathValid unavailable')
                self.validating = True
                token = self.validation_token
                self.validation_started = time.monotonic()
                request = IsPathValid.Request(); request.path = path
                self.validator.call_async(request).add_done_callback(
                    lambda f: self.on_validated(f, token, snapshot))
                self.publish_status('VALIDATING: wait for VALIDATED before confirming execution')
            response.success, response.message = True, self.status_text
        except Exception as exc:
            self.invalidate()
            response.success, response.message = False, str(exc)
            self.publish_status('PREVIEW REJECTED: '+str(exc))
        return response

    def on_validated(self, future, token, snapshot):
        if token != self.validation_token:
            return
        self.validating = False
        try:
            if not future.result().is_valid:
                raise ValueError('path collision/invalid costmap poses')
            if not self.health_ok():
                raise ValueError('health changed during validation')
            if math.dist(self.robot_pose()[:2], snapshot['robot'][:2]) > 0.1:
                raise ValueError('robot moved during validation')
            self.preview = snapshot
            self.publish_status('VALIDATED: confirm Execute (or Resume for paused route)')
        except Exception as exc:
            self.invalidate()
            self.publish_status('VALIDATION FAILED: '+str(exc))

    def on_start(self, _, response):
        return self.start_preview(response, resume=False)

    def on_resume(self, _, response):
        return self.start_preview(response, resume=True)

    def start_preview(self, response, resume):
        try:
            if self.offline():
                raise ValueError('offline mode: execution disabled; use Preview')
            if self.busy() or self.validating or self.preview is None:
                raise ValueError('fresh validated preview required; no active/pending task')
            p = self.preview
            if p['resume'] != resume or resume != (self.paused_path is not None):
                raise ValueError('use Resume for a paused preview, Execute for a new route')
            if p['revision'] != self.editor.revision or p['map_key'] != self.map_key:
                raise ValueError('preview revision/map changed')
            if time.monotonic()-p['time'] > float(self.get_parameter('preview_ttl').value):
                raise ValueError('preview expired')
            if not self.health_ok():
                raise ValueError('navigation health not READY')
            robot = self.robot_pose()
            yaw_diff = math.atan2(math.sin(robot[2]-p['robot'][2]), math.cos(robot[2]-p['robot'][2]))
            if math.dist(robot[:2], p['robot'][:2]) > 0.1 or abs(yaw_diff) > 0.15:
                raise ValueError('robot moved: preview again')
            if not self.submit_path(copy.deepcopy(p['path'])):
                raise ValueError('Navigation Capability unavailable')
            self.paused_path = None
            self.invalidate()
            response.success, response.message = True, 'confirmed path submitted unchanged'
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self.publish_status('EXECUTION REJECTED: '+str(exc))
        return response

    def submit_path(self, path):
        if not self.follow_client.server_is_ready():
            return False
        self.start_pending = True
        self.cancel_pending = self.pause_pending = False
        self.active_path = path
        self.progress = 0
        try:
            goal = FollowRoute.Goal(); goal.path = path
            future = self.follow_client.send_goal_async(goal, feedback_callback=self.on_feedback)
            future.add_done_callback(self.on_goal_response)
        except Exception:
            self.start_pending = False
            self.active_path = None
            raise
        self.publish_status('PENDING: awaiting Navigation Capability acceptance')
        return True

    def on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.start_pending = True
            self.publish_status('SUBMIT STATE UNKNOWN: '+str(exc)+'; remain locked, verify stop independently')
            return
        self.start_pending = False
        try:
            if not handle.accepted:
                raise ValueError('Navigation Capability rejected goal')
            self.active_goal = handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_pending or self.pause_pending:
                self.request_cancel()
            else:
                self.publish_status('RUNNING')
        except Exception as exc:
            if self.active_goal is not None:
                self.publish_status('ACCEPTED GOAL STATE UNKNOWN: '+str(exc)+'; remain locked')
                return
            self.active_path = None
            self.cancel_pending = self.pause_pending = False
            self.publish_status('SUBMIT FAILED: '+str(exc))

    def request_cancel(self):
        if self.active_goal is not None:
            try:
                self.active_goal.cancel_goal_async().add_done_callback(self.on_cancel_ack)
            except Exception as exc:
                self.publish_status('CANCEL ERROR: '+str(exc)+'; task remains locked')
        self.publish_status('PAUSING' if self.pause_pending else 'CANCELING')

    def on_cancel_ack(self, future):
        try:
            if not future.result().goals_canceling:
                self.pause_pending = False
                self.publish_status('CANCEL NOT ACKNOWLEDGED: task locked until terminal result; retry/stop safely')
        except Exception as exc:
            self.publish_status('CANCEL ACK ERROR: '+str(exc))

    def on_cancel(self, _, response):
        self.invalidate()
        self.paused_path = None
        self.pause_pending = False
        self.cancel_pending = self.busy()
        if self.busy():
            self.request_cancel()
        else:
            self.publish_status('CANCELED / IDLE')
        response.success, response.message = True, 'cancel latched, including pending goal; wait for terminal result'
        return response

    def on_pause(self, _, response):
        if not self.busy():
            response.success, response.message = False, 'no active or pending route'
            return response
        self.pause_pending = True
        self.cancel_pending = False
        self.request_cancel()
        response.success, response.message = True, 'pause requested; wait for confirmed PAUSED'
        return response

    def on_result(self, future):
        try:
            wrapped = future.result()
            if self.pause_pending and wrapped.status == GoalStatus.STATUS_CANCELED:
                self.paused_path = self.active_path
                self.paused_pose = self.current_map_pose()
                text = 'PAUSED: Preview remaining route, then confirm Resume; editing locked until Cancel'
            else:
                text = f'FINISHED status={wrapped.status}: {wrapped.result.error_code} {wrapped.result.message}'
            self.active_goal = self.active_path = None
            self.start_pending = self.pause_pending = self.cancel_pending = False
            self.invalidate()
            self.publish_status(text)
        except Exception as exc:
            # Unknown terminal state must not unlock editing or permit another goal.
            self.publish_status('RESULT UNKNOWN: '+str(exc)+'; remain locked, verify stop before restarting tool')

    def on_feedback(self, feedback):
        # Panel status heartbeat avoids high-rate log/JSON churn.
        self.feedback_state = feedback.feedback.state
        value = feedback.feedback.distance_remaining
        self.feedback_remaining = float(value) if math.isfinite(value) else None

    def on_lio(self, _msg):
        pose = self.current_map_pose()
        if pose and all(math.isfinite(v) for v in pose):
            self.lio_dirty |= self.append_trail(self.lio_points, pose)

    def on_wheel(self, msg):
        from .path_tool import yaw_from_quaternion, align_relative_pose
        p = msg.pose.pose.position
        wheel = (float(p.x), float(p.y), yaw_from_quaternion(msg.pose.pose.orientation))
        if not all(math.isfinite(v) for v in wheel):
            return
        if self.wheel_origin is None:
            origin = self.current_map_pose()
            if origin is None:
                return
            self.wheel_origin, self.map_origin = wheel, origin
        pose = align_relative_pose(wheel, self.wheel_origin, self.map_origin)
        self.wheel_dirty |= self.append_trail(self.wheel_points, pose)

    def tick(self):
        if self.validating and time.monotonic()-self.validation_started > 3.0:
            self.invalidate()
            self.publish_status('VALIDATION TIMEOUT: preview again')
        if self.preview and (time.monotonic()-self.preview['time'] > float(self.get_parameter('preview_ttl').value)
                             or (not self.offline() and not self.health_ok())):
            self.invalidate()
            self.publish_status('PREVIEW EXPIRED: preview again')
        if self.active_goal is not None and self.active_path is not None:
            robot = self.current_map_pose()
            if robot is not None:
                # Monotonic local progress window, never globally jump across a crossing.
                poses = self.active_path.poses
                indices = range(self.progress, min(len(poses), self.progress+21))
                best = min(indices, key=lambda i: math.hypot(poses[i].pose.position.x-robot[0],
                                                            poses[i].pose.position.y-robot[1]))
                if math.hypot(poses[best].pose.position.x-robot[0], poses[best].pose.position.y-robot[1]) < 0.3:
                    self.progress = best
        if self.lio_dirty:
            self.lio_pub.publish(self.make_path(self.lio_points)); self.lio_dirty = False
        if self.wheel_dirty:
            self.wheel_pub.publish(self.make_path(self.wheel_points)); self.wheel_dirty = False
        self.publish_state()
