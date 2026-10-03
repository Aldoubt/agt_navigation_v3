"""Persistent research task action with explicit cancellation and command fences.

Only the epoch controller produces motion in this profile. An uncertain pending
acceptance or cancellation retains the capability lease until Nav2 is terminal.
"""
import copy
import math
import threading
import time
import rclpy

from action_msgs.msg import GoalStatus
from agt_navigation_interfaces.action import ExecuteTaskSegment
from agt_navigation_interfaces.msg import (GlobalQuality, NavigationMode, OdomQuality,
                                           RecoveryStatus, TaskContext)
from agt_robot_interfaces.msg import BaseState, LocalizationStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import IsPathValid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from tf2_ros import Buffer, TransformListener


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def message_stamp(message):
    return message.header.stamp if hasattr(message, 'header') else message.stamp


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def heading_error(a, b):
    if not math.isfinite(a) or not math.isfinite(b):
        return math.inf
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def closest_segment(point, poses):
    """Return a clamped XY projection on actual path segments, not vertices."""
    if not math.isfinite(point.x) or not math.isfinite(point.y):
        return None
    best = None
    for index, (first, second) in enumerate(zip(poses, poses[1:])):
        a, b = first.pose.position, second.pose.position
        dx, dy = b.x - a.x, b.y - a.y
        length_sq = dx*dx + dy*dy
        if length_sq <= 1e-12:
            continue
        fraction = max(0.0, min(1.0, ((point.x-a.x)*dx + (point.y-a.y)*dy) / length_sq))
        x, y = a.x + fraction*dx, a.y + fraction*dy
        result = (math.hypot(point.x-x, point.y-y), index, fraction, x, y, math.atan2(dy, dx))
        if best is None or result[0] < best[0]:
            best = result
    return best


def quaternion_product(a, b):
    return (a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
            a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
            a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
            a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2])


def transformed_pose(pose, transform):
    out = copy.deepcopy(pose)
    t, q = transform.transform.translation, transform.transform.rotation
    quat = (q.x, q.y, q.z, q.w)
    point = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0.0)
    rotated = quaternion_product(quaternion_product(quat, point), (-q.x, -q.y, -q.z, q.w))
    out.pose.position.x, out.pose.position.y, out.pose.position.z = rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z
    pq = pose.pose.orientation
    result = quaternion_product(quat, (pq.x, pq.y, pq.z, pq.w))
    out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = result
    out.header.frame_id = transform.header.frame_id
    return out


class ResearchTaskExecutor:
    def __init__(self, node, group):
        self.node, self.group = node, group
        defaults = {
            'enable_research_task_continuity': False, 'research_field_verified': False,
            'research_controller_id': 'ResearchFollowPath',
            'research_goal_checker_id': 'general_goal_checker',
            'research_source_timeout_sec': 0.3, 'research_handover_timeout_sec': 3.0,
            'research_global_quality_timeout_sec': 3.0, 'research_recovery_timeout_sec': 3.0,
            'research_local_path_max_age_sec': 0.5,
            'research_goal_tolerance_m': 0.0, 'research_goal_yaw_tolerance_rad': 0.0,
            'research_owner_id': 'agt_task_continuity',
            'research_task_topic': '/agt/research/task_context',
            'research_mode_topic': '/agt/research/navigation_mode',
            'research_row_path_topic': '/agt/local_row/path',
            'research_global_quality_topic': '/agt/localization/quality',
            'research_odom_quality_topic': '/agt/odometry/quality',
            'research_recovery_topic': '/agt/localization/recovery_status',
            'research_localization_topic': '/agt/localization/status',
            'research_base_state_topic': '/agt/base/state',
        }
        for name, default in defaults.items():
            node.declare_parameter(name, default)
        for name in ('research_source_timeout_sec', 'research_global_quality_timeout_sec',
                     'research_recovery_timeout_sec', 'research_handover_timeout_sec',
                     'research_local_path_max_age_sec'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.param('research_owner_id') or not self.param('research_controller_id'):
            raise ValueError('research owner and controller IDs must be explicit')
        self.enabled = bool(self.param('enable_research_task_continuity'))
        self.latest = {}
        self.lock = threading.RLock()
        self.context = None
        self.active = False
        self.uncertain = False
        self.last_request_epoch = 0
        self.child_result = None
        self.child_acceptance = None
        self.goal = None
        self.local_since = None
        self.last_local_pose = None
        self.last_local_stamp = 0
        self.handover_retry = False
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node, spin_thread=False)
        self.publisher = node.create_publisher(TaskContext, self.param('research_task_topic'), 1)
        for key, typ, topic in (
                ('mode', NavigationMode, self.param('research_mode_topic')),
                ('global', GlobalQuality, self.param('research_global_quality_topic')),
                ('quality', OdomQuality, self.param('research_odom_quality_topic')),
                ('recovery', RecoveryStatus, self.param('research_recovery_topic')),
                ('localization', LocalizationStatus, self.param('research_localization_topic')),
                ('base', BaseState, self.param('research_base_state_topic')),
                ('path', Path, self.param('research_row_path_topic')),
                ('odom', Odometry, '/agt/odometry/local')):
            node.create_subscription(typ, topic, lambda msg, k=key: self.receive(k, msg), 10,
                                     callback_group=group)
        self.planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose', callback_group=group)
        self.server = ActionServer(node, ExecuteTaskSegment, '/navigation/execute_task_segment',
                                   execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
                                   cancel_callback=self.cancel, callback_group=group)
        node.create_timer(0.05, self.heartbeat, callback_group=group, clock=self._steady_clock)

    def param(self, name):
        return self.node.get_parameter(name).value

    def receive(self, key, message):
        with self.lock:
            old = self.latest.get(key)
            same_epoch = (old and getattr(message, 'odom_epoch', 0) == getattr(old[0], 'odom_epoch', 0)
                          and getattr(message, 'control_epoch', 0) == getattr(old[0], 'control_epoch', 0))
            if same_epoch and ns(message_stamp(message)) < ns(message_stamp(old[0])) and key != 'odom':
                return
            self.latest[key] = (message, time.monotonic())

    def fresh(self, key, timeout=None):
        with self.lock:
            entry = self.latest.get(key)
        if not entry:
            return None
        message, receipt = entry
        limit = self.param('research_source_timeout_sec') if timeout is None else timeout
        if timeout is None and key in ('global', 'recovery'):
            limit = self.param('research_global_quality_timeout_sec' if key == 'global'
                               else 'research_recovery_timeout_sec')
        age = (self.node.get_clock().now().nanoseconds - ns(message_stamp(message))) * 1e-9
        if key == 'mode':
            if (message.owner_id != self.param('research_owner_id') or message.control_epoch <= 0
                    or message.odom_epoch <= 0
                    or (self.context and (message.task_id, message.odom_epoch) != (
                        self.context.task_id, self.context.odom_epoch))):
                return None
            if message.authorized and ns(message.valid_until) <= self.node.get_clock().now().nanoseconds:
                return None
        return message if ns(message_stamp(message)) > 0 and -0.01 <= age <= limit and time.monotonic() - receipt <= limit else None

    def trusted_global_ready(self, segment):
        """Research uses measured typed evidence without requiring a BT navigator."""
        quality, global_quality = self.fresh('quality'), self.fresh('global')
        loc, base = self.fresh('localization'), self.fresh('base')
        if quality is None or global_quality is None or loc is None or base is None:
            return False
        return bool(
            quality.valid and quality.odom_epoch > 0 and global_quality.valid
            and global_quality.ambiguity_valid and global_quality.odom_epoch == quality.odom_epoch
            and (global_quality.map_id, global_quality.map_version, global_quality.map_hash) == (
                segment.map_id, segment.map_version, segment.map_hash)
            and loc.state == LocalizationStatus.STATE_LOCALIZED and loc.local_odom_fresh
            and loc.global_correction_valid and (loc.map_id, loc.map_version) == (
                segment.map_id, segment.map_version)
            and base.robot_profile == segment.robot_profile and base.source_valid and base.drive_permitted
            and not base.emergency_stop and not base.manual_override and not base.fault_active
            and base.measured_velocity_valid and math.isfinite(base.measured_velocity.linear.x)
            and math.isfinite(base.measured_velocity.angular.z))

    def publish_context(self):
        with self.lock:
            if self.context is None:
                return
            self.context.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(copy.deepcopy(self.context))

    def heartbeat(self):
        if not self.active:
            return
        # Fresh header is only a task-owner heartbeat; all sensor ages are checked
        # independently by policy and guard.
        with self.lock:
            recovery = self.fresh('recovery')
            global_quality = self.fresh('global')
            if self.context:
                s = self.context.segment
                same = lambda x: x and (x.map_id, x.map_version, x.map_hash, x.odom_epoch) == (
                    s.map_id, s.map_version, s.map_hash, self.context.odom_epoch)
                verification_age = math.inf if recovery is None else (
                    self.node.get_clock().now().nanoseconds - ns(recovery.verified_observation_stamp)) * 1e-9
                self.context.recovery_complete = bool(
                    same(recovery) and same(global_quality) and recovery.complete and global_quality.valid
                    and bool(recovery.job_id) and recovery.job_id == global_quality.job_id
                    and global_quality.ambiguity_valid and 0 <= verification_age <= self.param('research_recovery_timeout_sec'))
                mode = self.fresh('mode')
                odom = self.fresh('odom')
                quality = self.fresh('quality')
                if quality and quality.odom_epoch != self.context.odom_epoch:
                    self.context.anchor_valid = False
                    self.context.handover_complete = False
                if mode and mode.mode in (NavigationMode.LOCAL_TASK, NavigationMode.RECOVERING):
                    if self.local_since is None:
                        self.local_since = time.monotonic()
                    if odom and ns(odom.header.stamp) > self.last_local_stamp:
                        p = odom.pose.pose.position
                        if self.last_local_pose:
                            self.context.local_distance_m += math.hypot(p.x-self.last_local_pose[0], p.y-self.last_local_pose[1])
                        self.last_local_pose = (p.x, p.y)
                        self.last_local_stamp = ns(odom.header.stamp)
                if self.local_since is not None:
                    self.context.local_elapsed_sec = time.monotonic() - self.local_since
        self.publish_context()

    def revoke(self, deactivate=False):
        with self.lock:
            if self.context:
                self.context.handover_complete = False
                if deactivate:
                    self.context.active = False
        self.publish_context()

    def cancel(self, _goal):
        self.revoke()
        return CancelResponse.ACCEPT

    async def sleep(self, seconds=0.02):
        future = Future()
        timer = None
        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(True)
        timer = self.node.create_timer(seconds, wake, callback_group=self.group, clock=self._steady_clock)
        try:
            await future
        finally:
            self.node.destroy_timer(timer)

    def result(self, goal, code, message, canceled=False):
        out = ExecuteTaskSegment.Result()
        out.success = False
        out.error_code, out.message = code, message
        if self.context:
            out.local_distance_m = self.context.local_distance_m
        if canceled:
            goal.canceled()
        else:
            goal.abort()
        return out

    def valid_segment(self, segment):
        values = (segment.max_local_distance_m, segment.max_local_duration_sec,
                  segment.max_position_std_m, segment.max_yaw_std_rad,
                  segment.max_linear_mps, segment.max_angular_rps)
        if not all(math.isfinite(v) and v > 0 for v in values):
            return False
        if segment.allow_local and (not math.isfinite(segment.max_local_lateral_error_m)
                or segment.max_local_lateral_error_m <= 0
                or not math.isfinite(segment.max_local_heading_error_rad)
                or not 0 < segment.max_local_heading_error_rad < math.pi/2):
            return False
        if not all((segment.route_id, segment.segment_id, segment.row_id,
                    segment.map_id, segment.map_version, segment.map_hash)):
            return False
        if not segment.requires_global_completion or segment.map_goal.header.frame_id != 'map':
            return False
        path = segment.map_path
        if path.header.frame_id != 'map' or len(path.poses) < 2:
            return False
        for pose in path.poses + [segment.map_goal]:
            p, q = pose.pose.position, pose.pose.orientation
            if pose.header.frame_id not in ('', 'map') or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                return False
            if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                return False
        end, goal = path.poses[-1].pose.position, segment.map_goal.pose.position
        if (math.hypot(end.x - goal.x, end.y - goal.y) > self.param('research_goal_tolerance_m')
                or heading_error(yaw(path.poses[-1].pose.orientation), yaw(segment.map_goal.pose.orientation))
                > self.param('research_goal_yaw_tolerance_rad')):
            return False
        if segment.allow_local:
            # This first implementation authorizes one straight row segment.
            # Turns, crossings and neighboring rows require a new global task.
            first = path.poses[0].pose.position
            dx, dy = end.x-first.x, end.y-first.y
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                return False
            dx, dy = dx/length, dy/length
            direction, previous = math.atan2(dy, dx), -math.inf
            for pose in path.poses:
                p = pose.pose.position
                along = (p.x-first.x)*dx + (p.y-first.y)*dy
                across = abs((p.x-first.x)*dy - (p.y-first.y)*dx)
                if (along < previous-1e-6 or across > segment.max_local_lateral_error_m
                        or heading_error(yaw(pose.pose.orientation), direction) > segment.max_local_heading_error_rad):
                    return False
                previous = along
        return True

    def create_anchor(self, segment, odom):
        transform = self.tf.lookup_transform('odom', 'map', rclpy.time.Time.from_msg(odom.header.stamp), timeout=Duration(seconds=0.2))
        converted = [transformed_pose(p, transform) for p in segment.map_path.poses]
        pos = odom.pose.pose.position
        projection = closest_segment(pos, converted)
        allowed_distance = (segment.max_local_lateral_error_m if segment.allow_local
                            else self.param('research_goal_tolerance_m'))
        allowed_heading = (segment.max_local_heading_error_rad if segment.allow_local
                           else self.param('research_goal_yaw_tolerance_rad'))
        if (projection is None or projection[0] > allowed_distance
                or heading_error(yaw(odom.pose.pose.orientation), projection[5]) > allowed_heading):
            raise ValueError('initial measured anchor is outside the frozen route or forward heading bounds')
        path = Path()
        path.header.frame_id = 'odom'
        anchor = PoseStamped()
        anchor.header = copy.deepcopy(odom.header)
        anchor.pose = copy.deepcopy(odom.pose.pose)
        # Freeze the segment's actual endpoint. The local travel budget starts
        # when LOCAL_TASK is first entered, independently of prior GLOBAL travel.
        start = copy.deepcopy(converted[projection[1]])
        start.pose.position.x, start.pose.position.y = projection[3], projection[4]
        start.pose.position.z += projection[2] * (converted[projection[1]+1].pose.position.z-start.pose.position.z)
        path.poses = [start]
        for pose in converted[projection[1]+1:]:
            p, previous = pose.pose.position, path.poses[-1].pose.position
            if math.hypot(p.x-previous.x, p.y-previous.y) > 1e-6:
                path.poses.append(pose)
        if len(path.poses) < 2:
            raise ValueError('authorized local interval is empty at the global anchor')
        return anchor, path

    def local_plan(self):
        observed = self.fresh('path', self.param('research_local_path_max_age_sec'))
        if observed is None or observed.header.frame_id != 'odom' or len(observed.poses) < 2:
            return None
        # The terminal remains in the initial odom epoch. Current observer geometry
        # may bend laterally, but cannot move the authorized longitudinal end.
        authorized = self.context.authorized_odom_path.poses
        odom, quality = self.fresh('odom'), self.fresh('quality')
        if (odom is None or quality is None or not quality.valid
                or quality.odom_epoch != self.context.odom_epoch):
            return None
        q = odom.pose.pose.orientation
        if (not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01):
            return None
        current = closest_segment(odom.pose.pose.position, authorized)
        if (current is None or current[0] > self.context.segment.max_local_lateral_error_m
                or heading_error(yaw(odom.pose.pose.orientation), current[5])
                > self.context.segment.max_local_heading_error_rad):
            return None
        anchor = authorized[0].pose.position
        terminal = self.context.authorized_odom_path.poses[-1].pose.position
        dx, dy = terminal.x - anchor.x, terminal.y - anchor.y
        norm = math.hypot(dx, dy)
        if norm <= 0:
            return None
        dx, dy = dx / norm, dy / norm
        current_position = odom.pose.pose.position
        if ((current_position.x-anchor.x)*dx + (current_position.y-anchor.y)*dy >= norm):
            return None
        out = copy.deepcopy(observed)
        out.poses = []
        for pose in observed.poses:
            p = pose.pose.position
            q = pose.pose.orientation
            if pose.header.frame_id not in ('', 'odom') or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                return None
            if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                return None
            longitudinal = (p.x - anchor.x) * dx + (p.y - anchor.y) * dy
            if longitudinal > norm:
                break
            if longitudinal >= -0.01:
                projection = closest_segment(p, authorized)
                if (projection is None or projection[0] > self.context.segment.max_local_lateral_error_m
                        or heading_error(yaw(q), projection[5]) > self.context.segment.max_local_heading_error_rad):
                    return None
                out.poses.append(pose)
        return out if len(out.poses) >= 2 else None

    async def wait_authorized(self, stamp, frame, deadline):
        while time.monotonic() < deadline:
            mode = self.fresh('mode')
            if mode and mode.task_id == self.context.task_id and mode.odom_epoch == self.context.odom_epoch:
                if mode.mode == NavigationMode.SAFE:
                    return None
                mode_frame = 'map' if mode.mode == NavigationMode.GLOBAL else 'odom'
                if (mode.mode == NavigationMode.DEGRADED or mode_frame != frame
                        or (mode.mode == NavigationMode.GLOBAL
                            and mode.reason == 'global_quality_low_authority_revoked')
                        or (mode.plan_stamp == stamp and not mode.authorized
                            and mode.reason == 'awaiting_terminal_barrier_and_new_plan')):
                    self.handover_retry = True
                    return None
                if mode.authorized and mode.plan_stamp == stamp:
                    return mode
            if self.goal.is_cancel_requested or not self.node._payload_ok():
                return None
            await self.sleep()
        return None

    def late_acceptance(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.uncertain = False
                if not self.active:
                    self.node._release()
                return
            self.node.nav_goal = handle
            handle.cancel_goal_async()
            result = handle.get_result_async()
            result.add_done_callback(self.terminal_cleanup)
        except Exception as exc:
            self.node.get_logger().error(f'late research acceptance remains uncertain: {exc}')

    def terminal_cleanup(self, future):
        try:
            wrapped = future.result()
            if wrapped.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                return
        except Exception:
            return
        self.uncertain = False
        if not self.active:
            self.node._release()

    async def stop_child(self):
        self.revoke()
        # Confirm the sole mode owner has withdrawn permission before requesting
        # action cancellation. Expiry also keeps guard closed if owner is absent.
        revoke_deadline = time.monotonic() + self.param('research_handover_timeout_sec')
        while self.context is not None and time.monotonic() < revoke_deadline:
            mode = self.fresh('mode')
            if mode is None or (mode.task_id == self.context.task_id and not mode.authorized):
                break
            await self.sleep()
        if self.node.nav_goal is None:
            return not self.uncertain
        handle = self.node.nav_goal
        handle.cancel_goal_async()
        future = self.child_result or handle.get_result_async()
        deadline = time.monotonic() + self.param('research_handover_timeout_sec')
        while not future.done() and time.monotonic() < deadline:
            await self.sleep()
        if not future.done():
            self.uncertain = True
            future.add_done_callback(self.terminal_cleanup)
            return False
        wrapped = future.result()
        if wrapped.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.uncertain = True
            return False
        self.node.nav_goal = None
        self.child_result = None
        return True

    async def start_child(self, path):
        # stop_child is called before every new submission. The path stamp is a
        # new source identity, published before controller setPlan observes it.
        self.handover_retry = False
        stamp = self.node.get_clock().now().to_msg()
        if ns(stamp) <= 0 or ns(stamp) <= ns(self.context.plan_stamp):
            self.revoke()
            return None
        path = copy.deepcopy(path)
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header = copy.deepcopy(path.header)
        with self.lock:
            self.context.plan_stamp = stamp
            if path.header.frame_id == 'map':
                self.context.segment.map_path.header.stamp = stamp
            else:
                # Keep frozen terminal geometry, update only this plan identity.
                self.context.authorized_odom_path.header.stamp = stamp
            self.context.handover_complete = True
        self.publish_context()
        mode = await self.wait_authorized(stamp, path.header.frame_id,
                                         time.monotonic() + self.param('research_handover_timeout_sec'))
        if mode is None:
            self.revoke()
            return None
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = self.param('research_controller_id')
        request.goal_checker_id = self.param('research_goal_checker_id')
        future = self.node.follow_client.send_goal_async(request)
        self.child_acceptance = future
        deadline = time.monotonic() + self.param('research_handover_timeout_sec')
        while not future.done() and time.monotonic() < deadline and not self.goal.is_cancel_requested:
            await self.sleep()
        if not future.done():
            self.revoke()
            self.uncertain = True
            future.add_done_callback(self.late_acceptance)
            return None
        handle = future.result()
        if not handle.accepted:
            self.revoke()
            return None
        self.node.nav_goal = handle
        self.child_result = handle.get_result_async()
        if self.goal.is_cancel_requested:
            self.revoke()
            handle.cancel_goal_async()
        return mode.mode

    async def validate_path(self, path):
        request = IsPathValid.Request()
        request.path = path
        future = self.node.path_validation.call_async(request)
        deadline = time.monotonic() + self.param('research_handover_timeout_sec')
        while not future.done() and time.monotonic() < deadline:
            await self.sleep()
        return future.done() and future.result().is_valid

    async def replan_global(self):
        if not self.planner.wait_for_server(timeout_sec=0.5):
            return None
        request = ComputePathToPose.Goal()
        request.goal = self.context.segment.map_goal
        request.use_start = False
        acceptance = self.planner.send_goal_async(request)
        deadline = time.monotonic() + self.param('research_handover_timeout_sec')
        while not acceptance.done() and time.monotonic() < deadline and not self.goal.is_cancel_requested:
            await self.sleep()
        if not acceptance.done():
            def cancel_late(f):
                handle = f.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            acceptance.add_done_callback(cancel_late)
            return None
        handle = acceptance.result()
        if not handle.accepted:
            return None
        future = handle.get_result_async()
        while not future.done() and time.monotonic() < deadline and not self.goal.is_cancel_requested:
            await self.sleep()
        if not future.done():
            handle.cancel_goal_async()
            return None
        wrapped = future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        path = wrapped.result.path
        return path if path.header.frame_id == 'map' and len(path.poses) >= 2 else None

    def reached_global_goal(self):
        mode = self.fresh('mode')
        if (mode is None or mode.mode != NavigationMode.GLOBAL or not mode.authorized
                or mode.plan_stamp != self.context.plan_stamp
                or not self.trusted_global_ready(self.context.segment)):
            return False
        odom = self.fresh('odom')
        global_quality = self.fresh('global')
        if odom is None or global_quality is None or not global_quality.valid:
            return False
        transform = self.tf.lookup_transform('map', 'odom', rclpy.time.Time.from_msg(odom.header.stamp))
        measured = PoseStamped()
        measured.header = odom.header
        measured.pose = odom.pose.pose
        measured = transformed_pose(measured, transform)
        desired = self.context.segment.map_goal
        p, g = measured.pose.position, desired.pose.position
        dyaw = math.atan2(math.sin(yaw(measured.pose.orientation) - yaw(desired.pose.orientation)),
                          math.cos(yaw(measured.pose.orientation) - yaw(desired.pose.orientation)))
        return (math.hypot(p.x - g.x, p.y - g.y) <= self.param('research_goal_tolerance_m')
                and abs(dyaw) <= self.param('research_goal_yaw_tolerance_rad'))

    async def execute(self, goal):
        if not self.enabled or not self.param('research_field_verified'):
            return self.result(goal, 'RESEARCH_DISABLED', 'research action requires explicit field verified profile')
        if (self.param('research_goal_tolerance_m') <= 0 or self.param('research_goal_yaw_tolerance_rad') <= 0
                or not math.isfinite(self.param('research_goal_tolerance_m'))
                or not math.isfinite(self.param('research_goal_yaw_tolerance_rad'))):
            return self.result(goal, 'FIELD_PARAMETERS_MISSING', 'goal tolerances need field verification')
        req, segment = goal.request, goal.request.segment
        if not req.task_id or req.request_epoch <= self.last_request_epoch or not self.valid_segment(segment):
            return self.result(goal, 'INVALID_SEGMENT', 'fixed identities, normalized map path, global goal and positive budgets required')
        if segment.robot_profile != self.node.get_parameter('robot_profile').value:
            return self.result(goal, 'PROFILE_MISMATCH', 'segment robot_profile differs from selected hardware')
        duration = req.timeout.sec + req.timeout.nanosec * 1e-9
        if not math.isfinite(duration) or duration <= 0:
            return self.result(goal, 'INVALID_TIMEOUT', 'task timeout must be positive')
        quality, odom, global_quality = self.fresh('quality'), self.fresh('odom'), self.fresh('global')
        if (not self.trusted_global_ready(segment) or not self.node._payload_ok() or quality is None or not quality.valid or odom is None
                or global_quality is None or not global_quality.valid or not global_quality.ambiguity_valid
                or global_quality.odom_epoch != quality.odom_epoch
                or (global_quality.map_id, global_quality.map_version, global_quality.map_hash) != (segment.map_id, segment.map_version, segment.map_hash)):
            return self.result(goal, 'TRUSTED_ANCHOR_UNAVAILABLE', 'fresh trusted global identity and local odometry required')
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        if (quality.odom_epoch <= 0 or odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link'
                or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01):
            return self.result(goal, 'ODOMETRY_INVALID', 'canonical measured odometry and normalized pose required')
        if not self.node._claim():
            return self.result(goal, 'BUSY', 'another navigation action or uncertain Nav2 handle owns control')
        self.active, self.goal = True, goal
        self.context = None
        self.child_result = None
        self.child_acceptance = None
        self.local_since = None
        self.last_local_pose = None
        self.last_local_stamp = 0
        self.node._research_active = True
        self.last_request_epoch = req.request_epoch
        deadline = time.monotonic() + duration
        try:
            anchor, local_interval = self.create_anchor(segment, odom)
            self.context = TaskContext()
            self.context.task_id, self.context.segment = req.task_id, copy.deepcopy(segment)
            self.context.request_epoch, self.context.odom_epoch = req.request_epoch, quality.odom_epoch
            self.context.active = self.context.anchor_valid = True
            self.context.anchor_odom_pose, self.context.authorized_odom_path = anchor, local_interval
            self.publish_context()
            if not self.node.follow_client.wait_for_server(timeout_sec=2.0) or not self.node.path_validation.wait_for_service(timeout_sec=2.0):
                return self.result(goal, 'NAV2_UNAVAILABLE', 'FollowPath and path validation are required')
            if not await self.validate_path(segment.map_path):
                return self.result(goal, 'PATH_INVALID', 'fixed map path failed collision validation')
            # Give the mode owner time to choose a path class. No command is
            # installed during this decision window.
            initial = time.monotonic() + self.param('research_handover_timeout_sec')
            while time.monotonic() < initial:
                mode = self.fresh('mode')
                if mode and mode.task_id == req.task_id and mode.mode in (
                        NavigationMode.GLOBAL, NavigationMode.LOCAL_TASK, NavigationMode.RECOVERING):
                    break
                await self.sleep()
            else:
                return self.result(goal, 'POLICY_NOT_READY', 'research mode owner did not confirm a usable path class')
            active_mode = None
            first_plan = True
            active_local_source = 0
            while time.monotonic() < deadline:
                mode = self.fresh('mode')
                if goal.is_cancel_requested:
                    if not await self.stop_child():
                        return self.result(goal, 'CANCEL_UNCONFIRMED',
                                           'command authority withdrawn; Nav2 handle quarantined until terminal')
                    return self.result(goal, 'CANCELED', 'research task canceled', True)
                if not self.node._payload_ok() or self.node.payload_lost:
                    await self.stop_child()
                    return self.result(goal, 'ARM_NOT_DRIVE_SAFE', 'payload drive permission lost')
                if mode is None or mode.task_id != req.task_id or mode.mode == NavigationMode.SAFE:
                    await self.stop_child()
                    return self.result(goal, 'SAFE_STOP', 'mode expired or policy entered SAFE')
                feedback = ExecuteTaskSegment.Feedback()
                feedback.mode, feedback.state = mode, str(mode.mode)
                feedback.distance_remaining_m = mode.remaining_distance_m
                goal.publish_feedback(feedback)
                if mode.mode == NavigationMode.DEGRADED:
                    if self.node.nav_goal is not None and not await self.stop_child():
                        return self.result(goal, 'CANCEL_UNCONFIRMED', 'old Nav2 action remains nonterminal')
                    await self.sleep()
                    continue
                if (mode.mode == NavigationMode.GLOBAL and not mode.authorized
                        and mode.reason == 'global_quality_low_authority_revoked'):
                    # Quality withdrawal precedes the degradation dwell. Stop
                    # promptly, then let the owner choose GLOBAL or LOCAL_TASK;
                    # a new map plan cannot resolve an odom-mode transition.
                    if self.node.nav_goal is not None and not await self.stop_child():
                        return self.result(goal, 'CANCEL_UNCONFIRMED', 'old Nav2 action remains nonterminal')
                    await self.sleep()
                    continue
                desired_mode = mode.mode
                if desired_mode == NavigationMode.RECOVERING:
                    desired_mode = NavigationMode.LOCAL_TASK
                live_local_path = self.fresh('path', self.param('research_local_path_max_age_sec'))
                local_plan_expired = (active_mode in (NavigationMode.LOCAL_TASK, NavigationMode.RECOVERING)
                                      and (live_local_path is None or
                                           (self.node.get_clock().now().nanoseconds - active_local_source) * 1e-9 > self.param('research_local_path_max_age_sec')))
                current_class = NavigationMode.LOCAL_TASK if active_mode == NavigationMode.RECOVERING else active_mode
                epoch_revoked = self.node.nav_goal is not None and (not mode.authorized or mode.plan_stamp != self.context.plan_stamp)
                if desired_mode != current_class or epoch_revoked or local_plan_expired or self.node.nav_goal is None:
                    if not await self.stop_child():
                        return self.result(goal, 'CANCEL_UNCONFIRMED', 'Nav2 terminal acknowledgement missing; capability quarantined')
                    if desired_mode == NavigationMode.GLOBAL:
                        path = copy.deepcopy(segment.map_path) if first_plan else await self.replan_global()
                        if path is None or not await self.validate_path(path):
                            return self.result(goal, 'REPLAN_FAILED', 'restored global path unavailable or unsafe')
                    else:
                        path = self.local_plan()
                        if path is None:
                            return self.result(goal, 'LOCAL_PATH_UNAVAILABLE', 'fresh odom path within frozen authorized terminal required')
                        active_local_source = ns(path.header.stamp)
                    active_mode = await self.start_child(path)
                    if active_mode is None:
                        if self.handover_retry or goal.is_cancel_requested:
                            await self.sleep()
                            continue
                        return self.result(goal, 'HANDOVER_FAILED', 'new epoch plan rejected or timed out')
                    first_plan = False
                if self.child_result is not None and self.child_result.done():
                    wrapped = self.child_result.result()
                    if wrapped.status == GoalStatus.STATUS_SUCCEEDED and active_mode == NavigationMode.GLOBAL and self.reached_global_goal():
                        result = ExecuteTaskSegment.Result()
                        result.success, result.message = True, 'trusted global task goal reached'
                        result.local_distance_m = self.context.local_distance_m
                        goal.succeed()
                        self.node.nav_goal = None
                        self.child_result = None
                        return result
                    if wrapped.status == GoalStatus.STATUS_SUCCEEDED and active_mode != NavigationMode.GLOBAL:
                        # Local controller terminal is never map task completion.
                        self.node.nav_goal = None
                        self.child_result = None
                    else:
                        return self.result(goal, 'NAV2_FAILED', f'Nav2 terminal {wrapped.status} without trusted global completion')
                await self.sleep(0.05)
            await self.stop_child()
            return self.result(goal, 'TASK_TIMEOUT', 'outer timeout includes all local and recovery time')
        except Exception as exc:
            self.node.get_logger().error(f'research task failed: {exc}')
            await self.stop_child()
            return self.result(goal, 'RESEARCH_ERROR', str(exc))
        finally:
            self.revoke(deactivate=True)
            if self.node.nav_goal is not None and not self.uncertain:
                await self.stop_child()
            self.active = False
            self.node._research_active = False
            if not self.uncertain:
                self.node._release()
