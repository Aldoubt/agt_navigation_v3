"""Single research NavigationMode owner; sensor validity remains separate."""
import math
import time
from dataclasses import fields

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from agt_navigation_interfaces.msg import (GlobalQuality, LocalRowState, NavigationMode,
                                           OdomQuality, RecoveryStatus, TaskContext)
from agt_robot_interfaces.msg import BaseState, LocalizationStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
from .policy import Config, ContinuityPolicy, Evidence, Observation, Task


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class TaskContinuityNode(Node):
    def __init__(self):
        super().__init__('agt_task_continuity')
        default = Config()
        for f in fields(Config):
            self.declare_parameter('research_enabled' if f.name == 'enabled' else f.name,
                                   getattr(default, f.name))
        self.declare_parameter('require_payload_drive_permission', False)
        self.declare_parameter('payload_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('relocalization_backoff_sec', 2.0)
        self.declare_parameter('relocalization_request_topic', '/agt/relocalization/request')
        topics = {'task_topic': '/agt/research/task_context',
                  'global_quality_topic': '/agt/localization/quality',
                  'row_topic': '/agt/local_row/state',
                  'odom_quality_topic': '/agt/odometry/quality',
                  'odom_topic': '/agt/odometry/local',
                  'localization_topic': '/agt/localization/status',
                  'payload_topic': '/agt/payload/drive_permission',
                  'base_state_topic': '/agt/base/state',
                  'recovery_status_topic': '/agt/localization/recovery_status',
                  'mode_topic': '/agt/research/navigation_mode'}
        for name, topic in topics.items():
            self.declare_parameter(name, topic)
        config = Config(**{f.name: self.get_parameter(
            'research_enabled' if f.name == 'enabled' else f.name).value for f in fields(Config)})
        for value in (config.source_timeout_sec, config.global_quality_timeout_sec,
                      config.task_timeout_sec, config.authorization_ttl_sec,
                      config.degrade_dwell_sec, config.acquire_dwell_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('policy timeouts and dwell times must be finite and positive')
        if config.field_verified and not config.motion_parameters_valid():
            raise ValueError('verified research policy requires explicit measured safety and calibrated quality parameters')
        self.policy = ContinuityPolicy(config)
        self.config = config
        self.latest = {}
        self.publisher = self.create_publisher(NavigationMode, self.get_parameter('mode_topic').value, 1)
        self.relocalization_pub = self.create_publisher(Empty, self.get_parameter('relocalization_request_topic').value, 1)
        self._last_relocalization = -math.inf
        for key, msg_type, name in (
                ('task', TaskContext, 'task_topic'), ('global', GlobalQuality, 'global_quality_topic'),
                ('row', LocalRowState, 'row_topic'), ('quality', OdomQuality, 'odom_quality_topic'),
                ('odom', Odometry, 'odom_topic'), ('localization', LocalizationStatus, 'localization_topic'),
                ('payload', Bool, 'payload_topic'), ('base', BaseState, 'base_state_topic')):
            self.create_subscription(msg_type, self.get_parameter(name).value,
                                     lambda msg, k=key: self._receive(k, msg), 10)
        self.create_subscription(RecoveryStatus, self.get_parameter('recovery_status_topic').value,
                                 lambda msg: self._receive('recovery', msg), 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be finite and positive')
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate, self._tick, clock=self._steady_clock)
        self._last_reason = None
        if not math.isfinite(self.get_parameter('relocalization_backoff_sec').value) or self.get_parameter('relocalization_backoff_sec').value <= 0:
            raise ValueError('relocalization backoff must be finite and positive')

    def _receive(self, key, msg):
        previous = self.latest.get(key)
        if (previous and hasattr(msg, 'header') and stamp_ns(msg.header.stamp) < stamp_ns(previous[0].header.stamp)
                and getattr(msg, 'odom_epoch', 0) == getattr(previous[0], 'odom_epoch', 0)):
            # An odometry reversal is deliberately delivered to the policy and
            # epoch monitor. Other old messages cannot refresh freshness.
            if key != 'odom':
                return
        self.latest[key] = (msg, time.monotonic())

    def _get(self, key):
        entry = self.latest.get(key)
        return (None, math.inf) if entry is None else (entry[0], time.monotonic() - entry[1])

    def _tick(self):
        current = self.get_clock().now()
        now = current.nanoseconds * 1e-9
        task_msg, task_age = self._get('task')
        row, row_age = self._get('row')
        quality, quality_age = self._get('quality')
        global_msg, global_age = self._get('global')
        odom, odom_age = self._get('odom')
        loc, loc_age = self._get('localization')
        payload, payload_age = self._get('payload')
        base, base_age = self._get('base')
        recovery, recovery_age = self._get('recovery')
        task = Task()
        if task_msg:
            s = task_msg.segment
            local = task_msg.authorized_odom_path
            frame = ('odom' if stamp_ns(local.header.stamp) == stamp_ns(task_msg.plan_stamp)
                     and local.header.frame_id == 'odom' else s.map_path.header.frame_id)
            terminal_distance = 0.0
            if odom and local.poses:
                first = local.poses[0].pose.position
                terminal = local.poses[-1].pose.position
                pos = odom.pose.pose.position
                dx, dy = terminal.x-first.x, terminal.y-first.y
                length = math.hypot(dx, dy)
                if length > 0:
                    terminal_distance = max(0.0, ((terminal.x-pos.x)*dx + (terminal.y-pos.y)*dy) / length)
            task = Task(stamp_sec(task_msg.header.stamp), task_age, task_msg.task_id,
                        task_msg.request_epoch, task_msg.odom_epoch, task_msg.active,
                        task_msg.anchor_valid, s.allow_local, s.map_id, s.map_version, s.map_hash,
                        s.max_local_distance_m, s.max_local_duration_sec, s.max_position_std_m,
                        s.max_yaw_std_rad, s.max_linear_mps, s.max_angular_rps,
                        stamp_ns(task_msg.plan_stamp), frame, task_msg.handover_complete,
                        task_msg.recovery_complete, terminal_distance)
        row_evidence = Evidence() if row is None else Evidence(
            stamp=stamp_sec(row.header.stamp), receipt_age=row_age, epoch=row.odom_epoch,
            valid=row.ground_valid and row.row_valid, quality=row.quality,
            clearance_valid=row.clearance_valid, clearance_left=row.clearance_left_m,
            clearance_right=row.clearance_right_m, clearance_front=row.clearance_front_m)
        odom_evidence = Evidence() if quality is None else Evidence(
            stamp=stamp_sec(quality.header.stamp), receipt_age=quality_age, epoch=quality.odom_epoch,
            valid=quality.valid, quality=quality.quality,
            position_std=quality.position_std_m, yaw_std=quality.yaw_std_rad)
        global_evidence = Evidence() if global_msg is None else Evidence(
            stamp=stamp_sec(global_msg.header.stamp), receipt_age=global_age,
            epoch=global_msg.odom_epoch, valid=global_msg.valid and global_msg.ambiguity_valid,
            quality=global_msg.quality, map_id=global_msg.map_id, map_version=global_msg.map_version,
            map_hash=global_msg.map_hash)
        payload_allowed = not self.get_parameter('require_payload_drive_permission').value or (
            payload is not None and payload.data and payload_age <= self.get_parameter('payload_timeout_sec').value)
        loc_valid = (loc is not None and loc_age <= self.config.source_timeout_sec and
                     -0.01 <= now - stamp_sec(loc.stamp) <= self.config.source_timeout_sec and
                     loc.state == LocalizationStatus.STATE_LOCALIZED and loc.local_odom_fresh and
                     loc.global_correction_valid and loc.map_id == task.map_id and loc.map_version == task.map_version)
        base_valid = (base is not None and task_msg is not None and base_age <= self.config.source_timeout_sec
                      and -0.01 <= now - stamp_sec(base.stamp) <= self.config.source_timeout_sec
                      and base.robot_profile == task_msg.segment.robot_profile and base.source_valid
                      and base.drive_permitted and not base.emergency_stop and not base.manual_override
                      and not base.fault_active and base.measured_velocity_valid
                      and math.isfinite(base.measured_velocity.linear.x)
                      and math.isfinite(base.measured_velocity.angular.z))
        measured_speed = math.nan if odom is None else odom.twist.twist.linear.x
        if base_valid and math.isfinite(measured_speed):
            measured_speed = max(abs(measured_speed), abs(base.measured_velocity.linear.x))
        recovery_active = bool(
            recovery is not None and recovery.active and recovery.job_id
            and recovery_age <= self.config.source_timeout_sec
            and -0.01 <= now - stamp_sec(recovery.header.stamp) <= self.config.source_timeout_sec
            and (recovery.map_id, recovery.map_version, recovery.map_hash, recovery.odom_epoch) == (
                task.map_id, task.map_version, task.map_hash, task.odom_epoch))
        observation = Observation(now, time.monotonic(), task, global_evidence, row_evidence,
                                  odom_evidence, 0.0 if odom is None else stamp_sec(odom.header.stamp),
                                  odom_age, math.nan if odom is None else odom.pose.pose.position.x,
                                  math.nan if odom is None else odom.pose.pose.position.y,
                                  measured_speed,
                                  payload_allowed, loc_valid, base_valid, recovery_active)
        decision = self.policy.step(observation)
        out = NavigationMode()
        out.header.stamp = current.to_msg()
        out.mode = int(decision.mode)
        out.owner_id = self.config.owner_id
        out.task_id = task.task_id
        out.control_epoch = decision.control_epoch
        out.odom_epoch = task.odom_epoch
        out.authorized = decision.authorized
        ttl_ns = current.nanoseconds + int(self.config.authorization_ttl_sec * 1e9)
        out.valid_until.sec, out.valid_until.nanosec = divmod(ttl_ns, 1000000000)
        out.max_linear_mps, out.max_angular_rps = task.max_linear, task.max_angular
        out.remaining_distance_m, out.remaining_time_sec = decision.remaining_distance, decision.remaining_time
        out.reason = decision.reason
        if task_msg:
            out.route_id, out.segment_id, out.row_id = task_msg.segment.route_id, task_msg.segment.segment_id, task_msg.segment.row_id
            out.plan_stamp = task_msg.plan_stamp
        self.publisher.publish(out)
        if (self.config.enabled and decision.mode in (2, 3) and not recovery_active and
                time.monotonic() - self._last_relocalization >= self.get_parameter('relocalization_backoff_sec').value):
            self.relocalization_pub.publish(Empty())
            self._last_relocalization = time.monotonic()
        if decision.reason != self._last_reason:
            self.get_logger().info(f'{decision.mode.name}: {decision.reason}; authorized={decision.authorized}')
            self._last_reason = decision.reason


def main():
    rclpy.init()
    node = TaskContinuityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
