"""Research odometry evidence. A live stream alone never grants local motion."""

from __future__ import annotations

import math
import secrets
import time

import rclpy
from agt_navigation_interfaces.msg import OdomQuality
from agt_robot_interfaces.msg import AdapterStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.qos import qos_profile_sensor_data


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid_quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class OdomQualityNode(Node):
    def __init__(self):
        super().__init__('agt_odom_quality')
        defaults = {
            'odom_topic': '/agt/odometry/local',
            'adapter_status_topic': '/agt/odometry/adapter_status',
            'output_topic': '/agt/odometry/quality',
            'odom_frame': 'odom', 'base_frame': 'base_link',
            'field_verified': False, 'covariance_calibrated': False,
            'source_timeout_sec': 0.3, 'adapter_timeout_sec': 0.5,
            'future_tolerance_sec': 0.05,
            # Positive limits/model coefficients must come from calibration.
            'max_measured_linear_mps': 0.0, 'max_measured_angular_rps': 0.0,
            'jump_slack_m': 0.0, 'jump_slack_rad': 0.0,
            'position_std_floor_m': 0.0, 'yaw_std_floor_rad': 0.0,
            'position_drift_per_meter': 0.0, 'yaw_drift_per_meter': 0.0,
            'position_drift_per_second': 0.0, 'yaw_drift_per_second': 0.0,
            'max_position_std_m': 0.0, 'max_yaw_std_rad': 0.0,
            'publish_rate_hz': 10.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key,value in defaults.items():
            if isinstance(value,float):
                configured=float(self.value(key))
                if not math.isfinite(configured) or configured<0:
                    raise ValueError(key+' requires a finite nonnegative value')
        for key in ('source_timeout_sec','adapter_timeout_sec','publish_rate_hz'):
            if self.value(key)<=0: raise ValueError(key+' must be positive')
        self.epoch = secrets.randbits(63) or 1
        self.last = None
        self.received = 0.0
        self.adapter = None
        self.adapter_received = 0.0
        self.distance = 0.0
        self.start_stamp = 0
        self.reason = 'no_input'
        self.clock_ns = None
        self.pub = self.create_publisher(OdomQuality, self.value('output_topic'), 10)
        self.create_subscription(Odometry, self.value('odom_topic'), self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(AdapterStatus, self.value('adapter_status_topic'),
                                 self.on_adapter, 10)
        self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1/max(1.0, float(self.value('publish_rate_hz'))), self.publish,clock=self.steady_clock)

    def value(self, name):
        return self.get_parameter(name).value

    def reset(self, reason):
        self.epoch += 1
        self.last = None
        self.distance = 0.0
        self.start_stamp = 0
        self.reason = reason

    def on_adapter(self, msg):
        self.adapter = msg
        self.adapter_received = time.monotonic()

    def on_odom(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        stamp = stamp_ns(msg.header.stamp)
        if (msg.header.frame_id != self.value('odom_frame') or
                msg.child_frame_id != self.value('base_frame')):
            self.reset('frame_mismatch')
            return
        p = msg.pose.pose.position
        try:
            heading = yaw(msg.pose.pose.orientation)
        except ValueError as exc:
            self.reset(str(exc))
            return
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.reset('nonfinite_pose')
            return
        if stamp <= 0 or now_ns-stamp > float(self.value('source_timeout_sec'))*1e9:
            self.reset('source_stale')
            return
        if stamp-now_ns > float(self.value('future_tolerance_sec'))*1e9:
            self.reset('source_future')
            return
        if self.last is not None:
            old_stamp = stamp_ns(self.last.header.stamp)
            if stamp == old_stamp:
                if msg.pose.pose!=self.last.pose.pose:
                    self.reset('pose_changed_under_duplicate_stamp')
                return
            if stamp < old_stamp:
                self.reset('source_time_regression')
                return
            old = self.last.pose.pose.position
            delta = math.sqrt((p.x-old.x)**2+(p.y-old.y)**2+(p.z-old.z)**2)
            angle = abs(math.atan2(math.sin(heading-yaw(self.last.pose.pose.orientation)),
                                   math.cos(heading-yaw(self.last.pose.pose.orientation))))
            dt = (stamp-old_stamp)*1e-9
            linear = float(self.value('max_measured_linear_mps'))
            angular = float(self.value('max_measured_angular_rps'))
            if ((linear > 0 and delta > linear*dt+float(self.value('jump_slack_m'))) or
                    (angular > 0 and angle > angular*dt+float(self.value('jump_slack_rad')))):
                self.reset('pose_discontinuity')
                return
            self.distance += delta
        if not self.start_stamp:
            self.start_stamp = stamp
        self.last = msg
        self.received = time.monotonic()
        self.reason = 'observing'

    def publish(self):
        now = self.get_clock().now()
        if self.clock_ns is not None and now.nanoseconds < self.clock_ns:
            self.reset('ros_clock_regression')
        self.clock_ns = now.nanoseconds
        msg = OdomQuality()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.value('odom_frame')
        msg.odom_epoch = self.epoch
        msg.position_std_m = msg.yaw_std_rad = math.inf
        msg.input_age_sec = msg.output_age_sec = math.inf
        reason = self.reason
        if self.last is not None:
            msg.header.stamp = self.last.header.stamp
            msg.output_age_sec = max(0.0, (now.nanoseconds-stamp_ns(msg.header.stamp))*1e-9)
            if self.adapter is not None:
                msg.input_age_sec = float(self.adapter.input_age_sec)
            adapter_fresh = (self.adapter is not None and
                             time.monotonic()-self.adapter_received <= self.value('adapter_timeout_sec') and
                             stamp_ns(self.adapter.stamp)>0 and
                             -self.value('future_tolerance_sec') <= (now.nanoseconds-stamp_ns(self.adapter.stamp))*1e-9 <= self.value('adapter_timeout_sec') and
                             self.adapter.state == AdapterStatus.STATE_FRESH and
                             math.isfinite(msg.input_age_sec) and
                             0.0 <= msg.input_age_sec <= self.value('source_timeout_sec'))
            source_fresh = (time.monotonic()-self.received <= self.value('source_timeout_sec') and
                            msg.output_age_sec <= self.value('source_timeout_sec'))
            calibrated = bool(self.value('field_verified'))
            limit_p, limit_y = float(self.value('max_position_std_m')), float(self.value('max_yaw_std_rad'))
            if not source_fresh:
                reason = 'source_stale'
            elif not adapter_fresh:
                reason = 'adapter_evidence_missing_or_stale'
            elif not calibrated:
                reason = 'field_calibration_required'
            elif not (limit_p > 0 and limit_y > 0 and
                      self.value('max_measured_linear_mps') > 0 and
                      self.value('max_measured_angular_rps') > 0):
                reason = 'calibration_limits_missing'
            else:
                elapsed = max(0.0, (stamp_ns(self.last.header.stamp)-self.start_stamp)*1e-9)
                floor_p, floor_y = float(self.value('position_std_floor_m')), float(self.value('yaw_std_floor_rad'))
                p_var = max(float(self.last.pose.covariance[0]), float(self.last.pose.covariance[7]))
                y_var = float(self.last.pose.covariance[35])
                if self.value('covariance_calibrated') and all(math.isfinite(v) and v > 0 for v in (p_var, y_var)):
                    msg.position_std_m, msg.yaw_std_rad = math.sqrt(p_var), math.sqrt(y_var)
                elif floor_p > 0 and floor_y > 0 and all(
                        math.isfinite(float(self.value(k))) and self.value(k) > 0 for k in
                        ('position_drift_per_meter', 'yaw_drift_per_meter',
                         'position_drift_per_second', 'yaw_drift_per_second')):
                    msg.position_std_m = (floor_p+self.distance*self.value('position_drift_per_meter')+
                                          elapsed*self.value('position_drift_per_second'))
                    msg.yaw_std_rad = (floor_y+self.distance*self.value('yaw_drift_per_meter')+
                                      elapsed*self.value('yaw_drift_per_second'))
                msg.valid = (msg.position_std_m <= limit_p and msg.yaw_std_rad <= limit_y)
                msg.quality = math.exp(-(msg.position_std_m/limit_p)**2-(msg.yaw_std_rad/limit_y)**2) if msg.valid else 0.0
                reason = 'calibrated_evidence' if msg.valid else 'uncertainty_unknown_or_budget_exhausted'
        msg.reason = reason
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomQualityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
