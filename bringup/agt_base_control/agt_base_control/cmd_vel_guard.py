from __future__ import annotations

import math
import json
import time
import signal

import rclpy
from agt_robot_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool


class CmdVelGuard(Node):
    """Clamp, slew-limit and refresh velocity commands at a fixed rate.

    Nav2's velocity smoother publishes /cmd_vel_smoothed. This guard is the
    only software path expected to publish /mux/cmd_vel to the Bunker driver.
    It does not publish odometry or TF.
    """

    def __init__(self) -> None:
        super().__init__('agt_cmd_vel_guard')
        self.declare_parameter('require_chassis_feedback', False)
        self.chassis_at=0.;self.chassis_ok=False
        if self.get_parameter('require_chassis_feedback').value:
            from bunker_msgs.msg import BunkerStatus
            self.create_subscription(BunkerStatus,'/bunker_status',self._on_chassis,10)
        self.declare_parameter('require_deadman', False)
        self.declare_parameter('allow_navigation', False)
        self.deadman_at = 0.0
        self.deadman_held = False
        self.create_subscription(Bool, '/agt/hmi/deadman', self._on_deadman, 1)
        self.declare_parameter('kinematic_profile', 'bunker')
        profile = str(self.get_parameter('kinematic_profile').value)
        if profile not in ('bunker', 'fwmini'):
            raise ValueError('kinematic_profile must be bunker or fwmini')
        self.fwmini = profile == 'fwmini'
        self.declare_parameter('max_linear_y', 0.03)
        self.declare_parameter('input_topic', '/cmd_vel_smoothed')
        self.declare_parameter('manual_input_topic', '/agt/hmi/cmd_vel')
        self.declare_parameter('control_mode_topic', '/agt/control/mode')
        self.declare_parameter('default_control_mode', 'navigation')
        self.declare_parameter('output_topic', '/mux/cmd_vel')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('localization_status_topic', '/agt/localization/status')
        self.declare_parameter('require_localization_status', True)
        self.declare_parameter('localization_status_timeout_sec', 0.75)
        self.declare_parameter('allow_degraded_localization', False)
        self.declare_parameter('max_linear_x', 0.55)
        self.declare_parameter('max_reverse_x', 0.20)
        self.declare_parameter('max_angular_z', 0.65)
        self.declare_parameter('max_linear_accel', 0.45)
        self.declare_parameter('max_linear_decel', 0.80)
        self.declare_parameter('max_angular_accel', 0.80)

        self.target = Twist()
        self.manual_target = Twist()
        self.output = Twist()
        self.last_rx_ns = 0
        self.last_manual_rx_ns = 0
        self.last_tick_ns = self.get_clock().now().nanoseconds
        self.last_localization_rx_ns = 0
        self.localization_status = None
        self._last_gate_reason = None

        input_topic = self.get_parameter('input_topic').value
        manual_input_topic = self.get_parameter('manual_input_topic').value
        control_mode_topic = self.get_parameter('control_mode_topic').value
        default_mode = str(self.get_parameter('default_control_mode').value).strip().lower()
        self.control_mode = default_mode if default_mode in ('navigation', 'manual') else 'navigation'
        output_topic = self.get_parameter('output_topic').value
        self.create_subscription(Twist, input_topic, self._on_cmd, 20)
        self.create_subscription(Twist, manual_input_topic, self._on_manual_cmd, 20)
        self.create_subscription(String, control_mode_topic, self._on_control_mode, 10)
        self.create_subscription(
            LocalizationStatus,
            str(self.get_parameter('localization_status_topic').value),
            self._on_localization_status,
            20,
        )
        if self.fwmini:
            if output_topic != '/fwmini/cmd_vel_guarded':
                raise ValueError('FW-mini guard output must be /fwmini/cmd_vel_guarded')
            self.pub = self.create_publisher(TwistStamped, output_topic, 1)
            self.fw_status = None
            self.fw_status_at = 0.0
            self.fw_epoch = -1
            self.fw_blocked_epoch = -1
            self.fw_target = Twist()
            self.fw_rx_at = 0.0
            self.fw_opened_at = 0.0
            self.fw_last_tick = time.monotonic()
            self.fw_fault = self.create_publisher(String, '/fwmini/guard_fault', 10)
            self.create_subscription(String, '/fwmini/control_status', self._fw_on_status,
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        else:
            self.pub = self.create_publisher(Twist, output_topic, 20)

        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'cmd_vel guard: nav={input_topic}, manual={manual_input_topic} -> {output_topic} '
            f'@ {rate:.1f} Hz; default mode={self.control_mode}; '
            'motion is fail-closed on LocalizationStatus'
        )

    def _on_cmd(self, msg: Twist) -> None:
        if self.fwmini:
            if self.control_mode == 'navigation':
                if bool(self.get_parameter('allow_navigation').value): self._fw_command(msg)
                else: self._fw_reject('autonomous_source_not_enabled')
            return
        if self.control_mode != 'navigation':
            return
        if not self._chassis_ready() or not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)) or any(v!=0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
            self.target=Twist();self.manual_target=Twist();self.output=Twist();self.last_rx_ns=self.last_manual_rx_ns=0;self.pub.publish(Twist());return
        now_ns = self.get_clock().now().nanoseconds
        localization_ok, _ = self._localization_allows_motion(now_ns)
        if not localization_ok:
            # Never cache motion intent while localization is unsafe. This
            # guarantees that reopening the gate cannot replay a command that
            # Nav2 produced during LOST / RELOCALIZING / stale-status time.
            self.target = Twist()
            self.last_rx_ns = 0
            return
        self.target = msg
        self.last_rx_ns = now_ns

    def _on_manual_cmd(self, msg: Twist) -> None:
        if self.fwmini:
            if self.control_mode == 'manual': self._fw_command(msg)
            return
        if self.control_mode != 'manual':
            return
        if not self._chassis_ready() or not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)) or any(v!=0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
            self.target=Twist();self.manual_target=Twist();self.output=Twist();self.last_rx_ns=self.last_manual_rx_ns=0;self.pub.publish(Twist());return
        now_ns = self.get_clock().now().nanoseconds
        localization_ok, _ = self._localization_allows_motion(now_ns)
        if not localization_ok:
            self.manual_target = Twist()
            self.last_manual_rx_ns = 0
            return
        self.manual_target = msg
        self.last_manual_rx_ns = now_ns

    def _on_control_mode(self, msg: String) -> None:
        requested = str(msg.data).strip().lower()
        if requested not in ('navigation', 'manual'):
            self.get_logger().warning(
                f'ignoring unsupported control mode {requested!r}; expected navigation or manual')
            return
        if requested == self.control_mode:
            return
        if self.fwmini:
            self._fw_reject('source_handoff_requires_reenable')
            self.control_mode = requested
            return
        # A source handoff is always a hard stop. Neither Nav2 nor the HMI may
        # replay intent produced before the operator made the handoff.
        self.control_mode = requested
        self.target = Twist()
        self.manual_target = Twist()
        self.last_rx_ns = 0
        self.last_manual_rx_ns = 0
        self.output = Twist()
        self.pub.publish(self.output)
        self.get_logger().info(f'control mode changed to {self.control_mode}; fresh command required')

    def _on_localization_status(self, msg: LocalizationStatus) -> None:
        if self.fwmini:
            self.localization_status = msg
            self.last_localization_rx_ns = self.get_clock().now().nanoseconds
            if bool(self.get_parameter('require_localization_status').value):
                allowed, reason = self._localization_allows_motion(self.last_localization_rx_ns)
                if not allowed: self._fw_reject(reason)
            return
        self.localization_status = msg
        self.last_localization_rx_ns = self.get_clock().now().nanoseconds
        allowed, reason = self._localization_allows_motion(self.last_localization_rx_ns)
        self._report_gate(reason, allowed)
        if not allowed:
            # A localization fault is a hard stop, not a slew-limited stop. Clear
            # the cached upstream command as well so recovery requires a fresh
            # Nav2 command after localization becomes valid again.
            self.target = Twist()
            self.manual_target = Twist()
            self.last_rx_ns = 0
            self.last_manual_rx_ns = 0
            self.output = Twist()
            self.pub.publish(self.output)

    def _localization_allows_motion(self, now_ns: int) -> tuple[bool, str]:
        if not bool(self.get_parameter('require_localization_status').value):
            return True, 'localization_gate_disabled'
        if self.localization_status is None or self.last_localization_rx_ns <= 0:
            return False, 'localization_status_missing'

        max_age = float(self.get_parameter('localization_status_timeout_sec').value)
        age = max(0.0, (now_ns - self.last_localization_rx_ns) / 1e9)
        if age > max_age:
            return False, f'localization_status_stale:{age:.2f}s'

        status = self.localization_status
        valid_states = {int(LocalizationStatus.STATE_LOCALIZED)}
        if bool(self.get_parameter('allow_degraded_localization').value):
            valid_states.add(int(LocalizationStatus.STATE_DEGRADED))
        if int(status.state) not in valid_states:
            return False, f'localization_state:{int(status.state)}'
        if not bool(status.local_odom_fresh):
            return False, 'local_odom_not_fresh'
        if not bool(status.global_correction_valid):
            return False, 'global_correction_invalid'
        return True, 'localized'

    def _report_gate(self, reason: str, allowed: bool) -> None:
        # Dynamic details such as stale age should not turn a 50 Hz timer into
        # a 50 Hz warning stream. Log only when the gate class changes.
        reason_class = reason.split(':', 1)[0]
        key = (allowed, reason_class)
        if key == self._last_gate_reason:
            return
        self._last_gate_reason = key
        if allowed:
            self.get_logger().info(f'motion gate OPEN: {reason}')
        else:
            self.get_logger().warning(f'motion gate CLOSED: {reason}')

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def _slew(current: float, target: float, limit: float, dt: float) -> float:
        delta = target - current
        if math.isclose(delta, 0.0, abs_tol=1e-9):
            return target
        step = max(limit * dt, 0.0)
        return current + max(-step, min(step, delta))

    def _tick(self) -> None:
        if self.fwmini:
            self._fw_tick()
            return
        now_ns = self.get_clock().now().nanoseconds
        dt = max((now_ns - self.last_tick_ns) / 1e9, 1e-4)
        self.last_tick_ns = now_ns

        timeout = float(self.get_parameter('command_timeout_sec').value)
        if self.control_mode == 'manual':
            active_target = self.manual_target
            active_rx_ns = self.last_manual_rx_ns
        else:
            active_target = self.target
            active_rx_ns = self.last_rx_ns
        stale = active_rx_ns <= 0 or (now_ns - active_rx_ns) / 1e9 > timeout

        localization_ok, localization_reason = self._localization_allows_motion(now_ns)
        self._report_gate(localization_reason, localization_ok)

        if stale or not localization_ok or not self._deadman_ok() or not self._chassis_ready():
            self.target=Twist();self.manual_target=Twist();self.last_rx_ns=self.last_manual_rx_ns=0
            # Stale upstream motion command is a fault. Do not continue ramping an
            # old command. Localization failure is also an immediate hard stop.
            self.output = Twist()
            self.pub.publish(self.output)
            return

        target_x = self._clamp(
            float(active_target.linear.x),
            -float(self.get_parameter('max_reverse_x').value),
            float(self.get_parameter('max_linear_x').value),
        )
        target_w = self._clamp(
            float(active_target.angular.z),
            -float(self.get_parameter('max_angular_z').value),
            float(self.get_parameter('max_angular_z').value),
        )

        accel_x = float(self.get_parameter('max_linear_accel').value)
        decel_x = float(self.get_parameter('max_linear_decel').value)
        accel_w = float(self.get_parameter('max_angular_accel').value)

        linear_limit = decel_x if abs(target_x) < abs(self.output.linear.x) else accel_x
        self.output.linear.x = self._slew(
            self.output.linear.x, target_x, linear_limit, dt
        )
        self.output.angular.z = self._slew(
            self.output.angular.z, target_w, accel_w, dt
        )

        # Bunker is non-holonomic; unused axes are always forced to zero.
        self.output.linear.y = 0.0
        self.output.linear.z = 0.0
        self.output.angular.x = 0.0
        self.output.angular.y = 0.0
        self.pub.publish(self.output)

    def _on_chassis(self,msg):
        self.chassis_at=time.monotonic()
        self.chassis_ok=msg.vehicle_state==0 and msg.control_mode==1 and msg.error_code==0
        if not self.chassis_ok:
            self.target=Twist();self.manual_target=Twist();self.output=Twist();self.last_rx_ns=self.last_manual_rx_ns=0
            self.pub.publish(Twist())

    def _chassis_ready(self):
        return (not self.get_parameter('require_chassis_feedback').value or
                (self.chassis_ok and time.monotonic()-self.chassis_at<.3))

    def _deadman_ok(self):
        return (self.control_mode != 'manual' or not bool(self.get_parameter('require_deadman').value)
                or (self.deadman_held and time.monotonic()-self.deadman_at < .2))

    def _on_deadman(self, msg):
        self.deadman_held = msg.data
        self.deadman_at = time.monotonic()
        if not msg.data and bool(self.get_parameter('require_deadman').value) and self.control_mode == 'manual':
            self.target = Twist(); self.manual_target = Twist(); self.output = Twist()
            self.last_rx_ns = self.last_manual_rx_ns = 0
            if self.fwmini:
                self.fw_target = Twist(); self.fw_rx_at = 0.
                self._fw_publish(Twist())
            else: self.pub.publish(Twist())

    def _fw_on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            epoch = int(status['epoch'])
            if status['state'] not in ('DISABLED', 'STOPPING', 'SWITCHING', 'READY', 'ACTIVE', 'FAULT'):
                raise ValueError('unknown state')
            if epoch != self.fw_epoch:
                self.fw_target = Twist()
                self.fw_rx_at = 0.0
                self.output = Twist()
                self.fw_opened_at = time.monotonic()
                self.fw_epoch = epoch
                self._fw_publish(Twist())
            self.fw_status = status
            self.fw_status_at = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self._fw_reject('invalid_adapter_status')

    def _fw_publish(self, velocity: Twist) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fwmini_epoch_' + str(self.fw_epoch)
        msg.twist = velocity
        self.pub.publish(msg)

    def _fw_reject(self, reason: str) -> None:
        self.fw_target = Twist()
        self.fw_rx_at = 0.0
        self.output = Twist()
        self._fw_publish(Twist())
        if self.fw_blocked_epoch != self.fw_epoch:
            self.fw_fault.publish(String(data=reason))
            self.fw_blocked_epoch = self.fw_epoch
            self.get_logger().warning('FW-mini command rejected: ' + reason)

    def _fw_allowed(self) -> bool:
        return (self.fw_status is not None and self.fw_status['state'] == 'ACTIVE'
                and self.fw_status.get('motion_allowed') is True
                and time.monotonic() - self.fw_status_at < 0.2
                and self.fw_blocked_epoch != self.fw_epoch
                and (self.control_mode == 'manual' or bool(self.get_parameter('allow_navigation').value))
                and self._deadman_ok()
                and self._localization_allows_motion(self.get_clock().now().nanoseconds)[0])

    def _fw_command(self, msg: Twist) -> None:
        if not self._fw_allowed():
            return  # Never cache commands while disabled or switching.
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if not all(math.isfinite(v) for v in values):
            self._fw_reject('nonfinite_command')
            return
        gear = self.fw_status['target_gear']
        if (any(v != 0.0 for v in (msg.linear.z, msg.angular.x, msg.angular.y))
                or gear not in (6, 8) or (gear == 6 and msg.linear.y != 0.0)
                or (gear == 8 and msg.angular.z != 0.0)):
            self._fw_reject('unsupported_velocity_axis')
            return
        self.fw_target = msg
        self.fw_rx_at = time.monotonic()

    def _fw_tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now-self.fw_last_tick, 0.0001), 0.05)
        self.fw_last_tick = now
        if not self._fw_allowed():
            self.fw_target = Twist()
            self.fw_rx_at = 0.0
            self.output = Twist()
            self._fw_publish(self.output)
            if self.fw_status and self.fw_status['state'] == 'ACTIVE' and now-self.fw_status_at >= 0.2:
                self._fw_reject('adapter_status_stale')
            return
        timeout = float(self.get_parameter('command_timeout_sec').value)
        if now-(self.fw_rx_at or self.fw_opened_at) >= timeout:
            self._fw_reject('upstream_command_timeout')
            return
        target = self.fw_target
        desired = (
            self._clamp(target.linear.x, -float(self.get_parameter('max_reverse_x').value), float(self.get_parameter('max_linear_x').value)),
            self._clamp(target.linear.y, -float(self.get_parameter('max_linear_y').value), float(self.get_parameter('max_linear_y').value)),
            self._clamp(target.angular.z, -float(self.get_parameter('max_angular_z').value), float(self.get_parameter('max_angular_z').value)),
        )
        for obj, attr, value, angular in ((self.output.linear, 'x', desired[0], False),
                                        (self.output.linear, 'y', desired[1], False),
                                        (self.output.angular, 'z', desired[2], True)):
            current = getattr(obj, attr)
            limit = float(self.get_parameter('max_angular_accel' if angular else
                          ('max_linear_decel' if abs(value) < abs(current) else 'max_linear_accel')).value)
            setattr(obj, attr, self._slew(current, value, limit, dt))
        self._fw_publish(self.output)


def main(args=None) -> None:
    # Keep the context valid until final zero commands have been published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    running = [True]
    def stop_signal(_number, _frame):
        running[0] = False
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    node = CmdVelGuard()
    try:
        while running[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        for _ in range(3):
            if not rclpy.ok():
                break
            if node.fwmini:
                node._fw_publish(zero)
            else:
                node.pub.publish(zero)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
