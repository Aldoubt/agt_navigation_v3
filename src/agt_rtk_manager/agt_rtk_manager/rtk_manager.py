from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from agt_asensing_driver.msg import INSStatus
from agt_robot_interfaces.msg import RTKStatus


@dataclass
class _Inputs:
    fix: Optional[NavSatFix] = None
    ins: Optional[INSStatus] = None
    fix_rx_ns: int = 0
    ins_rx_ns: int = 0


class RTKManager(Node):
    """Convert raw INS/GNSS health into a quality-gated navigation status.

    This node deliberately does not publish odometry or TF. FAST-LIO2 remains
    the owner of continuous local odometry; RTK is an auxiliary global source.
    """

    def __init__(self) -> None:
        super().__init__('agt_rtk_manager')

        self.declare_parameter('fix_topic', '/ins/navsatfix')
        self.declare_parameter('status_topic', '/ins/status')
        self.declare_parameter('output_topic', '/agt/rtk/status')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('max_fix_age_sec', 1.0)
        self.declare_parameter('max_status_age_sec', 1.0)
        self.declare_parameter('min_satellites', 8)
        self.declare_parameter('fixed_max_position_std_m', 0.15)
        self.declare_parameter('float_max_position_std_m', 0.60)
        self.declare_parameter('max_heading_std_deg', 8.0)
        self.declare_parameter('allow_float_for_auxiliary', True)
        self.declare_parameter('map_origin_file', '')

        self.inputs = _Inputs()
        self._validate_origin_file()

        fix_topic = self.get_parameter('fix_topic').value
        status_topic = self.get_parameter('status_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 20)
        self.create_subscription(INSStatus, status_topic, self._on_ins, 20)
        self.pub = self.create_publisher(RTKStatus, output_topic, 10)

        rate = max(float(self.get_parameter('publish_rate_hz').value), 0.5)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'RTK manager: fix={fix_topic}, status={status_topic}, output={output_topic}'
        )

    def _validate_origin_file(self) -> None:
        path = str(self.get_parameter('map_origin_file').value).strip()
        if not path:
            self.get_logger().warn(
                'map_origin_file is empty; geographic-to-map alignment is disabled. '
                'This is valid for localization-only/runtime data recording.'
            )
            return
        expanded = Path(path).expanduser()
        if not expanded.exists():
            self.get_logger().error(f'map_origin_file does not exist: {expanded}')
        else:
            self.get_logger().info(f'map origin asset: {expanded}')

    def _on_fix(self, msg: NavSatFix) -> None:
        self.inputs.fix = msg
        self.inputs.fix_rx_ns = self.get_clock().now().nanoseconds

    def _on_ins(self, msg: INSStatus) -> None:
        self.inputs.ins = msg
        self.inputs.ins_rx_ns = self.get_clock().now().nanoseconds

    def _age_sec(self, rx_ns: int, now_ns: int) -> float:
        if rx_ns <= 0:
            return float('inf')
        return max(0.0, (now_ns - rx_ns) / 1e9)

    def _classify(self, now_ns: int) -> tuple[int, bool, str]:
        fix = self.inputs.fix
        ins = self.inputs.ins
        if fix is None or ins is None:
            return RTKStatus.QUALITY_UNKNOWN, False, 'waiting_for_fix_and_status'

        fix_age = self._age_sec(self.inputs.fix_rx_ns, now_ns)
        status_age = self._age_sec(self.inputs.ins_rx_ns, now_ns)
        if fix_age > float(self.get_parameter('max_fix_age_sec').value):
            return RTKStatus.QUALITY_INVALID, False, f'fix_stale:{fix_age:.2f}s'
        if status_age > float(self.get_parameter('max_status_age_sec').value):
            return RTKStatus.QUALITY_INVALID, False, f'ins_status_stale:{status_age:.2f}s'
        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return RTKStatus.QUALITY_INVALID, False, 'navsat_no_fix'

        min_sat = int(self.get_parameter('min_satellites').value)
        if int(ins.num_satellite) < min_sat:
            return RTKStatus.QUALITY_SINGLE, False, f'low_satellite_count:{ins.num_satellite}'

        position_std = float(ins.position_std)
        heading_std = float(ins.heading_std)
        heading_ok = heading_std <= float(self.get_parameter('max_heading_std_deg').value)

        if bool(ins.rtk_fixed):
            pos_ok = position_std <= float(
                self.get_parameter('fixed_max_position_std_m').value
            )
            usable = pos_ok and heading_ok
            reason = 'rtk_fixed' if usable else 'rtk_fixed_quality_rejected'
            return RTKStatus.QUALITY_FIXED, usable, reason

        float_ok = position_std <= float(
            self.get_parameter('float_max_position_std_m').value
        )
        allow_float = bool(self.get_parameter('allow_float_for_auxiliary').value)
        if float_ok:
            usable = allow_float and heading_ok
            reason = 'rtk_float_auxiliary' if usable else 'rtk_float_not_usable'
            return RTKStatus.QUALITY_FLOAT, usable, reason

        return RTKStatus.QUALITY_SINGLE, False, 'standalone_or_low_quality'

    def _publish(self) -> None:
        now = self.get_clock().now()
        quality, usable, reason = self._classify(now.nanoseconds)
        out = RTKStatus()
        out.stamp = now.to_msg()
        out.quality = quality
        out.usable = usable
        out.reason = reason

        if self.inputs.fix is not None:
            fix = self.inputs.fix
            out.navsat_status = int(fix.status.status)
            out.latitude = float(fix.latitude)
            out.longitude = float(fix.longitude)
            out.altitude = float(fix.altitude)

        if self.inputs.ins is not None:
            ins = self.inputs.ins
            out.rtk_fixed = bool(ins.rtk_fixed)
            out.ins_status = int(ins.ins_status)
            out.position_type = int(ins.position_type)
            out.heading_type = int(ins.heading_type)
            out.num_satellite = int(ins.num_satellite)
            out.position_std = float(ins.position_std)
            out.heading_std = float(ins.heading_std)

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RTKManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
