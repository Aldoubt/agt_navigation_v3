from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from agt_asensing_driver.msg import INSStatus
from agt_robot_interfaces.msg import RTKStatus


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class _Inputs:
    fix: Optional[NavSatFix] = None
    ins: Optional[INSStatus] = None
    fix_rx_ns: int = 0
    ins_rx_ns: int = 0


@dataclass(frozen=True)
class _MapOrigin:
    map_id: str
    map_version: str
    frame_id: str
    latitude: float
    longitude: float
    altitude: float
    map_yaw_from_enu_rad: float


def _geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float):
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + altitude_m) * cos_lat * cos_lon
    y = (n + altitude_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


def _geodetic_to_enu(fix: NavSatFix, origin: _MapOrigin):
    x, y, z = _geodetic_to_ecef(fix.latitude, fix.longitude, fix.altitude)
    x0, y0, z0 = _geodetic_to_ecef(
        origin.latitude, origin.longitude, origin.altitude)
    dx, dy, dz = x - x0, y - y0, z - z0

    lat0 = math.radians(origin.latitude)
    lon0 = math.radians(origin.longitude)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


class RTKManager(Node):
    """Quality-gate INS/GNSS and expose an optional map-frame position observation.

    This node deliberately does not publish odometry or TF. FAST-LIO2 remains
    the owner of continuous local odometry and the localization manager remains
    the only owner allowed to correct map->odom.
    """

    def __init__(self) -> None:
        super().__init__('agt_rtk_manager')

        self.declare_parameter('fix_topic', '/ins/navsatfix')
        self.declare_parameter('status_topic', '/ins/status')
        self.declare_parameter('output_topic', '/agt/rtk/status')
        self.declare_parameter('map_pose_topic', '/agt/rtk/map_pose')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('max_fix_age_sec', 1.0)
        self.declare_parameter('max_status_age_sec', 1.0)
        self.declare_parameter('min_satellites', 8)
        self.declare_parameter('fixed_max_position_std_m', 0.15)
        self.declare_parameter('float_max_position_std_m', 0.60)
        self.declare_parameter('max_heading_std_deg', 8.0)
        self.declare_parameter('allow_float_for_auxiliary', True)
        self.declare_parameter('publish_unfixed_map_pose', True)
        self.declare_parameter('map_origin_file', '')

        self.inputs = _Inputs()
        self.origin = self._load_origin_file()

        fix_topic = self.get_parameter('fix_topic').value
        status_topic = self.get_parameter('status_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 20)
        self.create_subscription(INSStatus, status_topic, self._on_ins, 20)
        self.pub = self.create_publisher(RTKStatus, output_topic, 10)
        self.map_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('map_pose_topic').value,
            10,
        )

        rate = max(float(self.get_parameter('publish_rate_hz').value), 0.5)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'RTK manager: fix={fix_topic}, status={status_topic}, output={output_topic}'
        )
        self.get_logger().info(
            'RTK policy: observation only; this node never publishes map->odom.')

    def _load_origin_file(self) -> Optional[_MapOrigin]:
        path_text = str(self.get_parameter('map_origin_file').value).strip()
        if not path_text:
            self.get_logger().warn(
                'map_origin_file is empty; WGS84->map observation is disabled.')
            return None

        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().error(f'map_origin_file does not exist: {path}')
            return None

        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            ref = data['geographic_reference']
            # heading_deg is defined as the 2D rotation applied to an ENU vector
            # to express it in map coordinates. Positive is counter-clockwise.
            yaw_deg = float(ref.get('map_yaw_from_enu_deg', ref.get('heading_deg', 0.0)))
            origin = _MapOrigin(
                map_id=str(data.get('map_id', '')),
                map_version=str(data.get('map_version', '')),
                frame_id=str(data.get('frame_id', 'map')),
                latitude=float(ref['latitude']),
                longitude=float(ref['longitude']),
                altitude=float(ref.get('altitude', 0.0)),
                map_yaw_from_enu_rad=math.radians(yaw_deg),
            )
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f'invalid map origin file {path}: {exc}')
            return None

        self.get_logger().info(
            f'map origin loaded: {origin.map_id}/{origin.map_version}, '
            f'frame={origin.frame_id}')
        return origin

    def _on_fix(self, msg: NavSatFix) -> None:
        self.inputs.fix = msg
        self.inputs.fix_rx_ns = self.get_clock().now().nanoseconds

    def _on_ins(self, msg: INSStatus) -> None:
        self.inputs.ins = msg
        self.inputs.ins_rx_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _age_sec(rx_ns: int, now_ns: int) -> float:
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
                self.get_parameter('fixed_max_position_std_m').value)
            usable = pos_ok and heading_ok
            reason = 'rtk_fixed' if usable else 'rtk_fixed_quality_rejected'
            return RTKStatus.QUALITY_FIXED, usable, reason

        float_ok = position_std <= float(
            self.get_parameter('float_max_position_std_m').value)
        allow_float = bool(self.get_parameter('allow_float_for_auxiliary').value)
        if float_ok:
            usable = allow_float and heading_ok
            reason = 'rtk_float_auxiliary' if usable else 'rtk_float_not_usable'
            return RTKStatus.QUALITY_FLOAT, usable, reason

        return RTKStatus.QUALITY_SINGLE, False, 'standalone_or_low_quality'

    def _publish_map_pose(self, quality: int, position_std: float) -> None:
        if self.origin is None or self.inputs.fix is None:
            return
        if quality in (RTKStatus.QUALITY_UNKNOWN, RTKStatus.QUALITY_INVALID):
            return
        if quality != RTKStatus.QUALITY_FIXED and not bool(
                self.get_parameter('publish_unfixed_map_pose').value):
            return

        east, north, up = _geodetic_to_enu(self.inputs.fix, self.origin)
        c = math.cos(self.origin.map_yaw_from_enu_rad)
        s = math.sin(self.origin.map_yaw_from_enu_rad)
        map_x = c * east - s * north
        map_y = s * east + c * north

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.inputs.fix.header.stamp
        msg.header.frame_id = self.origin.frame_id
        msg.pose.pose.position.x = map_x
        msg.pose.pose.position.y = map_y
        msg.pose.pose.position.z = up
        # GNSS provides a position observation here. Orientation is intentionally
        # unobserved and assigned very large covariance.
        msg.pose.pose.orientation.w = 1.0
        sigma = max(float(position_std), 0.01)
        msg.pose.covariance[0] = sigma * sigma
        msg.pose.covariance[7] = sigma * sigma
        msg.pose.covariance[14] = sigma * sigma
        msg.pose.covariance[21] = 1.0e6
        msg.pose.covariance[28] = 1.0e6
        msg.pose.covariance[35] = 1.0e6
        self.map_pose_pub.publish(msg)

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
        self._publish_map_pose(quality, float(out.position_std))


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
