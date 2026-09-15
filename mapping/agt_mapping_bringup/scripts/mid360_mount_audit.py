#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_sec(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9


def _rate_hz(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 1.0e-9 else 0.0


def _gap_summary(stamps: list[float]) -> dict:
    if len(stamps) < 2:
        return {
            'mean_sec': 0.0,
            'max_sec': 0.0,
            'p95_sec': 0.0,
            'non_monotonic_count': 0,
        }
    gaps = [b - a for a, b in zip(stamps[:-1], stamps[1:])]
    positive = [g for g in gaps if g >= 0.0]
    ordered = sorted(positive)
    if ordered:
        idx = min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)
        p95 = ordered[max(0, idx)]
    else:
        p95 = 0.0
    return {
        'mean_sec': _mean(positive),
        'max_sec': max(positive, default=0.0),
        'p95_sec': p95,
        'non_monotonic_count': sum(1 for g in gaps if g < 0.0),
    }


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return statistics.fmean(values) if values else 0.0


def _std(values: Iterable[float]) -> float:
    values = list(values)
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def _yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1.0e-12:
        return 0.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def _angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class VectorStats:
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    z: list[float] = field(default_factory=list)

    def add(self, x: float, y: float, z: float) -> None:
        if all(math.isfinite(v) for v in (x, y, z)):
            self.x.append(float(x))
            self.y.append(float(y))
            self.z.append(float(z))

    def summary(self) -> dict:
        def axis(values: list[float]) -> dict:
            return {
                'mean': _mean(values),
                'std': _std(values),
                'peak_abs': max((abs(v) for v in values), default=0.0),
            }
        return {'x': axis(self.x), 'y': axis(self.y), 'z': axis(self.z)}


class Mid360MountAudit(Node):
    """Read-only audit node for MID360 mount/LIO replay and field sessions."""

    def __init__(self) -> None:
        super().__init__('mid360_mount_audit')
        p = self.declare_parameter
        p('imu_topic', '/agt/sensors/imu/data')
        p('odom_topic', '/agt/odometry/local')
        p('raw_cloud_topic', '/agt/livox/points')
        p('obstacle_cloud_topic', '/agt/navigation/points_obstacles')
        p('base_frame', 'base_link')
        p('lidar_frame', 'livox_frame')
        p('duration_sec', 60.0)
        p('report_file', '~/.ros/agt_mount_audit/mid360_mount_audit.yaml')
        p('tf_sample_rate_hz', 2.0)

        self.imu_stamps: list[float] = []
        self.odom_stamps: list[float] = []
        self.raw_cloud_stamps: list[float] = []
        self.obstacle_cloud_stamps: list[float] = []
        self.accel = VectorStats()
        self.gyro = VectorStats()
        self.accel_norm: list[float] = []
        self.gyro_norm: list[float] = []

        self.odom_first = None
        self.odom_last = None
        self.odom_prev = None
        self.max_odom_step_m = 0.0
        self.max_odom_yaw_step_deg = 0.0

        self.raw_points = 0
        self.obstacle_points = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_success = 0
        self.tf_failure = 0
        self.tf_first = None
        self.tf_last = None
        self.tf_max_translation_delta_m = 0.0
        self.tf_max_yaw_delta_deg = 0.0

        self.first_sensor_stamp = None
        self.latest_sensor_stamp = None
        self.finalized = False

        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 100)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('raw_cloud_topic').value),
            self.on_raw_cloud, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('obstacle_cloud_topic').value),
            self.on_obstacle_cloud, qos_profile_sensor_data)

        tf_rate = max(0.2, float(self.get_parameter('tf_sample_rate_hz').value))
        self.create_timer(1.0 / tf_rate, self.sample_tf)
        self.create_timer(0.5, self.maybe_finalize)

        self.get_logger().info(
            'MID360 mount audit started; report='
            f"{os.path.expanduser(str(self.get_parameter('report_file').value))}")

    def _observe_stamp(self, stamp: float) -> None:
        if stamp <= 0.0 or not math.isfinite(stamp):
            return
        if self.first_sensor_stamp is None:
            self.first_sensor_stamp = stamp
        self.latest_sensor_stamp = max(stamp, self.latest_sensor_stamp or stamp)

    def on_imu(self, msg: Imu) -> None:
        if self.finalized:
            return
        stamp = _stamp_sec(msg)
        self.imu_stamps.append(stamp)
        self._observe_stamp(stamp)
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.accel.add(a.x, a.y, a.z)
        self.gyro.add(g.x, g.y, g.z)
        an = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
        gn = math.sqrt(g.x*g.x + g.y*g.y + g.z*g.z)
        if math.isfinite(an):
            self.accel_norm.append(an)
        if math.isfinite(gn):
            self.gyro_norm.append(gn)

    def on_odom(self, msg: Odometry) -> None:
        if self.finalized:
            return
        stamp = _stamp_sec(msg)
        self.odom_stamps.append(stamp)
        self._observe_stamp(stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = {
            'stamp': stamp,
            'x': float(p.x), 'y': float(p.y), 'z': float(p.z),
            'qx': float(q.x), 'qy': float(q.y), 'qz': float(q.z), 'qw': float(q.w),
            'yaw': _yaw_from_xyzw(q.x, q.y, q.z, q.w),
        }
        if self.odom_first is None:
            self.odom_first = pose
        if self.odom_prev is not None:
            dx = pose['x'] - self.odom_prev['x']
            dy = pose['y'] - self.odom_prev['y']
            dz = pose['z'] - self.odom_prev['z']
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            yaw_step = abs(math.degrees(_angle_wrap(pose['yaw'] - self.odom_prev['yaw'])))
            self.max_odom_step_m = max(self.max_odom_step_m, step)
            self.max_odom_yaw_step_deg = max(self.max_odom_yaw_step_deg, yaw_step)
        self.odom_prev = pose
        self.odom_last = pose

    def on_raw_cloud(self, msg: PointCloud2) -> None:
        if self.finalized:
            return
        stamp = _stamp_sec(msg)
        self.raw_cloud_stamps.append(stamp)
        self._observe_stamp(stamp)
        self.raw_points += int(msg.width) * int(msg.height)

    def on_obstacle_cloud(self, msg: PointCloud2) -> None:
        if self.finalized:
            return
        stamp = _stamp_sec(msg)
        self.obstacle_cloud_stamps.append(stamp)
        self._observe_stamp(stamp)
        self.obstacle_points += int(msg.width) * int(msg.height)

    @staticmethod
    def _tf_pose(transform) -> dict:
        tr = transform.translation
        qr = transform.rotation
        return {
            'x': float(tr.x), 'y': float(tr.y), 'z': float(tr.z),
            'qx': float(qr.x), 'qy': float(qr.y), 'qz': float(qr.z), 'qw': float(qr.w),
            'yaw': _yaw_from_xyzw(qr.x, qr.y, qr.z, qr.w),
        }

    def sample_tf(self) -> None:
        if self.finalized:
            return
        base = str(self.get_parameter('base_frame').value)
        lidar = str(self.get_parameter('lidar_frame').value)
        try:
            stamped = self.tf_buffer.lookup_transform(base, lidar, Time())
            pose = self._tf_pose(stamped.transform)
            self.tf_success += 1
            if self.tf_first is None:
                self.tf_first = pose
            if self.tf_last is not None:
                dx = pose['x'] - self.tf_last['x']
                dy = pose['y'] - self.tf_last['y']
                dz = pose['z'] - self.tf_last['z']
                self.tf_max_translation_delta_m = max(
                    self.tf_max_translation_delta_m,
                    math.sqrt(dx*dx + dy*dy + dz*dz))
                self.tf_max_yaw_delta_deg = max(
                    self.tf_max_yaw_delta_deg,
                    abs(math.degrees(_angle_wrap(pose['yaw'] - self.tf_last['yaw']))))
            self.tf_last = pose
        except TransformException:
            self.tf_failure += 1

    def maybe_finalize(self) -> None:
        if self.finalized:
            return
        duration = float(self.get_parameter('duration_sec').value)
        if duration <= 0.0:
            return
        if self.first_sensor_stamp is None or self.latest_sensor_stamp is None:
            return
        if self.latest_sensor_stamp - self.first_sensor_stamp >= duration:
            self.write_report(final_reason='duration_reached')

    def _odom_summary(self) -> dict:
        summary = {
            'samples': len(self.odom_stamps),
            'rate_hz': _rate_hz(self.odom_stamps),
            'timestamp_gaps': _gap_summary(self.odom_stamps),
            'max_translation_step_m': self.max_odom_step_m,
            'max_yaw_step_deg': self.max_odom_yaw_step_deg,
        }
        if self.odom_first is None or self.odom_last is None:
            return summary
        dx = self.odom_last['x'] - self.odom_first['x']
        dy = self.odom_last['y'] - self.odom_first['y']
        dz = self.odom_last['z'] - self.odom_first['z']
        summary.update({
            'start_xyz': [
                self.odom_first['x'], self.odom_first['y'], self.odom_first['z']],
            'end_xyz': [
                self.odom_last['x'], self.odom_last['y'], self.odom_last['z']],
            'delta_xyz': [dx, dy, dz],
            'start_to_end_translation_m': math.sqrt(dx*dx + dy*dy + dz*dz),
            'start_to_end_yaw_deg': math.degrees(
                _angle_wrap(self.odom_last['yaw'] - self.odom_first['yaw'])),
            'z_delta_m': dz,
        })
        return summary

    def build_report(self, final_reason: str) -> dict:
        raw_rate = _rate_hz(self.raw_cloud_stamps)
        obstacle_rate = _rate_hz(self.obstacle_cloud_stamps)
        return {
            'format_version': 1,
            'result': 'OBSERVATION',
            'final_reason': final_reason,
            'duration_sec': (
                (self.latest_sensor_stamp - self.first_sensor_stamp)
                if self.first_sensor_stamp is not None and self.latest_sensor_stamp is not None
                else 0.0),
            'topics': {
                'imu': str(self.get_parameter('imu_topic').value),
                'odom': str(self.get_parameter('odom_topic').value),
                'raw_cloud': str(self.get_parameter('raw_cloud_topic').value),
                'obstacle_cloud': str(self.get_parameter('obstacle_cloud_topic').value),
            },
            'imu': {
                'samples': len(self.imu_stamps),
                'rate_hz': _rate_hz(self.imu_stamps),
                'timestamp_gaps': _gap_summary(self.imu_stamps),
                'acceleration_xyz': self.accel.summary(),
                'angular_velocity_xyz': self.gyro.summary(),
                'accel_norm_mean': _mean(self.accel_norm),
                'accel_norm_std': _std(self.accel_norm),
                'gyro_norm_mean_rps': _mean(self.gyro_norm),
                'gyro_norm_peak_rps': max(self.gyro_norm, default=0.0),
            },
            'odometry': self._odom_summary(),
            'pointcloud': {
                'raw_clouds': len(self.raw_cloud_stamps),
                'raw_rate_hz': raw_rate,
                'raw_timestamp_gaps': _gap_summary(self.raw_cloud_stamps),
                'raw_points': self.raw_points,
                'obstacle_clouds': len(self.obstacle_cloud_stamps),
                'obstacle_rate_hz': obstacle_rate,
                'obstacle_timestamp_gaps': _gap_summary(self.obstacle_cloud_stamps),
                'obstacle_points': self.obstacle_points,
                'obstacle_points_over_raw_points': (
                    float(self.obstacle_points) / float(self.raw_points)
                    if self.raw_points else 0.0),
            },
            'mount_tf': {
                'base_frame': str(self.get_parameter('base_frame').value),
                'lidar_frame': str(self.get_parameter('lidar_frame').value),
                'lookup_success': self.tf_success,
                'lookup_failure': self.tf_failure,
                'first': self.tf_first,
                'last': self.tf_last,
                'max_translation_step_m': self.tf_max_translation_delta_m,
                'max_yaw_step_deg': self.tf_max_yaw_delta_deg,
            },
            'interpretation_note': (
                'This report measures the software-observed TF/LIO replay. '
                'A static robot_description TF cannot prove physical damping-mount rigidity; '
                'vehicle mechanical A/B remains required for that question.'
            ),
        }

    def write_report(self, final_reason: str = 'shutdown') -> None:
        if self.finalized:
            return
        path = Path(os.path.expanduser(str(self.get_parameter('report_file').value)))
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = self.build_report(final_reason)
        path.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
            encoding='utf-8')
        self.finalized = True
        self.get_logger().info(f'MID360 mount audit report written: {path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mid360MountAudit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
