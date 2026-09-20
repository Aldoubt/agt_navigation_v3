#!/usr/bin/env python3
"""Verify the base_footprint static offset and observed in-place rotation center."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrapped_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def evaluate_center(profile: dict, static_pose: dict | None, samples: list[dict]) -> dict:
    failures: list[str] = []
    static_result = {'status': 'FAIL', 'pose': static_pose, 'reasons': []}
    if static_pose is None:
        static_result['reasons'].append('base_footprint_to_base_link_not_observed')
    else:
        xy_tolerance = float(profile['static_xy_tolerance_m'])
        z_expected = float(profile['static_z_expected_m'])
        z_tolerance = float(profile['static_z_tolerance_m'])
        angle_tolerance = float(profile['static_angle_tolerance_rad'])
        if abs(static_pose['x']) > xy_tolerance or abs(static_pose['y']) > xy_tolerance:
            static_result['reasons'].append('static_xy_offset_out_of_tolerance')
        if abs(static_pose['z'] - z_expected) > z_tolerance:
            static_result['reasons'].append('static_z_offset_out_of_tolerance')
        if any(abs(static_pose[name]) > angle_tolerance for name in ('roll', 'pitch', 'yaw')):
            static_result['reasons'].append('static_rotation_out_of_tolerance')
        if not static_result['reasons']:
            static_result['status'] = 'PASS'
    if static_result['reasons']:
        failures.extend(static_result['reasons'])

    path_length = 0.0
    yaw_travel = 0.0
    max_excursion = 0.0
    if samples:
        origin_x, origin_y = samples[0]['x'], samples[0]['y']
        for previous, current in zip(samples, samples[1:]):
            path_length += math.hypot(current['x'] - previous['x'], current['y'] - previous['y'])
            yaw_travel += abs(wrapped_delta(current['yaw'], previous['yaw']))
            max_excursion = max(
                max_excursion,
                math.hypot(current['x'] - origin_x, current['y'] - origin_y),
            )
    effective_radius = path_length / yaw_travel if yaw_travel > 1e-9 else None
    motion_reasons = []
    if len(samples) < int(profile['minimum_motion_samples']):
        motion_reasons.append('insufficient_motion_samples')
    if yaw_travel < float(profile['minimum_observed_yaw_rad']):
        motion_reasons.append('insufficient_observed_yaw')
    if max_excursion > float(profile['maximum_planar_excursion_m']):
        motion_reasons.append('planar_excursion_exceeded')
    if effective_radius is None or effective_radius > float(profile['maximum_effective_radius_m']):
        motion_reasons.append('effective_rotation_radius_exceeded')
    failures.extend(motion_reasons)

    return {
        'verdict': 'PASS' if not failures else 'FAIL',
        'failures': failures,
        'static_transform': static_result,
        'motion': {
            'status': 'PASS' if not motion_reasons else 'FAIL',
            'sample_count': len(samples),
            'yaw_travel_rad': yaw_travel,
            'path_length_m': path_length,
            'max_planar_excursion_m': max_excursion,
            'effective_radius_m': effective_radius,
            'reasons': motion_reasons,
        },
    }


class CenterAudit(Node):
    def __init__(self, profile: dict) -> None:
        super().__init__('agt_base_footprint_center_audit')
        self.profile = profile
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.last_command_time = 0.0
        self.command_is_rotation = False
        self.samples: list[dict] = []
        self.last_stamp: tuple[int, int] | None = None
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_sub = self.create_subscription(
            Twist, str(profile['command_topic']), self._on_command, qos)

    def _on_command(self, message: Twist) -> None:
        linear_speed = math.hypot(message.linear.x, message.linear.y)
        self.command_is_rotation = (
            abs(message.angular.z) >= float(self.profile['minimum_command_yaw_rate_radps'])
            and linear_speed <= float(self.profile['maximum_command_linear_speed_mps'])
        )
        self.last_command_time = time.monotonic()

    def read_static_pose(self) -> dict | None:
        try:
            transform = self.buffer.lookup_transform(
                'base_footprint', 'base_link', Time(), timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(
            rotation.x, rotation.y, rotation.z, rotation.w)
        return {
            'x': translation.x,
            'y': translation.y,
            'z': translation.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }

    def sample_motion(self) -> None:
        fresh = time.monotonic() - self.last_command_time <= float(
            self.profile['command_timeout_sec'])
        if not (fresh and self.command_is_rotation):
            return
        try:
            transform = self.buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return
        stamp = transform.header.stamp
        stamp_key = (stamp.sec, stamp.nanosec)
        if stamp_key == self.last_stamp:
            return
        self.last_stamp = stamp_key
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        _, _, yaw = quaternion_to_rpy(rotation.x, rotation.y, rotation.z, rotation.w)
        self.samples.append({
            'stamp_sec': stamp.sec + stamp.nanosec / 1e9,
            'x': translation.x,
            'y': translation.y,
            'yaw': yaw,
        })


def load_profile(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if data.get('schema_version') != 1 or 'base_footprint_audit' not in data:
        raise ValueError('profile requires schema_version: 1 and base_footprint_audit')
    return data['base_footprint_audit']


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration-sec', type=float, default=0.0)
    args = parser.parse_args(argv)
    try:
        profile = load_profile(args.profile)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'profile error: {exc}', file=sys.stderr)
        return 2
    duration = args.duration_sec or float(profile.get('sample_duration_sec', 30.0))
    if duration <= 0.0:
        print('duration must be positive', file=sys.stderr)
        return 2

    print(
        'Observer only: during the sample window, use the approved controller to command '
        'a low-speed in-place rotation. This tool never publishes velocity commands.')
    rclpy.init(args=None)
    node = CenterAudit(profile)
    static_pose = None
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        static_pose = node.read_static_pose() or static_pose
        node.sample_motion()

    result = evaluate_center(profile, static_pose, node.samples)
    payload = {
        'schema_version': 1,
        'kind': 'base_footprint_center_audit',
        'sample_duration_sec': duration,
        'command_topic': str(profile['command_topic']),
        **result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    motion = result['motion']
    print(
        f'{result["verdict"]} base_footprint center: samples={motion["sample_count"]} '
        f'yaw={motion["yaw_travel_rad"]:.3f} rad '
        f'excursion={motion["max_planar_excursion_m"]:.3f} m '
        f'radius={motion["effective_radius_m"] if motion["effective_radius_m"] is not None else "n/a"}')
    for failure in result['failures']:
        print(f'FAIL {failure}', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
