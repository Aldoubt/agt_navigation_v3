#!/usr/bin/env python3
"""Read-only P3.1 OFF/ON rosbag comparison for self-obstacle experiments."""

from __future__ import annotations

import argparse
import bisect
import math
from collections import Counter
from pathlib import Path
from typing import Any

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    '/agt/livox/points',
    '/agt/navigation/points_obstacles',
    '/local_costmap/costmap_raw',
    '/local_costmap/costmap',  # compatibility fallback for older field bags
    '/agt/odometry/local',
    '/cmd_vel',
)
FOOTPRINT = {'front_m': 0.55, 'rear_m': 0.55, 'half_width_m': 0.43}


def stamp_seconds(message: Any, fallback_ns: int) -> float:
    header = getattr(message, 'header', None)
    if header and (header.stamp.sec or header.stamp.nanosec):
        return header.stamp.sec + header.stamp.nanosec * 1e-9
    return fallback_ns * 1e-9


def yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def ratio(numerator: int | float, denominator: int | float) -> float | None:
    return float(numerator) / float(denominator) if denominator else None


def read_statistics(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {'status': 'NOT_PROVIDED'}
    if not path.is_file():
        return {'status': 'MISSING', 'path': str(path)}
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {
        'status': 'AVAILABLE',
        'path': str(path),
        'input_points': int(data.get('input_points', 0)),
        'output_points': int(data.get('output_points', 0)),
        'removed_ratio_of_input': data.get('removed_ratio_of_input'),
        'rear_removed_points': int(data.get('rear_removed_points', 0)),
        'rear_sector_removed_ratio_of_input': data.get('rear_sector_removed_ratio_of_input'),
        'rear_sector_enabled': data.get('rear_sector_enabled'),
    }


def costmap_geometry(message: Any) -> tuple[str, int, int, float, float, float, list[int]]:
    """Return (frame, width, height, resolution, origin_x, origin_y, data)."""
    if hasattr(message, 'info'):  # nav_msgs/OccupancyGrid
        info = message.info
        return (message.header.frame_id, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, list(message.data))
    # nav2_msgs/Costmap as published by local_costmap/costmap_raw.
    metadata = message.metadata
    return (message.header.frame_id, metadata.size_x, metadata.size_y, metadata.resolution,
            metadata.origin.x, metadata.origin.y, list(message.data))


def footprint_costs(costmap: Any, pose: tuple[float, float, float]) -> list[int]:
    _, width, height, resolution, origin_x, origin_y, data = costmap_geometry(costmap)
    base_x, base_y, base_yaw = pose
    # The padded footprint is smaller than this envelope; only visit nearby cells.
    ix0 = max(0, int(math.floor((base_x - 0.75 - origin_x) / resolution)))
    ix1 = min(width - 1, int(math.ceil((base_x + 0.75 - origin_x) / resolution)))
    iy0 = max(0, int(math.floor((base_y - 0.75 - origin_y) / resolution)))
    iy1 = min(height - 1, int(math.ceil((base_y + 0.75 - origin_y) / resolution)))
    cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
    values: list[int] = []
    for grid_y in range(iy0, iy1 + 1):
        world_y = origin_y + (grid_y + 0.5) * resolution
        for grid_x in range(ix0, ix1 + 1):
            world_x = origin_x + (grid_x + 0.5) * resolution
            dx, dy = world_x - base_x, world_y - base_y
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            if (-FOOTPRINT['rear_m'] <= local_x <= FOOTPRINT['front_m'] and
                    abs(local_y) <= FOOTPRINT['half_width_m']):
                values.append(int(data[grid_y * width + grid_x]))
    return values


def weighted_command_ratios(commands: list[tuple[float, float, float]]) -> dict[str, Any]:
    if len(commands) < 2:
        return {'status': 'INSUFFICIENT_CMD_VEL'}
    duration = commands[-1][0] - commands[0][0]
    stopped = rotating = 0.0
    for index, (time_s, linear_x, angular_z) in enumerate(commands[:-1]):
        delta = commands[index + 1][0] - time_s
        if abs(linear_x) <= 0.02 and abs(angular_z) <= 0.02:
            stopped += delta
        if abs(linear_x) <= 0.02 and abs(angular_z) >= 0.20:
            rotating += delta
    return {
        'status': 'AVAILABLE', 'samples': len(commands), 'duration_s': duration,
        'stopped_time_s': stopped, 'stopped_time_ratio': ratio(stopped, duration),
        'rotate_in_place_time_s': rotating,
        'rotate_in_place_time_ratio': ratio(rotating, duration),
    }


def analyze_bag(bag: Path, statistics_path: Path | None) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    clouds: dict[str, list[tuple[float, int]]] = {'input': [], 'obstacle': []}
    odometry: list[tuple[float, float, float, float]] = []
    commands: list[tuple[float, float, float]] = []
    raw_costmaps: list[tuple[float, Any]] = []
    fallback_costmaps: list[tuple[float, Any]] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        time_s = stamp_seconds(message, timestamp_ns)
        if topic == '/agt/livox/points':
            clouds['input'].append((time_s, int(message.width) * int(message.height)))
        elif topic == '/agt/navigation/points_obstacles':
            clouds['obstacle'].append((time_s, int(message.width) * int(message.height)))
        elif topic == '/agt/odometry/local':
            position = message.pose.pose.position
            odometry.append((time_s, position.x, position.y, yaw(message.pose.pose.orientation)))
        elif topic == '/cmd_vel':
            commands.append((time_s, message.linear.x, message.angular.z))
        elif topic == '/local_costmap/costmap_raw':
            raw_costmaps.append((time_s, message))
        elif topic == '/local_costmap/costmap':
            fallback_costmaps.append((time_s, message))

    input_points = sum(count for _, count in clouds['input'])
    output_points = sum(count for _, count in clouds['obstacle'])
    costmaps = raw_costmaps or fallback_costmaps
    costmap_topic = '/local_costmap/costmap_raw' if raw_costmaps else '/local_costmap/costmap'
    odom_times = [sample[0] for sample in odometry]
    footprint_samples: list[list[int]] = []
    for time_s, costmap in costmaps:
        if not odometry:
            break
        insert = bisect.bisect_left(odom_times, time_s)
        candidates = odometry[max(0, insert - 1):min(len(odometry), insert + 1)]
        nearest = min(candidates, key=lambda item: abs(item[0] - time_s))
        footprint_samples.append(footprint_costs(costmap, nearest[1:]))

    nonempty = [sample for sample in footprint_samples if sample]
    footprint_total = sum(len(sample) for sample in nonempty)
    # OccupancyGrid has a standard 0..100 scale; Costmap raw normally uses
    # 0..254.  Report both thresholds, preserving their distinct semantics.
    max_values = [max(sample) for sample in nonempty]
    return {
        'bag': str(bag),
        'topics_found': sorted(topic_types),
        'obstacle_cloud': {
            'input_topic': '/agt/livox/points', 'input_messages': len(clouds['input']),
            'input_points_total': input_points, 'input_points_mean': mean([x[1] for x in clouds['input']]),
            'output_topic': '/agt/navigation/points_obstacles', 'output_messages': len(clouds['obstacle']),
            'output_points_total': output_points, 'output_points_mean': mean([x[1] for x in clouds['obstacle']]),
            'input_to_output_change_ratio': ratio(input_points - output_points, input_points),
            'rear_sector_statistics': read_statistics(statistics_path),
        },
        'local_costmap': {
            'topic_used': costmap_topic if costmaps else None,
            'samples': len(costmaps), 'odom_samples': len(odometry),
            'footprint_samples': len(nonempty), 'footprint_cells_total': footprint_total,
            'footprint_value_100_samples': sum(value >= 100 for value in max_values),
            'footprint_value_253_samples': sum(value >= 253 for value in max_values),
            'footprint_max_value': max(max_values) if max_values else None,
            'footprint_obstacle_cell_ratio_ge_100': ratio(
                sum(value >= 100 for sample in nonempty for value in sample), footprint_total),
            'footprint_obstacle_cell_ratio_ge_253': ratio(
                sum(value >= 253 for sample in nonempty for value in sample), footprint_total),
            'status': 'AVAILABLE' if nonempty else 'MISSING_COSTMAP_OR_ODOM',
        },
        'cmd_vel': weighted_command_ratios(commands),
    }


def comparison(off: dict[str, Any], on: dict[str, Any]) -> dict[str, Any]:
    def delta(section: str, key: str) -> float | None:
        before, after = off[section].get(key), on[section].get(key)
        return after - before if isinstance(before, (int, float)) and isinstance(after, (int, float)) else None
    return {
        'obstacle_output_points_total_delta_on_minus_off': delta('obstacle_cloud', 'output_points_total'),
        'costmap_value_100_samples_delta_on_minus_off': delta('local_costmap', 'footprint_value_100_samples'),
        'costmap_obstacle_cell_ratio_ge_100_delta_on_minus_off': delta(
            'local_costmap', 'footprint_obstacle_cell_ratio_ge_100'),
        'stopped_time_ratio_delta_on_minus_off': delta('cmd_vel', 'stopped_time_ratio'),
        'rotate_in_place_time_ratio_delta_on_minus_off': delta('cmd_vel', 'rotate_in_place_time_ratio'),
    }


def markdown(metrics: dict[str, Any]) -> str:
    off, on = metrics['off'], metrics['on']
    rows = []
    for label, section, key in (
        ('Obstacle output points (total)', 'obstacle_cloud', 'output_points_total'),
        ('Rear-sector deleted ratio', 'obstacle_cloud', 'rear_sector_statistics'),
        ('Footprint value≥100 samples', 'local_costmap', 'footprint_value_100_samples'),
        ('Footprint obstacle-cell ratio ≥100', 'local_costmap', 'footprint_obstacle_cell_ratio_ge_100'),
        ('Stopped time ratio', 'cmd_vel', 'stopped_time_ratio'),
        ('Rotate-in-place time ratio', 'cmd_vel', 'rotate_in_place_time_ratio'),
    ):
        def value(run: dict[str, Any]) -> Any:
            result = run[section].get(key)
            return result.get('rear_sector_removed_ratio_of_input', 'NOT_AVAILABLE') if isinstance(result, dict) else result
        rows.append(f'| {label} | {value(off)} | {value(on)} |')
    return "\n".join([
        '# P3.1 Self-Obstacle OFF/ON Result', '',
        '该报告由 `tools/p3_1_self_obstacle_replay_analyzer.py` 只读生成。', '',
        '| Metric | OFF | ON |', '|---|---:|---:|', *rows, '',
        '## Interpretation boundary', '',
        'rear-sector 的专属删除比例只能由节点退出时写出的 `statistics_output` YAML 证明。'
        '若该文件缺失，bag 内 input→output 点数差包含 self/range/voxel 等全部过滤，不能归因于 rear sector。', '',
        'PASS 的最小证据是：ON 组 rear-sector 删除数大于零、footprint 内高代价样本下降，'
        '且外部测试障碍仍保留。仅点数下降或仅停止比例变化不足以证明安全改进。', '',
        '## Metrics artifact', '',
        '`p3_1_metrics.yaml` 是本报告的机器可读原始汇总。',
    ]) + '\n'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--off', required=True, type=Path, help='P3.1 rear-sector OFF rosbag directory')
    parser.add_argument('--on', required=True, type=Path, help='P3.1 rear-sector ON rosbag directory')
    parser.add_argument('--off-statistics', type=Path, help='Optional OFF final statistics_output YAML')
    parser.add_argument('--on-statistics', type=Path, help='Optional ON final statistics_output YAML')
    parser.add_argument('--metrics', required=True, type=Path, help='Output YAML path')
    parser.add_argument('--report', required=True, type=Path, help='Output Markdown path')
    args = parser.parse_args()
    for bag in (args.off, args.on):
        if not (bag / 'metadata.yaml').is_file():
            parser.error(f'not a rosbag directory (metadata.yaml missing): {bag}')
    metrics = {
        'format_version': 1,
        'experiment': 'P3.1 self-obstacle rear-sector OFF/ON replay analysis',
        'footprint_assumption': FOOTPRINT,
        'off': analyze_bag(args.off, args.off_statistics),
        'on': analyze_bag(args.on, args.on_statistics),
    }
    metrics['comparison_on_minus_off'] = comparison(metrics['off'], metrics['on'])
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.metrics.write_text(yaml.safe_dump(metrics, sort_keys=False, allow_unicode=True), encoding='utf-8')
    args.report.write_text(markdown(metrics), encoding='utf-8')


if __name__ == '__main__':
    main()
