"""Cross-layer planner -> controller correlation and diagnosis rules."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence

from .controller_analysis import CmdVelSample, count_sign_changes
from .path_analysis import PathMetric


@dataclass
class CorrelationRow:
    path_index: int
    path_timestamp_ns: int
    next_path_timestamp_ns: int
    path_mean_curvature: float
    path_max_curvature: float
    path_length_change_ratio: float
    cmd_samples: int
    cmd_angular_sign_change: int
    cmd_oscillation_score: float
    odom_samples: int
    odom_mean_linear_speed: float
    odom_max_angular_speed: float


def correlate_path_and_cmd(
    paths: Sequence[PathMetric],
    commands: Sequence[CmdVelSample],
    odometry=(),
    deadband: float = 0.02,
) -> List[CorrelationRow]:
    """Assign each command sample to the path currently being published.

    A planner publishes a new path at an arbitrary time, so the interval from
    one plan timestamp to the next is the most useful offline approximation of
    the path being tracked. The final interval ends at the last command.
    """
    paths = sorted(paths, key=lambda item: item.timestamp_ns)
    commands = sorted(commands, key=lambda item: item.timestamp_ns)
    rows: List[CorrelationRow] = []
    for index, path in enumerate(paths):
        next_stamp = paths[index + 1].timestamp_ns if index + 1 < len(paths) else (
            commands[-1].timestamp_ns if commands else path.timestamp_ns
        )
        selected = [
            item for item in commands
            if path.timestamp_ns <= item.timestamp_ns < max(next_stamp, path.timestamp_ns + 1)
        ]
        values = [item.angular_z for item in selected]
        changes = count_sign_changes(values, deadband)
        active = sum(abs(value) > deadband for value in values)
        previous_length = paths[index - 1].path_length if index else path.path_length
        change_ratio = abs(path.path_length - previous_length) / max(abs(previous_length), 1e-6)
        selected_odom = [
            item for item in odometry
            if path.timestamp_ns <= item.timestamp_ns < max(next_stamp, path.timestamp_ns + 1)
        ]
        rows.append(CorrelationRow(
            path_index=index,
            path_timestamp_ns=path.timestamp_ns,
            next_path_timestamp_ns=next_stamp,
            path_mean_curvature=path.mean_curvature,
            path_max_curvature=path.max_curvature,
            path_length_change_ratio=change_ratio,
            cmd_samples=len(selected),
            cmd_angular_sign_change=changes,
            cmd_oscillation_score=changes / max(active - 1, 1),
            odom_samples=len(selected_odom),
            odom_mean_linear_speed=(
                sum(item.linear_speed for item in selected_odom) / len(selected_odom)
                if selected_odom else 0.0
            ),
            odom_max_angular_speed=max((abs(item.angular_speed) for item in selected_odom), default=0.0),
        ))
    return rows


def write_correlation_csv(rows: Iterable[CorrelationRow], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'path_index', 'path_timestamp', 'next_path_timestamp',
            'path_mean_curvature', 'path_max_curvature', 'path_length_change_ratio',
            'cmd_samples', 'cmd_angular_sign_change', 'cmd_oscillation_score',
            'odom_samples', 'odom_mean_linear_speed', 'odom_max_angular_speed',
        ])
        for row in rows:
            writer.writerow([
                row.path_index, f'{row.path_timestamp_ns / 1e9:.9f}',
                f'{row.next_path_timestamp_ns / 1e9:.9f}',
                f'{row.path_mean_curvature:.6f}', f'{row.path_max_curvature:.6f}',
                f'{row.path_length_change_ratio:.6f}', row.cmd_samples,
                row.cmd_angular_sign_change, f'{row.cmd_oscillation_score:.6f}',
                row.odom_samples, f'{row.odom_mean_linear_speed:.6f}', f'{row.odom_max_angular_speed:.6f}',
            ])


def diagnose(paths, costmaps, controller, config, correlation, planner_quality=None):
    thresholds = config['thresholds']
    global_maps = [item for item in costmaps if item.topic == config['topics']['global_costmap']]
    all_maps = global_maps or list(costmaps)
    max_unknown = max((item.unknown_ratio for item in all_maps), default=0.0)
    max_occupied = max((item.occupied_ratio for item in all_maps), default=0.0)
    avg_mean_curvature = sum(item.mean_curvature for item in paths) / max(len(paths), 1)
    max_curvature = max((item.max_curvature for item in paths), default=0.0)
    high_curvature = (
        avg_mean_curvature >= thresholds['high_mean_curvature_rad_per_m']
        or max_curvature >= thresholds['high_max_curvature_rad_per_m']
    )
    high_unknown = max_unknown >= thresholds['unknown_ratio_warning']
    high_obstacle = max_occupied >= thresholds['occupied_ratio_warning']
    high_oscillation = (
        controller.oscillation_score >= thresholds['controller_oscillation_score']
        and controller.angular_sign_change >= thresholds.get('min_sign_changes_for_oscillation', 5)
    )
    planner_quality = list(planner_quality or paths)
    mean_smoothness = sum(item.smoothness_score for item in planner_quality) / max(len(planner_quality), 1)
    path_smooth = mean_smoothness >= thresholds.get('smoothness_score_smooth', 0.70)
    primary_map_suspect = (
        max_unknown > thresholds.get('unknown_ratio_primary_map_suspect', 0.50)
        and high_curvature
        and not high_oscillation
    )
    unstable_paths = sum(
        row.path_length_change_ratio >= thresholds['high_path_length_change_ratio']
        for row in correlation
    )
    path_unstable = high_curvature or unstable_paths > 0
    associated_oscillation = sum(
        row.cmd_angular_sign_change > 0 and (
            row.path_mean_curvature >= thresholds['high_mean_curvature_rad_per_m']
            or row.path_max_curvature >= thresholds['high_max_curvature_rad_per_m']
        )
        for row in correlation
    )

    causes = []
    if primary_map_suspect:
        causes.append(
            'Map representation is the primary suspect. Planner is reacting to incomplete free space.'
        )
    elif high_unknown and high_curvature and high_oscillation:
        causes.append('Unknown area in map causes planner path instability.')
    elif high_unknown and high_curvature:
        causes.append(
            'Map unknown area and path curvature are high, indicating a planner-side '
            'detour/path-geometry issue; cmd_vel oscillation evidence is currently weak.'
        )
    if path_smooth and high_oscillation:
        if max_unknown < 0.10:
            causes.append('Base controller or odometry issue. Check controller and localization data.')
        else:
            causes.append('Controller tracking issue. Check RPP/MPPI parameters.')
    elif not high_curvature and high_oscillation:
        causes.append('Controller tracking instability. Check RPP parameters.')
    if path_unstable and high_obstacle:
        causes.append('Costmap obstacle inflation or map quality issue.')
    if associated_oscillation:
        causes.append(
            f'Planner/controller correlation: {associated_oscillation} path interval(s) '
            'combine high curvature with angular sign changes.'
        )
    if not causes:
        causes.append(
            'No single rule exceeded all configured thresholds; inspect the per-message CSVs '
            'and plots before changing navigation parameters.'
        )
    return {
        'max_global_unknown_ratio': max_unknown,
        'max_occupied_ratio': max_occupied,
        'average_mean_curvature': avg_mean_curvature,
        'maximum_curvature': max_curvature,
        'high_unknown': high_unknown,
        'high_curvature': high_curvature,
        'high_obstacle': high_obstacle,
        'high_oscillation': high_oscillation,
        'mean_smoothness_score': mean_smoothness,
        'path_smooth': path_smooth,
        'primary_map_suspect': primary_map_suspect,
        'unstable_path_count': unstable_paths,
        'associated_oscillation_count': associated_oscillation,
        'causes': causes,
    }
