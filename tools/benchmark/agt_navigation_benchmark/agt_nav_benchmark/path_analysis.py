"""Path geometry metrics."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import numpy as np


@dataclass
class PathMetric:
    timestamp_ns: int
    path_length: float
    num_points: int
    mean_curvature: float
    max_curvature: float
    sharp_turn_count: int
    curvature_std: float = 0.0
    curvature_p50: float = 0.0
    curvature_p90: float = 0.0
    curvature_p95: float = 0.0
    curvature_p99: float = 0.0
    heading_change_mean: float = 0.0
    heading_change_std: float = 0.0
    sharp_turn_ratio: float = 0.0
    smoothness_score: float = 1.0


def _wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def analyze_path_message(
    message,
    timestamp_ns: int,
    sharp_turn_angle_rad: float = 0.5,
    smoothness_curvature_scale: float = 1.0,
) -> PathMetric:
    points = np.asarray([(pose.pose.position.x, pose.pose.position.y) for pose in message.poses], dtype=float)
    return analyze_xy_points(points, timestamp_ns, sharp_turn_angle_rad, smoothness_curvature_scale)


def analyze_xy_points(
    points,
    timestamp_ns: int,
    sharp_turn_angle_rad: float = 0.5,
    smoothness_curvature_scale: float = 1.0,
) -> PathMetric:
    """Analyze a planner path represented as an ``N x 2`` XY array."""
    points = np.asarray(points, dtype=float).reshape((-1, 2))
    count = len(points)
    if count < 2:
        return PathMetric(timestamp_ns, 0.0, count, 0.0, 0.0, 0)

    deltas = np.diff(points, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    path_length = float(np.sum(distances))
    valid = distances > 1e-6
    if count < 3 or not np.any(valid):
        return PathMetric(timestamp_ns, path_length, count, 0.0, 0.0, 0)

    # Remove repeated points before deriving headings, avoiding artificial spikes.
    valid_points = points[np.r_[True, valid]]
    segment = np.diff(valid_points, axis=0)
    segment_distance = np.linalg.norm(segment, axis=1)
    headings = np.arctan2(segment[:, 1], segment[:, 0])
    turn_angles = _wrap_angle(np.diff(headings))
    curvature_distance = 0.5 * (segment_distance[:-1] + segment_distance[1:])
    valid_curvature = curvature_distance > 1e-6
    curvatures = np.abs(turn_angles[valid_curvature] / curvature_distance[valid_curvature])
    sharp_turn_count = int(np.count_nonzero(np.abs(turn_angles) >= sharp_turn_angle_rad))
    heading_changes = np.abs(turn_angles)
    sharp_turn_ratio = float(sharp_turn_count / max(len(heading_changes), 1))
    if len(curvatures) == 0:
        return PathMetric(
            timestamp_ns, path_length, count, 0.0, 0.0, sharp_turn_count,
            heading_change_mean=float(np.mean(heading_changes)) if len(heading_changes) else 0.0,
            heading_change_std=float(np.std(heading_changes)) if len(heading_changes) else 0.0,
            sharp_turn_ratio=sharp_turn_ratio,
        )
    scale = max(float(smoothness_curvature_scale), 1e-6)
    # A score of 1 means no heading change and no curvature. Both penalties
    # are bounded so a single geometric spike cannot produce a negative score.
    curvature_penalty = min(float(np.percentile(curvatures, 95)) / scale, 1.0)
    smoothness_score = max(0.0, (1.0 - curvature_penalty) * (1.0 - sharp_turn_ratio))
    return PathMetric(
        timestamp_ns, path_length, count, float(np.mean(curvatures)),
        float(np.max(curvatures)), sharp_turn_count,
        curvature_std=float(np.std(curvatures)),
        curvature_p50=float(np.percentile(curvatures, 50)),
        curvature_p90=float(np.percentile(curvatures, 90)),
        curvature_p95=float(np.percentile(curvatures, 95)),
        curvature_p99=float(np.percentile(curvatures, 99)),
        heading_change_mean=float(np.mean(heading_changes)),
        heading_change_std=float(np.std(heading_changes)),
        sharp_turn_ratio=sharp_turn_ratio,
        smoothness_score=smoothness_score,
    )


def write_path_csv(metrics: Iterable[PathMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'path_length', 'num_points', 'mean_curvature',
            'max_curvature', 'sharp_turn_count',
        ])
        for item in metrics:
            writer.writerow([
                f'{item.timestamp_ns / 1e9:.9f}', f'{item.path_length:.6f}', item.num_points,
                f'{item.mean_curvature:.6f}', f'{item.max_curvature:.6f}', item.sharp_turn_count,
            ])


def write_planner_quality_csv(metrics: Iterable[PathMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'path_length', 'num_points', 'curvature_mean', 'curvature_std',
            'curvature_p50', 'curvature_p90', 'curvature_p95', 'curvature_p99',
            'heading_change_mean', 'heading_change_std', 'sharp_turn_ratio',
            'smoothness_score',
        ])
        for item in metrics:
            writer.writerow([
                f'{item.timestamp_ns / 1e9:.9f}', f'{item.path_length:.6f}', item.num_points,
                f'{item.mean_curvature:.6f}', f'{item.curvature_std:.6f}',
                f'{item.curvature_p50:.6f}', f'{item.curvature_p90:.6f}',
                f'{item.curvature_p95:.6f}', f'{item.curvature_p99:.6f}',
                f'{item.heading_change_mean:.6f}', f'{item.heading_change_std:.6f}',
                f'{item.sharp_turn_ratio:.6f}', f'{item.smoothness_score:.6f}',
            ])
