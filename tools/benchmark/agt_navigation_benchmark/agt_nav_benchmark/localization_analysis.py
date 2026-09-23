"""Lightweight offline metrics for the recorded local odometry layer."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import numpy as np


@dataclass
class OdometrySample:
    timestamp_ns: int
    x: float
    y: float
    linear_speed: float
    angular_speed: float
    frame_id: str = ''


@dataclass
class LocalizationMetric:
    timestamp_ns: int
    duration_sec: float
    num_messages: int
    displacement_m: float
    traveled_distance_m: float
    mean_linear_speed: float
    max_linear_speed: float
    max_angular_speed: float


def sample_from_message(message, timestamp_ns: int) -> OdometrySample:
    pose = message.pose.pose.position
    twist = message.twist.twist
    return OdometrySample(
        timestamp_ns=timestamp_ns,
        x=float(pose.x),
        y=float(pose.y),
        linear_speed=float(twist.linear.x),
        angular_speed=float(twist.angular.z),
        frame_id=str(getattr(getattr(message, 'header', None), 'frame_id', '') or ''),
    )


def analyze_odometry(samples: List[OdometrySample]) -> LocalizationMetric:
    samples = sorted(samples, key=lambda item: item.timestamp_ns)
    if not samples:
        return LocalizationMetric(0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
    points = np.asarray([(item.x, item.y) for item in samples], dtype=float)
    distance = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points) > 1 else 0.0
    displacement = float(np.linalg.norm(points[-1] - points[0]))
    speeds = np.asarray([item.linear_speed for item in samples], dtype=float)
    angular = np.asarray([item.angular_speed for item in samples], dtype=float)
    return LocalizationMetric(
        timestamp_ns=samples[0].timestamp_ns,
        duration_sec=float((samples[-1].timestamp_ns - samples[0].timestamp_ns) / 1e9),
        num_messages=len(samples),
        displacement_m=displacement,
        traveled_distance_m=distance,
        mean_linear_speed=float(np.mean(np.abs(speeds))),
        max_linear_speed=float(np.max(np.abs(speeds))),
        max_angular_speed=float(np.max(np.abs(angular))),
    )


def write_localization_csv(metric: LocalizationMetric, output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'duration_sec', 'num_messages', 'displacement_m',
            'traveled_distance_m', 'mean_linear_speed', 'max_linear_speed', 'max_angular_speed',
        ])
        writer.writerow([
            f'{metric.timestamp_ns / 1e9:.9f}', f'{metric.duration_sec:.6f}', metric.num_messages,
            f'{metric.displacement_m:.6f}', f'{metric.traveled_distance_m:.6f}',
            f'{metric.mean_linear_speed:.6f}', f'{metric.max_linear_speed:.6f}',
            f'{metric.max_angular_speed:.6f}',
        ])
