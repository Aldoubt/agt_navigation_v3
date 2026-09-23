"""cmd_vel statistics and angular velocity sign-change analysis."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import numpy as np


@dataclass
class CmdVelSample:
    timestamp_ns: int
    linear_x: float
    angular_z: float


@dataclass
class ControllerMetric:
    timestamp_ns: int
    duration_sec: float
    num_messages: int
    active_messages: int
    linear_x_mean: float
    linear_x_max: float
    angular_z_mean: float
    angular_z_max_abs: float
    angular_sign_change: int
    oscillation_score: float


def count_sign_changes(values: Iterable[float], deadband: float = 0.02) -> int:
    signs = [1 if value > deadband else -1 if value < -deadband else 0 for value in values]
    active = [sign for sign in signs if sign]
    return sum(left != right for left, right in zip(active, active[1:]))


def analyze_cmd_vel(samples: List[CmdVelSample], deadband: float = 0.02) -> ControllerMetric:
    if not samples:
        return ControllerMetric(0, 0.0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0, 0.0)
    samples = sorted(samples, key=lambda item: item.timestamp_ns)
    linear = np.asarray([sample.linear_x for sample in samples], dtype=float)
    angular = np.asarray([sample.angular_z for sample in samples], dtype=float)
    active = int(np.count_nonzero(np.abs(angular) > deadband))
    changes = count_sign_changes(angular, deadband)
    score = float(changes / max(active - 1, 1))
    return ControllerMetric(
        timestamp_ns=samples[0].timestamp_ns,
        duration_sec=float((samples[-1].timestamp_ns - samples[0].timestamp_ns) / 1e9),
        num_messages=len(samples),
        active_messages=active,
        linear_x_mean=float(np.mean(linear)),
        linear_x_max=float(np.max(np.abs(linear))),
        angular_z_mean=float(np.mean(angular)),
        angular_z_max_abs=float(np.max(np.abs(angular))),
        angular_sign_change=changes,
        oscillation_score=min(score, 1.0),
    )


def write_controller_csv(metric: ControllerMetric, output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'duration_sec', 'num_messages', 'active_messages',
            'linear_x_mean', 'linear_x_max', 'angular_z_mean', 'angular_z_max_abs',
            'angular_sign_change', 'oscillation_score',
        ])
        writer.writerow([
            f'{metric.timestamp_ns / 1e9:.9f}', f'{metric.duration_sec:.6f}', metric.num_messages,
            metric.active_messages, f'{metric.linear_x_mean:.6f}', f'{metric.linear_x_max:.6f}',
            f'{metric.angular_z_mean:.6f}', f'{metric.angular_z_max_abs:.6f}',
            metric.angular_sign_change, f'{metric.oscillation_score:.6f}',
        ])
