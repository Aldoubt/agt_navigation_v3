"""Matplotlib figures for quick offline inspection."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _relative_seconds(items):
    if not items:
        return []
    start = items[0].timestamp_ns
    return [(item.timestamp_ns - start) / 1e9 for item in items]


def plot_path_curvature(metrics, output_path: Path) -> None:
    times = _relative_seconds(metrics)
    fig, axis = plt.subplots(figsize=(10, 4.8))
    axis.plot(times, [item.mean_curvature for item in metrics], label='mean curvature')
    axis.plot(times, [item.max_curvature for item in metrics], label='max curvature')
    axis.set(xlabel='time since first plan (s)', ylabel='curvature (rad/m)', title='Planner path curvature')
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_unknown_ratio(metrics, output_path: Path) -> None:
    fig, axis = plt.subplots(figsize=(10, 4.8))
    first_stamp = min((item.timestamp_ns for item in metrics), default=0)
    topics = sorted({item.topic for item in metrics})
    for topic in topics:
        selected = [item for item in metrics if item.topic == topic]
        times = [(item.timestamp_ns - first_stamp) / 1e9 for item in selected]
        label = topic.strip('/').replace('/', '/') or 'map'
        axis.plot(times, [item.unknown_ratio * 100.0 for item in selected], label=label)
    axis.axhline(30.0, color='tab:red', linestyle='--', linewidth=1, label='warning 30%')
    axis.set(xlabel='time since first costmap (s)', ylabel='unknown cells (%)', title='OccupancyGrid unknown ratio')
    axis.grid(True, alpha=0.3)
    if topics:
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_cmd_vel_angular(samples, output_path: Path, deadband: float = 0.02) -> None:
    times = _relative_seconds(samples)
    fig, axis = plt.subplots(figsize=(10, 4.8))
    axis.plot(times, [item.angular_z for item in samples], linewidth=0.9, label='angular.z')
    axis.axhline(deadband, color='gray', linestyle=':', linewidth=0.8)
    axis.axhline(-deadband, color='gray', linestyle=':', linewidth=0.8)
    axis.axhline(0.0, color='black', linewidth=0.6)
    axis.set(xlabel='time since first cmd_vel (s)', ylabel='angular velocity (rad/s)', title='cmd_vel angular velocity')
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_tracking_error(samples, output_path: Path, warning_threshold: float = 0.10) -> None:
    if samples:
        start = samples[0].odom_timestamp_ns
        times = [(item.odom_timestamp_ns - start) / 1e9 for item in samples]
    else:
        times = []
    fig, axis = plt.subplots(figsize=(10, 4.8))
    axis.plot(times, [item.cross_track_error for item in samples], linewidth=0.9, label='cross-track error')
    axis.axhline(warning_threshold, color='tab:red', linestyle='--', linewidth=1, label='warning threshold')
    axis.set(
        xlabel='time since first matched odometry (s)',
        ylabel='distance to nearest path point (m)',
        title='Planner path tracking error',
    )
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_plan_sequence(metrics, output_path: Path) -> None:
    times = [(item.timestamp_ns - metrics[0].timestamp_ns) / 1e9 for item in metrics] if metrics else []
    fig, axis = plt.subplots(figsize=(10, 4.8))
    axis.plot(times, [item.plan_change_distance for item in metrics], marker='o', label='plan deviation')
    axis.plot(times, [item.costmap_trigger_score for item in metrics], marker='x', label='costmap trigger score')
    axis.set(xlabel='time since first plan (s)', ylabel='distance / normalized score', title='Planner runtime sequence')
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)


def plot_plan_curvature_timeline(metrics, output_path: Path) -> None:
    times = [(item.timestamp_ns - metrics[0].timestamp_ns) / 1e9 for item in metrics] if metrics else []
    fig, axis = plt.subplots(figsize=(10, 4.8))
    axis.plot(times, [item.curvature_p95 for item in metrics], marker='o', label='curvature P95')
    axis.set(xlabel='time since first plan (s)', ylabel='curvature (rad/m)', title='Planner curvature timeline')
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
