"""Offline audit of recorded planner output and its costmap context."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml


@dataclass
class PlannerRuntimeMetric:
    timestamp_ns: int
    path_length: float
    num_points: int
    curvature_p95: float
    smoothness_score: float
    goal_distance: float
    plan_change_distance: float
    plan_change_max_distance: float
    costmap_trigger_score: float


@dataclass
class ReplanSummary:
    plan_count: int
    replan_frequency_hz: float
    interval_mean_sec: float
    interval_min_sec: float
    interval_max_sec: float
    mean_path_deviation: float
    max_path_deviation: float
    mean_costmap_trigger_score: float
    max_costmap_trigger_score: float


def _path_points(message) -> np.ndarray:
    return np.asarray(
        [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses],
        dtype=float,
    ).reshape((-1, 2))


def _resample_path(points: np.ndarray, count: int = 100) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    if len(points) == 1:
        return np.repeat(points, count, axis=0)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(segment_lengths)]
    if cumulative[-1] <= 1e-9:
        return np.repeat(points[:1], count, axis=0)
    samples = np.linspace(0.0, cumulative[-1], count)
    return np.column_stack([
        np.interp(samples, cumulative, points[:, 0]),
        np.interp(samples, cumulative, points[:, 1]),
    ])


def path_deviation(previous_points: np.ndarray, current_points: np.ndarray):
    """Compare two paths after normalized arc-length resampling."""
    if len(previous_points) == 0 or len(current_points) == 0:
        return 0.0, 0.0
    previous = _resample_path(previous_points)
    current = _resample_path(current_points)
    distances = np.linalg.norm(previous - current, axis=1)
    return float(np.mean(distances)), float(np.max(distances))


def _costmap_change(previous_message, current_message) -> float:
    if previous_message is None or current_message is None:
        return 0.0
    previous = np.asarray(previous_message.data, dtype=np.int16)
    current = np.asarray(current_message.data, dtype=np.int16)
    if previous.shape != current.shape:
        return 1.0
    # Compare semantic occupancy classes so cost value changes within the same
    # class do not look like a complete map change.
    def occupancy_class(values):
        return np.where(values < 0, 0, np.where(values >= 100, 2, 1))
    return float(np.mean(occupancy_class(previous) != occupancy_class(current)))


def _latest_costmap(records, timestamp_ns: int):
    candidates = [record for record in records if record.timestamp_ns <= timestamp_ns]
    return max(candidates, key=lambda item: item.timestamp_ns) if candidates else None


def analyze_plan_runtime(plan_records, path_metrics, costmap_records, thresholds=None):
    """Build per-plan runtime metrics and aggregate replanning statistics."""
    thresholds = thresholds or {}
    plans = sorted(zip(plan_records, path_metrics), key=lambda item: item[0].timestamp_ns)
    costmaps_by_topic = {}
    for record in costmap_records:
        costmaps_by_topic.setdefault(record.topic, []).append(record)
    for records in costmaps_by_topic.values():
        records.sort(key=lambda item: item.timestamp_ns)

    metrics = []
    previous_points = None
    previous_timestamp = None
    previous_costmaps = {}
    for record, path_metric in plans:
        points = _path_points(record.message)
        if len(points) >= 2:
            goal_distance = float(np.linalg.norm(points[-1] - points[0]))
        else:
            goal_distance = 0.0
        change_mean = 0.0
        change_max = 0.0
        if previous_points is not None:
            change_mean, change_max = path_deviation(previous_points, points)

        costmap_changes = []
        current_costmaps = {}
        for topic, records in costmaps_by_topic.items():
            current = _latest_costmap(records, record.timestamp_ns)
            current_costmaps[topic] = current
            previous = previous_costmaps.get(topic)
            if previous is not None and current is not None:
                costmap_changes.append(_costmap_change(previous.message, current.message))
        trigger_score = float(np.mean(costmap_changes)) if costmap_changes else 0.0
        metrics.append(PlannerRuntimeMetric(
            timestamp_ns=record.timestamp_ns,
            path_length=float(path_metric.path_length),
            num_points=int(path_metric.num_points),
            curvature_p95=float(path_metric.curvature_p95),
            smoothness_score=float(path_metric.smoothness_score),
            goal_distance=goal_distance,
            plan_change_distance=change_mean,
            plan_change_max_distance=change_max,
            costmap_trigger_score=trigger_score,
        ))
        previous_points = points
        previous_timestamp = record.timestamp_ns
        previous_costmaps = current_costmaps

    intervals = np.diff([item.timestamp_ns for item, _ in plans]).astype(float) / 1e9 if len(plans) > 1 else np.asarray([])
    deviations = np.asarray([item.plan_change_distance for item in metrics[1:]], dtype=float)
    deviation_maxima = np.asarray([item.plan_change_max_distance for item in metrics[1:]], dtype=float)
    triggers = np.asarray([item.costmap_trigger_score for item in metrics[1:]], dtype=float)
    duration = float((plans[-1][0].timestamp_ns - plans[0][0].timestamp_ns) / 1e9) if len(plans) > 1 else 0.0
    return metrics, ReplanSummary(
        plan_count=len(plans),
        replan_frequency_hz=float((len(plans) - 1) / duration) if duration > 0.0 else 0.0,
        interval_mean_sec=float(np.mean(intervals)) if len(intervals) else 0.0,
        interval_min_sec=float(np.min(intervals)) if len(intervals) else 0.0,
        interval_max_sec=float(np.max(intervals)) if len(intervals) else 0.0,
        mean_path_deviation=float(np.mean(deviations)) if len(deviations) else 0.0,
        max_path_deviation=float(np.max(deviation_maxima)) if len(deviation_maxima) else 0.0,
        mean_costmap_trigger_score=float(np.mean(triggers)) if len(triggers) else 0.0,
        max_costmap_trigger_score=float(np.max(triggers)) if len(triggers) else 0.0,
    )


def write_runtime_metrics_csv(metrics: Iterable[PlannerRuntimeMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'path_length', 'num_points', 'curvature_p95',
            'smoothness_score', 'goal_distance', 'plan_change_distance',
            'costmap_trigger_score',
        ])
        for item in metrics:
            writer.writerow([
                f'{item.timestamp_ns / 1e9:.9f}', f'{item.path_length:.6f}', item.num_points,
                f'{item.curvature_p95:.6f}', f'{item.smoothness_score:.6f}',
                f'{item.goal_distance:.6f}', f'{item.plan_change_distance:.6f}',
                f'{item.costmap_trigger_score:.6f}',
            ])


def _parameter_value(value):
    type_id = int(getattr(value, 'type', 0))
    fields = {
        1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
        5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
        8: 'double_array_value', 9: 'string_array_value',
    }
    field = fields.get(type_id)
    if field is None:
        return None
    result = getattr(value, field, None)
    return list(result) if type_id in (5, 6, 7, 8, 9) else result


def write_planner_runtime_config(parameter_records, output_path: Path):
    """Persist planner_server parameter events, or an explicit MISSING marker."""
    parameters = {}
    event_count = 0
    planner_event_count = 0
    for record in parameter_records:
        event = record.message
        event_count += 1
        node = str(getattr(event, 'node', '') or '').strip('/')
        if node not in ('planner_server',) and not node.endswith('/planner_server'):
            continue
        planner_event_count += 1
        for field in ('new_parameters', 'changed_parameters'):
            for parameter in getattr(event, field, []):
                parameters[str(parameter.name)] = _parameter_value(parameter.value)
        for parameter in getattr(event, 'deleted_parameters', []):
            parameters[str(parameter.name)] = None
    if not parameters:
        output_path.write_text('MISSING\n', encoding='utf-8')
        return {'status': 'MISSING', 'event_count': event_count, 'planner_event_count': planner_event_count, 'parameters': {}}
    payload = {
        'status': 'FOUND',
        'event_count': event_count,
        'planner_event_count': planner_event_count,
        'parameters': parameters,
    }
    with output_path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=True, allow_unicode=True)
    return payload


def write_planner_runtime_report(output_path: Path, metrics, summary: ReplanSummary, config_info, thresholds=None):
    thresholds = thresholds or {}
    small_change = float(thresholds.get('small_plan_change_m', 0.10))
    frequent_interval = float(thresholds.get('frequent_replan_interval_sec', 2.0))
    high_costmap_change = float(thresholds.get('high_costmap_trigger_score', 0.10))
    significant_plan_change = summary.mean_path_deviation > small_change
    if not metrics:
        case = 'INSUFFICIENT_DATA'
        diagnosis = 'No valid /plan messages were found.'
    elif summary.mean_path_deviation <= small_change:
        case = 'A'
        diagnosis = 'Continuous plans are nearly identical; the recorded planner output is stable.'
    elif summary.interval_mean_sec <= frequent_interval and summary.mean_costmap_trigger_score < high_costmap_change and significant_plan_change:
        case = 'B'
        diagnosis = 'Plans change frequently while costmap changes little. Investigate planner parameters, goal handling, or runtime state.'
    elif significant_plan_change and summary.mean_costmap_trigger_score >= high_costmap_change:
        case = 'C'
        diagnosis = 'Plan changes are temporally associated with costmap changes; costmap updates may be influencing replanning.'
    else:
        case = 'MIXED'
        diagnosis = 'Plan and costmap changes do not satisfy a single dominant heuristic case.'
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Planner Runtime Audit Report\n\n')
        stream.write('## Plan Sequence\n\n')
        stream.write('| plan id | timestamp | path length (m) | points | curvature P95 | smoothness | goal distance (m) | plan change (m) | costmap trigger |\n')
        stream.write('|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n')
        for index, item in enumerate(metrics):
            stream.write(
                f'| {index} | {item.timestamp_ns / 1e9:.3f} | {item.path_length:.3f} | {item.num_points} | '
                f'{item.curvature_p95:.3f} | {item.smoothness_score:.3f} | {item.goal_distance:.3f} | '
                f'{item.plan_change_distance:.3f} | {item.costmap_trigger_score:.3f} |\n'
            )
        stream.write('\n## Replanning\n\n')
        stream.write(f'- Plan count: `{summary.plan_count}`\n')
        stream.write(f'- Replan frequency: `{summary.replan_frequency_hz:.3f} Hz`\n')
        stream.write(f'- Replan interval mean/min/max: `{summary.interval_mean_sec:.3f} / {summary.interval_min_sec:.3f} / {summary.interval_max_sec:.3f} s`\n')
        stream.write(f'- Mean path deviation: `{summary.mean_path_deviation:.3f} m`\n')
        stream.write(f'- Maximum path deviation: `{summary.max_path_deviation:.3f} m`\n')
        stream.write(f'- Mean/max costmap trigger score: `{summary.mean_costmap_trigger_score:.3f} / {summary.max_costmap_trigger_score:.3f}`\n')
        stream.write('\n## Planner Runtime Config\n\n')
        stream.write(f"- parameter_events status: `{config_info['status']}`\n")
        stream.write(f"- planner_server events: `{config_info['planner_event_count']}`\n")
        stream.write('- YAML: `planner_runtime_config.yaml`\n\n')
        stream.write('## Diagnosis\n\n')
        stream.write(f'- Case: `{case}`\n')
        stream.write(f'- Threshold: small plan change `{small_change:.3f} m`, frequent replan interval `{frequent_interval:.3f} s`, high costmap trigger `{high_costmap_change:.3f}`\n\n')
        stream.write(f'**Diagnosis:** {diagnosis}\n\n')
        stream.write('This is an offline temporal-correlation heuristic. It does not prove that a costmap update caused a planner decision.\n\n')
        stream.write('- `planner_runtime_metrics.csv`\n- `plan_sequence.png`\n- `plan_curvature_timeline.png`\n')
    return case, diagnosis
