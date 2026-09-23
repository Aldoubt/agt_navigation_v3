"""Offline runtime audit for odometry publishers, frequency and TF evidence.

rosbag2 records topic/type metadata and serialized messages, but they do not
normally retain the ROS publisher node name or TF authority.  This module
therefore never infers publisher count from message count or frequency.  It
reports publisher provenance as ``UNKNOWN`` for present topics and ``MISSING``
for topics absent from the bag.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np


DEFAULT_ODOMETRY_TOPICS = (
    '/odom',
    '/agt/chassis/odometry',
    '/agt/odometry/local',
)
DEFAULT_TF_TOPICS = ('/tf', '/tf_static')
DEFAULT_TARGET_EDGES = (
    ('odom', 'base_link'),
    ('base_link', 'base_footprint'),
)


@dataclass
class OdomTopicMetric:
    topic: str
    status: str
    message_count: int
    duration_sec: float
    frequency_hz: float | None
    frequency_class: str
    frequency_assessment: str
    publisher_count: str
    publisher_nodes: str
    anomalous_intervals: int


@dataclass
class TfEdgeEvidence:
    parent_frame: str
    child_frame: str
    observed_topics: Tuple[str, ...]
    transform_count: int
    source_nodes: str
    status: str


@dataclass
class OdomPlanRelation:
    topic: str
    status: str
    anomalous_intervals: int
    plan_count: int
    plan_change_count: int
    plan_changes_during_anomaly: int
    assessment: str


@dataclass
class OdomRuntimeResult:
    odom_metrics: List[OdomTopicMetric]
    tf_edges: List[TfEdgeEvidence]
    target_edges: List[TfEdgeEvidence]
    plan_relations: List[OdomPlanRelation]


def _frame_name(value) -> str:
    return str(value or '').strip().lstrip('/')


def _frequency_class(frequency_hz: float | None, tolerance_ratio: float) -> Tuple[str, str]:
    if frequency_hz is None:
        return 'INSUFFICIENT_DATA', 'INSUFFICIENT_DATA'
    for nominal in (50.0, 100.0):
        if abs(frequency_hz - nominal) <= nominal * tolerance_ratio:
            return f'NEAR_{int(nominal)}HZ', f'EXPECTED_{int(nominal)}HZ'
    return 'OTHER', 'ABNORMAL_FREQUENCY'


def _anomaly_intervals(records, metric_frequency_class: str, config: Mapping):
    if len(records) < 2:
        return []
    timestamps = sorted(int(item.timestamp_ns) for item in records)
    nominal = 100.0 if metric_frequency_class == 'NEAR_100HZ' else 50.0
    if metric_frequency_class not in ('NEAR_50HZ', 'NEAR_100HZ'):
        nominal = float(config.get('nominal_frequency_hz', 50.0))
    expected_dt = 1.0 / max(nominal, 1e-9)
    low_factor = float(config.get('interval_low_factor', 0.5))
    high_factor = float(config.get('interval_high_factor', 1.5))
    result = []
    for before, after in zip(timestamps[:-1], timestamps[1:]):
        dt = (after - before) / 1e9
        if dt <= 0.0 or dt < expected_dt * low_factor or dt > expected_dt * high_factor:
            result.append((before, after))
    return result


def _metric_for_topic(topic: str, available_topics: Mapping[str, str], records, config: Mapping) -> OdomTopicMetric:
    if topic not in available_topics:
        return OdomTopicMetric(
            topic, 'MISSING', 0, 0.0, None, 'MISSING', 'MISSING',
            'MISSING', 'MISSING', 0,
        )
    records = sorted(records, key=lambda item: item.timestamp_ns)
    if not records:
        return OdomTopicMetric(
            topic, 'PRESENT_NO_MESSAGES', 0, 0.0, None, 'INSUFFICIENT_DATA',
            'INSUFFICIENT_DATA', 'UNKNOWN', 'UNKNOWN', 0,
        )
    timestamps = np.asarray([int(item.timestamp_ns) for item in records], dtype=np.int64)
    duration_sec = float((timestamps[-1] - timestamps[0]) / 1e9) if len(records) > 1 else 0.0
    frequency = float((len(records) - 1) / duration_sec) if duration_sec > 0.0 else None
    frequency_class, assessment = _frequency_class(
        frequency, float(config.get('frequency_tolerance_ratio', 0.10)),
    )
    anomalies = _anomaly_intervals(records, frequency_class, config)
    return OdomTopicMetric(
        topic,
        'FOUND',
        len(records),
        duration_sec,
        frequency,
        frequency_class,
        assessment,
        'UNKNOWN',
        'UNKNOWN',
        len(anomalies),
    )


def _tf_edges(tf_records: Iterable) -> Dict[Tuple[str, str], Dict]:
    evidence: Dict[Tuple[str, str], Dict] = {}
    for record in tf_records:
        for transform in getattr(record.message, 'transforms', []):
            parent = _frame_name(getattr(getattr(transform, 'header', None), 'frame_id', ''))
            child = _frame_name(getattr(transform, 'child_frame_id', ''))
            if not parent or not child:
                continue
            key = (parent, child)
            item = evidence.setdefault(key, {'topics': set(), 'count': 0})
            item['topics'].add(record.topic)
            item['count'] += 1
    return evidence


def _edge_status(item: Dict | None) -> Tuple[Tuple[str, ...], int, str, str]:
    if item is None:
        return (), 0, 'UNKNOWN', 'MISSING'
    topics = tuple(sorted(item['topics']))
    if len(topics) > 1:
        status = 'POSSIBLE_CONFLICT_MULTIPLE_TF_TOPICS'
    else:
        status = 'OBSERVED_SOURCE_UNKNOWN'
    return topics, int(item['count']), 'UNKNOWN', status


def _build_edge(evidence: Dict, edge: Tuple[str, str]) -> TfEdgeEvidence:
    item = evidence.get(edge)
    reverse_item = evidence.get((edge[1], edge[0]))
    if item is None and reverse_item is not None:
        topics, count, source_nodes, _ = _edge_status(reverse_item)
        return TfEdgeEvidence(
            edge[0], edge[1], topics, count, source_nodes,
            'OBSERVED_REVERSE_EDGE_SOURCE_UNKNOWN',
        )
    topics, count, source_nodes, status = _edge_status(item)
    return TfEdgeEvidence(edge[0], edge[1], topics, count, source_nodes, status)


def _plan_relation(topic: str, records, metric, plan_records, path_metrics, config: Mapping) -> OdomPlanRelation:
    if metric.status == 'MISSING':
        return OdomPlanRelation(topic, 'MISSING_ODOM', 0, len(plan_records), 0, 0, 'MISSING')
    if not records:
        return OdomPlanRelation(topic, 'NO_ODOM_MESSAGES', 0, len(plan_records), 0, 0, 'MISSING')
    if not plan_records or not path_metrics:
        return OdomPlanRelation(topic, 'MISSING_PLAN', 0, len(plan_records), 0, 0, 'MISSING')

    paired = sorted(zip(plan_records, path_metrics), key=lambda item: item[0].timestamp_ns)
    threshold = float(config.get('plan_change_threshold_m', 0.10))
    changed_plan_times = [
        int(record.timestamp_ns)
        for index, (record, path_metric) in enumerate(paired)
        if index > 0 and float(getattr(path_metric, 'plan_change_distance', 0.0)) >= threshold
    ]
    # PathMetric does not carry runtime deviation in the current pipeline;
    # callers may attach it dynamically, while this fallback keeps the module
    # useful with plain path metrics and explicitly reports no detected change.
    intervals = _anomaly_intervals(
        records,
        metric.frequency_class,
        config,
    )
    changed_during = sum(
        any(start <= timestamp <= end for start, end in intervals)
        for timestamp in changed_plan_times
    )
    if not intervals:
        assessment = 'NO_ODOM_FREQUENCY_ANOMALY'
    elif changed_during:
        assessment = 'PLAN_CHANGE_OVERLAPS_ODOM_ANOMALY'
    else:
        assessment = 'NO_PLAN_CHANGE_DURING_ODOM_ANOMALY'
    return OdomPlanRelation(
        topic,
        'FOUND',
        len(intervals),
        len(plan_records),
        len(changed_plan_times),
        changed_during,
        assessment,
    )


def analyze_odom_runtime(
    available_topics: Mapping[str, str],
    grouped: Mapping[str, Sequence],
    plan_records: Sequence = (),
    path_metrics: Sequence = (),
    config: Mapping | None = None,
) -> OdomRuntimeResult:
    """Analyze all configured odometry candidates and recorded TF edges."""
    config = config or {}
    odometry_topics = tuple(config.get('odometry_topics', DEFAULT_ODOMETRY_TOPICS))
    tf_topics = tuple(config.get('tf_topics', DEFAULT_TF_TOPICS))
    target_edges = tuple(
        tuple(edge) for edge in config.get('target_edges', DEFAULT_TARGET_EDGES)
    )

    metrics = [
        _metric_for_topic(topic, available_topics, grouped.get(topic, []), config)
        for topic in odometry_topics
    ]
    tf_records = [
        record
        for topic in tf_topics
        for record in grouped.get(topic, [])
    ]
    evidence = _tf_edges(tf_records)
    edges = [
        TfEdgeEvidence(parent, child, tuple(sorted(item['topics'])), int(item['count']), 'UNKNOWN',
                       'POSSIBLE_CONFLICT_MULTIPLE_TF_TOPICS' if len(item['topics']) > 1 else 'OBSERVED_SOURCE_UNKNOWN')
        for (parent, child), item in sorted(evidence.items())
    ]
    target_evidence = [_build_edge(evidence, edge) for edge in target_edges]
    relations = [
        _plan_relation(
            metric.topic,
            grouped.get(metric.topic, []),
            metric,
            plan_records,
            path_metrics,
            config,
        )
        for metric in metrics
    ]
    return OdomRuntimeResult(metrics, edges, target_evidence, relations)


def write_odom_frequency_csv(metrics: Iterable[OdomTopicMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'topic', 'status', 'message_count', 'duration_sec', 'frequency_hz',
            'frequency_class', 'frequency_assessment', 'publisher_count',
            'publisher_nodes', 'anomalous_intervals',
        ])
        for item in metrics:
            writer.writerow([
                item.topic, item.status, item.message_count, f'{item.duration_sec:.6f}',
                '' if item.frequency_hz is None else f'{item.frequency_hz:.6f}',
                item.frequency_class, item.frequency_assessment, item.publisher_count,
                item.publisher_nodes, item.anomalous_intervals,
            ])


def write_tf_conflict_report(
    output_path: Path,
    result: OdomRuntimeResult,
    tf_topics: Sequence[str] = DEFAULT_TF_TOPICS,
    available_topics: Mapping[str, str] | None = None,
) -> None:
    available_topics = available_topics or {}
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# TF Conflict Report\n\n')
        stream.write('本报告基于 rosbag 中 `/tf` 和 `/tf_static` 消息。标准 TFMessage 不携带 publisher node/TF authority，因此 source node 统一标记为 `UNKNOWN`，不把重复 transform 消息误判为多个节点。\n\n')
        stream.write('## TF Topics\n\n')
        for topic in tf_topics:
            observed = any(topic in edge.observed_topics for edge in result.tf_edges)
            if topic not in available_topics:
                status = 'MISSING'
            elif observed:
                status = 'FOUND'
            else:
                status = 'FOUND_NO_DECODED_EDGE'
            stream.write(f'- `{topic}`: `{status}`\n')
        stream.write('\n## Target Edges\n\n')
        stream.write('| parent | child | status | recorded TF topics | transform count | source nodes |\n')
        stream.write('|---|---|---|---|---:|---|\n')
        for edge in result.target_edges:
            stream.write(
                f'| `{edge.parent_frame}` | `{edge.child_frame}` | `{edge.status}` | '
                f'`{", ".join(edge.observed_topics) or "MISSING"}` | {edge.transform_count} | '
                f'`{edge.source_nodes}` |\n'
            )
        reverse_targets = [
            edge for edge in result.target_edges
            if edge.status == 'OBSERVED_REVERSE_EDGE_SOURCE_UNKNOWN'
        ]
        if reverse_targets:
            stream.write('\n`OBSERVED_REVERSE_EDGE_SOURCE_UNKNOWN` means the requested parent/child direction was not recorded, but the inverse TF edge was observed; verify frame convention before diagnosing a conflict.\n')
        stream.write('\n## All Observed Direct Edges\n\n')
        if not result.tf_edges:
            stream.write('`MISSING`: no direct TF edges were decoded from the selected topics.\n')
        else:
            stream.write('| parent | child | status | recorded TF topics | transform count |\n')
            stream.write('|---|---|---|---|---:|\n')
            for edge in result.tf_edges:
                stream.write(
                    f'| `{edge.parent_frame}` | `{edge.child_frame}` | `{edge.status}` | '
                    f'`{", ".join(edge.observed_topics)}` | {edge.transform_count} |\n'
                )
        stream.write('\n## Interpretation\n\n')
        if any(edge.status == 'POSSIBLE_CONFLICT_MULTIPLE_TF_TOPICS' for edge in result.tf_edges):
            stream.write('- `WARNING`: the same frame edge was recorded on both `/tf` and `/tf_static`; verify runtime publishers and authority live.\n')
        else:
            stream.write('- No multi-topic TF evidence was found in the bag. Publisher node multiplicity remains `UNKNOWN` offline.\n')
        stream.write('- To identify actual nodes, use live `ros2 topic info -v /tf` and inspect `tf2_monitor` while the stack is running.\n')


def write_odom_runtime_report(
    output_path: Path,
    result: OdomRuntimeResult,
    plan_topic: str = '/plan',
) -> None:
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Odom Runtime Audit Report\n\n')
        stream.write('本报告只使用 rosbag 记录时间、消息和 TF 内容。publisher node 数量及 TF authority 不在标准 rosbag2 消息记录中，因此不可离线可靠确定时标来源。\n\n')
        stream.write('## Publisher Count\n\n')
        stream.write('| topic | status | publisher_count | publisher_nodes | message_count |\n')
        stream.write('|---|---|---|---|---:|\n')
        for item in result.odom_metrics:
            stream.write(f'| `{item.topic}` | `{item.status}` | `{item.publisher_count}` | `{item.publisher_nodes}` | {item.message_count} |\n')
        stream.write('\n`UNKNOWN` means the topic exists in the bag but publisher provenance was not recorded; it is not an assumption of one publisher.\n\n')

        stream.write('## Frequency Analysis\n\n')
        stream.write('| topic | frequency | class | assessment | anomalous intervals |\n')
        stream.write('|---|---:|---|---|---:|\n')
        for item in result.odom_metrics:
            frequency = 'MISSING' if item.frequency_hz is None else f'{item.frequency_hz:.3f} Hz'
            stream.write(f'| `{item.topic}` | `{frequency}` | `{item.frequency_class}` | `{item.frequency_assessment}` | {item.anomalous_intervals} |\n')
        stream.write('\n- `NEAR_50HZ` and `NEAR_100HZ` use the configured ±10% classification tolerance.\n')
        stream.write('- Frequency is evidence of timing behavior only; it does not prove publisher count.\n')
        stream.write('- CSV: `odom_frequency.csv`\n\n')

        stream.write('## TF Source Analysis\n\n')
        stream.write('- Report: `tf_conflict_report.md`\n')
        stream.write('- Required edges are checked explicitly: `odom -> base_link`, `base_link -> base_footprint`.\n\n')

        stream.write('## Planner Association\n\n')
        stream.write(f'Plan topic: `{plan_topic}`\n\n')
        stream.write('| odom topic | status | anomalous intervals | plan count | plan changes | changes during anomaly | assessment |\n')
        stream.write('|---|---|---:|---:|---:|---:|---|\n')
        for item in result.plan_relations:
            stream.write(
                f'| `{item.topic}` | `{item.status}` | {item.anomalous_intervals} | {item.plan_count} | '
                f'{item.plan_change_count} | {item.plan_changes_during_anomaly} | `{item.assessment}` |\n'
            )
        stream.write('\nPlan-change association uses recorded plan timestamps and the existing planner runtime change metric when available. It is temporal association, not causal proof.\n\n')

        stream.write('## Diagnosis\n\n')
        found = [item for item in result.odom_metrics if item.status == 'FOUND']
        if not found:
            stream.write('`MISSING`: no configured odometry topic was present in the bag; no frequency or runtime diagnosis is made.\n')
        else:
            near_100 = [item for item in found if item.frequency_class == 'NEAR_100HZ']
            abnormal = [item for item in found if item.frequency_assessment == 'ABNORMAL_FREQUENCY']
            overlap = sum(item.plan_changes_during_anomaly for item in result.plan_relations)
            if near_100:
                stream.write('`WARNING`: at least one odometry topic is near 100 Hz. This is compatible with two 50 Hz publishers only if they share the final topic; publisher count remains `UNKNOWN`.\n')
            elif abnormal:
                stream.write('`WARNING`: at least one odometry topic has a frequency outside the configured 50/100 Hz bands. Inspect runtime scheduling, replay, and publisher ownership.\n')
            else:
                stream.write('No odometry frequency outside the configured 50/100 Hz bands was detected for present topics.\n')
            if overlap:
                stream.write(f'Plan changes overlapping odometry timing anomalies: `{overlap}`. This is a correlation signal for runtime investigation.\n')
            else:
                stream.write('No recorded plan change was found inside an odometry timing anomaly interval.\n')
        stream.write('\nThis audit does not modify the chassis code, Nav2 code, or runtime parameters.\n')
