"""Offline legacy-versus-shadow localization comparison utilities.

This module is deliberately read-only.  It consumes rosbag2 data emitted by the
legacy localization manager and the v1 shadow manager; it does not create ROS
nodes, publish TF, or change any localization runtime behaviour.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Any, Iterable, Sequence


LEGACY_METRICS_TOPIC = '/agt/localization/metrics'
LEGACY_STATUS_TOPIC = '/agt/localization/status'
LOCAL_ODOM_TOPIC = '/agt/odometry/local'
GLOBAL_POSE_TOPIC = '/agt/relocalization/pose'
TRACKING_POSE_TOPIC = '/agt/map_tracking/pose'
TRACKING_STATUS_TOPIC = '/agt/map_tracking/status'
SHADOW_STATE_TOPIC = '/agt/localization/v1/shadow/state'
SHADOW_DIAGNOSTICS_TOPIC = '/agt/localization/v1/shadow/diagnostics'
SHADOW_MAP_ODOM_TOPIC = '/agt/localization/v1/shadow/map_odom'

REPLAY_REQUIRED_TOPICS = {
    LOCAL_ODOM_TOPIC: 'nav_msgs/msg/Odometry',
    LEGACY_STATUS_TOPIC: 'agt_robot_interfaces/msg/LocalizationStatus',
    GLOBAL_POSE_TOPIC: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    TRACKING_POSE_TOPIC: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    TRACKING_STATUS_TOPIC: 'std_msgs/msg/String',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}

RECORD_TOPICS = (
    LOCAL_ODOM_TOPIC,
    GLOBAL_POSE_TOPIC,
    TRACKING_POSE_TOPIC,
    TRACKING_STATUS_TOPIC,
    LEGACY_STATUS_TOPIC,
    LEGACY_METRICS_TOPIC,
    SHADOW_STATE_TOPIC,
    SHADOW_MAP_ODOM_TOPIC,
    SHADOW_DIAGNOSTICS_TOPIC,
    '/tf',
    '/tf_static',
    '/clock',
)

STATUS_ENUM_NAMES = {
    0: 'BOOT',
    1: 'WAIT_LOCAL_ODOM',
    2: 'WAIT_GLOBAL',
    3: 'LOCALIZED',
    4: 'DEGRADED',
    5: 'LOST',
    6: 'RELOCALIZING',
}


@dataclass(frozen=True)
class LegacySample:
    timestamp_ns: int
    state: str
    x: float | None
    y: float | None
    z: float | None
    yaw: float | None
    innovation_translation: float | None
    innovation_yaw: float | None


@dataclass(frozen=True)
class ShadowSample:
    timestamp_ns: int
    state: str
    x: float | None
    y: float | None
    z: float | None
    yaw: float | None
    innovation_translation: float | None
    innovation_yaw: float | None
    correction_decision: str
    reason: str


@dataclass(frozen=True)
class ComparisonRow:
    timestamp_ns: int
    legacy_state: str
    shadow_state: str
    translation_error: float | None
    yaw_error: float | None
    innovation: str
    correction_accept: str
    sync_offset_sec: float


@dataclass(frozen=True)
class ReplayContractResult:
    """Result of the strict, parity-qualified localization replay contract."""

    status: str
    missing_topics: tuple[str, ...]
    zero_message_topics: tuple[str, ...]
    type_mismatches: tuple[tuple[str, str, str], ...]
    topic_counts: dict[str, int]

    @property
    def ok(self) -> bool:
        return self.status == 'OK'


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def timestamp_from_message(message: Any, fallback_ns: int) -> int:
    """Use an explicit ``stamp`` first, then Header.stamp, then bag storage time."""
    stamp = getattr(message, 'stamp', None)
    if stamp is None:
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return int(fallback_ns)
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else int(fallback_ns)


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _state_from_status(message: Any) -> str:
    return STATUS_ENUM_NAMES.get(int(getattr(message, 'state', -1)), 'UNKNOWN')


def _nearest_index(sorted_timestamps: Sequence[int], timestamp_ns: int) -> int | None:
    if not sorted_timestamps:
        return None
    index = bisect.bisect_left(sorted_timestamps, timestamp_ns)
    candidates = []
    if index < len(sorted_timestamps):
        candidates.append(index)
    if index:
        candidates.append(index - 1)
    return min(candidates, key=lambda item: abs(sorted_timestamps[item] - timestamp_ns))


def _vector_error(legacy: LegacySample, shadow: ShadowSample) -> float | None:
    values = (legacy.x, legacy.y, legacy.z, shadow.x, shadow.y, shadow.z)
    if any(value is None for value in values):
        return None
    return math.sqrt(
        (shadow.x - legacy.x) ** 2 +
        (shadow.y - legacy.y) ** 2 +
        (shadow.z - legacy.z) ** 2
    )


def _yaw_error(legacy: LegacySample, shadow: ShadowSample) -> float | None:
    if legacy.yaw is None or shadow.yaw is None:
        return None
    return abs(wrap_to_pi(shadow.yaw - legacy.yaw))


def _innovation_text(shadow: ShadowSample, legacy: LegacySample) -> str:
    translation = shadow.innovation_translation
    yaw = shadow.innovation_yaw
    if translation is None:
        translation = legacy.innovation_translation
    if yaw is None:
        yaw = legacy.innovation_yaw
    return json.dumps(
        {'translation_m': translation, 'yaw_rad': yaw},
        allow_nan=False,
        separators=(',', ':'),
    )


def _normalise_decision(decision: str) -> str:
    value = (decision or 'UNKNOWN').upper()
    if value in {'ACCEPTED', 'ACCEPT'}:
        return 'accepted'
    if value in {'REJECTED', 'REJECT'}:
        return 'rejected'
    if value in {'NONE', 'PENDING'}:
        return value.lower()
    return 'unknown'


def synchronize_samples(
    legacy_samples: Sequence[LegacySample],
    shadow_samples: Sequence[ShadowSample],
    max_sync_sec: float,
) -> tuple[list[ComparisonRow], int]:
    """Nearest-neighbour time synchronization without interpolation.

    Each shadow sample is matched once to the nearest legacy sample.  An
    unmatched shadow sample is counted but never silently converted into a
    zero-error result.
    """
    legacy_sorted = sorted(legacy_samples, key=lambda item: item.timestamp_ns)
    shadow_sorted = sorted(shadow_samples, key=lambda item: item.timestamp_ns)
    timestamps = [item.timestamp_ns for item in legacy_sorted]
    max_offset_ns = int(max_sync_sec * 1_000_000_000)
    rows: list[ComparisonRow] = []
    unmatched = 0
    for shadow in shadow_sorted:
        index = _nearest_index(timestamps, shadow.timestamp_ns)
        if index is None:
            unmatched += 1
            continue
        legacy = legacy_sorted[index]
        offset_ns = shadow.timestamp_ns - legacy.timestamp_ns
        if abs(offset_ns) > max_offset_ns:
            unmatched += 1
            continue
        rows.append(ComparisonRow(
            timestamp_ns=shadow.timestamp_ns,
            legacy_state=legacy.state,
            shadow_state=shadow.state,
            translation_error=_vector_error(legacy, shadow),
            yaw_error=_yaw_error(legacy, shadow),
            innovation=_innovation_text(shadow, legacy),
            correction_accept=_normalise_decision(shadow.correction_decision),
            sync_offset_sec=offset_ns / 1_000_000_000.0,
        ))
    return rows, unmatched


def _available_topics(bag_path: str) -> dict[str, str]:
    from .bag_reader import RosbagReader
    return RosbagReader(bag_path).available_topics()


def _read_records(bag_path: str, topics: Iterable[str]):
    from .bag_reader import RosbagReader
    reader = RosbagReader(bag_path)
    return list(reader.records(list(topics)))


def load_legacy_samples(bag_path: str) -> tuple[list[LegacySample], dict[str, int], list[str]]:
    topics = _available_topics(bag_path)
    requested = [topic for topic in (LEGACY_METRICS_TOPIC, LEGACY_STATUS_TOPIC) if topic in topics]
    warnings: list[str] = []
    if LEGACY_METRICS_TOPIC not in topics:
        warnings.append(f'MISSING {LEGACY_METRICS_TOPIC}; falling back to legacy status without map->odom values.')
    if not requested:
        return [], {topic: 0 for topic in (LEGACY_METRICS_TOPIC, LEGACY_STATUS_TOPIC)}, warnings + [
            'No legacy localization metrics or status topic is available.'
        ]

    metrics: list[LegacySample] = []
    statuses: list[LegacySample] = []
    counts = {topic: 0 for topic in (LEGACY_METRICS_TOPIC, LEGACY_STATUS_TOPIC)}
    for record in _read_records(bag_path, requested):
        message = record.message
        timestamp_ns = timestamp_from_message(message, record.timestamp_ns)
        counts[record.topic] += 1
        if record.topic == LEGACY_METRICS_TOPIC:
            metrics.append(LegacySample(
                timestamp_ns=timestamp_ns,
                state=str(getattr(message, 'state', 'UNKNOWN') or 'UNKNOWN'),
                x=_finite(getattr(message, 'map_odom_x', None)),
                y=_finite(getattr(message, 'map_odom_y', None)),
                z=_finite(getattr(message, 'map_odom_z', None)),
                yaw=_finite(getattr(message, 'map_odom_yaw', None)),
                innovation_translation=_finite(getattr(message, 'tracking_innovation_translation', None)),
                innovation_yaw=_finite(getattr(message, 'tracking_innovation_yaw', None)),
            ))
        else:
            statuses.append(LegacySample(
                timestamp_ns=timestamp_ns,
                state=_state_from_status(message),
                x=None, y=None, z=None, yaw=None,
                innovation_translation=None, innovation_yaw=None,
            ))
    return (metrics or statuses), counts, warnings


def _parse_diagnostic(message: Any) -> dict[str, Any]:
    try:
        parsed = json.loads(str(getattr(message, 'data', '')))
    except json.JSONDecodeError:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def load_shadow_samples(bag_path: str, association_sec: float = 1.0) -> tuple[list[ShadowSample], dict[str, int], list[str]]:
    topics = _available_topics(bag_path)
    requested = [topic for topic in (SHADOW_STATE_TOPIC, SHADOW_DIAGNOSTICS_TOPIC) if topic in topics]
    warnings: list[str] = []
    if SHADOW_STATE_TOPIC not in topics:
        return [], {topic: 0 for topic in (SHADOW_STATE_TOPIC, SHADOW_DIAGNOSTICS_TOPIC)}, [
            f'MISSING required {SHADOW_STATE_TOPIC}.'
        ]
    if SHADOW_DIAGNOSTICS_TOPIC not in topics:
        warnings.append(f'MISSING {SHADOW_DIAGNOSTICS_TOPIC}; correction decision and shadow map->odom will be unavailable.')

    states: list[tuple[int, ShadowSample]] = []
    diagnostics: list[tuple[int, dict[str, Any]]] = []
    counts = {topic: 0 for topic in (SHADOW_STATE_TOPIC, SHADOW_DIAGNOSTICS_TOPIC)}
    for record in _read_records(bag_path, requested):
        counts[record.topic] += 1
        if record.topic == SHADOW_STATE_TOPIC:
            message = record.message
            states.append((record.timestamp_ns, ShadowSample(
                timestamp_ns=timestamp_from_message(message, record.timestamp_ns),
                state=str(getattr(message, 'state', 'UNKNOWN') or 'UNKNOWN'),
                x=None, y=None, z=None, yaw=None,
                innovation_translation=None, innovation_yaw=None,
                correction_decision='UNKNOWN',
                reason=str(getattr(message, 'reason', '') or ''),
            )))
        else:
            diagnostics.append((record.timestamp_ns, _parse_diagnostic(record.message)))

    if not diagnostics:
        return [item[1] for item in states], counts, warnings

    state_storage_times = [item[0] for item in states]
    association_limit_ns = int(association_sec * 1_000_000_000)
    samples: list[ShadowSample] = []
    unpaired = 0
    for diagnostic_storage_ns, diagnostic in diagnostics:
        index = _nearest_index(state_storage_times, diagnostic_storage_ns)
        if index is None or abs(state_storage_times[index] - diagnostic_storage_ns) > association_limit_ns:
            unpaired += 1
            continue
        state = states[index][1]
        samples.append(ShadowSample(
            timestamp_ns=state.timestamp_ns,
            state=str(diagnostic.get('v1_public_state') or state.state),
            x=_finite(diagnostic.get('shadow_map_odom_x')),
            y=_finite(diagnostic.get('shadow_map_odom_y')),
            z=_finite(diagnostic.get('shadow_map_odom_z')),
            yaw=_finite(diagnostic.get('shadow_map_odom_yaw')),
            innovation_translation=_finite(diagnostic.get('tracking_innovation_translation_m')),
            innovation_yaw=_finite(diagnostic.get('tracking_innovation_yaw_rad')),
            correction_decision=str(diagnostic.get('correction_decision') or 'UNKNOWN'),
            reason=str(diagnostic.get('accept_reject_reason') or state.reason),
        ))
    if unpaired:
        warnings.append(f'{unpaired} shadow diagnostics could not be associated with a shadow state by bag storage time.')
    return samples, counts, warnings


def inspect_input_contract(bag_path: str) -> dict[str, int]:
    topics = _available_topics(bag_path)
    selected = (LOCAL_ODOM_TOPIC, GLOBAL_POSE_TOPIC, TRACKING_POSE_TOPIC, TRACKING_STATUS_TOPIC)
    counts = {topic: 0 for topic in selected}
    present = [topic for topic in selected if topic in topics]
    if not present:
        return counts
    for record in _read_records(bag_path, present):
        counts[record.topic] += 1
    return counts


def evaluate_replay_contract(
    topic_types: dict[str, str],
    topic_counts: dict[str, int],
) -> ReplayContractResult:
    """Evaluate required topic names, exact ROS types, and nonzero evidence.

    A topic advertised in rosbag metadata with zero messages is deliberately a
    contract failure: it cannot drive a deterministic localization replay.
    """
    missing = []
    zero_messages = []
    type_mismatches = []
    normalized_counts: dict[str, int] = {}
    for topic, expected_type in REPLAY_REQUIRED_TOPICS.items():
        actual_type = topic_types.get(topic)
        count = int(topic_counts.get(topic, 0))
        normalized_counts[topic] = count
        if actual_type is None:
            missing.append(topic)
            continue
        if actual_type != expected_type:
            type_mismatches.append((topic, expected_type, actual_type))
        if count <= 0:
            zero_messages.append(topic)
    status = 'OK' if not (missing or zero_messages or type_mismatches) else 'BLOCKED_INPUT_CONTRACT'
    return ReplayContractResult(
        status=status,
        missing_topics=tuple(missing),
        zero_message_topics=tuple(zero_messages),
        type_mismatches=tuple(type_mismatches),
        topic_counts=normalized_counts,
    )


def check_replay_contract(bag_path: str) -> ReplayContractResult:
    """Read a bag once and apply the strict Patch 3B replay contract."""
    topic_types = _available_topics(bag_path)
    counts = {topic: 0 for topic in REPLAY_REQUIRED_TOPICS if topic in topic_types}
    if counts:
        for record in _read_records(bag_path, counts):
            counts[record.topic] += 1
    return evaluate_replay_contract(topic_types, counts)


def write_replay_contract_report(result: ReplayContractResult, bag_path: str, output_path: Path) -> None:
    """Write a compact human-readable contract decision."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '# Localization Replay Contract Check',
        '',
        f'- Bag: `{bag_path}`',
        f'- Result: **{result.status}**',
        '',
        '## Required Topics',
        '',
        '| Topic | Required type | Message count |',
        '| --- | --- | ---: |',
    ]
    lines.extend(
        f'| `{topic}` | `{expected}` | {result.topic_counts[topic]} |'
        for topic, expected in REPLAY_REQUIRED_TOPICS.items()
    )
    lines.extend(['', '## Missing Topics', ''])
    lines.extend(f'- `{topic}`' for topic in result.missing_topics) if result.missing_topics else lines.append('- None')
    lines.extend(['', '## Zero-message Topics', ''])
    lines.extend(f'- `{topic}`' for topic in result.zero_message_topics) if result.zero_message_topics else lines.append('- None')
    lines.extend(['', '## Type Mismatches', ''])
    if result.type_mismatches:
        lines.extend(
            f'- `{topic}`: expected `{expected}`, recorded `{actual}`'
            for topic, expected, actual in result.type_mismatches
        )
    else:
        lines.append('- None')
    lines.extend([
        '',
        'A `BLOCKED_INPUT_CONTRACT` result means this bag must not be used for localization shadow correction parity. '
        'This checker does not publish TF, infer a missing observation, or activate v1.',
        '',
    ])
    output_path.write_text('\n'.join(lines), encoding='utf-8')


def write_csv(rows: Sequence[ComparisonRow], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'legacy_state', 'shadow_state', 'translation_error',
            'yaw_error', 'innovation', 'correction_accept',
        ])
        for row in rows:
            writer.writerow([
                f'{row.timestamp_ns / 1_000_000_000.0:.9f}',
                row.legacy_state,
                row.shadow_state,
                '' if row.translation_error is None else f'{row.translation_error:.9f}',
                '' if row.yaw_error is None else f'{row.yaw_error:.9f}',
                row.innovation,
                row.correction_accept,
            ])


def _format_float(value: float | None, digits: int = 6) -> str:
    return 'N/A' if value is None else f'{value:.{digits}f}'


def write_report(
    output_path: Path,
    legacy_bag: str,
    shadow_bag: str,
    input_contract: dict[str, int],
    legacy_counts: dict[str, int],
    shadow_counts: dict[str, int],
    rows: Sequence[ComparisonRow],
    unmatched: int,
    warnings: Sequence[str],
    max_sync_sec: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    numeric_translation = [row.translation_error for row in rows if row.translation_error is not None]
    numeric_yaw = [row.yaw_error for row in rows if row.yaw_error is not None]
    state_matches = sum(row.legacy_state == row.shadow_state for row in rows)
    offsets = [abs(row.sync_offset_sec) for row in rows]
    state_pairs = Counter((row.legacy_state, row.shadow_state) for row in rows)
    decisions = Counter(row.correction_accept for row in rows)
    has_correction_observation = input_contract[GLOBAL_POSE_TOPIC] > 0 or input_contract[TRACKING_POSE_TOPIC] > 0
    has_full_observations = all(input_contract[topic] > 0 for topic in (
        LOCAL_ODOM_TOPIC, GLOBAL_POSE_TOPIC, TRACKING_POSE_TOPIC,
    ))

    lines = [
        '# Localization Shadow Metrics',
        '',
        'This is an offline comparison only. It does not publish TF, change the TF owner, or activate v1.',
        '',
        '## Inputs',
        '',
        f'- Legacy bag: `{legacy_bag}`',
        f'- Shadow bag: `{shadow_bag}`',
        f'- Synchronization: nearest message timestamp, maximum offset `{max_sync_sec:.3f}s`.',
        '',
        '## Recorded Localization Contract',
        '',
        '| Topic | Messages |',
        '| --- | ---: |',
    ]
    lines.extend(f'| `{topic}` | {count} |' for topic, count in input_contract.items())
    lines.extend([
        '',
        'A full correction-parity replay requires local odometry, at least one global relocalization pose, and map-tracking observations. ' +
        ('All three were recorded.' if has_full_observations else 'This input does not meet that complete-replay precondition.'),
        '',
        '## Stream Counts',
        '',
        f'- Legacy metrics: {legacy_counts.get(LEGACY_METRICS_TOPIC, 0)}',
        f'- Legacy status: {legacy_counts.get(LEGACY_STATUS_TOPIC, 0)}',
        f'- Shadow state: {shadow_counts.get(SHADOW_STATE_TOPIC, 0)}',
        f'- Shadow diagnostics: {shadow_counts.get(SHADOW_DIAGNOSTICS_TOPIC, 0)}',
        '',
        '## Time Synchronization',
        '',
        f'- Matched rows: {len(rows)}',
        f'- Unmatched shadow samples: {unmatched}',
        f'- Mean absolute timestamp offset: {_format_float(mean(offsets) if offsets else None)} s',
        f'- Maximum absolute timestamp offset: {_format_float(max(offsets) if offsets else None)} s',
        '',
        '## State and Correction Comparison',
        '',
        f'- Exact state matches: {state_matches}/{len(rows)}' if rows else '- Exact state matches: N/A',
        f'- Translation error samples: {len(numeric_translation)}; mean: {_format_float(mean(numeric_translation) if numeric_translation else None)} m; max: {_format_float(max(numeric_translation) if numeric_translation else None)} m',
        f'- Yaw error samples: {len(numeric_yaw)}; mean: {_format_float(mean(numeric_yaw) if numeric_yaw else None)} rad; max: {_format_float(max(numeric_yaw) if numeric_yaw else None)} rad',
        '- State pairs: ' + (', '.join(
            f'`{legacy}` -> `{shadow}`: {count}'
            for (legacy, shadow), count in sorted(state_pairs.items())
        ) if state_pairs else 'N/A') + '.',
        '- Shadow correction decisions: ' + (', '.join(
            f'`{decision}`: {count}' for decision, count in sorted(decisions.items())
        ) if decisions else 'N/A') + '.',
        '- `innovation` in the CSV is JSON with `translation_m` and `yaw_rad`; it is retained even when no correction was accepted.',
        '',
        '## Validation Result',
        '',
    ])
    if not has_correction_observation:
        lines.extend([
            '**BLOCKED_INPUT_CONTRACT** — neither global relocalization nor map-tracking pose messages were recorded. ' +
            'The legacy correction present in metrics cannot be replayed into shadow, so numeric correction parity is not established.',
        ])
    elif not numeric_translation or not numeric_yaw:
        lines.extend([
            '**INCOMPLETE_SHADOW_OUTPUT** — correction observations exist, but shadow diagnostics contain no comparable map->odom correction.',
        ])
    else:
        lines.extend([
            '**NUMERIC_COMPARISON_AVAILABLE** — inspect the CSV and the reported error statistics before promoting any shadow behaviour.',
        ])
    lines.extend(['', '## Warnings', ''])
    lines.extend(f'- {warning}' for warning in warnings) if warnings else lines.append('- None')
    lines.append('')
    output_path.write_text('\n'.join(lines), encoding='utf-8')
