"""Offline planner-to-odometry tracking and cmd_vel dynamics analysis."""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, List, Sequence

import numpy as np


@dataclass
class PlanSnapshot:
    timestamp_ns: int
    points: np.ndarray
    smoothness_score: float
    frame_id: str = ''


@dataclass
class TrackingSample:
    odom_timestamp_ns: int
    plan_timestamp_ns: int
    odom_x: float
    odom_y: float
    nearest_x: float
    nearest_y: float
    nearest_point_index: int
    cross_track_error: float


@dataclass
class TrackingSummary:
    num_odom_messages: int
    matched_messages: int
    unmatched_messages: int
    mean_error: float
    max_error: float
    p95_error: float
    plan_frame_ids: tuple = ()
    odometry_frame_ids: tuple = ()
    source_odometry_frame_ids: tuple = ()
    frame_mismatch: bool = False
    tf_transformed_messages: int = 0
    tf_untransformed_messages: int = 0


@dataclass
class CmdVelDynamics:
    timestamp_ns: int
    linear_x: float
    angular_z: float
    dt_sec: float
    linear_acceleration: float
    angular_acceleration: float


@dataclass
class DynamicsSummary:
    num_messages: int
    mean_abs_linear_acceleration: float
    max_abs_linear_acceleration: float
    mean_abs_angular_acceleration: float
    max_abs_angular_acceleration: float
    p95_abs_angular_acceleration: float


def make_plan_snapshot(message, timestamp_ns: int, smoothness_score: float = 0.0) -> PlanSnapshot:
    points = np.asarray(
        [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses],
        dtype=float,
    )
    frame_id = str(getattr(getattr(message, 'header', None), 'frame_id', '') or '')
    return PlanSnapshot(timestamp_ns, points, float(smoothness_score), frame_id)


def _select_plan(snapshots: Sequence[PlanSnapshot], timestamp_ns: int, plan_hold_sec: float):
    if not snapshots or timestamp_ns < snapshots[0].timestamp_ns:
        return None
    for index in range(len(snapshots) - 1, -1, -1):
        if snapshots[index].timestamp_ns <= timestamp_ns:
            if index == len(snapshots) - 1:
                valid_until = snapshots[index].timestamp_ns + int(plan_hold_sec * 1e9)
            else:
                valid_until = snapshots[index + 1].timestamp_ns
            return snapshots[index] if timestamp_ns < valid_until else None
    return None


def _quaternion_to_yaw(quaternion) -> float:
    return float(np.arctan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    ))


def transform_odometry_to_frame(odometry: Iterable, tf_records: Iterable, target_frame: str):
    """Apply the latest direct 2D TF edge to odometry samples.

    This intentionally handles the common ``map -> odom`` case used by Nav2.
    It does not invent a transform when TF is missing or when only a multi-hop
    chain is available; those samples retain their original frame and are
    reported as untransformed.
    """
    odometry = sorted(odometry, key=lambda item: item.timestamp_ns)
    if not odometry or not target_frame:
        return list(odometry), 0, len(odometry)

    edges = []
    for record in tf_records:
        for transform in getattr(record.message, 'transforms', []):
            parent = str(getattr(transform.header, 'frame_id', '') or '')
            child = str(getattr(transform, 'child_frame_id', '') or '')
            if not parent or not child:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            edges.append((int(record.timestamp_ns), parent, child,
                          float(translation.x), float(translation.y),
                          _quaternion_to_yaw(rotation)))
    edges.sort(key=lambda item: item[0])

    transformed = []
    transformed_count = 0
    untransformed_count = 0
    for sample in odometry:
        source_frame = str(getattr(sample, 'frame_id', '') or '')
        if not source_frame or source_frame == target_frame:
            transformed.append(replace(sample, frame_id=target_frame) if source_frame == target_frame else sample)
            if source_frame == target_frame:
                transformed_count += 1
            else:
                untransformed_count += 1
            continue
        candidates = [edge for edge in edges if edge[1] == target_frame and edge[2] == source_frame and edge[0] <= sample.timestamp_ns]
        inverse = False
        if not candidates:
            candidates = [edge for edge in edges if edge[1] == source_frame and edge[2] == target_frame and edge[0] <= sample.timestamp_ns]
            inverse = True
        if not candidates:
            transformed.append(sample)
            untransformed_count += 1
            continue
        _, _, _, tx, ty, yaw = candidates[-1]
        if inverse:
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            dx = float(sample.x) - tx
            dy = float(sample.y) - ty
            x = cos_yaw * dx + sin_yaw * dy
            y = -sin_yaw * dx + cos_yaw * dy
        else:
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            x = cos_yaw * float(sample.x) - sin_yaw * float(sample.y) + tx
            y = sin_yaw * float(sample.x) + cos_yaw * float(sample.y) + ty
        transformed.append(replace(sample, x=float(x), y=float(y), frame_id=target_frame))
        transformed_count += 1
    return transformed, transformed_count, untransformed_count


def analyze_tracking(
    snapshots: Sequence[PlanSnapshot],
    odometry: Iterable,
    plan_hold_sec: float = 2.0,
):
    snapshots = sorted(snapshots, key=lambda item: item.timestamp_ns)
    samples: List[TrackingSample] = []
    odometry = sorted(odometry, key=lambda item: item.timestamp_ns)
    unmatched = 0
    for odom in odometry:
        plan = _select_plan(snapshots, odom.timestamp_ns, plan_hold_sec)
        if plan is None or len(plan.points) == 0:
            unmatched += 1
            continue
        position = np.asarray([odom.x, odom.y], dtype=float)
        distances = np.linalg.norm(plan.points - position, axis=1)
        nearest_index = int(np.argmin(distances))
        nearest = plan.points[nearest_index]
        samples.append(TrackingSample(
            odom_timestamp_ns=odom.timestamp_ns,
            plan_timestamp_ns=plan.timestamp_ns,
            odom_x=float(odom.x),
            odom_y=float(odom.y),
            nearest_x=float(nearest[0]),
            nearest_y=float(nearest[1]),
            nearest_point_index=nearest_index,
            cross_track_error=float(distances[nearest_index]),
        ))
    errors = np.asarray([item.cross_track_error for item in samples], dtype=float)
    summary = TrackingSummary(
        num_odom_messages=len(odometry),
        matched_messages=len(samples),
        unmatched_messages=unmatched,
        mean_error=float(np.mean(errors)) if len(errors) else 0.0,
        max_error=float(np.max(errors)) if len(errors) else 0.0,
        p95_error=float(np.percentile(errors, 95)) if len(errors) else 0.0,
        plan_frame_ids=tuple(sorted({item.frame_id for item in snapshots if item.frame_id})),
        odometry_frame_ids=tuple(sorted({str(getattr(item, 'frame_id', '') or '') for item in odometry if getattr(item, 'frame_id', '')})),
    )
    summary.frame_mismatch = bool(
        summary.plan_frame_ids and summary.odometry_frame_ids
        and set(summary.plan_frame_ids).isdisjoint(summary.odometry_frame_ids)
    )
    return samples, summary


def write_tracking_csv(samples: Iterable[TrackingSample], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'odom_timestamp', 'plan_timestamp', 'odom_x', 'odom_y',
            'nearest_x', 'nearest_y', 'nearest_point_index', 'cross_track_error',
        ])
        for item in samples:
            writer.writerow([
                f'{item.odom_timestamp_ns / 1e9:.9f}',
                f'{item.plan_timestamp_ns / 1e9:.9f}',
                f'{item.odom_x:.6f}', f'{item.odom_y:.6f}',
                f'{item.nearest_x:.6f}', f'{item.nearest_y:.6f}',
                item.nearest_point_index, f'{item.cross_track_error:.6f}',
            ])


def analyze_cmd_vel_dynamics(samples: Iterable) -> tuple[List[CmdVelDynamics], DynamicsSummary]:
    samples = sorted(samples, key=lambda item: item.timestamp_ns)
    dynamics: List[CmdVelDynamics] = []
    previous = None
    for sample in samples:
        if previous is None:
            dt = 0.0
            linear_acceleration = 0.0
            angular_acceleration = 0.0
        else:
            dt = max((sample.timestamp_ns - previous.timestamp_ns) / 1e9, 0.0)
            if dt > 1e-9:
                linear_acceleration = (sample.linear_x - previous.linear_x) / dt
                angular_acceleration = (sample.angular_z - previous.angular_z) / dt
            else:
                linear_acceleration = 0.0
                angular_acceleration = 0.0
        dynamics.append(CmdVelDynamics(
            sample.timestamp_ns, sample.linear_x, sample.angular_z,
            dt, linear_acceleration, angular_acceleration,
        ))
        previous = sample
    angular = np.asarray([abs(item.angular_acceleration) for item in dynamics], dtype=float)
    linear = np.asarray([abs(item.linear_acceleration) for item in dynamics], dtype=float)
    summary = DynamicsSummary(
        num_messages=len(dynamics),
        mean_abs_linear_acceleration=float(np.mean(linear)) if len(linear) else 0.0,
        max_abs_linear_acceleration=float(np.max(linear)) if len(linear) else 0.0,
        mean_abs_angular_acceleration=float(np.mean(angular)) if len(angular) else 0.0,
        max_abs_angular_acceleration=float(np.max(angular)) if len(angular) else 0.0,
        p95_abs_angular_acceleration=float(np.percentile(angular, 95)) if len(angular) else 0.0,
    )
    return dynamics, summary


def write_cmd_vel_dynamics_csv(dynamics: Iterable[CmdVelDynamics], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp', 'linear_x', 'angular_z', 'dt_sec',
            'linear_acceleration', 'angular_acceleration', 'abs_angular_acceleration',
        ])
        for item in dynamics:
            writer.writerow([
                f'{item.timestamp_ns / 1e9:.9f}', f'{item.linear_x:.6f}', f'{item.angular_z:.6f}',
                f'{item.dt_sec:.6f}', f'{item.linear_acceleration:.6f}',
                f'{item.angular_acceleration:.6f}', f'{abs(item.angular_acceleration):.6f}',
            ])


def assess_controller_tracking(paths, tracking: TrackingSummary, thresholds=None):
    """Classify planner geometry and odometry-to-plan tracking evidence.

    The classification deliberately stays heuristic: it describes the evidence
    in the bag and does not claim that a single layer is the proven root cause.
    """
    thresholds = thresholds or {}
    smooth_threshold = float(thresholds.get('smoothness_score_smooth', 0.70))
    mean_error_threshold = float(thresholds.get('high_mean_error_m', 0.10))
    p95_error_threshold = float(thresholds.get('high_p95_error_m', 0.20))
    mean_smoothness = float(np.mean([item.smoothness_score for item in paths])) if paths else 0.0
    path_bad = bool(paths) and mean_smoothness < smooth_threshold
    tracking_bad = bool(tracking.matched_messages) and (
        tracking.mean_error >= mean_error_threshold or tracking.p95_error >= p95_error_threshold
    )
    if tracking.frame_mismatch:
        case = 'FRAME_MISMATCH'
        diagnosis = (
            'Plan and odometry frames differ, so the reported raw cross-track error is not '
            'physically comparable until TF transformation is applied.'
        )
    elif not tracking.matched_messages:
        case = 'INSUFFICIENT_DATA'
        diagnosis = 'No odometry samples were matched to a valid /plan time window.'
    elif path_bad and tracking_bad:
        case = 'C'
        diagnosis = 'Planner geometry and controller tracking both show degradation.'
    elif path_bad and not tracking_bad:
        case = 'B'
        diagnosis = 'Path geometry is unstable, while odometry remains close to the path. Investigate the planner.'
    elif not path_bad and tracking_bad:
        case = 'A'
        diagnosis = 'Path is smooth, but tracking error is high. Investigate the controller or base/odometry layer.'
    else:
        case = 'STABLE'
        diagnosis = 'No high planner smoothness loss or high cross-track error was detected.'
    return {
        'case': case,
        'mean_smoothness': mean_smoothness,
        'smoothness_threshold': smooth_threshold,
        'path_bad': path_bad,
        'tracking_bad': tracking_bad,
        'mean_error_threshold_m': mean_error_threshold,
        'p95_error_threshold_m': p95_error_threshold,
        'diagnosis': diagnosis,
    }


def write_controller_tracking_report(
    output_path: Path,
    paths,
    tracking: TrackingSummary,
    dynamics: DynamicsSummary,
    assessment,
) -> None:
    """Write the standalone planner/controller tracking report."""
    max_curvature_p95 = max((item.curvature_p95 for item in paths), default=0.0)
    mean_sharp_turn_ratio = float(np.mean([item.sharp_turn_ratio for item in paths])) if paths else 0.0
    planner_assessment = 'SMOOTH' if not assessment['path_bad'] else 'UNSTABLE'
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Controller Tracking Analysis Report\n\n')
        stream.write('## Planner\n\n')
        stream.write(f"- Path smoothness score: `{assessment['mean_smoothness']:.3f}`\n")
        stream.write(f'- Curvature P95 (maximum across paths): `{max_curvature_p95:.3f} rad/m`\n')
        stream.write(f'- Sharp turn ratio (mean): `{mean_sharp_turn_ratio:.3f}`\n')
        stream.write(f'- Assessment: `{planner_assessment}`\n\n')

        stream.write('## Tracking\n\n')
        stream.write(f'- Odometry messages: `{tracking.num_odom_messages}`\n')
        stream.write(f'- Matched to a plan: `{tracking.matched_messages}`\n')
        stream.write(f'- Unmatched: `{tracking.unmatched_messages}`\n')
        stream.write(f"- Plan frame(s): `{', '.join(tracking.plan_frame_ids) or 'unknown'}`\n")
        stream.write(f"- Odometry frame(s): `{', '.join(tracking.odometry_frame_ids) or 'unknown'}`\n")
        stream.write(f"- Source odometry frame(s): `{', '.join(tracking.source_odometry_frame_ids) or 'unknown'}`\n")
        stream.write(f'- Frame mismatch: `{tracking.frame_mismatch}`\n')
        stream.write(f'- TF-transformed odometry: `{tracking.tf_transformed_messages}`\n')
        stream.write(f'- TF-untransformed odometry: `{tracking.tf_untransformed_messages}`\n')
        stream.write(f'- Mean cross-track error: `{tracking.mean_error:.4f} m`\n')
        stream.write(f'- Maximum cross-track error: `{tracking.max_error:.4f} m`\n')
        stream.write(f'- P95 cross-track error: `{tracking.p95_error:.4f} m`\n')
        stream.write('- CSV: `controller_tracking.csv`\n\n')

        stream.write('## Controller Dynamics\n\n')
        stream.write(f'- cmd_vel messages: `{dynamics.num_messages}`\n')
        stream.write(f'- Mean absolute linear acceleration: `{dynamics.mean_abs_linear_acceleration:.4f} m/s^2`\n')
        stream.write(f'- Maximum absolute linear acceleration: `{dynamics.max_abs_linear_acceleration:.4f} m/s^2`\n')
        stream.write(f'- Mean absolute angular acceleration: `{dynamics.mean_abs_angular_acceleration:.4f} rad/s^2`\n')
        stream.write(f'- Maximum absolute angular acceleration: `{dynamics.max_abs_angular_acceleration:.4f} rad/s^2`\n')
        stream.write(f'- P95 absolute angular acceleration: `{dynamics.p95_abs_angular_acceleration:.4f} rad/s^2`\n')
        stream.write('- CSV: `cmd_vel_dynamics.csv`\n\n')

        stream.write('## Diagnosis\n\n')
        stream.write(f"- Case: `{assessment['case']}`\n")
        stream.write(f"- Planner smoothness threshold: `{assessment['smoothness_threshold']:.3f}`\n")
        stream.write(f"- High mean tracking-error threshold: `{assessment['mean_error_threshold_m']:.3f} m`\n")
        stream.write(f"- High P95 tracking-error threshold: `{assessment['p95_error_threshold_m']:.3f} m`\n\n")
        stream.write(f"**Diagnosis:** {assessment['diagnosis']}\n\n")
        if tracking.frame_mismatch:
            stream.write('WARNING: Raw nearest-waypoint distances are retained for auditability, but should not be used as controller tracking error until `/tf` is applied.\n\n')
        stream.write('The result is an offline evidence classification. Review the timestamp association and per-sample CSV before changing controller or planner parameters.\n')
