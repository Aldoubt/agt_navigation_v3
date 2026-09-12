"""Side-channel, deterministic evidence recorder for offline acceptance replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from agt_robot_interfaces.msg import LocalizationStatus


PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'
DISABLED = 'DISABLED'
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT)


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lo, hi = int(math.floor(position)), int(math.ceil(position))
    if lo == hi:
        return float(ordered[lo])
    return float(ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo))


def _trend(samples: list[tuple[float, float]]) -> float | None:
    """Least-squares delay slope in seconds delay / second replay time."""
    if len(samples) < 2:
        return None
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    mean_x, mean_y = statistics.fmean(xs), statistics.fmean(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator <= 1.0e-12:
        return None
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denominator


def _numeric_summary(values: list[float]) -> dict:
    return {
        'count': len(values), 'p50': _percentile(values, 50),
        'p95': _percentile(values, 95), 'p99': _percentile(values, 99),
        'max': max(values) if values else None,
        'mean': (sum(values) / len(values)) if values else None,
        'stddev': (math.sqrt(sum((value - (sum(values) / len(values))) ** 2 for value in values) / len(values))
                   if values else None),
    }


class ReplayMetrics:
    """ROS-independent accumulation and acceptance rules, intentionally deterministic."""

    def __init__(self, *, min_lio_messages: int = 10,
                 max_lio_delay_sec: float = 0.50,
                 max_lio_delay_growth_sec_per_sec: float = 0.002,
                 min_lio_delay_growth_sec: float = 0.10,
                 min_localized_ratio: float = 0.80,
                 tracker_expected: bool = False) -> None:
        self.min_lio_messages = min_lio_messages
        self.max_lio_delay_sec = max_lio_delay_sec
        self.max_lio_delay_growth_sec_per_sec = max_lio_delay_growth_sec_per_sec
        self.min_lio_delay_growth_sec = min_lio_delay_growth_sec
        self.min_localized_ratio = min_localized_ratio
        self.tracker_expected = tracker_expected
        self.clock_start: float | None = None
        self.clock_end: float | None = None
        self.lio_samples: list[tuple[float, float]] = []
        self.upstream_samples: list[tuple[float, float]] = []
        self.first_local_odom: float | None = None
        self.localization_events = 0
        self.anchor_time: float | None = None
        self.localization_state = 'UNKNOWN'
        self.localization_state_since: float | None = None
        self.localized_seconds = 0.0
        self.degraded_count = 0
        self.lost_count = 0
        self._last_localization_state: str | None = None
        self.tracker_states: dict[str, int] = {}
        self.tracker_decisions: dict[str, int] = {}
        self.tracker_reason_codes: dict[str, int] = {}
        self.tracker_metrics = {key: [] for key in (
            'fitness', 'overlap', 'translation_innovation_m',
            'translation_innovation_dx_m', 'translation_innovation_dy_m',
            'translation_innovation_dz_m', 'yaw_innovation_deg',
            'yaw_innovation_signed_deg', 'hessian_condition_number')}
        self.tracker_accepted_metrics = {key: [] for key in (
            'translation_innovation_m', 'translation_innovation_dx_m',
            'translation_innovation_dy_m', 'translation_innovation_dz_m',
            'yaw_innovation_deg', 'yaw_innovation_signed_deg')}
        self.manual = {
            'manual_seed_received': False,
            'manual_seed_rejected_count': 0,
            'manual_gicp_attempts': 0,
            'manual_gicp_accepted': False,
            'manual_gicp_last_reason': None,
            'status_counts': {},
            'last_status': None,
            'relocalization_pose_published': False,
            'relocalization_pose_count': 0,
            'last_fitness': None,
            'last_overlap': None,
        }
        self.stationary_windows: list[dict] = []
        self.nav = {
            'map_server_active': False, 'planner_server_active': False,
            'planner_action_available': False, 'tf_map_odom': False,
            'tf_odom_base': False, 'planner_test': 'NOT_REQUESTED',
            'map_server_ever_active': False, 'planner_server_ever_active': False,
            'map_server_final_state': 'UNKNOWN', 'planner_server_final_state': 'UNKNOWN',
            'tf_map_odom_runtime': self._tf_evidence(),
            'tf_odom_base_runtime': self._tf_evidence(),
        }

    @staticmethod
    def _tf_evidence() -> dict:
        return {'ever_seen': False, 'first_seen_clock': None,
                'last_seen_clock': None, 'sample_count': 0,
                'check_count': 0, 'availability': 0.0}

    def advance_clock(self, replay_time: float) -> None:
        if self.clock_start is None:
            self.clock_start = replay_time
        if self.clock_end is not None and self.localization_state_since is not None:
            delta = max(0.0, replay_time - self.clock_end)
            if self.localization_state == 'LOCALIZED':
                self.localized_seconds += delta
        self.clock_end = replay_time

    def observe_lio(self, replay_time: float, header_time: float) -> None:
        self.advance_clock(replay_time)
        if self.first_local_odom is None:
            self.first_local_odom = replay_time
        self.lio_samples.append((replay_time, replay_time - header_time))

    def observe_upstream_lio(self, replay_time: float, header_time: float) -> None:
        self.advance_clock(replay_time)
        self.upstream_samples.append((replay_time, replay_time - header_time))

    def observe_localization(self, replay_time: float, state: str, global_anchor: bool) -> None:
        self.advance_clock(replay_time)
        self.localization_events += 1
        if global_anchor and self.anchor_time is None:
            self.anchor_time = replay_time
        if self._last_localization_state != state:
            if state == 'DEGRADED':
                self.degraded_count += 1
            if state == 'LOST':
                self.lost_count += 1
            self._last_localization_state = state
        self.localization_state = state
        self.localization_state_since = replay_time

    def observe_tracker(self, payload: dict) -> None:
        state = str(payload.get('state', '')).strip()
        if not state:
            return
        self.tracker_states[state] = self.tracker_states.get(state, 0) + 1
        decision = str(payload.get('decision', '')).strip()
        if decision:
            self.tracker_decisions[decision] = self.tracker_decisions.get(decision, 0) + 1
        for code in payload.get('reason_codes', []):
            code = str(code).strip()
            if code:
                self.tracker_reason_codes[code] = self.tracker_reason_codes.get(code, 0) + 1
        for key, values in self.tracker_metrics.items():
            value = payload.get(key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                values.append(float(value))
        if decision == 'ACCEPT':
            for key, values in self.tracker_accepted_metrics.items():
                value = payload.get(key)
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    values.append(float(value))

    def observe_manual_status(self, payload: dict) -> None:
        state = str(payload.get('state', '')).strip()
        if not state:
            return
        manual = self.manual
        manual['status_counts'][state] = manual['status_counts'].get(state, 0) + 1
        manual['last_status'] = state
        reason = str(payload.get('detail', '')).strip() or None
        if state == 'MANUAL_SEED_REJECTED':
            # A rejection is emitted by the manual-seed callback, so it proves
            # delivery even though the stationary/GICP gate declined the seed.
            manual['manual_seed_received'] = True
            manual['manual_seed_rejected_count'] += 1
            manual['manual_gicp_last_reason'] = reason
        elif state == 'MANUAL_GICP_REFINING':
            manual['manual_seed_received'] = True
            manual['manual_gicp_attempts'] += 1
            manual['manual_gicp_last_reason'] = reason
        elif state == 'MANUAL_GICP_REJECTED':
            manual['manual_gicp_last_reason'] = reason
        elif state == 'MANUAL_GICP_ACCEPTED':
            manual['manual_seed_received'] = True
            manual['manual_gicp_accepted'] = True
            manual['manual_gicp_last_reason'] = reason
            for source, target in (('fitness', 'last_fitness'), ('overlap', 'last_overlap')):
                value = payload.get(source)
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    manual[target] = float(value)

    def observe_relocalization_pose(self) -> None:
        self.manual['relocalization_pose_published'] = True
        self.manual['relocalization_pose_count'] += 1

    def close_stationary_window(self, window: dict | None) -> None:
        if window and window['end_bag_time'] - window['start_bag_time'] >= 0.75:
            self.stationary_windows.append(window)

    def observe_tf(self, key: str, replay_time: float | None) -> None:
        evidence = self.nav[f'{key}_runtime']
        evidence['ever_seen'] = True
        evidence['sample_count'] += 1
        if replay_time is not None:
            if evidence['first_seen_clock'] is None:
                evidence['first_seen_clock'] = replay_time
            evidence['last_seen_clock'] = replay_time

    def observe_tf_check(self, key: str) -> None:
        self.nav[f'{key}_runtime']['check_count'] += 1

    def finish_runtime_observations(self) -> None:
        for key in ('tf_map_odom', 'tf_odom_base'):
            evidence = self.nav[f'{key}_runtime']
            evidence['availability'] = (evidence['sample_count'] / evidence['check_count']
                                        if evidence['check_count'] else 0.0)

    def _lio_summary(self) -> tuple[str, dict, list[str]]:
        reasons = []
        delays = [sample[1] for sample in self.lio_samples]
        if not self.lio_samples:
            return FAIL, {'message_count': 0}, ['no local odometry received']
        span = self.lio_samples[-1][0] - self.lio_samples[0][0]
        frequency = ((len(self.lio_samples) - 1) / span) if span > 0.0 else None
        slope = _trend(self.lio_samples)
        growth = max(delays) - min(delays) if delays else 0.0
        summary = {
            'message_count': len(self.lio_samples), 'frequency_hz': frequency,
            'clock_to_header_delay_sec': _numeric_summary(delays),
            'delay_trend_sec_per_sec': slope, 'delay_growth_sec': growth,
        }
        upstream = self.upstream_samples
        if upstream:
            span_u = upstream[-1][0] - upstream[0][0]
            summary['batch_lio_upstream'] = {
                'topic': '/aft_mapped_to_init', 'message_count': len(upstream),
                'frequency_hz': ((len(upstream)-1)/span_u) if span_u > 0 else None,
                'clock_to_header_delay_sec': _numeric_summary([x[1] for x in upstream]),
                'adapter_forward_ratio': len(self.lio_samples) / len(upstream),
            }
        if len(self.lio_samples) < self.min_lio_messages:
            return FAIL, summary, [f'fewer than {self.min_lio_messages} local odometry messages']
        if (slope is not None and slope > self.max_lio_delay_growth_sec_per_sec
                and growth >= self.min_lio_delay_growth_sec):
            return FAIL, summary, ['clock-to-header delay grows over replay time (backlog)']
        if _percentile(delays, 95) is not None and _percentile(delays, 95) > self.max_lio_delay_sec:
            reasons.append('p95 clock-to-header delay exceeds advisory threshold')
        if any(delay < -1.0e-3 for delay in delays):
            reasons.append('some odometry headers are ahead of latest /clock')
        return (WARN if reasons else PASS), summary, reasons

    def _localization_summary(self) -> tuple[str, dict, list[str]]:
        duration = ((self.clock_end - self.clock_start)
                    if self.clock_start is not None and self.clock_end is not None else 0.0)
        ratio = self.localized_seconds / duration if duration > 0.0 else 0.0
        start = self.clock_start
        summary = {
            'time_to_first_local_odom_sec': (
                self.first_local_odom - start if start is not None and self.first_local_odom is not None else None),
            'time_to_global_anchor_sec': (
                self.anchor_time - start if start is not None and self.anchor_time is not None else None),
            'localized_duration_ratio': ratio,
            'degraded_count': self.degraded_count, 'lost_count': self.lost_count,
            'final_state': self.localization_state, 'status_message_count': self.localization_events,
            **self.manual,
            'stationary_windows': self.stationary_windows,
        }
        if self.first_local_odom is None or self.anchor_time is None or self.localization_events == 0:
            return FAIL, summary, ['local odometry, global anchor, or localization status is missing']
        if self.lost_count:
            return FAIL, summary, ['localization entered LOST']
        if self.localization_state != 'LOCALIZED':
            return FAIL, summary, [f'final localization state is {self.localization_state}']
        if ratio < self.min_localized_ratio:
            return WARN, summary, ['localized duration ratio is below advisory threshold']
        if self.degraded_count:
            return WARN, summary, ['localization entered DEGRADED']
        return PASS, summary, []

    def _tracker_summary(self) -> tuple[str, dict, list[str]]:
        attempts = sum(self.tracker_states.get(state, 0) for state in (
            'TRACKING_OK', 'HOLD', 'DEGRADED', 'RECOVERY_REQUIRED'))
        summary = {
            'attempts': attempts, 'states': dict(sorted(self.tracker_states.items())),
            'decisions': dict(sorted(self.tracker_decisions.items())),
            'failure_reason_histogram': dict(sorted(self.tracker_reason_codes.items())),
            'success_rate': (self.tracker_decisions.get('ACCEPT', 0) / attempts) if attempts else 0.0,
            'fitness': _numeric_summary(self.tracker_metrics['fitness']),
            'overlap': _numeric_summary(self.tracker_metrics['overlap']),
            'translation_innovation_m': _numeric_summary(self.tracker_metrics['translation_innovation_m']),
            'translation_innovation_dx_m': _numeric_summary(self.tracker_metrics['translation_innovation_dx_m']),
            'translation_innovation_dy_m': _numeric_summary(self.tracker_metrics['translation_innovation_dy_m']),
            'translation_innovation_dz_m': _numeric_summary(self.tracker_metrics['translation_innovation_dz_m']),
            'yaw_innovation_deg': _numeric_summary(self.tracker_metrics['yaw_innovation_deg']),
            'yaw_innovation_signed_deg': _numeric_summary(self.tracker_metrics['yaw_innovation_signed_deg']),
            'hessian_condition_number': _numeric_summary(self.tracker_metrics['hessian_condition_number']),
            'accepted_correction': {
                key: _numeric_summary(values)
                for key, values in self.tracker_accepted_metrics.items()
            },
        }
        if not self.tracker_expected:
            return DISABLED, summary, ['map tracking was not enabled for this replay']
        if attempts == 0:
            return FAIL, summary, ['map tracking was enabled but no attempts were observed']
        if self.tracker_states.get('RECOVERY_REQUIRED', 0):
            return FAIL, summary, ['map tracker entered RECOVERY_REQUIRED']
        if self.tracker_states.get('DEGRADED', 0) or self.tracker_states.get('HOLD', 0):
            return WARN, summary, ['map tracker had rejected/degraded samples; no recovery was requested']
        return PASS, summary, []

    def _nav_summary(self, map_status: str) -> tuple[str, dict, list[str]]:
        summary = dict(self.nav)
        missing = [key for key in ('map_server_active', 'planner_server_active',
                                   'planner_action_available', 'tf_map_odom', 'tf_odom_base')
                   if not summary[key]]
        if map_status == FAIL or missing:
            return FAIL, summary, ['missing ' + ', '.join(missing)] if missing else ['MAP failed']
        if summary['planner_test'] == 'NOT_REQUESTED':
            return PASS, summary, ['planner-only goal was not requested; rosbag cannot validate closed-loop control']
        return PASS, summary, []

    def summary(self, map_status: str, map_summary: dict, map_reasons: list[str]) -> dict:
        self.finish_runtime_observations()
        lio_status, lio, lio_reasons = self._lio_summary()
        loc_status, localization, loc_reasons = self._localization_summary()
        tracker_status, tracker, tracker_reasons = self._tracker_summary()
        nav_status, nav, nav_reasons = self._nav_summary(map_status)
        gates = {
            'MAP': {'status': map_status, 'details': map_summary, 'reasons': map_reasons},
            'LIO': {'status': lio_status, 'details': lio, 'reasons': lio_reasons},
            'LOCALIZATION': {'status': loc_status, 'details': localization, 'reasons': loc_reasons},
            'MAP TRACKER': {'status': tracker_status, 'details': tracker, 'reasons': tracker_reasons},
            'NAV2 PLAN': {'status': nav_status, 'details': nav, 'reasons': nav_reasons},
        }
        software_ready = not any(gate['status'] == FAIL for gate in gates.values())
        return {'gates': gates, 'overall': 'READY' if software_ready else 'NOT_READY',
                'software_runtime_ready': software_ready,
                'field_acceptance_ready': software_ready and map_status == PASS}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _inspect_relocalization_contract(package: dict, query_frame_mode: str) -> tuple[str, str]:
    """Check a declared formal PGO contract without invalidating immutable legacy maps."""
    contract = package.get('relocalization_contract')
    if contract is None:
        return 'LEGACY_UNDECLARED', ''
    if not isinstance(contract, dict):
        return 'INVALID', 'relocalization_contract must be a mapping'
    expected = {
        'formal_pose_semantics': 'T_map_body',
        'map_cloud_frame': 'body',
        'query_frame_mode': 'mapping_body',
        'query_frame': 'body',
    }
    mismatches = [f'{key}={contract.get(key)!r}' for key, value in expected.items()
                  if contract.get(key) != value]
    if mismatches:
        return 'INVALID', 'relocalization contract mismatch: ' + ', '.join(mismatches)
    if query_frame_mode not in ('mapping_body', 'body_aligned'):
        return 'MISMATCH', (
            f'formal PGO package requires mapping_body but replay requested {query_frame_mode!r}')
    return 'MATCH', ''


def inspect_frozen_map(package_dir: Path, navigation_map: Path, evidence_path: Path | None,
                       declared_status: str, query_frame_mode: str = 'mapping_body') -> tuple[str, dict, list[str]]:
    """Validate the frozen package without regenerating or modifying map assets."""
    reasons = []
    try:
        package = yaml.safe_load((package_dir / 'metadata.yaml').read_text(encoding='utf-8')) or {}
        assets = package.get('assets') or {}
        nav_asset, loc_asset, relocal_asset = (assets.get('navigation_map') or {},
                                                assets.get('localization_map') or {},
                                                assets.get('relocalization_assets') or {})
        expected_nav = package_dir / str(nav_asset.get('path', ''))
        expected_loc = package_dir / str(loc_asset.get('path', ''))
        expected_relocal = package_dir / str(relocal_asset.get('path', ''))
        if navigation_map.resolve() != expected_nav.resolve():
            reasons.append('launch navigation map is not the frozen package navigation map')
        if _sha256(expected_nav) != nav_asset.get('sha256'):
            reasons.append('navigation map hash mismatch')
        if _sha256(expected_loc) != loc_asset.get('sha256'):
            reasons.append('localization PCD hash mismatch')
        if not (expected_relocal / 'polar_context.db').is_file() or not (expected_relocal / 'voxelmaps_coords').is_dir():
            reasons.append('relocalization Polar Context or BBS assets are missing')
        contract_status, contract_reason = _inspect_relocalization_contract(
            package, query_frame_mode)
        if contract_reason:
            reasons.append(contract_reason)
        if evidence_path:
            evidence = yaml.safe_load(evidence_path.read_text(encoding='utf-8')) or {}
            evidence_status = evidence.get('acceptance_status')
            if evidence_status not in (PASS, 'REVIEW'):
                reasons.append(f'MAP evidence status is {evidence_status!r}')
            evidence_map = Path(str((evidence.get('map') or {}).get('map_directory', ''))).expanduser()
            if not evidence_map.is_dir() or evidence_map.resolve() != expected_nav.parent.resolve():
                reasons.append('P0.5 evidence does not identify this frozen navigation directory')
            evidence_source = str(evidence_path)
        else:
            evidence_source = 'launch-declared frozen P0/P0.5 result'
            if declared_status != PASS:
                reasons.append(f'launch-declared MAP gate is {declared_status!r}')
        details = {'map_id': package.get('map_id'), 'map_version': package.get('map_version'),
                   'package_dir': str(package_dir), 'navigation_map': str(expected_nav),
                   'p0_p05_evidence': evidence_source,
                   'relocalization_contract_status': contract_status,
                   'requested_query_frame_mode': query_frame_mode}
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        return FAIL, {}, [f'map package validation failed: {exc}']
    return (FAIL if reasons else (evidence_status if evidence_path else PASS)), details, reasons


class ReplayAuditNode(Node):
    def __init__(self) -> None:
        super().__init__('agt_replay_audit')
        default_package = '/home/yangxuan/ros2_ws/agt_data/maps/bunker_mid360_mapping_20260901_205036/v003-indexed'
        for name, value in {
            'report_dir': '', 'navigation_map': '', 'map_package_dir': default_package,
            'map_gate_evidence': '', 'map_gate_status': PASS,
            'global_query_frame_mode': 'mapping_body',
            'enable_map_tracking': False, 'min_lio_messages': 10,
            'max_lio_delay_sec': 0.50, 'max_lio_delay_growth_sec_per_sec': 0.002,
            'min_lio_delay_growth_sec': 0.10, 'min_localized_ratio': 0.80,
            'finalize_after_clock_idle_wall_sec': 10.0,
        }.items():
            self.declare_parameter(name, value)
        raw_report_dir = str(self.get_parameter('report_dir').value).strip()
        if not raw_report_dir:
            raise ValueError('report_dir is required when replay audit is enabled')
        report_root = Path(raw_report_dir).expanduser()
        self.report_dir = report_root / datetime.now(timezone.utc).strftime('replay_%Y%m%dT%H%M%SZ')
        self.report_dir.mkdir(parents=True, exist_ok=False)
        evidence = Path(str(self.get_parameter('map_gate_evidence').value)).expanduser()
        self.map_status, self.map_details, self.map_reasons = inspect_frozen_map(
            Path(str(self.get_parameter('map_package_dir').value)),
            Path(str(self.get_parameter('navigation_map').value)),
            evidence if str(evidence) else None,
            str(self.get_parameter('map_gate_status').value).upper(),
            str(self.get_parameter('global_query_frame_mode').value).strip())
        self.metrics = ReplayMetrics(
            min_lio_messages=int(self.get_parameter('min_lio_messages').value),
            max_lio_delay_sec=float(self.get_parameter('max_lio_delay_sec').value),
            max_lio_delay_growth_sec_per_sec=float(self.get_parameter('max_lio_delay_growth_sec_per_sec').value),
            min_lio_delay_growth_sec=float(self.get_parameter('min_lio_delay_growth_sec').value),
            min_localized_ratio=float(self.get_parameter('min_localized_ratio').value),
            tracker_expected=bool(self.get_parameter('enable_map_tracking').value))
        self._events = (self.report_dir / 'events.jsonl').open('w', encoding='utf-8')
        self._last_clock_wall: float | None = None
        self._previous_odom: Odometry | None = None
        self._stationary_window: dict | None = None
        self._finalized = False
        self._tf = Buffer(); self._listener = TransformListener(self._tf, self)
        self._lifecycle = {name: self.create_client(GetState, f'/{name}/get_state')
                           for name in ('map_server', 'planner_server')}
        self._lifecycle_pending = set()
        # rosbag2 /clock is normally best-effort.  The default reliable QoS
        # would silently receive no replay time and invalidate latency data.
        self.create_subscription(ClockMessage, '/clock', self._on_clock, CLOCK_QOS)
        self.create_subscription(Odometry, '/agt/odometry/local', self._on_odom, 100)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self._on_upstream_odom, 100)
        self.create_subscription(LocalizationStatus, '/agt/localization/status', self._on_localization, 50)
        self.create_subscription(String, '/agt/map_tracking/status', self._on_tracker, 50)
        self.create_subscription(String, '/agt/global_relocalization/status', self._on_global_relocalization, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/agt/relocalization/pose',
                                 self._on_relocalization_pose, 10)
        self.create_timer(1.0, self._on_wall_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Replay auditor recording to {self.report_dir}')

    @staticmethod
    def _stamp(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    def _event(self, kind: str, data: dict) -> None:
        self._events.write(json.dumps({'kind': kind, **data}, sort_keys=True) + '\n')
        self._events.flush()

    def _on_clock(self, msg: ClockMessage) -> None:
        replay_time = self._stamp(msg.clock)
        self.metrics.advance_clock(replay_time)
        self._last_clock_wall = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        if self.metrics.clock_end is None:
            return
        replay_time = self.metrics.clock_end
        header_time = self._stamp(msg.header.stamp)
        self.metrics.observe_lio(replay_time, header_time)
        motion = self._stationary_motion(msg)
        pose = msg.pose.pose
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                         1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        stationary = motion is not None and motion[0] <= 0.05 and motion[1] <= 0.08
        if stationary:
            if self._stationary_window is None:
                self._stationary_window = {'start_bag_time': header_time,
                                           'end_bag_time': header_time,
                                           'max_linear_speed_mps': motion[0],
                                           'max_angular_speed_rps': motion[1],
                                           'center_pose': None}
            window = self._stationary_window
            window['end_bag_time'] = header_time
            window['max_linear_speed_mps'] = max(window['max_linear_speed_mps'], motion[0])
            window['max_angular_speed_rps'] = max(window['max_angular_speed_rps'], motion[1])
            window['center_pose'] = {'bag_time': header_time, 'x': pose.position.x,
                                     'y': pose.position.y, 'z': pose.position.z, 'yaw': yaw}
        else:
            self.metrics.close_stationary_window(self._stationary_window)
            self._stationary_window = None
        self._event('lio', {'clock_sec': replay_time, 'header_sec': header_time,
                            'delay_sec': replay_time - header_time,
                            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
                            'yaw': yaw, 'stationary': stationary,
                            'linear_motion_mps': motion[0] if motion else None,
                            'angular_motion_rps': motion[1] if motion else None})

    def _on_upstream_odom(self, msg: Odometry) -> None:
        if self.metrics.clock_end is None:
            return
        now = self.metrics.clock_end
        stamp = self._stamp(msg.header.stamp)
        self.metrics.observe_upstream_lio(now, stamp)
        self._event('lio_upstream', {'clock_sec': now, 'header_sec': stamp,
                                     'delay_sec': now - stamp})

    def _stationary_motion(self, current: Odometry) -> tuple[float, float] | None:
        previous = self._previous_odom
        self._previous_odom = current
        if previous is None:
            return None
        t0, t1 = self._stamp(previous.header.stamp), self._stamp(current.header.stamp)
        dt = t1 - t0
        if dt <= 1.0e-4 or dt > 1.0:
            return None
        p0, p1 = previous.pose.pose.position, current.pose.pose.position
        pose_linear = math.sqrt((p1.x-p0.x)**2 + (p1.y-p0.y)**2 + (p1.z-p0.z)**2) / dt
        q0, q1 = previous.pose.pose.orientation, current.pose.pose.orientation
        n0 = math.sqrt(q0.x*q0.x + q0.y*q0.y + q0.z*q0.z + q0.w*q0.w)
        n1 = math.sqrt(q1.x*q1.x + q1.y*q1.y + q1.z*q1.z + q1.w*q1.w)
        if n0 <= 1.0e-12 or n1 <= 1.0e-12:
            return None
        dot = min(1.0, max(-1.0, abs((q0.x*q1.x + q0.y*q1.y + q0.z*q1.z + q0.w*q1.w)/(n0*n1))))
        pose_angular = 2.0 * math.acos(dot) / dt
        twist = current.twist.twist
        twist_linear = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        twist_angular = math.sqrt(twist.angular.x**2 + twist.angular.y**2 + twist.angular.z**2)
        return max(twist_linear, pose_linear), max(twist_angular, pose_angular)

    def _state_name(self, state: int) -> str:
        names = {LocalizationStatus.STATE_LOCALIZED: 'LOCALIZED',
                 LocalizationStatus.STATE_DEGRADED: 'DEGRADED',
                 LocalizationStatus.STATE_LOST: 'LOST'}
        return names.get(state, f'STATE_{state}')

    def _on_localization(self, msg: LocalizationStatus) -> None:
        if self.metrics.clock_end is None:
            return
        state = self._state_name(msg.state)
        self.metrics.observe_localization(self.metrics.clock_end, state, bool(msg.global_correction_valid))
        self._event('localization', {'clock_sec': self.metrics.clock_end, 'state': state,
                                     'global_correction_valid': bool(msg.global_correction_valid)})

    def _on_tracker(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._event('tracker_malformed', {'payload': msg.data})
            return
        self.metrics.observe_tracker(payload)
        self._event('tracker', payload)

    def _on_global_relocalization(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._event('global_relocalization_malformed', {'payload': msg.data})
            return
        self.metrics.observe_manual_status(payload)
        self._event('global_relocalization', {
            'clock_sec': self.metrics.clock_end, **payload})

    def _on_relocalization_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.metrics.observe_relocalization_pose()
        self._event('relocalization_pose', {
            'clock_sec': self.metrics.clock_end,
            'header_sec': self._stamp(msg.header.stamp),
            'frame_id': msg.header.frame_id,
        })

    def _request_lifecycle(self) -> None:
        for name, client in self._lifecycle.items():
            if name in self._lifecycle_pending or not client.service_is_ready():
                continue
            self._lifecycle_pending.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda done, node_name=name: self._on_lifecycle(node_name, done))

    def _on_lifecycle(self, name, future) -> None:
        self._lifecycle_pending.discard(name)
        try:
            state = future.result().current_state
            active = state.id == State.PRIMARY_STATE_ACTIVE
            final_state = state.label or str(state.id)
        except Exception:
            active = False
            final_state = 'UNAVAILABLE'
        self.metrics.nav[f'{name}_active'] = bool(active)
        self.metrics.nav[f'{name}_final_state'] = final_state
        if active:
            self.metrics.nav[f'{name}_ever_active'] = True

    def _update_nav_observations(self) -> None:
        self._request_lifecycle()
        services = {name for name, _ in self.get_service_names_and_types()}
        self.metrics.nav['planner_action_available'] = (
            '/compute_path_to_pose/_action/send_goal' in services)
        for target, source, key in (('map', 'odom', 'tf_map_odom'),
                                    ('odom', 'base_link', 'tf_odom_base')):
            self.metrics.observe_tf_check(key)
            try:
                self._tf.lookup_transform(target, source, rclpy.time.Time())
                self.metrics.nav[key] = True
                self.metrics.observe_tf(key, self.metrics.clock_end)
            except TransformException:
                pass

    def _on_wall_tick(self) -> None:
        self._update_nav_observations()
        if self._last_clock_wall is None or self._finalized:
            return
        idle = time.monotonic() - self._last_clock_wall
        if idle >= float(self.get_parameter('finalize_after_clock_idle_wall_sec').value):
            self.finalize()

    def finalize(self) -> None:
        if self._finalized:
            return
        # During launch SIGINT rclpy may already be shut down.  Keep the
        # observations accumulated so far and still write a valid report.
        if rclpy.ok():
            self._update_nav_observations()
        self.metrics.close_stationary_window(self._stationary_window)
        self._stationary_window = None
        report = self.metrics.summary(self.map_status, self.map_details, self.map_reasons)
        report['report_format_version'] = 1
        report['report_dir'] = str(self.report_dir)
        (self.report_dir / 'summary.yaml').write_text(yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
        self._events.close()
        self._finalized = True
        print('\nReplay acceptance summary')
        for name, gate in report['gates'].items():
            print(f"{name:<13} {gate['status']}")
        print(f"overall       {report['overall']}")
        print(f'report        {self.report_dir}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplayAuditNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        # ros2 launch may already have shut down the default context while
        # propagating SIGINT.  Preserve the evidence report in either order.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
