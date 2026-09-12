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
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from agt_robot_interfaces.msg import LocalizationStatus


PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'


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
        self.tracker_metrics = {key: [] for key in (
            'fitness', 'overlap', 'translation_innovation_m',
            'yaw_innovation_deg', 'hessian_condition_number')}
        self.nav = {
            'map_server_active': False, 'planner_server_active': False,
            'planner_action_available': False, 'tf_map_odom': False,
            'tf_odom_base': False, 'planner_test': 'NOT_REQUESTED',
        }

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
        for key, values in self.tracker_metrics.items():
            value = payload.get(key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                values.append(float(value))

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
            'fitness': _numeric_summary(self.tracker_metrics['fitness']),
            'overlap': _numeric_summary(self.tracker_metrics['overlap']),
            'translation_innovation_m': _numeric_summary(self.tracker_metrics['translation_innovation_m']),
            'yaw_innovation_deg': _numeric_summary(self.tracker_metrics['yaw_innovation_deg']),
            'hessian_condition_number': _numeric_summary(self.tracker_metrics['hessian_condition_number']),
        }
        if not self.tracker_expected:
            return WARN, summary, ['map tracking was not enabled for this replay']
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
        return {'gates': gates, 'overall': 'NOT_READY' if any(
            gate['status'] == FAIL for gate in gates.values()) else 'READY'}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_frozen_map(package_dir: Path, navigation_map: Path, evidence_path: Path | None,
                       declared_status: str) -> tuple[str, dict, list[str]]:
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
        if evidence_path:
            evidence = yaml.safe_load(evidence_path.read_text(encoding='utf-8')) or {}
            evidence_status = evidence.get('acceptance_status')
            if evidence_status != PASS:
                reasons.append(f'P0.5 evidence status is {evidence_status!r}')
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
                   'p0_p05_evidence': evidence_source}
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        return FAIL, {}, [f'map package validation failed: {exc}']
    return (FAIL if reasons else PASS), details, reasons


class ReplayAuditNode(Node):
    def __init__(self) -> None:
        super().__init__('agt_replay_audit')
        default_package = '/home/yangxuan/ros2_ws/agt_data/maps/bunker_mid360_mapping_20260901_205036/v003-indexed'
        for name, value in {
            'report_dir': '', 'navigation_map': '', 'map_package_dir': default_package,
            'map_gate_evidence': '', 'map_gate_status': PASS,
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
            str(self.get_parameter('map_gate_status').value).upper())
        self.metrics = ReplayMetrics(
            min_lio_messages=int(self.get_parameter('min_lio_messages').value),
            max_lio_delay_sec=float(self.get_parameter('max_lio_delay_sec').value),
            max_lio_delay_growth_sec_per_sec=float(self.get_parameter('max_lio_delay_growth_sec_per_sec').value),
            min_lio_delay_growth_sec=float(self.get_parameter('min_lio_delay_growth_sec').value),
            min_localized_ratio=float(self.get_parameter('min_localized_ratio').value),
            tracker_expected=bool(self.get_parameter('enable_map_tracking').value))
        self._events = (self.report_dir / 'events.jsonl').open('w', encoding='utf-8')
        self._last_clock_wall: float | None = None
        self._finalized = False
        self._tf = Buffer(); self._listener = TransformListener(self._tf, self)
        self._lifecycle = {name: self.create_client(GetState, f'/{name}/get_state')
                           for name in ('map_server', 'planner_server')}
        self._lifecycle_pending = set()
        self.create_subscription(ClockMessage, '/clock', self._on_clock, 100)
        self.create_subscription(Odometry, '/agt/odometry/local', self._on_odom, 100)
        self.create_subscription(LocalizationStatus, '/agt/localization/status', self._on_localization, 50)
        self.create_subscription(String, '/agt/map_tracking/status', self._on_tracker, 50)
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
        self._event('lio', {'clock_sec': replay_time, 'header_sec': header_time,
                            'delay_sec': replay_time - header_time})

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
            active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        except Exception:
            active = False
        self.metrics.nav[f'{name}_active'] = bool(active)

    def _update_nav_observations(self) -> None:
        self._request_lifecycle()
        services = {name for name, _ in self.get_service_names_and_types()}
        self.metrics.nav['planner_action_available'] = (
            '/compute_path_to_pose/_action/send_goal' in services)
        try:
            self._tf.lookup_transform('map', 'odom', rclpy.time.Time())
            self.metrics.nav['tf_map_odom'] = True
            self._tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.metrics.nav['tf_odom_base'] = True
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
        self._update_nav_observations()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
