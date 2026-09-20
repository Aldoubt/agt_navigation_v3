#!/usr/bin/env python3
"""Measure all hardware-acceptance topic rates concurrently."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def evaluate_rate(spec: dict, stamps_ns: list[int], minimum_span_sec: float) -> dict:
    count = len(stamps_ns)
    span_sec = (stamps_ns[-1] - stamps_ns[0]) / 1e9 if count >= 2 else 0.0
    rate_hz = (count - 1) / span_sec if span_sec > 0.0 else 0.0
    required = bool(spec.get('required', True))
    reasons: list[str] = []
    if count < 2:
        reasons.append('no_samples' if count == 0 else 'only_one_sample')
    elif span_sec < minimum_span_sec:
        reasons.append(f'sample_span_too_short:{span_sec:.3f}s')
    if rate_hz < float(spec['min_hz']):
        reasons.append(f'rate_below_min:{rate_hz:.3f}<{float(spec["min_hz"]):.3f}')
    if rate_hz > float(spec['max_hz']):
        reasons.append(f'rate_above_max:{rate_hz:.3f}>{float(spec["max_hz"]):.3f}')
    if not required and reasons:
        status = 'WARN'
    else:
        status = 'PASS' if not reasons else 'FAIL'
    return {
        'name': str(spec['name']),
        'required': required,
        'status': status,
        'count': count,
        'sample_span_sec': span_sec,
        'rate_hz': rate_hz,
        'min_hz': float(spec['min_hz']),
        'max_hz': float(spec['max_hz']),
        'reasons': reasons,
    }


class RateAudit(Node):
    def __init__(self) -> None:
        super().__init__('agt_hardware_topic_rate_audit')
        self.receipts: dict[str, list[int]] = {}
        self.subscriptions = []

    def add_topic(self, topic: str, type_name: str) -> None:
        message_type = get_message(type_name)
        self.receipts[topic] = []
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def callback(_message, *, key=topic):
            self.receipts[key].append(time.monotonic_ns())

        self.subscriptions.append(
            self.create_subscription(message_type, topic, callback, qos))


def load_profile(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if data.get('schema_version') != 1 or 'topic_rate_audit' not in data:
        raise ValueError('profile requires schema_version: 1 and topic_rate_audit')
    return data['topic_rate_audit']


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration-sec', type=float, default=0.0)
    args = parser.parse_args(argv)

    try:
        profile = load_profile(args.profile)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'profile error: {exc}', file=sys.stderr)
        return 2
    duration = args.duration_sec or float(profile.get('sample_duration_sec', 15.0))
    discovery_timeout = float(profile.get('discovery_timeout_sec', 8.0))
    minimum_span = float(profile.get('minimum_sample_span_sec', 5.0))
    if duration <= 0.0 or minimum_span <= 0.0:
        print('duration and minimum sample span must be positive', file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = RateAudit()
    missing_types: dict[str, str] = {}
    deadline = time.monotonic() + discovery_timeout
    unresolved = {str(item['name']) for item in profile['topics']}
    topic_types: dict[str, str] = {}
    while unresolved and time.monotonic() < deadline:
        discovered = dict(node.get_topic_names_and_types())
        for topic in list(unresolved):
            types = discovered.get(topic, [])
            if types:
                topic_types[topic] = types[0]
                unresolved.remove(topic)
        rclpy.spin_once(node, timeout_sec=0.2)

    for spec in profile['topics']:
        topic = str(spec['name'])
        if topic not in topic_types:
            missing_types[topic] = 'topic_not_discovered'
            node.receipts[topic] = []
            continue
        try:
            node.add_topic(topic, topic_types[topic])
        except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as exc:
            missing_types[topic] = f'type_load_failed:{exc}'
            node.receipts[topic] = []

    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = []
    for spec in profile['topics']:
        result = evaluate_rate(spec, node.receipts[str(spec['name'])], minimum_span)
        if str(spec['name']) in missing_types:
            result['reasons'].insert(0, missing_types[str(spec['name'])])
        results.append(result)

    failed = [item for item in results if item['required'] and item['status'] == 'FAIL']
    payload = {
        'schema_version': 1,
        'kind': 'topic_rate_audit',
        'verdict': 'PASS' if not failed else 'FAIL',
        'sample_duration_sec': duration,
        'topics': results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    for item in results:
        print(
            f'{item["status"]:4s} {item["name"]:<40s} '
            f'{item["rate_hz"]:8.3f} Hz samples={item["count"]}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == '__main__':
    raise SystemExit(main())
