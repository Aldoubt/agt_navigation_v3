#!/usr/bin/env python3
"""Trace every observed TF child frame to its DDS publisher and reject duplicates."""

from __future__ import annotations

import argparse
import json
import sys
import subprocess
from collections import defaultdict
from pathlib import Path

import yaml


def evaluate_observations(expected: list[dict], observations: dict, owners: dict) -> dict:
    frames = []
    failures = []
    for child in sorted(observations):
        entries = observations[child]
        parents = sorted({entry['parent'] for entry in entries})
        publishers = sorted({owners.get(entry['gid'], f'gid:{entry["gid"]}') for entry in entries})
        topics = sorted({entry['topic'] for entry in entries})
        status = 'PASS'
        reasons = []
        if len(parents) != 1:
            status = 'FAIL'
            reasons.append('multiple_parents')
        if len({entry['gid'] for entry in entries}) != 1:
            status = 'FAIL'
            reasons.append('multiple_publishers')
        if status == 'FAIL':
            failures.append(f'{child}:{"+".join(reasons)}')
        frames.append({
            'child': child,
            'parents': parents,
            'publishers': publishers,
            'publisher_gids': sorted({entry['gid'] for entry in entries}),
            'topics': topics,
            'message_count': sum(entry['count'] for entry in entries),
            'status': status,
            'reasons': reasons,
        })

    expected_results = []
    by_child = {item['child']: item for item in frames}
    for spec in expected:
        child = str(spec['child'])
        observed = by_child.get(child)
        reasons = []
        if observed is None:
            reasons.append('edge_not_observed')
        else:
            if observed['parents'] != [str(spec['parent'])]:
                reasons.append(
                    f'parent_mismatch:{observed["parents"]}!={str(spec["parent"])}')
            if observed['publishers'] != [str(spec['publisher'])]:
                reasons.append(
                    f'publisher_mismatch:{observed["publishers"]}!={str(spec["publisher"])}')
            if str(spec.get('topic', '')) and observed['topics'] != [str(spec['topic'])]:
                reasons.append(f'topic_mismatch:{observed["topics"]}')
        status = 'PASS' if not reasons else 'FAIL'
        if reasons:
            failures.append(f'{spec["parent"]}->{child}:{";".join(reasons)}')
        expected_results.append({**spec, 'status': status, 'reasons': reasons})

    return {
        'verdict': 'PASS' if not failures else 'FAIL',
        'failures': failures,
        'expected_edges': expected_results,
        'observed_frames': frames,
    }


def collect_probe(duration: float, timeout: float) -> tuple[dict, dict]:
    command = [
        'ros2', 'run', 'agt_system_bringup', 'agt_tf_authority_probe',
        '--duration-sec', str(duration),
    ]
    completed = subprocess.run(
        command, capture_output=True, text=True, timeout=duration + timeout + 10.0,
        check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(
            f'TF authority probe failed ({completed.returncode}): {detail}')

    observations: dict[str, dict[tuple[str, str, str], int]] = defaultdict(dict)
    owners: dict[str, str] = {}
    for raw_line in completed.stdout.splitlines():
        fields = raw_line.split('\t')
        if fields[0] == 'PUB' and len(fields) == 4:
            _, _topic, gid, owner = fields
            owners[gid] = owner
        elif fields[0] == 'TF' and len(fields) == 6:
            _, topic, parent, child, gid, count = fields
            key = (parent, gid, topic)
            observations[child][key] = observations[child].get(key, 0) + int(count)
    flattened = {
        child: [
            {'parent': parent, 'gid': gid, 'topic': topic, 'count': count}
            for (parent, gid, topic), count in entries.items()
        ]
        for child, entries in observations.items()
    }
    return flattened, owners


def load_profile(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if data.get('schema_version') != 1 or 'tf_audit' not in data:
        raise ValueError('profile requires schema_version: 1 and tf_audit')
    return data['tf_audit']


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

    duration = args.duration_sec or float(profile.get('sample_duration_sec', 8.0))
    discovery_timeout = float(profile.get('discovery_timeout_sec', 5.0))
    try:
        observations, owners = collect_probe(duration, discovery_timeout)
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError) as exc:
        print(f'probe error: {exc}', file=sys.stderr)
        return 2

    result = evaluate_observations(
        list(profile.get('expected_edges', [])),
        observations, owners)
    payload = {
        'schema_version': 1,
        'kind': 'tf_authority_audit',
        'sample_duration_sec': duration,
        **result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    for edge in result['expected_edges']:
        print(
            f'{edge["status"]:4s} {edge["parent"]} -> {edge["child"]} '
            f'owner={edge["publisher"]}')
    for failure in result['failures']:
        print(f'FAIL {failure}', file=sys.stderr)

    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
