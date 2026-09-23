"""Recorder profile generation and rosbag completeness checks."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
from pathlib import Path
from typing import Dict, Iterable, List

import yaml

from .bag_reader import RosbagReader
from .ros_graph_discovery import discover_ros_graph


@dataclass
class CategoryCheck:
    name: str
    description: str
    expected_topics: List[str]
    found_topics: List[str]
    missing_topics: List[str]
    status: str


def load_recorder_profile(path: Path) -> Dict:
    path = Path(path)
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def discover_live_topics() -> List[str]:
    return discover_ros_graph().topics


def _topic_aliases(profile_category: Dict, topic: str) -> List[str]:
    aliases = profile_category.get('aliases', {}) or {}
    values = aliases.get(topic, [])
    return [values] if isinstance(values, str) else list(values)


def _matches(available: Iterable[str], pattern: str) -> List[str]:
    return sorted(topic for topic in set(available) if fnmatchcase(topic, pattern))


def check_profile(profile: Dict, available_topics: Iterable[str]) -> List[CategoryCheck]:
    available = set(available_topics)
    checks = []
    for name, category in (profile.get('required_topics', {}) or {}).items():
        expected = list(category.get('topics', []) or [])
        found = []
        missing = []
        for topic in expected:
            candidates = [topic] + _topic_aliases(category, topic)
            matches = []
            for candidate in candidates:
                for match in _matches(available, candidate):
                    if match not in matches:
                        matches.append(match)
            if not matches:
                missing.append(topic)
            else:
                for match in matches:
                    if match not in found:
                        found.append(match)
        if not missing:
            status = 'PASS'
        elif found:
            status = 'PARTIAL'
        else:
            status = 'MISSING'
        checks.append(CategoryCheck(
            name=name,
            description=str(category.get('description', '')),
            expected_topics=expected,
            found_topics=found,
            missing_topics=missing,
            status=status,
        ))
    return checks


def completeness_confidence(checks: Iterable[CategoryCheck]) -> str:
    values = {item.name: item.status for item in checks}
    core = [values.get(name, 'MISSING') for name in ('planning', 'controller', 'localization')]
    if all(status == 'PASS' for status in core):
        return 'HIGH' if all(status == 'PASS' for status in values.values()) else 'MEDIUM'
    if all(status in ('PASS', 'PARTIAL') for status in core) and any(status == 'PASS' for status in core):
        return 'MEDIUM'
    return 'LOW'


def write_record_command(profile: Dict, available_topics: Iterable[str], output_path: Path) -> List[str]:
    checks = check_profile(profile, available_topics)
    topics = []
    for check in checks:
        for topic in check.found_topics:
            if topic not in topics:
                topics.append(topic)
    lines = ['#!/usr/bin/env bash', 'set -e', '', '# Generated from the live ROS 2 topic graph.', 'ros2 bag record \\']
    if topics:
        lines.extend([f'  {topic} \\' for topic in topics[:-1]])
        lines.append(f'  {topics[-1]}')
    else:
        lines = ['#!/usr/bin/env bash', 'set -e', '', '# No profile topics were found in the live ROS 2 graph.', 'echo "No matching topics found" >&2', 'exit 1']
    output_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    output_path.chmod(0o755)
    return topics


def write_completeness_report(bag_path: Path, profile: Dict, checks: Iterable[CategoryCheck], output_path: Path) -> str:
    checks = list(checks)
    confidence = completeness_confidence(checks)
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Dataset Completeness Report\n\n')
        stream.write(f'- bag: `{bag_path}`\n')
        stream.write(f'- confidence: `{confidence}`\n\n')
        stream.write('| category | status | found topics | missing topics |\n|---|---|---|---|\n')
        for check in checks:
            found = ', '.join(f'`{item}`' for item in check.found_topics) or '—'
            missing = ', '.join(f'`{item}`' for item in check.missing_topics) or '—'
            stream.write(f'| {check.name} | **{check.status}** | {found} | {missing} |\n')
        stream.write('\n## Category Details\n\n')
        for check in checks:
            stream.write(f'### {check.name.capitalize()}\n\n')
            if check.description:
                stream.write(f'{check.description}\n\n')
            stream.write(f'- Status: `{check.status}`\n')
            stream.write(f'- Expected: {", ".join(f"`{item}`" for item in check.expected_topics)}\n')
        stream.write('\n## Recording Recommendation\n\n')
        if confidence == 'HIGH':
            stream.write('The profile is complete enough for high-confidence offline diagnosis.\n')
        elif confidence == 'MEDIUM':
            stream.write('Core navigation data is available, but missing runtime/goal topics reduce causal confidence.\n')
        else:
            stream.write('Core navigation data is incomplete; re-record the missing planning/controller/localization topics before diagnosis.\n')
    return confidence


def analyze_bag_completeness(bag_path: Path, profile_path: Path, output_path: Path) -> str:
    profile = load_recorder_profile(profile_path)
    available = RosbagReader(str(bag_path)).available_topics()
    checks = check_profile(profile, available)
    return write_completeness_report(bag_path, profile, checks, output_path)
