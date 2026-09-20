#!/usr/bin/env python3
"""Generate a Markdown hardware-acceptance report from captured evidence."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import yaml


def load_json(path: Path) -> tuple[str, dict | None, str]:
    if not path.is_file():
        return 'NOT_RUN', None, 'evidence missing'
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        return 'FAIL', None, f'invalid evidence: {exc}'
    verdict = str(payload.get('verdict', 'FAIL'))
    return (verdict if verdict in {'PASS', 'FAIL'} else 'FAIL'), payload, ''


def runtime_status(path: Path) -> tuple[str, str]:
    if not path.is_file():
        return 'NOT_RUN', 'evidence missing'
    text = path.read_text(encoding='utf-8', errors='replace')
    if 'PASS runtime owner uniqueness' in text:
        return 'PASS', ''
    return 'FAIL', 'runtime uniqueness check did not pass'


def bag_status(directory: Path) -> tuple[str, int | None, str]:
    metadata = directory / 'navigation_bag' / 'metadata.yaml'
    if not metadata.is_file():
        if (directory / 'rosbag_record.log').is_file():
            return 'FAIL', None, 'recording was attempted but rosbag metadata is missing'
        return 'NOT_RUN', None, 'navigation_bag/metadata.yaml missing'
    try:
        data = yaml.safe_load(metadata.read_text(encoding='utf-8')) or {}
        info = data.get('rosbag2_bagfile_information', data)
        count = int(info.get('message_count', 0))
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        return 'FAIL', None, f'invalid rosbag metadata: {exc}'
    if count <= 0:
        return 'FAIL', count, 'bag contains no messages'
    return 'PASS', count, ''


def markdown_table(headers: list[str], rows: list[list[object]]) -> list[str]:
    def clean(value: object) -> str:
        return str(value).replace('|', '\\|').replace('\n', '<br>')

    return [
        '| ' + ' | '.join(headers) + ' |',
        '| ' + ' | '.join('---' for _ in headers) + ' |',
        *['| ' + ' | '.join(clean(value) for value in row) + ' |' for row in rows],
    ]


def build_report(evidence_dir: Path) -> tuple[str, str]:
    runtime, runtime_reason = runtime_status(evidence_dir / 'runtime_check.txt')
    tf_status, tf_data, tf_reason = load_json(evidence_dir / 'tf_authority.json')
    rate_status, rate_data, rate_reason = load_json(evidence_dir / 'topic_rates.json')
    center_status, center_data, center_reason = load_json(
        evidence_dir / 'base_footprint_center.json')
    if tf_status == 'NOT_RUN' and (evidence_dir / 'tf_authority.log').is_file():
        tf_status, tf_reason = 'FAIL', 'TF audit was attempted but produced no JSON evidence'
    if rate_status == 'NOT_RUN' and (evidence_dir / 'topic_rates.log').is_file():
        rate_status, rate_reason = 'FAIL', 'rate audit was attempted but produced no JSON evidence'
    if center_status == 'NOT_RUN' and (evidence_dir / 'base_footprint_center.log').is_file():
        center_status, center_reason = (
            'FAIL', 'rotation-center audit was attempted but produced no JSON evidence')
    recording, message_count, recording_reason = bag_status(evidence_dir)
    checks = [
        ('运行时唯一节点', runtime, runtime_reason),
        ('TF authority 唯一性', tf_status, tf_reason),
        ('关键 topic 频率', rate_status, rate_reason),
        ('base_footprint 运动中心', center_status, center_reason),
        ('导航数据记录', recording, recording_reason),
    ]
    statuses = [item[1] for item in checks]
    overall = 'FAIL' if 'FAIL' in statuses else ('NOT_RUN' if 'NOT_RUN' in statuses else 'PASS')

    lines = [
        '# MID360 + Bunker Hardware Acceptance Report',
        '',
        f'- Overall verdict: **{overall}**',
        f'- Generated (UTC): {datetime.now(timezone.utc).isoformat(timespec="seconds")}',
        f'- Evidence directory: `{evidence_dir.resolve()}`',
        '- Scope: v3.1 Runtime Refactor hardware acceptance; architecture unchanged.',
        '',
        '## Run context',
        '',
    ]
    context_path = evidence_dir / 'run_context.txt'
    if context_path.is_file():
        context = context_path.read_text(encoding='utf-8', errors='replace').strip()
        lines.extend(['```text', context, '```'])
    else:
        lines.append('Run context was not captured.')
    lines.extend([
        '',
        '## Summary',
        '',
        *markdown_table(
            ['Check', 'Status', 'Note'],
            [[name, status, reason or '-'] for name, status, reason in checks],
        ),
        '',
        '## TF publisher authority',
        '',
    ])
    if tf_data:
        lines.extend(markdown_table(
            ['Edge', 'Expected publisher', 'Topic', 'Status', 'Reason'],
            [[
                f'{edge["parent"]} → {edge["child"]}',
                edge['publisher'],
                edge.get('topic', '-'),
                edge['status'],
                ', '.join(edge.get('reasons', [])) or '-',
            ] for edge in tf_data.get('expected_edges', [])],
        ))
        duplicates = [
            frame for frame in tf_data.get('observed_frames', [])
            if frame.get('status') == 'FAIL'
        ]
        lines.extend(['', f'- Duplicate/conflicting observed child frames: {len(duplicates)}'])
    else:
        lines.append('No TF authority evidence was captured.')

    lines.extend(['', '## Topic rates', ''])
    if rate_data:
        lines.extend(markdown_table(
            ['Topic', 'Required', 'Measured Hz', 'Allowed Hz', 'Samples', 'Status'],
            [[
                item['name'],
                item['required'],
                f'{float(item["rate_hz"]):.3f}',
                f'{float(item["min_hz"]):.3f}–{float(item["max_hz"]):.3f}',
                item['count'],
                item['status'],
            ] for item in rate_data.get('topics', [])],
        ))
    else:
        lines.append('No topic-rate evidence was captured.')

    lines.extend(['', '## base_footprint rotation center', ''])
    if center_data:
        motion = center_data.get('motion', {})
        static = center_data.get('static_transform', {})
        radius = motion.get('effective_radius_m')
        lines.extend([
            f'- Static `base_footprint → base_link`: **{static.get("status", "FAIL")}**',
            f'- Gated motion samples: {motion.get("sample_count", 0)}',
            f'- Observed yaw travel: {float(motion.get("yaw_travel_rad", 0.0)):.3f} rad',
            f'- Maximum planar excursion: {float(motion.get("max_planar_excursion_m", 0.0)):.3f} m',
            f'- Effective rotation radius: {float(radius):.3f} m' if radius is not None else '- Effective rotation radius: n/a',
            f'- Verdict: **{center_data.get("verdict", "FAIL")}**',
        ])
    else:
        lines.append('No rotation-center evidence was captured.')

    lines.extend([
        '',
        '## Navigation recording',
        '',
        f'- Rosbag status: **{recording}**',
        f'- Recorded messages: {message_count if message_count is not None else "n/a"}',
        f'- Bag path: `{(evidence_dir / "navigation_bag").resolve()}`',
        '',
        '## Acceptance decision',
        '',
    ])
    if overall == 'PASS':
        lines.append('All required evidence passed. This run is accepted for the tested hardware/configuration.')
    elif overall == 'NOT_RUN':
        lines.append('Acceptance is pending because one or more evidence items were not run.')
    else:
        lines.append('Acceptance failed. Resolve failed checks and repeat the complete run; do not merge evidence from different hardware runs.')
    lines.extend([
        '',
        '> This report is evidence-only. It does not change launch composition, TF ownership, controller behavior, or safety limits.',
        '',
    ])
    return overall, '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.evidence_dir / 'acceptance_report.md'
    overall, report = build_report(args.evidence_dir)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(report, encoding='utf-8')
    print(f'{overall} report={output}')
    return 0 if overall == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
