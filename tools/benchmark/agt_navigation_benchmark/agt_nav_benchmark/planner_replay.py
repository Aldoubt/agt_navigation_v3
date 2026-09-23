"""Planner replay experiment orchestration.

The benchmark intentionally treats the actual Nav2 replay as an injectable
backend. A command backend must write an XY CSV plan to ``AGT_PLAN_OUTPUT``.
This keeps the benchmark offline and makes it possible to plug in a
PlannerServer/rosbag replay without embedding Nav2 launch policy here.
"""

from __future__ import annotations

import csv
import os
import shutil
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import yaml

from .bag_reader import RosbagReader
from .map_asset_validator import validate_pgm
from .path_analysis import PathMetric, analyze_path_message, analyze_xy_points


COMPARISON_FIELDS = [
    'map_type', 'unknown_ratio', 'path_length', 'num_points', 'curvature_mean',
    'curvature_p95', 'sharp_turn_ratio', 'smoothness_score', 'status', 'backend',
]


def _read_plan_csv(path: Path) -> List[List[float]]:
    points: List[List[float]] = []
    with path.open('r', encoding='utf-8') as stream:
        rows = csv.reader(stream)
        for row in rows:
            if not row:
                continue
            try:
                x, y = float(row[0]), float(row[1])
            except (ValueError, IndexError):
                # Permit a header such as x,y or timestamp,x,y.
                if not points:
                    continue
                raise ValueError(f'Invalid plan row in {path}: {row}')
            points.append([x, y])
    if len(points) < 2:
        raise ValueError(f'Plan file {path} must contain at least two x,y rows')
    return points


def _load_baseline_plan(bag_path: Path, sharp_turn_angle: float, curvature_scale: float) -> PathMetric:
    reader = RosbagReader(str(bag_path))
    records = list(reader.records(['/plan']))
    if not records:
        raise ValueError('No /plan messages found for baseline replay')
    metrics = [
        analyze_path_message(record.message, record.timestamp_ns, sharp_turn_angle, curvature_scale)
        for record in records
    ]
    return _aggregate_metrics(metrics)


def _aggregate_metrics(metrics: Iterable[PathMetric]) -> PathMetric:
    metrics = list(metrics)
    if not metrics:
        raise ValueError('Cannot aggregate an empty planner result')
    mean = lambda name: sum(float(getattr(item, name)) for item in metrics) / len(metrics)
    return PathMetric(
        timestamp_ns=metrics[0].timestamp_ns,
        path_length=mean('path_length'),
        num_points=int(round(mean('num_points'))),
        mean_curvature=mean('mean_curvature'),
        max_curvature=max(item.max_curvature for item in metrics),
        sharp_turn_count=sum(item.sharp_turn_count for item in metrics),
        curvature_std=mean('curvature_std'),
        curvature_p50=mean('curvature_p50'),
        curvature_p90=mean('curvature_p90'),
        curvature_p95=mean('curvature_p95'),
        curvature_p99=mean('curvature_p99'),
        heading_change_mean=mean('heading_change_mean'),
        heading_change_std=mean('heading_change_std'),
        sharp_turn_ratio=mean('sharp_turn_ratio'),
        smoothness_score=mean('smoothness_score'),
    )


def _run_backend(command: str, map_yaml: Path, output_plan: Path, map_type: str, experiment_dir: Path) -> Path:
    experiment_dir.mkdir(parents=True, exist_ok=True)
    output_plan.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update({
        'AGT_MAP_YAML': str(map_yaml.resolve()),
        'AGT_MAP_TYPE': map_type,
        'AGT_PLAN_OUTPUT': str(output_plan.resolve()),
        'AGT_EXPERIMENT_DIR': str(experiment_dir.resolve()),
    })
    rendered = command.format(
        map_yaml=str(map_yaml.resolve()),
        output_plan=str(output_plan.resolve()),
        map_type=map_type,
        experiment_dir=str(experiment_dir.resolve()),
    )
    completed = subprocess.run(rendered, shell=True, env=environment, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f'Planner backend failed for {map_type} with exit code {completed.returncode}')
    if not output_plan.exists():
        raise RuntimeError(f'Planner backend did not write {output_plan}')
    return output_plan


def _metric_row(map_type: str, unknown_ratio: float, metric: Optional[PathMetric], status: str, backend: str) -> Dict[str, object]:
    row = {field: '' for field in COMPARISON_FIELDS}
    row.update({'map_type': map_type, 'unknown_ratio': f'{unknown_ratio:.6f}', 'status': status, 'backend': backend})
    if metric is not None:
        row.update({
            'path_length': f'{metric.path_length:.6f}',
            'num_points': metric.num_points,
            'curvature_mean': f'{metric.mean_curvature:.6f}',
            'curvature_p95': f'{metric.curvature_p95:.6f}',
            'sharp_turn_ratio': f'{metric.sharp_turn_ratio:.6f}',
            'smoothness_score': f'{metric.smoothness_score:.6f}',
        })
    return row


def _write_experiment_files(experiment_dir: Path, map_type: str, map_yaml: Path, status: str, backend: str, planner_config: Dict):
    experiment_dir.mkdir(parents=True, exist_ok=True)
    map_info = yaml.safe_load(map_yaml.read_text(encoding='utf-8')) or {}
    map_info.update({'map_type': map_type, 'source_yaml': str(map_yaml.resolve())})
    (experiment_dir / 'map_info.yaml').write_text(yaml.safe_dump(map_info, sort_keys=False), encoding='utf-8')
    config = dict(planner_config)
    config.update({'map_type': map_type, 'status': status, 'backend': backend})
    (experiment_dir / 'planner_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    metadata = {
        'experiment': 'nav_test_002',
        'map_type': map_type,
        'map_yaml': str(map_yaml.resolve()),
        'status': status,
        'backend': backend,
    }
    (experiment_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')


def _write_result(path: Path, map_type: str, row: Dict[str, object], note: str = ''):
    path.write_text(
        f'# Planner Replay: {map_type}\n\n'
        f'- status: `{row["status"]}`\n- backend: `{row["backend"]}`\n'
        f'- unknown ratio: `{float(row["unknown_ratio"]) * 100.0:.2f}%`\n'
        + ''.join(f'- {key}: `{value}`\n' for key, value in row.items() if key in COMPARISON_FIELDS[2:8] and value != '')
        + (f'\n{note}\n' if note else ''),
        encoding='utf-8',
    )


def _write_comparison_report(path: Path, rows: List[Dict[str, object]]) -> None:
    by_type = {row['map_type']: row for row in rows}
    baseline = by_type['baseline_unknown']
    unknown_free = by_type['unknown_free']
    lines = [
        '# Planner A/B Test\n\n',
        '## Map Difference\n\n',
        f'- unknown before: `{float(baseline["unknown_ratio"]) * 100.0:.2f}%`\n',
        f'- unknown after: `{float(unknown_free["unknown_ratio"]) * 100.0:.2f}%`\n\n',
        '## Path Quality\n\n',
        '| metric | baseline | unknown_free |\n|---|---:|---:|\n',
    ]
    for field in ('path_length', 'num_points', 'curvature_mean', 'curvature_p95', 'sharp_turn_ratio', 'smoothness_score'):
        lines.append(f'| {field} | {baseline[field] or "PENDING"} | {unknown_free[field] or "PENDING"} |\n')
    lines.append('\n## Conclusion\n\n')
    if baseline['status'] != 'OK' or unknown_free['status'] != 'OK':
        lines.append('PENDING: a real unknown-free planner replay result is required before attributing improvement to unknown-space reduction.\n')
    elif float(unknown_free['unknown_ratio']) < float(baseline['unknown_ratio']) and float(unknown_free['curvature_p95']) <= 0.70 * float(baseline['curvature_p95']):
        lines.append('Unknown area is a major contributor to planner instability.\n')
    else:
        lines.append('Planner instability is not primarily caused by unknown space. Investigate inflation/planner parameters.\n')
    path.write_text(''.join(lines), encoding='utf-8')


def run_replay(
    bag_path: Path,
    ab_dir: Path,
    experiment_root: Path,
    reports_dir: Path,
    planner_command: Optional[str] = None,
    unknown_free_plan: Optional[Path] = None,
    sharp_turn_angle: float = 0.5,
    curvature_scale: float = 1.0,
) -> Path:
    original_yaml = ab_dir / 'original_map.yaml'
    unknown_free_yaml = ab_dir / 'unknown_free_map.yaml'
    if not original_yaml.exists() or not unknown_free_yaml.exists():
        raise FileNotFoundError(f'Missing A/B map assets under {ab_dir}; run map_unknown_ab_test.py first')
    original_pgm = original_yaml.parent / (yaml.safe_load(original_yaml.read_text(encoding='utf-8'))['image'])
    unknown_free_pgm = unknown_free_yaml.parent / (yaml.safe_load(unknown_free_yaml.read_text(encoding='utf-8'))['image'])
    original_ratio = validate_pgm(original_pgm, original_yaml)['unknown_ratio']
    unknown_free_ratio = validate_pgm(unknown_free_pgm, unknown_free_yaml)['unknown_ratio']

    experiment_root.mkdir(parents=True, exist_ok=True)
    comparison_dir = experiment_root / 'comparison'
    comparison_dir.mkdir(parents=True, exist_ok=True)
    planner_config = {'planner_id': 'navfn', 'planner_command': planner_command or '', 'plan_format': 'CSV with x,y columns'}
    rows = []
    baseline_metric = None
    baseline_status = 'PENDING'
    baseline_backend = 'bag_plan'
    try:
        if planner_command:
            baseline_plan = _run_backend(planner_command, original_yaml, experiment_root / 'baseline_unknown' / 'replayed_plan.csv', 'baseline_unknown', experiment_root / 'baseline_unknown')
            baseline_metric = analyze_xy_points(_read_plan_csv(baseline_plan), 0, sharp_turn_angle, curvature_scale)
            baseline_status = 'OK'
            baseline_backend = 'command'
        else:
            baseline_metric = _load_baseline_plan(bag_path, sharp_turn_angle, curvature_scale)
            baseline_status = 'OK'
    except Exception as exc:
        baseline_status = f'FAILED: {exc}'
    _write_experiment_files(experiment_root / 'baseline_unknown', 'baseline_unknown', original_yaml, baseline_status, baseline_backend, planner_config)
    baseline_row = _metric_row('baseline_unknown', original_ratio, baseline_metric, baseline_status, baseline_backend)
    rows.append(baseline_row)
    _write_result(experiment_root / 'baseline_unknown' / 'result.md', 'baseline_unknown', baseline_row)

    unknown_metric = None
    unknown_status = 'PENDING'
    unknown_backend = 'not_run'
    try:
        if unknown_free_plan:
            unknown_metric = analyze_xy_points(_read_plan_csv(unknown_free_plan), 0, sharp_turn_angle, curvature_scale)
            unknown_status = 'OK'
            unknown_backend = 'plan_file'
        elif planner_command:
            plan_path = _run_backend(planner_command, unknown_free_yaml, experiment_root / 'unknown_free' / 'replayed_plan.csv', 'unknown_free', experiment_root / 'unknown_free')
            unknown_metric = analyze_xy_points(_read_plan_csv(plan_path), 0, sharp_turn_angle, curvature_scale)
            unknown_status = 'OK'
            unknown_backend = 'command'
    except Exception as exc:
        unknown_status = f'FAILED: {exc}'
    _write_experiment_files(experiment_root / 'unknown_free', 'unknown_free', unknown_free_yaml, unknown_status, unknown_backend, planner_config)
    unknown_row = _metric_row('unknown_free', unknown_free_ratio, unknown_metric, unknown_status, unknown_backend)
    rows.append(unknown_row)
    note = 'Provide --planner-command or --unknown-free-plan to complete the A/B comparison.' if unknown_status == 'PENDING' else ''
    _write_result(experiment_root / 'unknown_free' / 'result.md', 'unknown_free', unknown_row, note)

    _write_experiment_files(comparison_dir, 'comparison', original_yaml, 'READY', 'comparison', planner_config)

    with (comparison_dir / 'planner_ab_comparison.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=COMPARISON_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    reports_dir.mkdir(parents=True, exist_ok=True)
    reports_csv = reports_dir / 'planner_ab_comparison.csv'
    shutil.copy2(comparison_dir / 'planner_ab_comparison.csv', reports_csv)
    report_path = reports_dir / 'planner_ab_report.md'
    _write_comparison_report(report_path, rows)
    (comparison_dir / 'result.md').write_text(report_path.read_text(encoding='utf-8'), encoding='utf-8')
    root_result = experiment_root / 'result.md'
    with root_result.open('a', encoding='utf-8') as stream:
        stream.write('\n## Planner Replay A/B\n\n')
        stream.write(f'- report: `{report_path.resolve()}`\n')
        stream.write(f'- comparison CSV: `{reports_csv.resolve()}`\n')
        stream.write(f'- baseline status: `{baseline_status}`\n- unknown-free status: `{unknown_status}`\n')
    return report_path
