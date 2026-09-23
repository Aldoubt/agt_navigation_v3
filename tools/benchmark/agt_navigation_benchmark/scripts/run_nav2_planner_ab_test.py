#!/usr/bin/env python3
"""Run real Nav2 planner_server A/B replay on the two generated maps."""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yaml

from agt_nav_benchmark.map_asset_validator import validate_pgm
from agt_nav_benchmark.nav2_replay_backend import (
    Nav2PlannerReplayBackend,
    extract_experiment_poses,
    pose_dict,
)
from agt_nav_benchmark.path_analysis import analyze_path_message


COMPARISON_FIELDS = [
    'map', 'unknown_ratio', 'path_length', 'num_points', 'curvature_mean',
    'curvature_p95', 'sharp_turn_ratio', 'smoothness_score', 'status',
    'planner_plugin',
]


def _resolve_bag(value: Path) -> Path:
    if value.exists():
        return value.resolve()
    candidate = Path('/home/yangxuan/ros2_ws/agt_data/field_acceptance') / value
    if candidate.exists():
        return candidate.resolve()
    raise FileNotFoundError(f'Bag not found: {value}')


def _map_image(map_yaml: Path) -> Path:
    image = (yaml.safe_load(map_yaml.read_text(encoding='utf-8')) or {}).get('image')
    if not image:
        raise ValueError(f'Map YAML has no image field: {map_yaml}')
    return (map_yaml.parent / image).resolve()


def _write_failed_plan(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        csv.writer(stream).writerow(['timestamp', 'x', 'y', 'yaw'])


def _row(map_name, unknown_ratio, metric, status, plugin):
    row = {field: '' for field in COMPARISON_FIELDS}
    row.update({
        'map': map_name,
        'unknown_ratio': f'{unknown_ratio:.6f}',
        'status': status,
        'planner_plugin': plugin,
    })
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


def _write_branch_result(path: Path, row, log_path: Path | None, error: str = ''):
    lines = [
        f'# Nav2 Planner Replay: {row["map"]}\n\n',
        f'- status: `{row["status"]}`\n',
        f'- planner plugin: `{row["planner_plugin"]}`\n',
        f'- unknown ratio: `{float(row["unknown_ratio"]) * 100.0:.2f}%`\n',
    ]
    for key in ('path_length', 'num_points', 'curvature_mean', 'curvature_p95', 'sharp_turn_ratio', 'smoothness_score'):
        if row[key] != '':
            lines.append(f'- {key}: `{row[key]}`\n')
    if log_path:
        lines.append(f'- replay log: `{log_path}`\n')
    if error:
        lines.append(f'\nError: `{error}`\n')
    path.write_text(''.join(lines), encoding='utf-8')


def _write_report(path: Path, rows):
    baseline, unknown_free = rows
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
        lines.append('PENDING: Nav2 planner replay did not produce valid paths for both map variants.\n')
    else:
        p95_improvement = 1.0 - float(unknown_free['curvature_p95']) / max(float(baseline['curvature_p95']), 1e-12)
        smoothness_improved = float(unknown_free['smoothness_score']) > float(baseline['smoothness_score'])
        if p95_improvement >= 0.30 and smoothness_improved:
            lines.append('Evidence: Unknown space caused planner degradation.\n')
        else:
            lines.append('Unknown space is not the dominant factor. Investigate:\n\n- inflation radius\n- planner parameters\n- obstacle representation\n')
    path.write_text(''.join(lines), encoding='utf-8')


def run_test(args):
    bag = _resolve_bag(args.bag)
    ab_dir = args.ab_dir.resolve()
    if not (ab_dir / 'original_map.yaml').exists() or not (ab_dir / 'unknown_free_map.yaml').exists():
        generator = Path(__file__).with_name('map_unknown_ab_test.py')
        subprocess.run([
            sys.executable, str(generator), '--bag', str(bag), '--output-dir', str(ab_dir),
        ], check=True)
    original_yaml = ab_dir / 'original_map.yaml'
    unknown_free_yaml = ab_dir / 'unknown_free_map.yaml'
    start, goal = extract_experiment_poses(bag)
    root = args.experiment_dir.resolve()
    baseline_dir = root / 'baseline'
    unknown_free_dir = root / 'unknown_free'
    comparison_dir = root / 'comparison'
    for directory in (baseline_dir, unknown_free_dir, comparison_dir):
        directory.mkdir(parents=True, exist_ok=True)
    (root / 'experiment_pose.yaml').write_text(yaml.safe_dump(pose_dict(start, goal), sort_keys=False), encoding='utf-8')
    reports_dir = args.reports_dir.resolve()
    reports_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for map_name, map_yaml, branch_dir in (
        ('baseline_unknown', original_yaml, baseline_dir),
        ('unknown_free', unknown_free_yaml, unknown_free_dir),
    ):
        shutil.copy2(map_yaml, branch_dir / 'map.yaml')
        (branch_dir / 'pose.yaml').write_text(yaml.safe_dump(pose_dict(start, goal), sort_keys=False), encoding='utf-8')
        filename = 'baseline_plan.csv' if map_name == 'baseline_unknown' else 'unknown_free_plan.csv'
        output_plan = branch_dir / filename
        backend = Nav2PlannerReplayBackend(
            map_yaml=map_yaml,
            start=start,
            goal=goal,
            planner_plugin=args.planner_plugin,
            planner_id=args.planner_id,
            namespace=f'agt_replay_{os.getpid()}_{map_name}',
            output_dir=branch_dir,
            timeout_sec=args.timeout_sec,
            inflation_radius=args.inflation_radius,
        )
        backend.write_planner_params(branch_dir / 'planner.yaml')
        status = 'FAILED'
        metric = None
        error = ''
        try:
            replay = backend.run(output_plan, branch_dir / 'planner.yaml')
            metric = analyze_path_message(replay.path, 0, args.sharp_turn_angle_rad, args.curvature_scale_rad_per_m)
            status = 'OK'
            log_path = replay.log_path
        except Exception as exc:
            error = str(exc)
            log_path = branch_dir / 'nav2_replay.log'
            _write_failed_plan(output_plan)
        ratio = validate_pgm(_map_image(map_yaml), map_yaml)['unknown_ratio']
        row = _row(map_name, ratio, metric, status, args.planner_plugin)
        rows.append(row)
        _write_branch_result(branch_dir / 'result.md', row, log_path, error)
        shutil.copy2(output_plan, reports_dir / output_plan.name)

    with (comparison_dir / 'planner_ab_comparison.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=COMPARISON_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    shutil.copy2(comparison_dir / 'planner_ab_comparison.csv', reports_dir / 'planner_ab_comparison.csv')
    report = reports_dir / 'planner_ab_report.md'
    _write_report(report, rows)
    (comparison_dir / 'result.md').write_text(report.read_text(encoding='utf-8'), encoding='utf-8')
    (comparison_dir / 'pose.yaml').write_text(yaml.safe_dump(pose_dict(start, goal), sort_keys=False), encoding='utf-8')
    shutil.copy2(original_yaml, comparison_dir / 'map.yaml')
    comparison_backend = Nav2PlannerReplayBackend(
        original_yaml, start, goal, args.planner_plugin, args.planner_id,
        namespace=f'agt_replay_{os.getpid()}_comparison', output_dir=comparison_dir,
        timeout_sec=args.timeout_sec, inflation_radius=args.inflation_radius,
    )
    comparison_backend.write_planner_params(comparison_dir / 'planner.yaml')
    print(f'Nav2 planner A/B report: {report}')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run real Nav2 planner_server on baseline and unknown-free maps.')
    parser.add_argument('--bag', type=Path, required=True, help='rosbag path or field_acceptance bag name')
    parser.add_argument('--ab-dir', type=Path, default=Path('reports/map_ab_test'))
    parser.add_argument('--experiment-dir', type=Path, default=Path('experiments/nav_test_002/planner_replay'))
    parser.add_argument('--reports-dir', type=Path, default=Path('reports'))
    parser.add_argument('--planner-plugin', default='nav2_navfn_planner/NavfnPlanner')
    parser.add_argument('--planner-id', default='GridBased')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--inflation-radius', type=float, default=0.25)
    parser.add_argument('--sharp-turn-angle-rad', type=float, default=0.5)
    parser.add_argument('--curvature-scale-rad-per-m', type=float, default=1.0)
    args = parser.parse_args(argv)
    run_test(args)


if __name__ == '__main__':
    main()
