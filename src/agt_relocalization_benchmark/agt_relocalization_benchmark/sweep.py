from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import time
from pathlib import Path

import yaml


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_cases(cases_dir: Path):
    path = cases_dir / 'cases.csv'
    with path.open('r', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f'no benchmark cases in {path}')
    for row in rows:
        row['ref_x'] = float(row['x'])
        row['ref_y'] = float(row['y'])
        row['ref_z'] = float(row['z'])
        row['ref_yaw'] = yaw_from_quat(
            float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw']))
    return rows


def command_for(candidate, map_path, scan_path, assets_dir, timeout_sec):
    cmd = [
        'ros2', 'run', 'agt_global_relocalization_native', 'bbs_gicp_localizer',
        '--map', str(map_path), '--scan', str(scan_path),
        '--timeout', str(timeout_sec),
        '--scan-leaf', str(candidate['scan_leaf']),
        '--bbs-min-level-res', str(candidate['bbs_min_level_res']),
        '--bbs-max-level', str(candidate['bbs_max_level']),
        '--bbs-score-threshold', str(candidate['bbs_score_threshold']),
        '--roll-pitch-range-deg', str(candidate['roll_pitch_range_deg']),
        '--gicp-max-corr', str(candidate['gicp_max_corr']),
        '--local-map-radius-xy', str(candidate['local_map_radius_xy']),
        '--local-map-half-height', str(candidate.get('local_map_half_height', 8.0)),
        '--min-local-map-points', str(candidate.get('min_local_map_points', 800)),
        '--threads', str(candidate.get('threads', 4)),
    ]
    if assets_dir:
        cmd += ['--assets-dir', str(assets_dir)]
    return cmd


def parse_backend_stdout(stdout):
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    if not lines:
        return {'success': False, 'message': 'empty backend stdout'}
    try:
        return json.loads(lines[-1])
    except Exception as exc:
        return {'success': False, 'message': f'invalid backend JSON: {exc}'}


def run_trial(candidate, case, args, acceptance):
    scan = args.cases / case['pcd']
    cmd = command_for(candidate, args.map, scan, args.assets, args.timeout)
    start = time.perf_counter()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout + 2.0, check=False)
        wall_ms = (time.perf_counter() - start) * 1000.0
    except subprocess.TimeoutExpired:
        return {
            'backend_success': False,
            'correct': False,
            'false_positive': False,
            'wall_ms': (time.perf_counter() - start) * 1000.0,
            'message': 'subprocess timeout',
        }
    result = parse_backend_stdout(proc.stdout)
    backend_success = bool(result.get('success', False)) and proc.returncode == 0
    trial = {
        'backend_success': backend_success,
        'wall_ms': wall_ms,
        'score': result.get('score'),
        'fitness': result.get('fitness'),
        'overlap': result.get('overlap'),
        'bbs_score': result.get('bbs_score'),
        'bbs_elapsed_ms': result.get('bbs_elapsed_ms'),
        'message': result.get('message', proc.stderr.strip()),
    }
    if not backend_success:
        trial.update({'correct': False, 'false_positive': False})
        return trial

    x, y, z = float(result['x']), float(result['y']), float(result.get('z', 0.0))
    yaw = yaw_from_quat(
        float(result['qx']), float(result['qy']), float(result['qz']), float(result['qw']))
    xy_error = math.hypot(x - case['ref_x'], y - case['ref_y'])
    z_error = abs(z - case['ref_z'])
    yaw_error_deg = abs(math.degrees(angle_diff(yaw, case['ref_yaw'])))
    correct = (
        xy_error <= float(acceptance['xy_error_m'])
        and z_error <= float(acceptance['z_error_m'])
        and yaw_error_deg <= float(acceptance['yaw_error_deg'])
    )
    trial.update({
        'x': x, 'y': y, 'z': z,
        'xy_error_m': xy_error,
        'z_error_m': z_error,
        'yaw_error_deg': yaw_error_deg,
        'correct': correct,
        'false_positive': not correct,
    })
    return trial


def safe_median(values, default=math.inf):
    vals = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return statistics.median(vals) if vals else default


def summarize_candidate(name, trials):
    correct = [t for t in trials if t['correct']]
    false_pos = [t for t in trials if t['false_positive']]
    total = len(trials)
    summary = {
        'name': name,
        'cases': total,
        'correct': len(correct),
        'backend_failures': sum(1 for t in trials if not t['backend_success']),
        'false_positives': len(false_pos),
        'success_rate': len(correct) / total if total else 0.0,
        'median_xy_error_m': safe_median([t.get('xy_error_m') for t in correct]),
        'median_yaw_error_deg': safe_median([t.get('yaw_error_deg') for t in correct]),
        'median_wall_ms': safe_median([t.get('wall_ms') for t in trials]),
    }
    return summary


def objective(summary):
    # False-positive global poses are the most dangerous failure for Nav2.
    return (
        summary['false_positives'],
        -summary['success_rate'],
        summary['median_xy_error_m'],
        summary['median_yaw_error_deg'],
        summary['median_wall_ms'],
    )


def recommend_gates(trials):
    correct = [t for t in trials if t['correct']]
    false_pos = [t for t in trials if t['false_positive']]
    if not correct:
        return {}

    def vals(key, rows):
        out = []
        for t in rows:
            v = t.get(key)
            if v is not None:
                try:
                    fv = float(v)
                except Exception:
                    continue
                if math.isfinite(fv):
                    out.append(fv)
        return out

    score_ok, score_bad = vals('score', correct), vals('score', false_pos)
    overlap_ok, overlap_bad = vals('overlap', correct), vals('overlap', false_pos)
    fit_ok, fit_bad = vals('fitness', correct), vals('fitness', false_pos)
    rec = {}

    if score_ok:
        base = min(score_ok)
        if score_bad and max(score_bad) < base:
            base = 0.5 * (max(score_bad) + base)
        rec['min_score'] = round(max(0.0, min(1.0, base)), 4)
    if overlap_ok:
        base = min(overlap_ok)
        if overlap_bad and max(overlap_bad) < base:
            base = 0.5 * (max(overlap_bad) + base)
        rec['min_overlap'] = round(max(0.0, min(1.0, base)), 4)
    if fit_ok:
        base = max(fit_ok)
        if fit_bad and min(fit_bad) > base:
            base = 0.5 * (min(fit_bad) + base)
        rec['max_fitness'] = round(max(0.0, base), 4)
    rec['note'] = (
        'Replay-derived gate suggestion only. Keep a margin and validate on a different rosbag before field use.'
    )
    return rec


def main(argv=None):
    parser = argparse.ArgumentParser(description='Sweep 3D-BBS/small_gicp parameters on captured replay cases.')
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--cases', required=True, type=Path)
    parser.add_argument('--assets', type=Path, default=None)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args(argv)
    args.map = args.map.expanduser().resolve()
    args.cases = args.cases.expanduser().resolve()
    args.assets = args.assets.expanduser().resolve() if args.assets else None
    args.output = args.output.expanduser().resolve()
    args.output.mkdir(parents=True, exist_ok=True)

    cfg = yaml.safe_load(args.config.expanduser().read_text(encoding='utf-8'))
    candidates = cfg.get('candidates', {})
    acceptance = cfg.get('acceptance', {})
    if not candidates:
        raise SystemExit('candidate config contains no candidates')
    for key in ('xy_error_m', 'z_error_m', 'yaw_error_deg'):
        if key not in acceptance:
            raise SystemExit(f'acceptance.{key} missing')

    cases = load_cases(args.cases)
    all_rows = []
    candidate_summaries = []
    trials_by_candidate = {}
    for name, candidate in candidates.items():
        print(f'=== candidate {name} ===', flush=True)
        trials = []
        for case in cases:
            trial = run_trial(candidate, case, args, acceptance)
            trial['candidate'] = name
            trial['case_id'] = case['case_id']
            trials.append(trial)
            all_rows.append(trial)
            print(
                f"{case['case_id']}: success={trial['backend_success']} correct={trial['correct']} "
                f"xy={trial.get('xy_error_m')} yaw={trial.get('yaw_error_deg')} wall_ms={trial['wall_ms']:.1f}",
                flush=True,
            )
        trials_by_candidate[name] = trials
        candidate_summaries.append(summarize_candidate(name, trials))

    best = min(candidate_summaries, key=objective)
    best_name = best['name']
    best_candidate = candidates[best_name]
    gates = recommend_gates(trials_by_candidate[best_name])

    fields = sorted({k for row in all_rows for k in row.keys()})
    with (args.output / 'trials.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(all_rows)
    # Public acceptance artifact name; retain trials.csv for backwards compatibility.
    (args.output / 'results.csv').write_text(
        (args.output / 'trials.csv').read_text(encoding='utf-8'), encoding='utf-8')

    summary_doc = {
        'best_candidate': best_name,
        'best_parameters': best_candidate,
        'recommended_quality_gates': gates,
        'acceptance': acceptance,
        'candidate_summaries': candidate_summaries,
        'warning': (
            'Same-bag replay is an optimistic closed-set smoke test because query scans contributed to the map. '
            'Validate the selected parameters on a separate rosbag before field acceptance.'
        ),
    }
    (args.output / 'summary.yaml').write_text(yaml.safe_dump(summary_doc, sort_keys=False), encoding='utf-8')
    (args.output / 'best_params.yaml').write_text(
        yaml.safe_dump({'backend': best_candidate, 'quality_gates': gates}, sort_keys=False), encoding='utf-8')

    print('=== BEST ===')
    print(yaml.safe_dump(summary_doc, sort_keys=False))


if __name__ == '__main__':
    main()
