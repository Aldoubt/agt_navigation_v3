#!/usr/bin/env python3
"""Run the offline planner A/B replay framework for nav_test_002."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.planner_replay import run_replay


def main(argv=None):
    parser = argparse.ArgumentParser(description='Prepare/run a baseline-vs-unknown-free planner replay.')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--ab-dir', type=Path, default=Path('reports/map_ab_test'))
    parser.add_argument('--experiment-dir', type=Path, default=Path('experiments/nav_test_002'))
    parser.add_argument('--reports-dir', type=Path, default=Path('reports'))
    parser.add_argument('--planner-command', help='Command template; it must write x,y CSV to {output_plan} or AGT_PLAN_OUTPUT')
    parser.add_argument('--unknown-free-plan', type=Path, help='Precomputed unknown-free x,y CSV plan')
    parser.add_argument('--sharp-turn-angle-rad', type=float, default=0.5)
    parser.add_argument('--curvature-scale-rad-per-m', type=float, default=1.0)
    args = parser.parse_args(argv)
    report = run_replay(
        args.bag, args.ab_dir, args.experiment_dir, args.reports_dir,
        args.planner_command, args.unknown_free_plan,
        args.sharp_turn_angle_rad, args.curvature_scale_rad_per_m,
    )
    print(f'Planner replay report: {report}')


if __name__ == '__main__':
    main()
