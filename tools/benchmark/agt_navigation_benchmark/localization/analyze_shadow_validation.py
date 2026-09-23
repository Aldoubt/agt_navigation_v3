#!/usr/bin/env python3
"""Compare legacy localization manager data with v1 shadow-manager output."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.localization_shadow import (
    inspect_input_contract,
    load_legacy_samples,
    load_shadow_samples,
    synchronize_samples,
    write_csv,
    write_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Offline legacy-versus-shadow localization parity analysis. Read-only; no ROS node is started.'
    )
    parser.add_argument('--legacy-bag', required=True, help='Bag containing legacy manager metrics/status and observations.')
    parser.add_argument('--shadow-bag', required=True, help='Bag containing /agt/localization/v1/shadow/* output.')
    parser.add_argument('--output-dir', required=True, help='Directory for localization_shadow_comparison.csv.')
    parser.add_argument(
        '--report', required=True,
        help='Markdown report path (normally agt_navigation_v3/docs/architecture/localization_shadow_metrics.md).',
    )
    parser.add_argument('--max-sync-sec', type=float, default=0.25, help='Maximum legacy-shadow timestamp difference (default: 0.25).')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_sync_sec <= 0.0:
        raise SystemExit('--max-sync-sec must be positive')
    legacy_bag = Path(args.legacy_bag).expanduser().resolve()
    shadow_bag = Path(args.shadow_bag).expanduser().resolve()
    for bag in (legacy_bag, shadow_bag):
        if not bag.is_dir():
            raise SystemExit(f'Bag directory does not exist: {bag}')

    legacy, legacy_counts, legacy_warnings = load_legacy_samples(str(legacy_bag))
    shadow, shadow_counts, shadow_warnings = load_shadow_samples(str(shadow_bag))
    rows, unmatched = synchronize_samples(legacy, shadow, args.max_sync_sec)
    output_dir = Path(args.output_dir).expanduser().resolve()
    csv_path = output_dir / 'localization_shadow_comparison.csv'
    write_csv(rows, csv_path)
    report_path = Path(args.report).expanduser().resolve()
    warnings = list(legacy_warnings) + list(shadow_warnings)
    if not legacy:
        warnings.append('No legacy samples were available for synchronization.')
    if not shadow:
        warnings.append('No shadow samples were available for synchronization.')
    write_report(
        report_path,
        str(legacy_bag),
        str(shadow_bag),
        inspect_input_contract(str(legacy_bag)),
        legacy_counts,
        shadow_counts,
        rows,
        unmatched,
        warnings,
        args.max_sync_sec,
    )
    print(f'CSV: {csv_path}')
    print(f'Report: {report_path}')
    print(f'Matched rows: {len(rows)}; unmatched shadow samples: {unmatched}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
