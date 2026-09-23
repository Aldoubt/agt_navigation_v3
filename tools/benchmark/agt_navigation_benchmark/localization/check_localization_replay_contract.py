#!/usr/bin/env python3
"""Strict read-only contract check for a localization shadow replay bag."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.localization_shadow import (
    check_replay_contract,
    write_replay_contract_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Check the mandatory topic/type/nonzero-message contract for localization replay.'
    )
    parser.add_argument('--bag', required=True, help='rosbag2 directory to inspect')
    parser.add_argument('--report', help='Optional Markdown report path')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag = Path(args.bag).expanduser().resolve()
    if not bag.is_dir():
        raise SystemExit(f'Bag directory does not exist: {bag}')
    result = check_replay_contract(str(bag))
    print(result.status)
    print('missing_topics:')
    if result.missing_topics:
        for topic in result.missing_topics:
            print(f'  - {topic}')
    else:
        print('  - none')
    print('zero_message_topics:')
    if result.zero_message_topics:
        for topic in result.zero_message_topics:
            print(f'  - {topic}')
    else:
        print('  - none')
    print('type_mismatches:')
    if result.type_mismatches:
        for topic, expected, actual in result.type_mismatches:
            print(f'  - {topic}: expected {expected}, recorded {actual}')
    else:
        print('  - none')
    if args.report:
        report = Path(args.report).expanduser().resolve()
        write_replay_contract_report(result, str(bag), report)
        print(f'report: {report}')
    return 0 if result.ok else 2


if __name__ == '__main__':
    raise SystemExit(main())
