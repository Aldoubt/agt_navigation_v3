#!/usr/bin/env python3
"""Verify controller profiles differ only in controller_server.FollowPath."""
import argparse
from pathlib import Path
import yaml


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--rpp', required=True, type=Path); parser.add_argument('--mppi', required=True, type=Path); parser.add_argument('--output', required=True, type=Path); args = parser.parse_args()
    rpp, mppi = yaml.safe_load(args.rpp.read_text()), yaml.safe_load(args.mppi.read_text())
    left = rpp['controller_server']['ros__parameters'].pop('FollowPath')
    right = mppi['controller_server']['ros__parameters'].pop('FollowPath')
    identical = rpp == mppi
    result = {'valid': identical, 'allowed_difference': 'controller_server.ros__parameters.FollowPath', 'rpp_plugin': left.get('plugin'), 'mppi_plugin': right.get('plugin'), 'non_controller_differences': [] if identical else 'present'}
    args.output.write_text(yaml.safe_dump(result, sort_keys=False))
    if not identical: raise SystemExit('non-controller profile differences found')


if __name__ == '__main__': main()
