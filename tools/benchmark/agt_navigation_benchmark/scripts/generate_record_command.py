#!/usr/bin/env python3
"""Generate a ros2 bag record command from the live ROS graph."""

from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.recorder_profile import (
    load_recorder_profile,
    write_record_command,
)
from agt_nav_benchmark.ros_graph_discovery import discover_ros_graph, write_navigation_stack_discovery_report
from agt_nav_benchmark.ros_runtime_diagnosis import diagnose_ros_runtime, write_runtime_diagnosis_report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Generate a recorder command from recorder_profile.yaml.')
    parser.add_argument('--profile', type=Path, default=Path(__file__).resolve().parents[1] / 'configs' / 'recorder_profile.yaml')
    parser.add_argument('--output', type=Path, default=Path('record_command.sh'))
    parser.add_argument('--discovery-report', type=Path, default=Path('reports/navigation_stack_discovery.md'))
    args = parser.parse_args(argv)
    profile = load_recorder_profile(args.profile)
    graph = discover_ros_graph()
    write_navigation_stack_discovery_report(graph, args.discovery_report)
    runtime_report = args.discovery_report.parent / 'runtime_diagnosis.md'
    write_runtime_diagnosis_report(diagnose_ros_runtime(graph), runtime_report)
    topics = write_record_command(profile, graph.topics, args.output)
    print(f'Generated: {args.output}')
    print(f'Discovery report: {args.discovery_report}')
    print(f'Runtime diagnosis: {runtime_report}')
    print(f'Matched topics: {len(topics)}')
    from agt_nav_benchmark.recorder_profile import check_profile
    for check in check_profile(profile, graph.topics):
        print(f'{check.name}: {check.status}')


if __name__ == '__main__':
    main()
