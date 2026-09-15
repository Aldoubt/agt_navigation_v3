#!/usr/bin/env python3
"""MQ4-P2: one MQ0 fixture, deterministic pairs, dynamic synthetic robot TF."""
import argparse
import csv
import hashlib
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(__file__))
from mq4_fixture_smoke import Smoke, audit_costmap, audit_raw, pose, samples, yaw_of  # noqa: E402


def pose_yaws(path):
    values = []
    for line in open(path):
        fields = line.split()
        qx, qy, qz, qw = map(float, fields[4:8])
        values.append(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return values


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def category_direction(pair_id):
    category, direction, _ = pair_id.split('_')
    return category, direction


def write_path(path, output):
    with open(output, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['index', 'x', 'y', 'yaw'])
        for index, item in enumerate(path.poses):
            writer.writerow([index, item.pose.position.x, item.pose.position.y, yaw_of(item.pose.orientation)])


def run_pair(node, action, pair, start_yaw, goal_yaw, request_index, output_dir):
    pair_id = pair['pair_id']
    category, direction = category_direction(pair_id)
    start = (float(pair['start']['x']), float(pair['start']['y']), start_yaw)
    goal_xy = (float(pair['goal']['x']), float(pair['goal']['y']), goal_yaw)
    report = {'pair_id': pair_id, 'request_index': request_index, 'category': category,
              'direction': direction, 'trajectory_start_index': pair['trajectory_index_start'],
              'trajectory_goal_index': pair['trajectory_index_goal'],
              'straight_line_distance_m': pair['straight_line_distance_m']}
    try:
        if node.get_state('planner_server') != 'active':
            raise RuntimeError('FIXTURE_FAILURE_PLANNER_NOT_ACTIVE')
        if not action.wait_for_server(timeout_sec=2):
            raise RuntimeError('FIXTURE_FAILURE_ACTION_NOT_AVAILABLE')
        node.set_synthetic_pose(*start)
        node.wait_for_synthetic_pose(start, timeout=2.0)
        request = ComputePathToPose.Goal()
        request.start = pose(*start)
        request.goal = pose(*goal_xy)
        request.planner_id = 'GridBased'
        request.use_start = True
        started = time.monotonic()
        future = action.send_goal_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        report['goal_accepted'] = bool(handle and handle.accepted)
        if not handle or not handle.accepted:
            report['failure_type'] = 'GOAL_REJECTED'; report['verdict'] = 'PLANNER_FAILURE'; return report
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        wrapped = future.result()
        if not wrapped:
            report['failure_type'] = 'ACTION_TIMEOUT'; report['verdict'] = 'PLANNER_FAILURE'; return report
        report['action_status'] = wrapped.status
        report['planner_success'] = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        report['wall_time_ms'] = (time.monotonic() - started) * 1000.0
        if not report['planner_success']:
            report['failure_type'] = 'PLANNER_RESULT_FAILURE'; report['verdict'] = 'PLANNER_FAILURE'; return report
        path = wrapped.result.path
        _, length = samples(path, min(0.05, node.map.info.resolution / 2.0))
        report['nav2_planning_time_ms'] = wrapped.result.planning_time.sec * 1000.0 + wrapped.result.planning_time.nanosec / 1e6
        report['path_pose_count'] = len(path.poses); report['path_length_m'] = length
        report['raw_map'] = audit_raw(node.map, path, min(0.05, node.map.info.resolution / 2.0))
        report['global_costmap'] = audit_costmap(node.costmap, path, min(0.05, node.map.info.resolution / 2.0))
        write_path(path, os.path.join(output_dir, f'{pair_id}.csv'))
        report['verdict'] = ('SEMANTIC_REVIEW' if report['raw_map']['occupied_samples'] or report['raw_map']['outside_samples'] or report['global_costmap']['lethal_samples'] or report['global_costmap']['outside_samples'] else 'PASS')
    except RuntimeError as error:
        report['failure_type'] = str(error)
        report['verdict'] = 'FIXTURE_FAILURE' if str(error).startswith('FIXTURE_FAILURE') or str(error) == 'TF_CONVERGENCE_TIMEOUT' else 'PLANNER_FAILURE'
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pairs', required=True); parser.add_argument('--poses', required=True)
    parser.add_argument('--map-yaml', required=True); parser.add_argument('--params', required=True)
    parser.add_argument('--pairs-output', required=True); parser.add_argument('--summary-output', required=True)
    parser.add_argument('--paths-output', required=True)
    parser.add_argument('--candidate', default='MQ0')
    args = parser.parse_args(); os.makedirs(args.paths_output, exist_ok=True)
    pairs = yaml.safe_load(open(args.pairs))['pairs']; yaws = pose_yaws(args.poses)
    report = {'candidate': args.candidate, 'nav2_params_sha256': hashlib.sha256(open(args.params, 'rb').read()).hexdigest(), 'fixture': {}, 'pairs': []}; rclpy.init(); node = Smoke()
    try:
        node.ensure_active('map_server', {'nodes': {'map_server': {}, 'planner_server': {}}})
        deadline = time.monotonic() + 30
        while node.map is None and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
        if node.map is None: raise RuntimeError('FIXTURE_FAILURE_MAP_NOT_READY')
        node.ensure_active('planner_server', {'nodes': {'map_server': {}, 'planner_server': {}}})
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        if not action.wait_for_server(timeout_sec=30): raise RuntimeError('FIXTURE_FAILURE_ACTION_NOT_AVAILABLE')
        deadline = time.monotonic() + 30
        while node.costmap is None and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
        if node.costmap is None: raise RuntimeError('FIXTURE_FAILURE_COSTMAP_NOT_READY')
        report['fixture']['ready'] = True
        for index, pair in enumerate(pairs):
            entry = run_pair(node, action, pair, yaws[pair['trajectory_index_start']], yaws[pair['trajectory_index_goal']], index, args.paths_output)
            report['pairs'].append(entry)
            if entry['verdict'] == 'FIXTURE_FAILURE': break
    except RuntimeError as error:
        report['fixture'] = {'ready': False, 'failure_type': str(error)}
    finally:
        with open(args.pairs_output, 'w') as stream: yaml.safe_dump(report, stream, sort_keys=False)
        successes = [p for p in report['pairs'] if p.get('planner_success')]
        summary = {'candidate': args.candidate, 'nav2_params_sha256': report['nav2_params_sha256'], 'total_pairs': len(pairs), 'planner_success': len(successes),
                   'planner_failure': len(report['pairs']) - len(successes),
                   'success_ratio': len(successes) / len(pairs), 'fixture_shutdown_clean': None}
        for field in ('category', 'direction'):
            summary[field] = {key: {'total': sum(p.get(field) == key for p in report['pairs']), 'success': sum(p.get(field) == key and p.get('planner_success') for p in report['pairs'])} for key in (['short', 'medium', 'long'] if field == 'category' else ['forward', 'reverse'])}
        for name, key in [('planning_time_ms', 'nav2_planning_time_ms'), ('wall_time_ms', 'wall_time_ms'), ('path_length_m', 'path_length_m')]:
            values = [p[key] for p in successes if key in p]; summary[name] = {'mean': sum(values)/len(values) if values else 0.0, 'p50': percentile(values,.5), 'p90': percentile(values,.9), 'max': max(values) if values else 0.0}
        semantic_keys = [('paths_with_raw_occupied', 'raw_map', 'occupied_samples'), ('paths_with_raw_unknown', 'raw_map', 'unknown_samples'), ('paths_with_raw_outside', 'raw_map', 'outside_samples'), ('paths_with_costmap_lethal', 'global_costmap', 'lethal_samples'), ('paths_with_costmap_unknown', 'global_costmap', 'unknown_samples'), ('paths_with_costmap_outside', 'global_costmap', 'outside_samples'), ('paths_with_inscribed', 'global_costmap', 'inscribed_samples')]
        summary['semantics'] = {name: sum(p.get(group, {}).get(key, 0) > 0 for p in successes) for name, group, key in semantic_keys}
        summary['semantics']['total_inscribed_samples'] = sum(p.get('global_costmap', {}).get('inscribed_samples', 0) for p in successes)
        summary['semantics']['max_inscribed_samples_per_path'] = max((p.get('global_costmap', {}).get('inscribed_samples', 0) for p in successes), default=0)
        fractions = [p['raw_map']['unknown_fraction'] for p in successes if 'raw_map' in p]; summary['mean_unknown_fraction'] = sum(fractions)/len(fractions) if fractions else 0.0; summary['max_unknown_fraction'] = max(fractions) if fractions else 0.0
        with open(args.summary_output, 'w') as stream: yaml.safe_dump(summary, stream, sort_keys=False)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
