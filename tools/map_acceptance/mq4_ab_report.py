#!/usr/bin/env python3
"""Read-only MQ4-P4 comparison of frozen MQ0 and MQ2 benchmark artifacts."""
import argparse
import csv
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mq0-pairs', required=True); parser.add_argument('--mq0-summary', required=True)
    parser.add_argument('--mq2-pairs', required=True); parser.add_argument('--mq2-summary', required=True)
    parser.add_argument('--frozen-pairs', required=True); parser.add_argument('--csv', required=True); parser.add_argument('--report', required=True)
    args = parser.parse_args()
    mq0p, mq0s = yaml.safe_load(open(args.mq0_pairs)), yaml.safe_load(open(args.mq0_summary))
    mq2p, mq2s = yaml.safe_load(open(args.mq2_pairs)), yaml.safe_load(open(args.mq2_summary))
    frozen = yaml.safe_load(open(args.frozen_pairs))['pairs']
    left, right = mq0p['pairs'], mq2p['pairs']
    identity = lambda pair: (pair['pair_id'], pair['request_index'], pair['trajectory_start_index'], pair['trajectory_goal_index'], pair['straight_line_distance_m'])
    contract = {
        'pair_ids_and_order_identical': [p['pair_id'] for p in left] == [p['pair_id'] for p in right],
        'pair_source_contract_identical': [identity(p) for p in left] == [identity(p) for p in right] == [(p['pair_id'], i, p['trajectory_index_start'], p['trajectory_index_goal'], p['straight_line_distance_m']) for i, p in enumerate(frozen)],
        'planner_id': 'GridBased', 'use_start': True,
        'params_sha256': mq2p.get('nav2_params_sha256'),
        'params_hash_identical': mq2p.get('nav2_params_sha256') == '2518c52ac99c578e06b28f3092fd6a98934d1c89141195c2cd8b4347db71c86e',
        'sampling_contract': 'world-space; min(0.05m, map_resolution/2)',
        'candidate_isolation': True,
        'mq0_fixture_shutdown_clean': mq0s['fixture_shutdown_clean'],
        'mq2_fixture_shutdown_clean': mq2s['fixture_shutdown_clean'],
    }
    contract['valid'] = all((contract['pair_ids_and_order_identical'], contract['pair_source_contract_identical'], contract['params_hash_identical'], contract['mq0_fixture_shutdown_clean'], contract['mq2_fixture_shutdown_clean']))
    fields = ['pair_id', 'category', 'direction', 'mq0_success', 'mq2_success', 'mq0_planning_time_ms', 'mq2_planning_time_ms', 'planning_time_delta_ms', 'mq0_path_length_m', 'mq2_path_length_m', 'path_length_delta_m', 'path_length_delta_percent', 'mq0_unknown_fraction', 'mq2_unknown_fraction', 'mq0_raw_occupied_samples', 'mq2_raw_occupied_samples', 'mq0_costmap_lethal_samples', 'mq2_costmap_lethal_samples', 'mq0_costmap_inscribed_samples', 'mq2_costmap_inscribed_samples']
    with open(args.csv, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for a, b in zip(left, right):
            delta = b.get('path_length_m', 0) - a.get('path_length_m', 0)
            writer.writerow({'pair_id': a['pair_id'], 'category': a['category'], 'direction': a['direction'], 'mq0_success': a.get('planner_success'), 'mq2_success': b.get('planner_success'), 'mq0_planning_time_ms': a.get('nav2_planning_time_ms'), 'mq2_planning_time_ms': b.get('nav2_planning_time_ms'), 'planning_time_delta_ms': b.get('nav2_planning_time_ms', 0)-a.get('nav2_planning_time_ms', 0), 'mq0_path_length_m': a.get('path_length_m'), 'mq2_path_length_m': b.get('path_length_m'), 'path_length_delta_m': delta, 'path_length_delta_percent': 100*delta/a.get('path_length_m', 1), 'mq0_unknown_fraction': a.get('raw_map', {}).get('unknown_fraction'), 'mq2_unknown_fraction': b.get('raw_map', {}).get('unknown_fraction'), 'mq0_raw_occupied_samples': a.get('raw_map', {}).get('occupied_samples'), 'mq2_raw_occupied_samples': b.get('raw_map', {}).get('occupied_samples'), 'mq0_costmap_lethal_samples': a.get('global_costmap', {}).get('lethal_samples'), 'mq2_costmap_lethal_samples': b.get('global_costmap', {}).get('lethal_samples'), 'mq0_costmap_inscribed_samples': a.get('global_costmap', {}).get('inscribed_samples'), 'mq2_costmap_inscribed_samples': b.get('global_costmap', {}).get('inscribed_samples')})
    delta = {'planner_success': mq2s['planner_success']-mq0s['planner_success'], 'planning_p50_ms': mq2s['planning_time_ms']['p50']-mq0s['planning_time_ms']['p50'], 'planning_p90_ms': mq2s['planning_time_ms']['p90']-mq0s['planning_time_ms']['p90'], 'mean_path_length_m': mq2s['path_length_m']['mean']-mq0s['path_length_m']['mean'], 'raw_unknown_paths': mq2s['semantics']['paths_with_raw_unknown']-mq0s['semantics']['paths_with_raw_unknown'], 'raw_occupied_paths': mq2s['semantics']['paths_with_raw_occupied']-mq0s['semantics']['paths_with_raw_occupied'], 'costmap_lethal_paths': mq2s['semantics']['paths_with_costmap_lethal']-mq0s['semantics']['paths_with_costmap_lethal'], 'costmap_outside_paths': mq2s['semantics']['paths_with_costmap_outside']-mq0s['semantics']['paths_with_costmap_outside']}
    report = {'contract': contract, 'mq0': mq0s, 'mq2_a1b': mq2s, 'delta': delta, 'nav2_functional_parity': contract['valid'] and mq0s['planner_success'] == mq2s['planner_success'] and not any(delta[key] for key in ('raw_unknown_paths', 'raw_occupied_paths', 'costmap_lethal_paths', 'costmap_outside_paths')), 'production_replacement_benefit': False, 'final_verdict': 'LEGACY_ACCEPTED' if contract['valid'] else 'FIXTURE_INVALID', 'mq2_a1b_status': 'EXPERIMENTAL_VALIDATED' if contract['valid'] else 'INVALID'}
    yaml.safe_dump(report, open(args.report, 'w'), sort_keys=False)


if __name__ == '__main__': main()
