"""Command line entry point for the Nav2 offline benchmark."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Any, Dict

import yaml

from .bag_reader import collect_records
from .controller_analysis import CmdVelSample, analyze_cmd_vel, write_controller_csv
from .costmap_analysis import analyze_costmap_message, write_costmap_csv
from .diagnostics import correlate_path_and_cmd, diagnose, write_correlation_csv
from .localization_analysis import analyze_odometry, sample_from_message, write_localization_csv
from .map_analyzer import analyze_map_message, write_map_quality_csv
from .map_asset_validator import write_occupancy_grid_asset_report
from .path_analysis import analyze_path_message, write_path_csv, write_planner_quality_csv
from .controller_tracking import (
    analyze_cmd_vel_dynamics,
    analyze_tracking,
    assess_controller_tracking,
    make_plan_snapshot,
    transform_odometry_to_frame,
    write_cmd_vel_dynamics_csv,
    write_controller_tracking_report,
    write_tracking_csv,
)
from .planner_runtime_analysis import (
    analyze_plan_runtime,
    write_planner_runtime_config,
    write_planner_runtime_report,
    write_runtime_metrics_csv,
)
from .odom_runtime_analysis import (
    DEFAULT_ODOMETRY_TOPICS,
    DEFAULT_TF_TOPICS,
    analyze_odom_runtime,
    write_odom_frequency_csv,
    write_odom_runtime_report,
    write_tf_conflict_report,
)
from .nav2_goal_analysis import (
    analyze_behavior_tree_logs,
    analyze_navigate_to_pose_action,
    build_goal_timeline,
    correlate_goals_plans,
    write_action_metrics_csv,
    write_goal_timeline_csv,
    write_nav2_runtime_report,
)
from .recorder_profile import analyze_bag_completeness
from .plotting import (
    plot_cmd_vel_angular,
    plot_path_curvature,
    plot_plan_curvature_timeline,
    plot_plan_sequence,
    plot_tracking_error,
    plot_unknown_ratio,
)


def _default_config_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / 'configs' / 'default.yaml'
    if source_path.exists():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('agt_navigation_benchmark')) / 'config' / 'default.yaml'
    except Exception:
        return source_path


def _default_recorder_profile_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / 'configs' / 'recorder_profile.yaml'
    if source_path.exists():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('agt_navigation_benchmark')) / 'config' / 'recorder_profile.yaml'
    except Exception:
        return source_path


def _load_config(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _metadata(bag: Path):
    metadata_path = bag / 'metadata.yaml'
    if not metadata_path.exists():
        return {}
    with metadata_path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def _format_ratio(value: float) -> str:
    return f'{value * 100.0:.1f}%'


def _write_report(
    report_path: Path, bag: Path, metadata: Dict[str, Any], available_topics,
    paths, costmaps, map_quality, controller, localization, diagnosis, output_dir: Path,
    odom_runtime=None,
) -> None:
    root = metadata.get('rosbag2_bagfile_information', metadata)
    duration_ns = ((root.get('duration') or {}).get('nanoseconds'))
    if duration_ns is None:
        stamps = [item.timestamp_ns for item in paths + costmaps]
        duration_sec = (max(stamps) - min(stamps)) / 1e9 if stamps else controller.duration_sec
    else:
        duration_sec = float(duration_ns) / 1e9
    topic_counts = {
        item.get('topic_metadata', {}).get('name'): item.get('message_count')
        for item in root.get('topics_with_message_count', [])
    }
    with report_path.open('w', encoding='utf-8') as stream:
        stream.write('# Navigation Offline Analysis Report\n\n')
        stream.write('## 1. Dataset\n\n')
        stream.write(f'- bag: `{bag}`\n- duration: `{duration_sec:.3f}s`\n')
        stream.write(f'- analyzed path messages: `{len(paths)}`\n')
        stream.write(f'- analyzed cmd_vel messages: `{controller.num_messages}`\n\n')
        stream.write('Topics found in bag:\n\n')
        for topic in sorted(available_topics):
            count = topic_counts.get(topic, 'unknown')
            stream.write(f'- `{topic}` ({available_topics[topic]}, {count} messages)\n')

        stream.write('\n## 2. Planner Analysis\n\n')
        mean_length = sum(item.path_length for item in paths) / max(len(paths), 1)
        mean_points = sum(item.num_points for item in paths) / max(len(paths), 1)
        max_curvature = max((item.max_curvature for item in paths), default=0.0)
        sharp_turns = sum(item.sharp_turn_count for item in paths)
        stream.write(f'- Average path length: `{mean_length:.3f} m`\n')
        stream.write(f'- Average waypoint count: `{mean_points:.1f}`\n')
        stream.write(f'- Maximum curvature: `{max_curvature:.3f} rad/m`\n')
        stream.write(f'- Sharp turns: `{sharp_turns}`\n')
        stream.write('- CSV: `path_metrics.csv`\n\n')

        mean_smoothness = sum(item.smoothness_score for item in paths) / max(len(paths), 1)
        max_p95 = max((item.curvature_p95 for item in paths), default=0.0)
        sharp_ratio = sum(item.sharp_turn_ratio for item in paths) / max(len(paths), 1)
        assessment = 'SMOOTH' if mean_smoothness >= 0.70 else 'UNSTABLE'
        stream.write('### Planner Quality\n\n')
        stream.write(f'- Smoothness score: `{mean_smoothness:.3f}`\n')
        stream.write(f'- Sharp turn ratio: `{sharp_ratio:.3f}`\n')
        stream.write(f'- Curvature P95: `{max_p95:.3f} rad/m`\n')
        stream.write(f'- Assessment: `{assessment}`\n')
        stream.write('- CSV: `planner_quality_metrics.csv`\n\n')

        stream.write('## 3. Map Quality\n\n')
        recorded_map = next((item for item in map_quality if item.topic == '/map'), None)
        if recorded_map is not None:
            stream.write(f'- Global Map unknown ratio: `{_format_ratio(recorded_map.unknown_ratio)}`\n')
            stream.write(f'- Largest unknown region: `{recorded_map.largest_unknown_area:.3f} m^2`\n')
            stream.write(f'- Unknown connected components: `{recorded_map.unknown_connected_components}`\n')
            stream.write(f'- Quality: `{recorded_map.quality}`\n- Risk: `{recorded_map.risk}`\n')
        else:
            stream.write('- Global Map: not present in the selected bag.\n')
        stream.write('- CSV: `map_quality_metrics.csv`\n\n')

        stream.write('## 4. Costmap Analysis\n\n')
        for topic in sorted({item.topic for item in costmaps}):
            selected = [item for item in costmaps if item.topic == topic]
            label = topic.strip('/') or 'map'
            unknown = max(item.unknown_ratio for item in selected)
            occupied = max(item.occupied_ratio for item in selected)
            free = sum(item.free_ratio for item in selected) / len(selected)
            stream.write(f'- `{label}`: unknown `{_format_ratio(unknown)}`, occupied max `{_format_ratio(occupied)}`, free mean `{_format_ratio(free)}`\n')
            if unknown >= diagnosis.get('unknown_ratio_warning', 0.30):
                stream.write('  - WARNING: High unknown area may cause planner detours.\n')
        stream.write('- CSV: `costmap_metrics.csv`\n\n')

        stream.write('## 5. Controller Analysis\n\n')
        stream.write(f'- linear.x mean/max: `{controller.linear_x_mean:.3f}/{controller.linear_x_max:.3f} m/s`\n')
        stream.write(f'- angular.z max absolute: `{controller.angular_z_max_abs:.3f} rad/s`\n')
        stream.write(f'- angular sign changes: `{controller.angular_sign_change}`\n')
        stream.write(f'- oscillation score: `{controller.oscillation_score:.3f}` (active-sign transition ratio)\n')
        stream.write('- CSV: `controller_metrics.csv` (also kept as `cmd_vel_metrics.csv`)\n\n')

        stream.write('### Controller Tracking\n\n')
        stream.write('- Detailed report: `controller_tracking_report.md` (alias: `controller_report.md`)\n')
        stream.write('- Cross-track error CSV: `controller_tracking.csv`\n')
        stream.write('- cmd_vel dynamics CSV: `cmd_vel_dynamics.csv`\n\n')

        stream.write('## 6. Localization Analysis\n\n')
        stream.write(f'- odometry messages: `{localization.num_messages}`\n')
        stream.write(f'- traveled distance / displacement: `{localization.traveled_distance_m:.3f} / {localization.displacement_m:.3f} m`\n')
        stream.write(f'- mean/max local linear speed: `{localization.mean_linear_speed:.3f}/{localization.max_linear_speed:.3f} m/s`\n')
        stream.write(f'- max local angular speed: `{localization.max_angular_speed:.3f} rad/s`\n')
        stream.write('- CSV: `localization_metrics.csv`\n\n')

        stream.write('## 7. Diagnosis\n\n')
        stream.write(f'- high unknown area: `{diagnosis["high_unknown"]}`\n')
        stream.write(f'- high path curvature: `{diagnosis["high_curvature"]}`\n')
        stream.write(f'- high controller oscillation: `{diagnosis["high_oscillation"]}`\n')
        stream.write(f'- planner path assessment: `{"SMOOTH" if diagnosis["path_smooth"] else "UNSTABLE"}`\n')
        stream.write(f'- primary map suspect: `{diagnosis["primary_map_suspect"]}`\n')
        stream.write(f'- high occupied ratio: `{diagnosis["high_obstacle"]}`\n')
        stream.write(f'- unstable path intervals: `{diagnosis["unstable_path_count"]}`\n\n')
        for cause in diagnosis['causes']:
            stream.write(f'- **Likely cause / observation:** {cause}\n')
        stream.write('\nThe rules are evidence-based heuristics, not a proof of root cause. Review the per-message correlation CSV and plots before tuning Nav2.\n\n')
        stream.write('## 8. Odom Runtime Audit\n\n')
        if odom_runtime is None:
            stream.write('- Status: `MISSING`\n')
        else:
            found_odom = [item for item in odom_runtime.odom_metrics if item.status == 'FOUND']
            stream.write(f'- Present odometry topics: `{len(found_odom)}` / `{len(odom_runtime.odom_metrics)}`\n')
            stream.write('- Publisher node/count provenance: `UNKNOWN` when only rosbag data is available\n')
            stream.write('- CSV: `odom_frequency.csv`\n')
            stream.write('- TF report: `tf_conflict_report.md`\n')
            stream.write('- Detailed report: `odom_runtime_report.md`\n\n')

        stream.write('## 9. Visualizations\n\n')
        for filename in (
            'path_curvature.png', 'unknown_ratio.png', 'cmd_vel_angular.png',
            'tracking_error.png', 'plan_sequence.png', 'plan_curvature_timeline.png',
        ):
            stream.write(f'- `{output_dir / filename}`\n')
        stream.write('- Detailed runtime audit: `planner_runtime_report.md`\n')
        stream.write('- Nav2 goal/action audit: `nav2_runtime_report.md`\n')
        stream.write('- Dataset completeness: `dataset_completeness_report.md`\n')


def run_analysis(bag_path: Path, output_dir: Path, config_path: Path) -> Path:
    config = _load_config(config_path)
    topics = config['topics']
    selected_topics = list(dict.fromkeys(
        topics[key] for key in ('plan', 'map', 'global_costmap', 'local_costmap', 'cmd_vel', 'odometry')
    ))
    available, grouped = collect_records(str(bag_path), selected_topics)
    nav2_runtime_config = config.get('nav2_runtime', {})
    explicit_goal_topics = [
        topic for topic in nav2_runtime_config.get('goal_topics', []) if topic in available
    ]
    behavior_topics = [
        topic for topic in nav2_runtime_config.get('behavior_tree_topics', []) if topic in available
    ]
    action_topics = {}
    for action_kind, configured in nav2_runtime_config.get('action_topics', {}).items():
        action_topics[action_kind] = [topic for topic in configured if topic in available]
    # Include standard NavigateToPose action names even when a bag uses a
    # non-default namespace.
    for topic in available:
        marker = '/_action/'
        if marker in topic and 'navigate_to_pose' in topic:
            suffix = topic.rsplit(marker, 1)[1]
            if suffix in ('goal', 'feedback', 'result', 'status'):
                action_topics.setdefault(suffix, [])
                if topic not in action_topics[suffix]:
                    action_topics[suffix].append(topic)
    odom_runtime_config = config.get('odom_runtime', {})
    odom_topics = tuple(odom_runtime_config.get('odometry_topics', DEFAULT_ODOMETRY_TOPICS))
    odom_tf_topics = tuple(odom_runtime_config.get('tf_topics', DEFAULT_TF_TOPICS))
    optional_keys = ('tf', 'tf_static', 'parameter_events')
    optional_topics = [topics[key] for key in optional_keys if topics.get(key) in available]
    optional_topics.extend(topic for topic in odom_topics if topic in available)
    optional_topics.extend(topic for topic in odom_tf_topics if topic in available)
    optional_topics.extend(explicit_goal_topics + behavior_topics)
    optional_topics.extend(topic for names in action_topics.values() for topic in names)
    optional_topics = list(dict.fromkeys(optional_topics))
    _, optional_grouped = collect_records(str(bag_path), optional_topics) if optional_topics else ({}, {})
    tf_topics = [topics[key] for key in ('tf', 'tf_static') if topics.get(key) in optional_grouped]
    thresholds = config['thresholds']

    paths = [
        analyze_path_message(
            record.message, record.timestamp_ns, thresholds['sharp_turn_angle_rad'],
            thresholds.get('smoothness_curvature_scale_rad_per_m', 1.0),
        )
        for record in grouped[topics['plan']]
    ]
    costmaps = [
        analyze_costmap_message(record.message, record.topic, record.timestamp_ns)
        for topic in (topics['map'], topics['global_costmap'], topics['local_costmap'])
        for record in grouped[topic]
    ]
    map_quality = [
        analyze_map_message(
            record.message, record.topic, record.timestamp_ns,
            int(config.get('map_quality', {}).get('connectivity', 8)),
        )
        for topic in (topics['map'], topics['global_costmap'], topics['local_costmap'])
        for record in grouped[topic]
    ]
    commands = [
        CmdVelSample(record.timestamp_ns, float(record.message.linear.x), float(record.message.angular.z))
        for record in grouped[topics['cmd_vel']]
    ]
    odometry = [
        sample_from_message(record.message, record.timestamp_ns)
        for record in grouped[topics['odometry']]
    ]
    localization = analyze_odometry(odometry)
    controller = analyze_cmd_vel(commands, thresholds['angular_deadband_rad_per_sec'])
    correlation = correlate_path_and_cmd(
        paths, commands, odometry, thresholds['angular_deadband_rad_per_sec']
    )
    diagnosis = diagnose(paths, costmaps, controller, config, correlation, paths)
    diagnosis['unknown_ratio_warning'] = thresholds['unknown_ratio_warning']

    tracking_config = config.get('tracking', {})
    plan_snapshots = [
        make_plan_snapshot(record.message, record.timestamp_ns, metric.smoothness_score)
        for record, metric in zip(grouped[topics['plan']], paths)
    ]
    target_frame = plan_snapshots[0].frame_id if plan_snapshots else ''
    source_odometry_frames = tuple(sorted({item.frame_id for item in odometry if item.frame_id}))
    odometry_for_tracking, transformed_count, untransformed_count = transform_odometry_to_frame(
        odometry,
        [record for topic in tf_topics for record in optional_grouped.get(topic, [])],
        target_frame,
    )
    tracking_samples, tracking_summary = analyze_tracking(
        plan_snapshots,
        odometry_for_tracking,
        plan_hold_sec=float(tracking_config.get('plan_hold_sec', 2.0)),
    )
    tracking_summary.tf_transformed_messages = transformed_count
    tracking_summary.tf_untransformed_messages = untransformed_count
    tracking_summary.source_odometry_frame_ids = source_odometry_frames
    dynamics, dynamics_summary = analyze_cmd_vel_dynamics(commands)
    tracking_assessment = assess_controller_tracking(paths, tracking_summary, tracking_config)

    runtime_config = config.get('planner_runtime', {})
    runtime_costmap_records = [
        record
        for topic in (topics['global_costmap'], topics['local_costmap'])
        for record in grouped[topic]
    ]
    runtime_metrics, replan_summary = analyze_plan_runtime(
        grouped[topics['plan']], paths, runtime_costmap_records, runtime_config,
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    write_path_csv(paths, output_dir / 'path_metrics.csv')
    write_planner_quality_csv(paths, output_dir / 'planner_quality_metrics.csv')
    write_costmap_csv(costmaps, output_dir / 'costmap_metrics.csv')
    write_map_quality_csv(map_quality, output_dir / 'map_quality_metrics.csv')
    write_controller_csv(controller, output_dir / 'cmd_vel_metrics.csv')
    write_controller_csv(controller, output_dir / 'controller_metrics.csv')
    write_localization_csv(localization, output_dir / 'localization_metrics.csv')
    write_correlation_csv(correlation, output_dir / 'planner_controller_correlation.csv')
    write_tracking_csv(tracking_samples, output_dir / 'controller_tracking.csv')
    write_cmd_vel_dynamics_csv(dynamics, output_dir / 'cmd_vel_dynamics.csv')
    write_runtime_metrics_csv(runtime_metrics, output_dir / 'planner_runtime_metrics.csv')
    odom_grouped = dict(grouped)
    odom_optional_topics = [topic for topic in odom_topics if topic in available]
    for topic in odom_optional_topics:
        odom_grouped.setdefault(topic, optional_grouped.get(topic, []))
    for topic in odom_tf_topics:
        odom_grouped.setdefault(topic, optional_grouped.get(topic, []))
    odom_runtime = analyze_odom_runtime(
        available,
        odom_grouped,
        grouped[topics['plan']],
        runtime_metrics,
        odom_runtime_config,
    )
    write_odom_frequency_csv(odom_runtime.odom_metrics, output_dir / 'odom_frequency.csv')
    write_tf_conflict_report(
        output_dir / 'tf_conflict_report.md',
        odom_runtime,
        odom_tf_topics,
        available,
    )
    write_odom_runtime_report(output_dir / 'odom_runtime_report.md', odom_runtime, topics['plan'])
    plot_path_curvature(paths, output_dir / 'path_curvature.png')
    plot_unknown_ratio(costmaps, output_dir / 'unknown_ratio.png')
    plot_cmd_vel_angular(commands, output_dir / 'cmd_vel_angular.png', thresholds['angular_deadband_rad_per_sec'])
    plot_tracking_error(
        tracking_samples,
        output_dir / 'tracking_error.png',
        float(tracking_config.get('high_mean_error_m', 0.10)),
    )
    plot_plan_sequence(runtime_metrics, output_dir / 'plan_sequence.png')
    plot_plan_curvature_timeline(runtime_metrics, output_dir / 'plan_curvature_timeline.png')
    parameter_topic = topics.get('parameter_events')
    parameter_records = optional_grouped.get(parameter_topic, []) if parameter_topic else []
    runtime_config_info = write_planner_runtime_config(
        parameter_records, output_dir / 'planner_runtime_config.yaml',
    )
    write_planner_runtime_report(
        output_dir / 'planner_runtime_report.md',
        runtime_metrics,
        replan_summary,
        runtime_config_info,
        runtime_config,
    )
    goal_items = build_goal_timeline(
        optional_grouped,
        explicit_topics=explicit_goal_topics,
        action_goal_topics=action_topics.get('goal', []),
    )
    action_summary = analyze_navigate_to_pose_action(optional_grouped, action_topics)
    behavior_summary = analyze_behavior_tree_logs(optional_grouped, behavior_topics)
    goal_plan_association = correlate_goals_plans(
        goal_items,
        grouped[topics['plan']],
        runtime_metrics,
        goal_change_threshold_m=float(nav2_runtime_config.get('goal_change_threshold_m', 0.05)),
        costmap_trigger_threshold=float(runtime_config.get('high_costmap_trigger_score', 0.10)),
    )
    write_goal_timeline_csv(goal_items, output_dir / 'goal_timeline.csv')
    write_action_metrics_csv(action_summary, output_dir / 'nav2_action_metrics.csv')
    write_nav2_runtime_report(
        output_dir / 'nav2_runtime_report.md',
        goal_items,
        action_summary,
        behavior_summary,
        goal_plan_association,
        replan_summary,
        available,
    )
    recorder_profile_path = _default_recorder_profile_path()
    analyze_bag_completeness(
        bag_path,
        recorder_profile_path,
        output_dir / 'dataset_completeness_report.md',
    )
    controller_report_path = output_dir / 'controller_tracking_report.md'
    write_controller_tracking_report(
        controller_report_path, paths, tracking_summary, dynamics_summary, tracking_assessment,
    )
    shutil.copyfile(controller_report_path, output_dir / 'controller_report.md')
    recorded_map_quality = next((item for item in map_quality if item.topic == topics['map']), None)
    if recorded_map_quality is not None:
        write_occupancy_grid_asset_report(recorded_map_quality, output_dir / 'map_asset_report.md')
    else:
        (output_dir / 'map_asset_report.md').write_text(
            '# Map Asset Validation Report\n\nNo `/map` OccupancyGrid was present in the selected bag.\n',
            encoding='utf-8',
        )

    report_path = output_dir / f'{bag_path.name}_report.md'
    _write_report(report_path, bag_path, _metadata(bag_path), available, paths, costmaps, map_quality, controller, localization, diagnosis, output_dir, odom_runtime)
    return report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description='Analyze a ROS 2 Humble Nav2 rosbag offline.')
    parser.add_argument('--bag', required=True, type=Path, help='rosbag2 sqlite3 directory')
    parser.add_argument('--output-dir', type=Path, default=Path('reports'))
    parser.add_argument('--config', type=Path, default=_default_config_path())
    args = parser.parse_args(argv)
    report_path = run_analysis(args.bag, args.output_dir, args.config)
    print(f'Analysis complete: {report_path}')


if __name__ == '__main__':
    main()
