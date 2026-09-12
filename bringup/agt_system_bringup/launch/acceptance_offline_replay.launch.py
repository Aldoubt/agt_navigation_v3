"""Rosbag-based software pre-acceptance.

No Bunker/CAN/camera hardware drivers are launched. The bag supplies recorded
sensor data; the stack validates LIO, localization, low-rate map tracking, TF,
map loading and Nav2 planning. A recorded bag cannot close the control loop
against new cmd_vel.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, launch_file: str, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    acceptance_params = (
        Path(get_package_share_directory('agt_nav2_bringup'))
        / 'config' / 'nav2_acceptance_params.yaml'
    )

    enable_map_tracking = LaunchConfiguration('enable_map_tracking')
    relocalization_executable = LaunchConfiguration('relocalization_executable')
    auto_relocalize = LaunchConfiguration('auto_relocalize')
    launch_rviz = LaunchConfiguration('launch_rviz')
    enable_replay_audit = LaunchConfiguration('enable_replay_audit')
    enable_seed_injector = LaunchConfiguration('enable_seed_injector')
    enable_pose_chain_recorder = LaunchConfiguration('enable_pose_chain_recorder')

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to test-field rosbag directory'),
        DeclareLaunchArgument('navigation_map', description='Acceptance Nav2 map.yaml'),
        DeclareLaunchArgument('localization_map', description='Matching localization PCD'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'gicp_constraint_mode', default_value='full_se3',
            description='Opt-in manual-seed experiment: full_se3, gravity_constrained, or gravity_prior.'),
        DeclareLaunchArgument('max_roll_delta_deg', default_value='10.0'),
        DeclareLaunchArgument('max_pitch_delta_deg', default_value='10.0'),
        DeclareLaunchArgument('cloud_contract_capture_dir', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=str(acceptance_params)),
        DeclareLaunchArgument(
            'relocalization_executable',
            default_value='manual_seed_relocalization',
            description='manual_seed_relocalization or global_relocalization.'),
        DeclareLaunchArgument(
            'auto_relocalize',
            default_value='false',
            description='Enable only when using the global relocalization backend.'),
        DeclareLaunchArgument(
            'relocalization_query_frame_mode', default_value='mapping_body',
            description='Legacy direct argument; formal PGO mode is mapping_body.'),
        DeclareLaunchArgument(
            'global_query_frame_mode', default_value='mapping_body',
            description='Formal PGO map query mode: mapping_body; base_link is deprecated compatibility.'),
        DeclareLaunchArgument(
            'bbs_query_frame_mode', default_value='mapping_body',
            description='Formal PGO candidate-BBS mode mapping_body; base_link is deprecated compatibility.'),
        DeclareLaunchArgument(
            'enable_map_tracking',
            default_value='false',
            description='Enable the 0.5 Hz local-map GICP tracker after a global anchor exists.'),
        DeclareLaunchArgument('map_tracker_apply_correction', default_value='true'),
        DeclareLaunchArgument('map_tracker_save_debug_cloud', default_value='false'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch the offline localization RViz view.'),
        DeclareLaunchArgument(
            'map_tracker_scan_topic',
            default_value='/agt/relocalization/input_cloud',
            description='PointCloud2 query topic used by the offline map tracker.'),
        DeclareLaunchArgument(
            'enable_replay_audit', default_value='false',
            description='Start the side-channel deterministic replay auditor.'),
        DeclareLaunchArgument(
            'report_dir', default_value='~/.ros/agt_acceptance/replay_reports',
            description='Parent directory for replay-audit JSONL/YAML reports.'),
        DeclareLaunchArgument(
            'map_package_dir',
            default_value=(
                '/home/yangxuan/ros2_ws/agt_data/maps/'
                'bunker_mid360_mapping_20260901_205036/v003-indexed'),
            description='Frozen v003-indexed package audited by replay_audit.'),
        DeclareLaunchArgument(
            'map_gate_evidence', default_value='',
            description='Optional persistent P0.5 existing_map_audit.yaml with acceptance_status=PASS.'),
        DeclareLaunchArgument(
            'map_gate_status', default_value='PASS',
            description='Frozen reviewed P0/P0.5 MAP gate result when no evidence file is supplied.'),
        DeclareLaunchArgument('enable_seed_injector', default_value='false'),
        DeclareLaunchArgument('enable_pose_chain_recorder', default_value='false'),
        DeclareLaunchArgument('pose_chain_report_dir', default_value='~/.ros/agt_pose_chain'),
        DeclareLaunchArgument('seed_target_clock_sec', default_value='0.0'),
        DeclareLaunchArgument('seed_x', default_value='0.0'), DeclareLaunchArgument('seed_y', default_value='0.0'),
        DeclareLaunchArgument('seed_z', default_value='0.0'), DeclareLaunchArgument('seed_qx', default_value='0.0'),
        DeclareLaunchArgument('seed_qy', default_value='0.0'), DeclareLaunchArgument('seed_qz', default_value='0.0'),
        DeclareLaunchArgument('seed_qw', default_value='1.0'),

        _include('agt_system_bringup', 'offline_relocalization_demo.launch.py', {
            'global_map': LaunchConfiguration('localization_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_sim_time': 'true',
            'launch_rviz': launch_rviz,
            'auto_relocalize': auto_relocalize,
            'relocalization_executable': relocalization_executable,
            'relocalization_query_frame_mode': LaunchConfiguration('global_query_frame_mode'),
            'bbs_query_frame_mode': LaunchConfiguration('bbs_query_frame_mode'),
            'gicp_constraint_mode': LaunchConfiguration('gicp_constraint_mode'),
            'max_roll_delta_deg': LaunchConfiguration('max_roll_delta_deg'),
            'max_pitch_delta_deg': LaunchConfiguration('max_pitch_delta_deg'),
            'cloud_contract_capture_dir': LaunchConfiguration('cloud_contract_capture_dir'),
        }),

        _include(
            'agt_map_tracker',
            'map_tracker.launch.py',
            {
                'global_map': LaunchConfiguration('localization_map'),
                'scan_topic': LaunchConfiguration('map_tracker_scan_topic'),
                'use_sim_time': 'true',
                'apply_correction': LaunchConfiguration('map_tracker_apply_correction'),
                'save_debug_cloud': LaunchConfiguration('map_tracker_save_debug_cloud'),
            },
            condition=IfCondition(enable_map_tracking),
        ),

        _include('agt_nav2_bringup', 'navigation.launch.py', {
            'map': LaunchConfiguration('navigation_map'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }),

        Node(
            package='agt_operator_console', executable='replay_audit',
            name='agt_replay_audit', output='screen',
            condition=IfCondition(enable_replay_audit),
            parameters=[{
                'use_sim_time': True,
                'report_dir': LaunchConfiguration('report_dir'),
                'navigation_map': LaunchConfiguration('navigation_map'),
                'map_package_dir': LaunchConfiguration('map_package_dir'),
                'map_gate_evidence': LaunchConfiguration('map_gate_evidence'),
                'map_gate_status': LaunchConfiguration('map_gate_status'),
                'global_query_frame_mode': LaunchConfiguration('global_query_frame_mode'),
                'enable_map_tracking': enable_map_tracking,
            }],
        ),
        Node(package='agt_operator_console', executable='deterministic_seed_injector',
             name='deterministic_seed_injector', condition=IfCondition(enable_seed_injector),
             parameters=[{'use_sim_time': True, 'target_clock_sec': LaunchConfiguration('seed_target_clock_sec'),
                          'x': LaunchConfiguration('seed_x'), 'y': LaunchConfiguration('seed_y'), 'z': LaunchConfiguration('seed_z'),
                          'qx': LaunchConfiguration('seed_qx'), 'qy': LaunchConfiguration('seed_qy'), 'qz': LaunchConfiguration('seed_qz'), 'qw': LaunchConfiguration('seed_qw')}]),
        Node(package='agt_operator_console', executable='pose_chain_recorder',
             name='pose_chain_recorder', condition=IfCondition(enable_pose_chain_recorder),
             parameters=[{'use_sim_time': True, 'report_dir': LaunchConfiguration('pose_chain_report_dir')}]),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play', LaunchConfiguration('bag'),
                        '--clock', '--rate', LaunchConfiguration('bag_rate'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
