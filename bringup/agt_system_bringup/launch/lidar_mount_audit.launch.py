"""Minimal MID360 mount/LIO audit runtime.

This launch intentionally excludes Nav2, global relocalization, Map Tracker, RTK,
mission runtime, and camera processes.  It supports both live MID360 input and a
raw LiDAR/IMU rosbag replay while preserving the same chassis TF and Batch-LIO
calibration sources used by the field runtime.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package: str, launch_file: str, arguments=None, condition=None, launch_directory='launch'):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / launch_directory / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    description_share = Path(get_package_share_directory('tracked_chassis_description'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    report_dir = LaunchConfiguration('report_dir')
    audit_report = PathJoinSubstitution([report_dir, 'mid360_mount_audit.yaml'])
    filter_report = PathJoinSubstitution([report_dir, 'obstacle_filter_statistics.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_mid360_driver',
            default_value='false',
            description='Start the live MID360 driver. Keep false for rosbag replay.'),
        DeclareLaunchArgument(
            'replay_bag',
            default_value='false',
            description='Replay only lidar_topic and imu_topic from bag with --clock.'),
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml')),
        DeclareLaunchArgument(
            'robot_description_calibration_file',
            default_value=str(description_share / 'config' / 'field_acceptance.yaml')),
        DeclareLaunchArgument(
            'report_dir',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), '.ros', 'agt_mount_audit'])),
        DeclareLaunchArgument(
            'audit_duration_sec',
            default_value='60.0',
            description='Sensor timestamp span captured by mid360_mount_audit.py.'),
        DeclareLaunchArgument(
            'debug_base_cloud_enabled',
            default_value='true',
            description='Publish /agt/debug/points_obstacles_base in base_link for RViz audit.'),

        # One physical TF authority for live and replay.
        _include('tracked_chassis_description', 'display.launch.py', {
            'calibration_file': LaunchConfiguration('robot_description_calibration_file'),
            'start_rviz': 'false',
        }),

        # Optional live source.  Replay mode leaves this disabled.
        _include(
            'livox_ros_driver2',
            'msg_MID360_launch.py',
            condition=IfCondition(LaunchConfiguration('enable_mid360_driver')),
            launch_directory='launch_ROS2',
        ),

        # Canonical local LIO + body/base adapter.
        _include('agt_navigation_runtime', 'navigation_lio.launch.py', {
            'use_sim_time': use_sim_time,
            'launch_batch_rviz': 'false',
            'batch_config': LaunchConfiguration('batch_config'),
            'lidar_topic': lidar_topic,
            'imu_topic': imu_topic,
        }),

        # Secondary PointCloud2 branch for self-filter / TF diagnostics.
        Node(
            package='agt_livox_tools',
            executable='livox_format_bridge',
            name='mount_audit_livox_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'mode': 'custom_to_pointcloud2',
                'input_topic': lidar_topic,
                'output_topic': '/agt/livox/points',
            }],
        ),
        _include('agt_pointcloud_preprocessor', 'obstacle_cloud.launch.py', {
            'use_sim_time': use_sim_time,
            'statistics_output': filter_report,
            'debug_log_interval_sec': '5.0',
            'debug_base_cloud_enabled': LaunchConfiguration(
                'debug_base_cloud_enabled'),
            'debug_base_cloud_topic': '/agt/debug/points_obstacles_base',
        }),

        # Read-only automatic YAML report.
        Node(
            package='agt_navigation_runtime',
            executable='mid360_mount_audit.py',
            name='mid360_mount_audit',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'imu_topic': imu_topic,
                'odom_topic': '/agt/odometry/local',
                'raw_cloud_topic': '/agt/livox/points',
                'obstacle_cloud_topic': '/agt/navigation/points_obstacles',
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'duration_sec': ParameterValue(
                    LaunchConfiguration('audit_duration_sec'), value_type=float),
                'report_file': audit_report,
            }],
        ),

        # Replay only raw sensor inputs.  Recorded TF/odom cannot override the
        # current calibration or hide a regression.
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play', LaunchConfiguration('bag'),
                        '--clock', '--rate', LaunchConfiguration('bag_rate'),
                        '--topics', lidar_topic, imu_topic,
                    ],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('replay_bag')),
                ),
            ],
        ),
    ])
