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

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to test-field rosbag directory'),
        DeclareLaunchArgument('navigation_map', description='Acceptance Nav2 map.yaml'),
        DeclareLaunchArgument('localization_map', description='Matching localization PCD'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
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
            'enable_map_tracking',
            default_value='false',
            description='Enable the 0.5 Hz local-map GICP tracker after a global anchor exists.'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch the offline localization RViz view.'),
        DeclareLaunchArgument(
            'map_tracker_scan_topic',
            default_value='/agt/relocalization/input_cloud',
            description='PointCloud2 query topic used by the offline map tracker.'),

        _include('agt_system_bringup', 'offline_relocalization_demo.launch.py', {
            'global_map': LaunchConfiguration('localization_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_sim_time': 'true',
            'launch_rviz': launch_rviz,
            'auto_relocalize': auto_relocalize,
            'relocalization_executable': relocalization_executable,
        }),

        _include(
            'agt_map_tracker',
            'map_tracker.launch.py',
            {
                'global_map': LaunchConfiguration('localization_map'),
                'scan_topic': LaunchConfiguration('map_tracker_scan_topic'),
                'use_sim_time': 'true',
            },
            condition=IfCondition(enable_map_tracking),
        ),

        _include('agt_nav2_bringup', 'navigation.launch.py', {
            'map': LaunchConfiguration('navigation_map'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }),

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
