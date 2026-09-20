"""Compose Gazebo, external-localization handoff, and Nav2 for P3 acceptance.

No SLAM, fake localization node, or global relocalization backend is started.
Publish a valid external ``/agt/relocalization/pose`` to exercise the frozen
LocalizationManager handoff and establish ``map -> odom``.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    gazebo_share = Path(get_package_share_directory('agt_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to the Nav2 map YAML used for acceptance.'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=''),
        DeclareLaunchArgument(
            'world', default_value=str(gazebo_share / 'worlds' / 'agt_mapping.world')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        _include('agt_simulation_bringup', 'gazebo_field_acceptance.launch.py', {
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }),
        _include('agt_localization_manager', 'localization_manager.launch.py', {
            'use_sim_time': 'true',
            'debug_identity_map_odom': 'false',
        }),
        _include('agt_simulation_bringup', 'nav2_sim_acceptance.launch.py', {
            'map': LaunchConfiguration('map'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true',
        }),
    ])
