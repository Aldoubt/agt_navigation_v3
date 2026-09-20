"""Start Nav2 for P3 simulation acceptance without AMCL or fake localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_share = Path(get_package_share_directory('agt_nav2_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to the Nav2 map YAML used for acceptance.'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value='',
            description='AGT Nav2 parameters for map, planner, controller and lifecycle nodes.'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'use_sim_time': 'true',
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])
