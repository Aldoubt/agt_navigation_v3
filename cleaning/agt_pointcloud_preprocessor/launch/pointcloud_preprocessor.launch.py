"""Compatibility name for the local perception atomic launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    canonical = Path(get_package_share_directory('agt_pointcloud_preprocessor')) / 'launch' / 'local_perception.launch.py'
    names = (
        'params_file', 'use_sim_time', 'rear_filter_enabled', 'rear_filter_center_deg',
        'rear_filter_width_deg', 'rear_filter_min_range_m', 'rear_filter_max_range_m',
        'statistics_output', 'debug_log_interval_sec', 'debug_base_cloud_enabled',
        'debug_base_cloud_topic',
    )
    defaults = {
        'params_file': '', 'use_sim_time': 'false', 'rear_filter_enabled': 'false',
        'rear_filter_center_deg': '180.0', 'rear_filter_width_deg': '70.0',
        'rear_filter_min_range_m': '0.5', 'rear_filter_max_range_m': '1.0',
        'statistics_output': '', 'debug_log_interval_sec': '0.0',
        'debug_base_cloud_enabled': 'false',
        'debug_base_cloud_topic': '/agt/debug/points_obstacles_base',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=defaults[name]) for name in names],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(canonical)),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items()),
    ])
