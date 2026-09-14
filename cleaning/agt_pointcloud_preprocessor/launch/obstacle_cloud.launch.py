from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('agt_pointcloud_preprocessor'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'rear_sector_enabled', default_value='false',
            description='Experimental navigation-only rear-sector exclusion; false preserves field behavior.'),
        DeclareLaunchArgument('rear_sector_center_deg', default_value='180.0'),
        DeclareLaunchArgument('rear_sector_width_deg', default_value='10.0'),
        DeclareLaunchArgument('rear_sector_min_range_m', default_value='0.5'),
        DeclareLaunchArgument('rear_sector_max_range_m', default_value='4.0'),
        DeclareLaunchArgument(
            'statistics_output', default_value='',
            description='Optional final YAML path for read-only filter statistics.'),
        DeclareLaunchArgument(
            'debug_log_interval_sec', default_value='0.0',
            description='Cumulative filter-statistics log interval; 0 disables periodic logs.'),
        DeclareLaunchArgument(
            'debug_base_cloud_enabled', default_value='false',
            description='Publish accepted obstacle points transformed to base_link for audit/RViz.'),
        DeclareLaunchArgument(
            'debug_base_cloud_topic', default_value='/agt/debug/points_obstacles_base'),
        Node(
            package='agt_pointcloud_preprocessor',
            executable='obstacle_cloud_node',
            name='agt_obstacle_cloud_preprocessor',
            output='screen',
            parameters=[
                str(share / 'config' / 'obstacle_cloud.yaml'),
                {
                    'rear_sector.enabled': ParameterValue(
                        LaunchConfiguration('rear_sector_enabled'), value_type=bool),
                    'rear_sector.center_deg': ParameterValue(
                        LaunchConfiguration('rear_sector_center_deg'), value_type=float),
                    'rear_sector.width_deg': ParameterValue(
                        LaunchConfiguration('rear_sector_width_deg'), value_type=float),
                    'rear_sector.min_range_m': ParameterValue(
                        LaunchConfiguration('rear_sector_min_range_m'), value_type=float),
                    'rear_sector.max_range_m': ParameterValue(
                        LaunchConfiguration('rear_sector_max_range_m'), value_type=float),
                    'statistics_output': LaunchConfiguration('statistics_output'),
                    'debug_log_interval_sec': ParameterValue(
                        LaunchConfiguration('debug_log_interval_sec'), value_type=float),
                    'debug_base_cloud.enabled': ParameterValue(
                        LaunchConfiguration('debug_base_cloud_enabled'), value_type=bool),
                    'debug_base_cloud.topic': LaunchConfiguration('debug_base_cloud_topic'),
                },
            ],
        )
    ])
