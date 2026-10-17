"""Standalone RViz hand-drawn path service; it owns no TF or Nav2 nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_id', default_value='demo_map'),
        DeclareLaunchArgument('map_version', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='agt_rviz_patrol', executable='rviz_path_tool',
             name='agt_rviz_path_tool', output='screen', parameters=[{
                 'preview_only': False,
                 'map_id': LaunchConfiguration('map_id'),
                 'map_version': LaunchConfiguration('map_version'),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
