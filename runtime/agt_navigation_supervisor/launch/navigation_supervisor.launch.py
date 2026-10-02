"""Standalone navigation health supervisor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_profile', default_value='bunker_v1'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('map_version', default_value=''),
        DeclareLaunchArgument('map_valid', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='agt_navigation_supervisor', executable='navigation_supervisor',
             name='agt_navigation_supervisor', output='screen', parameters=[{
                 'robot_profile': LaunchConfiguration('robot_profile'),
                 'map_id': LaunchConfiguration('map_id'),
                 'map_version': LaunchConfiguration('map_version'),
                 'map_valid': ParameterValue(LaunchConfiguration('map_valid'), value_type=bool),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
