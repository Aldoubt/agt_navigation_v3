"""Compatibility entry point; agt_robot_bringup owns every physical driver."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=''),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agt_robot_bringup'), 'launch',
                'robot_hardware.launch.py'])), launch_arguments={
                    'robot_config': LaunchConfiguration('robot_config'),
                    'robot': LaunchConfiguration('robot'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()),
    ])
