"""Standalone RViz UI. It creates no TF, localization, or navigation owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rviz(context):
    config = LaunchConfiguration('config_file').perform(context).strip()
    arguments = ['-d', config] if config else []
    return [Node(package='rviz2', executable='rviz2', name='agt_navigation_rviz',
                 arguments=arguments, output='screen', parameters=[{
                     'use_sim_time': ParameterValue(
                         LaunchConfiguration('use_sim_time'), value_type=bool),
                 }])]


def generate_launch_description():
    default = str(Path(get_package_share_directory('agt_rviz_patrol')) /
                  'config' / 'agt_rviz_demo.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_rviz),
    ])
