"""MQ4 planner-only fixture: no controller, robot driver, localization, or MPPI."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def validate(context):
    for key in ('map_yaml', 'nav2_params_file'):
        value = LaunchConfiguration(key).perform(context)
        if not Path(value).is_file():
            raise RuntimeError(f'MQ4 fixture missing {key}: {value}')
    return []

def generate_launch_description():
    params = LaunchConfiguration('nav2_params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'), DeclareLaunchArgument('nav2_params_file'),
        OpaqueFunction(function=validate),
        Node(package='tf2_ros', executable='static_transform_publisher', name='mq4_map_odom_tf',
             arguments=['0','0','0','0','0','0','map','odom']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': map_yaml, 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, {'use_sim_time': False}]),
    ])
