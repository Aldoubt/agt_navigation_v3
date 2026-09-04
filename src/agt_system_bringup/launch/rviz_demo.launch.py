import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, condition=None, arguments=None):
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )
    return action


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    map_id = LaunchConfiguration('map_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    enable_rtk = LaunchConfiguration('enable_rtk')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to converted Nav2 map.yaml'),
        DeclareLaunchArgument('map_id', default_value='rviz_demo_map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),

        include('agt_rtk_manager', 'rtk_manager.launch.py', condition=IfCondition(enable_rtk)),
        include('agt_nav2_bringup', 'navigation.launch.py', arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
        }),
        include('agt_base_control', 'cmd_vel_guard.launch.py'),
        include('agt_navigation_runtime', 'runtime.launch.py'),
        include('agt_rviz_patrol', 'rviz_patrol.launch.py', arguments={
            'map_id': map_id,
        }),

        Node(
            package='rviz2',
            executable='rviz2',
            name='agt_demo_rviz',
            output='screen',
            condition=IfCondition(launch_rviz),
        ),
    ])
