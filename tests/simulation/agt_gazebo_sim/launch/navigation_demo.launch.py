from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('x', default_value='2.8'),
        DeclareLaunchArgument('y', default_value='-2.2'),
        DeclareLaunchArgument('yaw', default_value='1.2'),
        include('agt_gazebo_sim', 'sim_world.launch.py', {'x': x, 'y': y, 'yaw': yaw}),
        include('agt_system_bringup', 'navigation.launch.py', {
            'map': map_yaml,
            'map_id': 'gazebo_mapping_v1',
            'use_sim_time': 'true',
        }),
    ])
