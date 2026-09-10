from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        include('agt_gazebo_sim', 'sim_world.launch.py'),
        include('agt_mapping_bringup', 'mapping_mode.launch.py', {
            'use_sim_time': 'true', 'launch_rviz': 'false', 'enable_pgo': 'true'}),
        Node(
            package='agt_gazebo_sim', executable='mapping_drive',
            name='agt_sim_mapping_drive', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
