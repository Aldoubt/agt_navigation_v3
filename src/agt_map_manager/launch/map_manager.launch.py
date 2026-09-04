from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_map_manager'))
    return LaunchDescription([
        Node(
            package='agt_map_manager',
            executable='map_manager',
            name='agt_map_manager',
            output='screen',
            parameters=[str(share / 'config' / 'map_manager.yaml')],
        )
    ])
