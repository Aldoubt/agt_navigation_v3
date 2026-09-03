from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_localization_manager'))
    return LaunchDescription([
        Node(
            package='agt_localization_manager',
            executable='localization_manager',
            name='agt_localization_manager',
            output='screen',
            parameters=[str(share / 'config' / 'localization_manager.yaml')],
        )
    ])
