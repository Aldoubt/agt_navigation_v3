from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_base_control'))
    return LaunchDescription([
        Node(
            package='agt_base_control',
            executable='cmd_vel_guard',
            name='agt_cmd_vel_guard',
            output='screen',
            parameters=[str(share / 'config' / 'cmd_vel_guard.yaml')],
        )
    ])
