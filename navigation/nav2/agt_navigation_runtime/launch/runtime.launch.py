import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agt_navigation_runtime')
    return LaunchDescription([
        Node(
            package='agt_navigation_runtime',
            executable='mission_runtime',
            name='mission_runtime',
            output='screen',
            parameters=[os.path.join(share, 'config', 'runtime.yaml')],
        )
    ])
