from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('agt_bunker_control'), 'config', 'command_guard.yaml')
    return LaunchDescription([
        Node(
            package='agt_bunker_control',
            executable='command_guard',
            name='agt_bunker_command_guard',
            output='screen',
            parameters=[params],
        )
    ])
