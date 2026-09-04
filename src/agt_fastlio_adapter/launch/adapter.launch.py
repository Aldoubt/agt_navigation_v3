from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_fastlio_adapter'))
    return LaunchDescription([
        Node(
            package='agt_fastlio_adapter',
            executable='fastlio_adapter',
            name='agt_fastlio_adapter',
            output='screen',
            parameters=[str(share / 'config' / 'adapter.yaml')],
        )
    ])
