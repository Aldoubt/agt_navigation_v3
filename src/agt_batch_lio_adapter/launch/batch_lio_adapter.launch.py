import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('agt_batch_lio_adapter'),
        'config', 'batch_lio_adapter.yaml')
    return LaunchDescription([
        Node(
            package='agt_batch_lio_adapter',
            executable='batch_lio_adapter',
            name='agt_batch_lio_adapter',
            output='screen',
            parameters=[cfg],
        )
    ])
