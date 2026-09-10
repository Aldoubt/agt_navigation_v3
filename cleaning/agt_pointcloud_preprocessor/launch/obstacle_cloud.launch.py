from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_pointcloud_preprocessor'))
    return LaunchDescription([
        Node(
            package='agt_pointcloud_preprocessor',
            executable='obstacle_cloud_node',
            name='agt_obstacle_cloud_preprocessor',
            output='screen',
            parameters=[str(share / 'config' / 'obstacle_cloud.yaml')],
        )
    ])
