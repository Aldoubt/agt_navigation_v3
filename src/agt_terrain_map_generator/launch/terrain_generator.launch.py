from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_terrain_map_generator'))
    params = share / 'config' / 'terrain_generator.yaml'

    return LaunchDescription([
        Node(
            package='agt_terrain_map_generator',
            executable='terrain_generator_node',
            name='agt_terrain_map_generator',
            output='screen',
            parameters=[str(params)],
        )
    ])
