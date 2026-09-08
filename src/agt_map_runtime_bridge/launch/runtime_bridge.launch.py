from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agt_map_runtime_bridge',
            executable='map_runtime_bridge_node',
            name='map_runtime_bridge',
            output='screen',
        )
    ])
