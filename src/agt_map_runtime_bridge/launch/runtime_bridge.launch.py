"""Launch file for map runtime bridge.

Starts the ROS2 runtime coordination node. Backend adapters are enabled
separately after Nav2/localization integration is validated.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agt_map_runtime_bridge',
            executable='map_runtime_bridge_node',
            name='map_runtime_bridge',
            output='screen',
            parameters=[{
                'runtime_namespace': '/agt/map/runtime'
            }],
        )
    ])
