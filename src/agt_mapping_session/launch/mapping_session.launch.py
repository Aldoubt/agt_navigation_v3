from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agt_mapping_session',
            executable='mapping_session',
            name='agt_mapping_session',
            output='screen'
        )
    ])
