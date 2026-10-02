"""The sole runtime adapter between guarded AGT Twist and Bunker driver topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('adapter', default_value='bunker', choices=['bunker']),
        DeclareLaunchArgument('input_topic', default_value='/agt/base/cmd_vel'),
        DeclareLaunchArgument('output_topic', default_value='/mux/cmd_vel'),
        Node(
            package='agt_base_runtime', executable='bunker_adapter',
            name='agt_bunker_base_adapter', output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }]),
    ])
