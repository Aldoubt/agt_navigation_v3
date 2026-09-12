from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='agt_sim_odometry_adapter', executable='sim_odometry_adapter',
            name='agt_sim_odometry_adapter', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/sim/ground_truth_odom',
                'output_topic': '/agt/odometry/local',
                'output_parent_frame': 'odom',
                'output_child_frame': 'base_link',
            }],
        ),
    ])
