from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('agt_yaw_constraint')) /
                 'config' / 'yaw_constraint.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('local_odom_topic', default_value='/agt/odometry/local'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('min_speed_threshold', default_value='0.05'),
        DeclareLaunchArgument('sync_max_dt_sec', default_value='0.2'),
        DeclareLaunchArgument('max_delta_yaw_jump_deg', default_value='45.0'),
        DeclareLaunchArgument(
            'debug_output_topic',
            default_value='/agt/localization/yaw_constraint_debug'),
        Node(
            package='agt_yaw_constraint',
            executable='yaw_constraint',
            name='agt_yaw_constraint',
            output='screen',
            parameters=[config, {
                'local_odom_topic': LaunchConfiguration('local_odom_topic'),
                'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'min_speed_threshold': LaunchConfiguration('min_speed_threshold'),
                'sync_max_dt_sec': LaunchConfiguration('sync_max_dt_sec'),
                'max_delta_yaw_jump_deg': LaunchConfiguration(
                    'max_delta_yaw_jump_deg'),
                'debug_output_topic': LaunchConfiguration('debug_output_topic'),
            }],
        ),
    ])
