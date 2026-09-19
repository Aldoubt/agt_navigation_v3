"""Standalone, non-TF Localization v1 shadow manager launch.

This file is intentionally not included by existing field or navigation launch
files.  Operators start it alongside an unchanged legacy stack when needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('local_odom_topic', default_value='/agt/odometry/local'),
        DeclareLaunchArgument('global_pose_topic', default_value='/agt/relocalization/pose'),
        DeclareLaunchArgument('tracking_pose_topic', default_value='/agt/map_tracking/pose'),
        DeclareLaunchArgument('tracking_status_topic', default_value='/agt/map_tracking/status'),
        DeclareLaunchArgument('legacy_status_topic', default_value='/agt/localization/status'),
        DeclareLaunchArgument('legacy_metrics_topic', default_value='/agt/localization/metrics'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('map_version', default_value=''),
        Node(
            package='agt_localization_ros',
            executable='shadow_manager',
            name='agt_localization_shadow_manager',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'local_odom_topic': LaunchConfiguration('local_odom_topic'),
                'global_pose_topic': LaunchConfiguration('global_pose_topic'),
                'tracking_pose_topic': LaunchConfiguration('tracking_pose_topic'),
                'tracking_status_topic': LaunchConfiguration('tracking_status_topic'),
                'legacy_status_topic': LaunchConfiguration('legacy_status_topic'),
                'legacy_metrics_topic': LaunchConfiguration('legacy_metrics_topic'),
                'map_id': LaunchConfiguration('map_id'),
                'map_version': LaunchConfiguration('map_version'),
            }],
        ),
    ])
