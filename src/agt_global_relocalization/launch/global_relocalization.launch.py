import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('agt_global_relocalization'),
        'config', 'global_relocalization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'global_map', default_value='',
            description='Fallback global PCD path when Map Manager is not supplying an active map.'),
        DeclareLaunchArgument(
            'relocalization_assets', default_value='',
            description='Optional prebuilt 3D-BBS assets directory.'),
        DeclareLaunchArgument(
            'follow_map_manager', default_value='true',
            description='Follow /agt/map/status for active PCD and BBS assets.'),
        DeclareLaunchArgument('sdk_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('local_map_radius_xy', default_value='35.0'),
        DeclareLaunchArgument('local_map_half_height', default_value='8.0'),

        Node(
            package='agt_global_relocalization',
            executable='global_relocalization',
            name='agt_global_relocalization',
            output='screen',
            parameters=[
                cfg,
                {
                    'global_map': LaunchConfiguration('global_map'),
                    'relocalization_assets': LaunchConfiguration('relocalization_assets'),
                    'follow_map_manager': LaunchConfiguration('follow_map_manager'),
                    'sdk_timeout_sec': LaunchConfiguration('sdk_timeout_sec'),
                    'backend_local_map_radius_xy': LaunchConfiguration('local_map_radius_xy'),
                    'backend_local_map_half_height': LaunchConfiguration('local_map_half_height'),
                },
            ],
        ),
    ])
