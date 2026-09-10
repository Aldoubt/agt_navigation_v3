import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('agt_global_relocalization'),
        'config', 'global_relocalization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_request', default_value='false'),
        DeclareLaunchArgument(
            'global_map', default_value='',
            description='Fallback global PCD path when Map Manager is not supplying an active map.'),
        DeclareLaunchArgument(
            'relocalization_assets', default_value='',
            description='Optional prebuilt 3D-BBS assets directory.'),
        DeclareLaunchArgument(
            'follow_map_manager', default_value='true',
            description='Follow /agt/map/status for active PCD and BBS assets.'),
        DeclareLaunchArgument('sdk_timeout_sec', default_value='18.0'),
        DeclareLaunchArgument('local_map_radius_xy', default_value='35.0'),
        DeclareLaunchArgument('local_map_half_height', default_value='8.0'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/agt/livox/points',
            description='Live PointCloud2 query stream; offline tests may override this.'),

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
                    'follow_map_manager': ParameterValue(
                        LaunchConfiguration('follow_map_manager'), value_type=bool),
                    'sdk_timeout_sec': ParameterValue(
                        LaunchConfiguration('sdk_timeout_sec'), value_type=float),
                    'backend_local_map_radius_xy': ParameterValue(
                        LaunchConfiguration('local_map_radius_xy'), value_type=float),
                    'backend_local_map_half_height': ParameterValue(
                        LaunchConfiguration('local_map_half_height'), value_type=float),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                    'auto_request': ParameterValue(LaunchConfiguration('auto_request'), value_type=bool),
                },
            ],
        ),
    ])
