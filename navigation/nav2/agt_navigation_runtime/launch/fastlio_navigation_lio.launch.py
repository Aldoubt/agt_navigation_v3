"""FAST-LIO2 local odometry for navigation, without mapping or PGO nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    adapter_share = Path(get_package_share_directory('agt_fastlio_adapter'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(runtime_share / 'config' / 'fastlio2_mid360_navigation.yaml')),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('body_to_base_calibration_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='fastlio2', executable='lio_node', namespace='fastlio2',
            name='lio_node', output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('fastlio_config'),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[
                ('/livox/lidar', LaunchConfiguration('lidar_topic')),
                ('/livox/imu', LaunchConfiguration('imu_topic')),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(adapter_share / 'launch' / 'adapter.launch.py')),
            launch_arguments={
                'config_file': str(adapter_share / 'config' / 'fastlio_navigation_adapter.yaml'),
                'body_to_base_calibration_file': LaunchConfiguration('body_to_base_calibration_file'),
            }.items(),
        ),
    ])
