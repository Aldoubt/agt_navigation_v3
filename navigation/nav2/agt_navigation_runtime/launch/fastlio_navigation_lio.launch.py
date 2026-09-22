"""FAST-LIO2 navigation odometry only; no PGO, map saving, or hardware nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from agt_navigation_runtime.lio_config import freeze_fastlio_config, require_same_calibration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _lio_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    runtime_config = freeze_fastlio_config(
        value('fastlio_config'), value('lidar_topic'), value('imu_topic'))
    calibration = value('body_to_base_calibration_file').strip() or runtime_config
    require_same_calibration(runtime_config, calibration)
    adapter_share = Path(get_package_share_directory('agt_fastlio_adapter'))
    use_sim_time = value('use_sim_time')
    return [
        Node(
            package='fastlio2', executable='lio_node', namespace='fastlio2',
            name='lio_node', output='screen',
            parameters=[{
                'config_path': runtime_config,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(adapter_share / 'launch' / 'adapter.launch.py')),
            launch_arguments={
                'config_file': str(adapter_share / 'config' / 'fastlio_navigation_adapter.yaml'),
                'body_to_base_calibration_file': calibration,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ]


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(runtime_share / 'config' / 'fastlio2_mid360_navigation.yaml')),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('body_to_base_calibration_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_lio_nodes),
    ])
