"""Select exactly one navigation LIO backend and its navigation adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(context):
    backend = LaunchConfiguration('lio_backend').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    if backend == 'fastlio2':
        launch_file = 'fastlio_navigation_lio.launch.py'
        arguments = {
            'fastlio_config': LaunchConfiguration('fastlio_config').perform(context),
            'lidar_topic': lidar_topic,
            'imu_topic': imu_topic,
            'body_to_base_calibration_file':
                LaunchConfiguration('body_to_base_calibration_file').perform(context),
            'use_sim_time': use_sim_time,
        }
    elif backend == 'batch_lio':
        launch_file = 'navigation_lio.launch.py'
        arguments = {
            'batch_config': LaunchConfiguration('batch_config').perform(context),
            'lidar_topic': lidar_topic,
            'imu_topic': imu_topic,
            'launch_batch_rviz': 'false',
            'use_sim_time': use_sim_time,
        }
    else:
        raise RuntimeError(f'unsupported lio_backend {backend!r}; expected fastlio2|batch_lio')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(runtime_share / 'launch' / launch_file)),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    share = Path(get_package_share_directory('agt_navigation_runtime'))
    return LaunchDescription([
        DeclareLaunchArgument('lio_backend', default_value='fastlio2',
                              choices=['fastlio2', 'batch_lio']),
        DeclareLaunchArgument('fastlio_config', default_value=str(
            share / 'config' / 'fastlio2_mid360_navigation.yaml')),
        DeclareLaunchArgument('batch_config', default_value=str(
            share / 'config' / 'batch_lio_mid360.yaml')),
        DeclareLaunchArgument('body_to_base_calibration_file', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_include),
    ])
