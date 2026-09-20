"""Local odometry, global relocalization and the sole map->odom owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, arguments=None, condition=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def _validate_assets(context):
    global_map = LaunchConfiguration('global_map').perform(context)
    assets = LaunchConfiguration('relocalization_assets').perform(context)
    if not global_map or not Path(global_map).expanduser().is_file():
        raise RuntimeError(f'global_map must be an existing PCD file: {global_map!r}')
    if assets and not Path(assets).expanduser().is_dir():
        raise RuntimeError(f'relocalization_assets must be a directory: {assets!r}')
    return []


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    batch_config = LaunchConfiguration('batch_config')
    return LaunchDescription([
        DeclareLaunchArgument('global_map', description='Absolute localization PCD path'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml')),
        DeclareLaunchArgument('auto_relocalize', default_value='true'),
        DeclareLaunchArgument('enable_map_tracking', default_value='false'),
        OpaqueFunction(function=_validate_assets),

        _include('agt_navigation_runtime', 'navigation_lio.launch.py', {
            'use_sim_time': use_sim_time,
            'batch_config': batch_config,
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }),
        # Single PointCloud2 conversion branch shared by relocalization and
        # navigation perception. The timing-preserving LIO input is untouched.
        _include('agt_livox_tools', 'livox_format_bridge.launch.py'),
        _include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'global_map': LaunchConfiguration('global_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'batch_lio_config_file': batch_config,
            'follow_map_manager': 'false',
            'scan_topic': '/agt/livox/points',
            'auto_request': LaunchConfiguration('auto_relocalize'),
            'use_sim_time': use_sim_time,
        }),
        # This is the only inclusion of agt_localization_manager in the four
        # top-level launches and therefore the only map->odom publisher.
        _include('agt_localization_manager', 'localization_manager.launch.py', {
            'use_sim_time': use_sim_time,
        }),
        _include(
            'agt_map_tracker', 'map_tracker.launch.py', {
                'global_map': LaunchConfiguration('global_map'),
                'scan_topic': '/agt/livox/points',
                'use_sim_time': use_sim_time,
            },
            condition=IfCondition(LaunchConfiguration('enable_map_tracking'))),
    ])
