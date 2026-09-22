"""One selected local odometry frontend and the sole map->odom owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from agt_navigation_runtime.lio_config import freeze_fastlio_config, validate_backend
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


def _localization_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    global_map, assets = value('global_map'), value('relocalization_assets')
    if not global_map or not Path(global_map).expanduser().is_file():
        raise RuntimeError(f'global_map must be an existing PCD file: {global_map!r}')
    if assets and not Path(assets).expanduser().is_dir():
        raise RuntimeError(f'relocalization_assets must be a directory: {assets!r}')
    backend = validate_backend(value('lio_backend'))
    use_sim_time = value('use_sim_time')
    common = {'use_sim_time': use_sim_time,
              'lidar_topic': value('lidar_topic'), 'imu_topic': value('imu_topic')}
    if backend == 'batch_lio':
        calibration = str(Path(value('batch_config')).expanduser().resolve())
        if not Path(calibration).is_file():
            raise RuntimeError(f'Batch-LIO configuration does not exist: {calibration}')
        lio_launch = 'navigation_lio.launch.py'
        lio_args = dict(common, batch_config=calibration)
    else:
        # Share one frozen calibration with relocalization and the FAST adapter.
        calibration = freeze_fastlio_config(
            value('fastlio_config'), common['lidar_topic'], common['imu_topic'])
        lio_launch = 'fastlio_navigation_lio.launch.py'
        lio_args = dict(common, fastlio_config=calibration,
                        body_to_base_calibration_file=calibration)

    return [
        # Only one frontend launch is constructed. The other backend is not started.
        _include('agt_navigation_runtime', lio_launch, lio_args),
        # Raw CustomMsg remains untouched; this is a separate PointCloud2 branch.
        _include('agt_livox_tools', 'livox_format_bridge.launch.py'),
        _include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'global_map': global_map,
            'relocalization_assets': assets,
            'body_to_base_calibration_file': calibration,
            # Retained compatibility argument for the existing Batch mode.
            'batch_lio_config_file': calibration if backend == 'batch_lio' else '',
            'follow_map_manager': 'false',
            'scan_topic': '/agt/livox/points',
            'auto_request': value('auto_relocalize'),
            'use_sim_time': use_sim_time,
        }),
        _include('agt_localization_manager', 'localization_manager.launch.py', {
            'use_sim_time': use_sim_time,
        }),
        _include(
            'agt_map_tracker', 'map_tracker.launch.py', {
                'global_map': global_map,
                'scan_topic': '/agt/livox/points',
                'use_sim_time': use_sim_time,
            },
            condition=IfCondition(LaunchConfiguration('enable_map_tracking'))),
    ]


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    return LaunchDescription([
        DeclareLaunchArgument('global_map', description='Absolute localization PCD path'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('lio_backend', default_value='batch_lio',
                              choices=['batch_lio', 'fastlio2']),
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml')),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(runtime_share / 'config' / 'fastlio2_mid360_navigation.yaml')),
        DeclareLaunchArgument('auto_relocalize', default_value='true'),
        DeclareLaunchArgument('enable_map_tracking', default_value='false'),
        OpaqueFunction(function=_localization_nodes),
    ])
