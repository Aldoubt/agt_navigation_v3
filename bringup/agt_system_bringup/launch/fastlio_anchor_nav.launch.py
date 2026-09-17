"""Field navigation: FAST-LIO2 local odometry plus one-shot 3D-BBS anchor.

This mode intentionally starts no mapping, PGO, or map-tracking process.
FAST-LIO2 provides continuous local motion; the global 3D-BBS/GICP result is
accepted once at startup to establish map->odom.  A later correction is only
possible through an explicit manual recovery request, never from a low-rate
map tracker.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def _validate_assets(context):
    checks = {
        'map': LaunchConfiguration('map').perform(context),
        'global_map': LaunchConfiguration('global_map').perform(context),
        'relocalization_assets': LaunchConfiguration('relocalization_assets').perform(context),
    }
    for label, value in checks.items():
        path = Path(os.path.expanduser(value))
        if not value or not (path.is_dir() if label == 'relocalization_assets' else path.is_file()):
            raise RuntimeError(f'fastlio_anchor_nav: required {label} does not exist: {value!r}')
    return []


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    localization_share = Path(get_package_share_directory('agt_localization_manager'))
    nav_share = Path(get_package_share_directory('agt_nav2_bringup'))
    rviz_share = Path(get_package_share_directory('agt_rviz_patrol'))

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to Nav2 map.yaml.'),
        DeclareLaunchArgument('global_map', description='Matching formal global_map.pcd.'),
        DeclareLaunchArgument('relocalization_assets', description='Matching 3D-BBS/Polar assets directory.'),
        DeclareLaunchArgument('map_id', default_value='fastlio_anchor_nav'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument(
            'body_to_base_calibration_file',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml'),
            description='Frozen body/lidar calibration only; this mode does not start Batch-LIO.'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=str(nav_share / 'config' / 'nav2_params_rpp.yaml'),
            description='Fixed RPP/Nav2 field baseline.'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=_validate_assets),

        # Continuous local odometry. This is the only LIO launched here.
        _include('agt_navigation_runtime', 'fastlio_navigation_lio.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'body_to_base_calibration_file': LaunchConfiguration('body_to_base_calibration_file'),
        }),

        # Keep the raw timing-preserving CustomMsg path for FAST-LIO2. This
        # separate PointCloud2 branch is only for 3D-BBS/GICP and obstacles.
        _include('agt_livox_tools', 'livox_format_bridge.launch.py'),
        _include('agt_pointcloud_preprocessor', 'obstacle_cloud.launch.py'),

        # auto_request is one-shot after successful startup localization. No
        # agt_map_tracker is launched, and manager tracker inputs are isolated.
        _include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'global_map': LaunchConfiguration('global_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'batch_lio_config_file': LaunchConfiguration('body_to_base_calibration_file'),
            'follow_map_manager': 'false',
            'scan_topic': '/agt/livox/points',
            'auto_request': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),
        Node(
            package='agt_localization_manager', executable='localization_manager',
            name='agt_localization_manager', output='screen',
            parameters=[
                str(localization_share / 'config' / 'localization_manager.yaml'),
                {
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                    # Explicitly disconnect low-rate map-tracking correction.
                    'tracking_pose_topic': '/agt/fastlio_anchor/disabled_tracking_pose',
                    'tracking_status_topic': '/agt/fastlio_anchor/disabled_tracking_status',
                    'tracking_recovery_auto_request': False,
                    'correction_smoothing_enabled': False,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('agt_rtk_manager')) / 'launch' / 'rtk_manager.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_rtk')),
        ),
        _include('agt_nav2_bringup', 'navigation.launch.py', {
            'map': LaunchConfiguration('map'),
            'controller_profile': 'rpp',
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),
        _include('agt_base_control', 'cmd_vel_guard.launch.py'),
        _include('agt_navigation_runtime', 'runtime.launch.py'),
        _include('agt_rviz_patrol', 'rviz_patrol.launch.py', {
            'map_id': LaunchConfiguration('map_id'),
        }),
        Node(
            package='rviz2', executable='rviz2', name='agt_fastlio_anchor_rviz',
            output='screen', arguments=['-d', str(rviz_share / 'config' / 'agt_rviz_demo.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
