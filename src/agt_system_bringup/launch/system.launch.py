from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, condition, launch_arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    enable_map_manager = LaunchConfiguration('enable_map_manager')
    enable_rtk = LaunchConfiguration('enable_rtk')
    enable_fastlio_adapter = LaunchConfiguration('enable_fastlio_adapter')
    enable_batch_lio_adapter = LaunchConfiguration('enable_batch_lio_adapter')
    enable_global_relocalization = LaunchConfiguration('enable_global_relocalization')
    enable_localization_manager = LaunchConfiguration('enable_localization_manager')
    enable_obstacle_cloud = LaunchConfiguration('enable_obstacle_cloud')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_base_guard = LaunchConfiguration('enable_base_guard')
    enable_runtime = LaunchConfiguration('enable_runtime')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Absolute path to derived Nav2 map YAML; required when enable_nav2=true.'),
        DeclareLaunchArgument('enable_map_manager', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_fastlio_adapter', default_value='false'),
        DeclareLaunchArgument('enable_batch_lio_adapter', default_value='false'),
        DeclareLaunchArgument('enable_global_relocalization', default_value='false'),
        DeclareLaunchArgument('enable_localization_manager', default_value='false'),
        DeclareLaunchArgument('enable_obstacle_cloud', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='false'),
        DeclareLaunchArgument('enable_base_guard', default_value='true'),
        DeclareLaunchArgument('enable_runtime', default_value='false'),

        _include('agt_map_manager', 'map_manager.launch.py', IfCondition(enable_map_manager)),
        _include('agt_rtk_manager', 'rtk_manager.launch.py', IfCondition(enable_rtk)),
        _include('agt_fastlio_adapter', 'adapter.launch.py', IfCondition(enable_fastlio_adapter)),
        _include('agt_batch_lio_adapter', 'batch_lio_adapter.launch.py', IfCondition(enable_batch_lio_adapter)),
        _include('agt_global_relocalization', 'global_relocalization.launch.py', IfCondition(enable_global_relocalization)),
        _include('agt_localization_manager', 'localization_manager.launch.py', IfCondition(enable_localization_manager)),
        _include('agt_pointcloud_preprocessor', 'obstacle_cloud.launch.py', IfCondition(enable_obstacle_cloud)),
        _include('agt_base_control', 'cmd_vel_guard.launch.py', IfCondition(enable_base_guard)),
        _include('agt_nav2_bringup', 'navigation.launch.py', IfCondition(enable_nav2), {'map': map_yaml}),
        _include('agt_navigation_runtime', 'runtime.launch.py', IfCondition(enable_runtime)),
    ])
