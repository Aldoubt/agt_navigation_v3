from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, *, condition=None, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rtk = LaunchConfiguration('start_rtk')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_base_guard = LaunchConfiguration('start_base_guard')
    start_mission_runtime = LaunchConfiguration('start_mission_runtime')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Derived Nav2 map YAML. Required when start_nav2:=true.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rtk', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('start_base_guard', default_value='true'),
        DeclareLaunchArgument('start_mission_runtime', default_value='true'),

        # Hardware drivers and FAST-LIO2 are intentionally not started here yet.
        # They remain independent acceptance boundaries until target-machine
        # integration is stable.
        _include(
            'agt_rtk_manager', 'rtk_manager.launch.py',
            condition=IfCondition(start_rtk),
        ),
        _include(
            'agt_nav2_bringup', 'navigation.launch.py',
            condition=IfCondition(start_nav2),
            arguments={'map': map_yaml, 'use_sim_time': use_sim_time},
        ),
        _include(
            'agt_base_control', 'cmd_vel_guard.launch.py',
            condition=IfCondition(start_base_guard),
        ),
        _include(
            'agt_navigation_runtime', 'runtime.launch.py',
            condition=IfCondition(start_mission_runtime),
        ),
    ])
