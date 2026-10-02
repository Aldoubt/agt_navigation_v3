"""Convenience composition of hardware, localization, and navigation layers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=arguments.items())


def generate_launch_description():
    names = {
        'robot_config': '', 'robot': '', 'use_sim_time': 'false',
        'global_map': '', 'map_id': '', 'map_version': '', 'relocalization_assets': '',
        'query_capture_dir': '', 'lio_backend': 'fastlio2', 'map': 'auto',
        'nav_config_dir': '', 'map_registry': '', 'launch_rviz': 'true',
        'motion_guard_backend': 'cpp', 'base_adapter': 'bunker',
    }
    declarations = [DeclareLaunchArgument(name, default_value=default)
                    for name, default in names.items()]
    hardware_args = {
        name: LaunchConfiguration(name)
        for name in ('robot_config', 'robot', 'use_sim_time')}
    localization_args = {
        name: LaunchConfiguration(name)
        for name in ('global_map', 'map_id', 'map_version', 'relocalization_assets',
                     'query_capture_dir', 'lio_backend', 'use_sim_time')}
    navigation_args = {
        name: LaunchConfiguration(name)
        for name in ('map', 'robot', 'robot_config', 'use_sim_time', 'nav_config_dir',
                     'map_registry', 'motion_guard_backend', 'base_adapter')}
    return LaunchDescription([
        *declarations,
        _include('agt_system_bringup', 'hardware.launch.py', hardware_args),
        _include('agt_system_bringup', 'localization.launch.py', localization_args),
        _include('agt_system_bringup', 'navigation.launch.py', navigation_args),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('agt_rviz_patrol'), 'launch', 'rviz.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
