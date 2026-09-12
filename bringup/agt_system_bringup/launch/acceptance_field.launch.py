"""Field acceptance entry point.

Explicit assets + manual-seed GICP + precision Nav2 profile + acceptance mission.
Nothing moves until /agt/acceptance/start is called.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, launch_file: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    acceptance_params = (
        Path(get_package_share_directory('agt_nav2_bringup'))
        / 'config' / 'nav2_acceptance_params.yaml'
    )
    waypoint_template = (
        Path(get_package_share_directory('agt_demo_task'))
        / 'config' / 'acceptance_waypoints.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('navigation_map', description='Acceptance Nav2 map.yaml'),
        DeclareLaunchArgument('localization_map', description='Matching cleaned localization PCD'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('waypoint_file', default_value=str(waypoint_template)),
        DeclareLaunchArgument('nav2_params_file', default_value=str(acceptance_params)),
        DeclareLaunchArgument('map_id', default_value='field_acceptance_v1'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_map_tracking', default_value='false'),
        DeclareLaunchArgument('bunker_can_port', default_value='can0'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'gimbal_port_name',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument(
            'capture_output_dir', default_value='~/.ros/agt_acceptance/captures'),
        DeclareLaunchArgument(
            'mission_output_dir', default_value='~/.ros/agt_acceptance/runs'),

        _include('agt_system_bringup', 'navigation_debug.launch.py', {
            'navigation_map': LaunchConfiguration('navigation_map'),
            'localization_map': LaunchConfiguration('localization_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'map_id': LaunchConfiguration('map_id'),
            'use_sim_time': 'false',
            'auto_relocalize': 'false',
            'relocalization_executable': 'manual_seed_relocalization',
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'enable_rtk': LaunchConfiguration('enable_rtk'),
            'enable_map_tracking': LaunchConfiguration('enable_map_tracking'),
            'bunker_can_port': LaunchConfiguration('bunker_can_port'),
            'camera_device_path': LaunchConfiguration('camera_device_path'),
            'gimbal_port_name': LaunchConfiguration('gimbal_port_name'),
            'capture_output_dir': LaunchConfiguration('capture_output_dir'),
            'enable_camera': 'true',
            'enable_demo_task': 'false',
        }),

        Node(
            package='agt_demo_task',
            executable='acceptance_patrol_task',
            name='acceptance_patrol_task',
            output='screen',
            parameters=[{
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'output_dir': LaunchConfiguration('mission_output_dir'),
                'capture_required': True,
                'linear_stop_threshold_mps': 0.03,
                'angular_stop_threshold_radps': 0.03,
                'stop_hold_sec': 1.0,
                'use_sim_time': False,
            }],
        ),
    ])
