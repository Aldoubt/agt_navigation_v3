"""Direct hardware navigation runtime for localization, Nav2, and camera capture debugging.

This launch deliberately accepts map asset paths as arguments.  It does not
resolve, validate, or activate an ``active_map.yaml`` through MapManager; use
the product ``hmi_field_demo.launch.py`` path for those lifecycle guarantees.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MAP_ROOT = Path('/home/yangxuan/ros2_ws/agt_data/maps')
DEFAULT_PACKAGE_ROOT = DEFAULT_MAP_ROOT / 'bunker_mid360_mapping_20260901_205036' / 'v003-indexed'


def _include(package: str, launch_file: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    debug_rviz = Path(get_package_share_directory('agt_demo_task')) / 'config' / 'navigation_debug.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(DEFAULT_PACKAGE_ROOT / 'navigation' / 'map.yaml')),
        DeclareLaunchArgument(
            'global_map', default_value=str(DEFAULT_PACKAGE_ROOT / 'localization' / 'global_map.pcd')),
        DeclareLaunchArgument(
            'relocalization_assets', default_value=str(DEFAULT_PACKAGE_ROOT / 'localization' / 'relocalization')),
        DeclareLaunchArgument('map_id', default_value='navigation_debug'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_relocalize', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_map_tracking', default_value='false'),
        DeclareLaunchArgument('bunker_can_port', default_value='can0'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'gimbal_port_name',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument('capture_output_dir', default_value='~/.ros/agt_navigation_debug/captures'),

        # The sensor session owns robot_description, MID360, CAN, RTK, and C1.
        _include('agt_system_bringup', 'sensor_session.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bunker_can_port': LaunchConfiguration('bunker_can_port'),
            'enable_rtk': LaunchConfiguration('enable_rtk'),
            # rviz_field_demo owns the one RTK manager used by this runtime.
            'enable_rtk_manager': 'false',
            'camera_device_path': LaunchConfiguration('camera_device_path'),
            'gimbal_port_name': LaunchConfiguration('gimbal_port_name'),
        }),
        # This composes the production localization/Nav2 nodes with explicit
        # assets, without the product MapManager active-map resolution path.
        _include('agt_system_bringup', 'rviz_field_demo.launch.py', {
            'map': LaunchConfiguration('map'),
            'global_map': LaunchConfiguration('global_map'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'map_id': LaunchConfiguration('map_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'auto_relocalize': LaunchConfiguration('auto_relocalize'),
            'enable_rtk': LaunchConfiguration('enable_rtk'),
            'enable_map_tracking': LaunchConfiguration('enable_map_tracking'),
            'launch_rviz': 'true',
            'rviz_config': str(debug_rviz),
        }),
        Node(
            package='agt_capability_camera',
            executable='camera_capture_service',
            name='camera_capture_service',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('capture_output_dir'),
            }],
        ),
        Node(
            package='agt_demo_task',
            executable='navigate_capture_task',
            name='navigate_capture_task',
            output='screen',
        ),
    ])
