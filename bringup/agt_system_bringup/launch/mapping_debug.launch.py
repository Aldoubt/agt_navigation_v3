"""Self-contained overwrite-only mapping runtime for field debugging.

This launch intentionally has no MapManager dependency at runtime.  It starts
the hardware session and the established FAST-LIO2 + PGO + OctoMap branch;
``save_mapping_debug.sh`` is responsible for replacing the fixed debug output.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


DEFAULT_DEBUG_ROOT = Path('/home/yangxuan/ros2_ws/agt_data/debug_mapping/current')


def _include(package: str, launch_file: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bunker_can_port', default_value='can0'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_camera_gimbal', default_value='true'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'gimbal_port_name',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument(
            'filter_statistics',
            default_value=str(Path.home() / '.ros' / 'agt_octomap' / 'rear_filter_statistics.yaml'),
            description='Debug-only OctoMap rear-filter statistics output.',
        ),

        # The debug runtime owns all hardware so it must not be run beside an
        # operator_console sensor session or another hardware bringup.
        _include('agt_system_bringup', 'sensor_session.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bunker_can_port': LaunchConfiguration('bunker_can_port'),
            'enable_rtk': LaunchConfiguration('enable_rtk'),
            'enable_camera_gimbal': LaunchConfiguration('enable_camera_gimbal'),
            'camera_device_path': LaunchConfiguration('camera_device_path'),
            'gimbal_port_name': LaunchConfiguration('gimbal_port_name'),
        }),
        _include('agt_mapping_bringup', 'mapping_mode.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_topic': '/livox/lidar',
            'imu_topic': '/livox/imu',
            'enable_pgo': 'true',
            'enable_octomap_navigation': 'true',
            'launch_rviz': 'true',
            'filter_statistics': LaunchConfiguration('filter_statistics'),
        }),
    ])
