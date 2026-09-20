"""Physical devices and the sole robot_state_publisher owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, enabled, arguments=None, launch_dir='launch'):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / launch_dir / launch_file)),
        condition=IfCondition(enabled),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    description_share = Path(get_package_share_directory('tracked_chassis_description'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_robot_description', default_value='true'),
        DeclareLaunchArgument(
            'robot_description_calibration_file',
            default_value=str(description_share / 'config' / 'field_acceptance.yaml')),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('enable_bunker_can', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_camera_gimbal', default_value='true'),
        DeclareLaunchArgument('bunker_can_port', default_value='can0'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'gimbal_port_name',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),

        # No other top-level launch includes tracked_chassis_description.
        _include(
            'tracked_chassis_description', 'display.launch.py',
            LaunchConfiguration('enable_robot_description'), {
                'calibration_file': LaunchConfiguration(
                    'robot_description_calibration_file'),
                'start_rviz': 'false',
            }),
        _include(
            'livox_ros_driver2', 'msg_MID360_launch.py',
            LaunchConfiguration('enable_mid360'), launch_dir='launch_ROS2'),
        _include(
            'bunker_base', 'bunker_base.launch.py',
            LaunchConfiguration('enable_bunker_can'), {
                'use_sim_time': use_sim_time,
                'port_name': LaunchConfiguration('bunker_can_port'),
                'odom_topic_name': '/wheel/odom',
                'publish_odom_tf': 'false',
            }),
        _include(
            'agt_asensing_driver', 'asensing.launch.py',
            LaunchConfiguration('enable_rtk')),
        _include(
            'autolabor_c1_bringup', 'autolabor_c1.launch.py',
            LaunchConfiguration('enable_camera_gimbal'), {
                'device_path': LaunchConfiguration('camera_device_path'),
                'port_name': LaunchConfiguration('gimbal_port_name'),
                'gui': 'false',
            }),
    ])
