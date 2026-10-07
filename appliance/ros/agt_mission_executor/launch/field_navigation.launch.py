"""Compose existing V3 Nav2/perception/guard; no replacement controller or TF."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav = Path(get_package_share_directory("agt_nav2_bringup")) / "launch/navigation.launch.py"
    params = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("params_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav)),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "nav2_params_file": params,
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="agt_pointcloud_preprocessor",
                executable="agt_pointcloud_preprocessor",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="agt_base_control",
                executable="cmd_vel_guard",
                parameters=[params],
                output="screen",
            ),
        ]
    )
