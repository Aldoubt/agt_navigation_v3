"""Start the Gazebo and simulated-local-odometry portion of P3 acceptance.

This launch intentionally creates only the local navigation chain.  The
simulation odometry adapter owns ``odom -> base_link``.  It never publishes
``map -> odom``; an accepted external relocalization pose must reach
LocalizationManager in the full-stack launch before that edge exists.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    gazebo_share = Path(get_package_share_directory('agt_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=str(gazebo_share / 'worlds' / 'agt_mapping.world'),
            description='Gazebo Classic world used by the tracked-chassis acceptance model.'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        _include('agt_gazebo_sim', 'sim_world.launch.py', {
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }),
        _include('agt_sim_odometry_adapter', 'sim_odometry_adapter.launch.py', {
            'use_sim_time': 'true',
        }),
    ])
