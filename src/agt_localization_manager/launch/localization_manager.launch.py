from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('agt_localization_manager'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('debug_identity_map_odom', default_value='false'),
        Node(
            package='agt_localization_manager',
            executable='localization_manager',
            name='agt_localization_manager',
            output='screen',
            parameters=[str(share / 'config' / 'localization_manager.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'debug_identity_map_odom': LaunchConfiguration('debug_identity_map_odom')}],
        )
    ])
