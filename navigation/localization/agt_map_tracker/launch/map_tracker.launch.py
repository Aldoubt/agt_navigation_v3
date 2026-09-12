from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_map_tracker'))
    return LaunchDescription([
        DeclareLaunchArgument('global_map', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value='/agt/livox/points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('apply_correction', default_value='true'),
        DeclareLaunchArgument('save_debug_cloud', default_value='false'),
        Node(
            package='agt_map_tracker', executable='map_tracker',
            name='agt_map_tracker', output='screen',
            parameters=[str(share / 'config' / 'map_tracker.yaml'), {
                'global_map': LaunchConfiguration('global_map'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'apply_correction': LaunchConfiguration('apply_correction'),
                'save_debug_cloud': LaunchConfiguration('save_debug_cloud'),
            }]),
    ])
