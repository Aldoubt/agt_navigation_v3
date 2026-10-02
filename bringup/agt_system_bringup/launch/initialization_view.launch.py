"""Map-only initialization aid. No planner, controller or TF fallback is started."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_system_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Resolved, validated navigation YAML'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='nav2_map_server', executable='map_server',
             name='agt_initialization_map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'frame_id': 'map'}],
             remappings=[('map', '/agt/initialization/map')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='agt_initialization_map_lifecycle', output='screen',
             parameters=[{'autostart': True, 'node_names': ['agt_initialization_map_server']}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('agt_rviz_patrol'), 'launch', 'rviz.launch.py'])),
            launch_arguments={
                'config_file': str(share / 'config/initialization.rviz'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
