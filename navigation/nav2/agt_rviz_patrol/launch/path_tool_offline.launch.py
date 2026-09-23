"""Offline RViz preview for the point-and-draw FollowPath operator flow."""

from pathlib import Path
from agt_map_manager.map_catalog import resolve_map

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _launch_preview(context):
    package_share = Path(get_package_share_directory('agt_rviz_patrol'))
    rviz_config = package_share / 'config' / 'agt_rviz_path_preview.rviz'

    selected = resolve_map(LaunchConfiguration('map').perform(context),
                           LaunchConfiguration('robot').perform(context),
                           Path(LaunchConfiguration('map_registry').perform(context)))
    map_yaml = str(selected.navigation_map)
    start_rviz = LaunchConfiguration('start_rviz')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    return [
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{'yaml_filename': map_yaml}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_path_tool_preview', output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='path_tool_preview_robot_tf', output='screen',
            arguments=[
                '--x', robot_x, '--y', robot_y, '--z', '0.0',
                '--yaw', robot_yaw, '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'map', '--child-frame-id', 'base_link',
            ]),
        Node(
            package='agt_rviz_patrol', executable='rviz_path_tool',
            name='agt_rviz_path_tool', output='screen',
            parameters=[{'preview_only': True}]),
        Node(
            package='rviz2', executable='rviz2', name='agt_path_tool_preview_rviz',
            output='screen', arguments=['-d', str(rviz_config)],
            condition=IfCondition(start_rviz)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='auto'),
        DeclareLaunchArgument('robot', default_value='bunker_v1'),
        DeclareLaunchArgument('map_registry',
                              default_value=EnvironmentVariable('AGT_MAP_REGISTRY', default_value='')),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_preview),
    ])
