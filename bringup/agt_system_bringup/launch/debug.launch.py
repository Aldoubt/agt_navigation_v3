"""Read-only visualization and preflight checks; starts no stack owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('run_preflight', default_value='true'),
        DeclareLaunchArgument(
            'require_camera', default_value='false',
            description='Require the C1 acquire-view action during preflight.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(nav2_share / 'rviz' / 'nav2_default_view.rviz')),
        Node(
            package='rviz2', executable='rviz2', name='agt_navigation_debug_rviz',
            output='screen', arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(
            package='agt_navigation_runtime', executable='demo_preflight',
            name='agt_demo_preflight', output='screen',
            condition=IfCondition(LaunchConfiguration('run_preflight')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'require_camera': ParameterValue(
                    LaunchConfiguration('require_camera'), value_type=bool),
            }]),
    ])
