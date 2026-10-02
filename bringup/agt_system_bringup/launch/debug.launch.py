"""Read-only visualization and preflight checks; starts no stack owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('agt_rviz_patrol'), 'launch', 'rviz.launch.py'])),
            launch_arguments={
                'config_file': LaunchConfiguration('rviz_config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_rviz'))),
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
