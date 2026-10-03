from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = str(Path(get_package_share_directory('agt_task_continuity')) / 'config' / 'task_continuity.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('research_enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='agt_task_continuity', executable='task_continuity', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'research_enabled': ParameterValue(LaunchConfiguration('research_enabled'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])])
