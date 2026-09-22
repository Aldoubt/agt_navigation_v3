from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('agt_fastlio_adapter'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share / 'config' / 'adapter.yaml')),
        DeclareLaunchArgument('body_to_base_calibration_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='agt_fastlio_adapter',
            executable='fastlio_adapter',
            name='agt_fastlio_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'body_to_base_calibration_file': LaunchConfiguration('body_to_base_calibration_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        )
    ])
