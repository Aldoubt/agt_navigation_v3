from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_livox_tools'))
    default_params = str(share / 'config' / 'custom_to_pointcloud2.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'livox_bridge_params_file',
            default_value=default_params,
            description='Unique parameter-file argument to avoid nested launch collisions.',
        ),
        Node(
            package='agt_livox_tools',
            executable='livox_format_bridge',
            name='livox_format_bridge',
            output='screen',
            parameters=[LaunchConfiguration('livox_bridge_params_file')],
        ),
    ])
