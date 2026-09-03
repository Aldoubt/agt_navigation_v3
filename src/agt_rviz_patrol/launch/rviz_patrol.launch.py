from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_preset = os.path.join(
        get_package_share_directory('agt_rviz_patrol'), 'config', 'front_sky_three_views.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_id', default_value='demo_map'),
        DeclareLaunchArgument('preset_file', default_value=default_preset),
        Node(
            package='agt_rviz_patrol', executable='rviz_patrol', output='screen',
            parameters=[{
                'map_id': LaunchConfiguration('map_id'),
                'preset_file': LaunchConfiguration('preset_file'),
            }]),
    ])
