from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('agt_rtk_manager'))
    params = str(share / 'config' / 'rtk_manager.yaml')
    return LaunchDescription([
        Node(
            package='agt_rtk_manager',
            executable='rtk_manager',
            name='agt_rtk_manager',
            output='screen',
            parameters=[params],
        )
    ])
