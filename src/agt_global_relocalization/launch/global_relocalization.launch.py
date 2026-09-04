import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('agt_global_relocalization'), 'config', 'global_relocalization.yaml')
    return LaunchDescription([
        Node(package='agt_global_relocalization', executable='global_relocalization', name='agt_global_relocalization', output='screen', parameters=[cfg]),
    ])
