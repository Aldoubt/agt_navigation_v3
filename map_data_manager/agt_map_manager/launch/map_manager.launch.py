from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_map_manager'))
    map_root = Path(os.environ.get('AGT_MAP_ROOT', '/home/yangxuan/ros2_ws/maps')).expanduser()
    state_file = map_root / 'active_map.yaml'
    registry_file = map_root / 'map_registry.yaml'
    edit_root = map_root.parent / 'map_edits'
    return LaunchDescription([
        Node(
            package='agt_map_manager',
            executable='map_manager',
            name='agt_map_manager',
            output='screen',
            parameters=[str(share / 'config' / 'map_manager.yaml'), {
                'map_root': str(map_root),
                'active_state_file': str(state_file),
                'registry_file': str(registry_file),
                'edit_root': str(edit_root),
                'verify_hashes_on_load': True,
            }],
        )
    ])
