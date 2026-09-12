"""Open the Qt HMI editor for one immutable Map Package without Nav2."""

import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agt_map_manager.map_package import discover_packages, validate_package


DEFAULT_MAP_ROOT = '/home/yangxuan/ros2_ws/agt_data/maps'
DEFAULT_HMI_RUNTIME_ROOT = '/home/yangxuan/ros2_ws/agt_data/hmi_runtime/editor'


def _write_runtime_config(runtime_dir: Path, navigation_map: str, map_id: str, map_version: str) -> None:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    config_path = runtime_dir / 'config.json'
    data = {
        'channel_config': {'channel_type': 'auto', 'rosbridge_config': {'ip': '127.0.0.1', 'port': '9090'}},
        'display_config': [], 'images': [],
        'key_value': {
            'agt_map_id': map_id,
            'agt_map_version': map_version,
            'agt_map_context_source': 'v3_map_editor',
        },
        'map_config': {'path': navigation_map},
        'robot_shape_config': {'color': '0x00000FF', 'is_ellipse': False, 'opacity': 0.5, 'shaped_points': []},
    }
    fd, temp_name = tempfile.mkstemp(prefix='.config.', suffix='.tmp', dir=str(runtime_dir))
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, config_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def _start_editor(context):
    root = Path(LaunchConfiguration('map_root').perform(context)).expanduser().resolve()
    map_id = LaunchConfiguration('map_id').perform(context).strip()
    map_version = LaunchConfiguration('map_version').perform(context).strip()
    if not map_id or not map_version:
        raise RuntimeError('map_id and map_version are required for the HMI editor')
    matches = [
        package for package in discover_packages(root, verify_hashes=False)
        if package.map_id == map_id and package.map_version == map_version
    ]
    if len(matches) != 1:
        raise RuntimeError(f'exact map package not found: {map_id}/{map_version}')
    package = validate_package(matches[0].metadata_path, verify_hashes=True)
    if not package.valid:
        raise RuntimeError(f'map package is invalid: {package.reason}')

    runtime_dir = Path(DEFAULT_HMI_RUNTIME_ROOT) / map_id / map_version
    _write_runtime_config(runtime_dir, package.asset_path('navigation_map'), map_id, map_version)
    hmi_start = Path(get_package_prefix('agt_robot_hmi')) / 'bin' / 'start.sh'
    if not hmi_start.is_file():
        raise RuntimeError(f'agt_robot_hmi launcher does not exist: {hmi_start}')
    manager_launch = Path(get_package_share_directory('agt_map_manager')) / 'launch' / 'map_manager.launch.py'
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(manager_launch))),
        # Editing is deliberately usable without Nav2/localization. The static
        # identity frame gives the Qt scene a valid map frame without claiming
        # that this is a live robot pose.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='agt_hmi_editor_map_frame', output='screen',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']),
        ExecuteProcess(cmd=[str(hmi_start)], name='agt_robot_hmi_editor', output='screen',
                       cwd=str(runtime_dir), additional_env={'AGT_HMI_CONFIG_DIR': str(runtime_dir)}),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_id'),
        DeclareLaunchArgument('map_version'),
        DeclareLaunchArgument('map_root', default_value=DEFAULT_MAP_ROOT),
        OpaqueFunction(function=_start_editor),
    ])
