"""Start the field navigation stack and Qt HMI from the active workspace map."""

import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from agt_map_manager.runtime_binding import resolve_active_map


DEFAULT_ACTIVE_STATE = '/home/yangxuan/ros2_ws/agt_data/maps/active_map.yaml'
DEFAULT_MISSION_ROOT = '/home/yangxuan/ros2_ws/agt_data/missions'
DEFAULT_HMI_RUNTIME_ROOT = '/home/yangxuan/ros2_ws/agt_data/hmi_runtime'


def _write_hmi_runtime_config(
    runtime_dir: Path,
    navigation_map: Path,
    map_id: str,
    map_version: str,
    generation: int,
) -> None:
    """Give HMI a V3-owned runtime context and the exact active map to open."""
    runtime_dir.mkdir(parents=True, exist_ok=True)
    config_path = runtime_dir / 'config.json'
    data = {
        'channel_config': {
            'channel_type': 'auto',
            'rosbridge_config': {'ip': '127.0.0.1', 'port': '9090'},
        },
        'display_config': [],
        'images': [],
        # These values are generated from active_map.yaml by V3. They are not
        # HMI preferences and must never be inferred from a last-opened file.
        'key_value': {
            'agt_map_id': map_id,
            'agt_map_version': map_version,
            'agt_map_generation': str(generation),
            'agt_map_context_source': 'v3_active_map',
        },
        'map_config': {'path': str(navigation_map)},
        'robot_shape_config': {
            'color': '0x00000FF', 'is_ellipse': False,
            'opacity': 0.5, 'shaped_points': [],
        },
    }
    fd, tmp_name = tempfile.mkstemp(prefix='.config.', suffix='.tmp', dir=str(runtime_dir))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_path, config_path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def _start_from_active_map(context):
    state_path = Path(LaunchConfiguration('active_state_file').perform(context)).expanduser()
    try:
        binding = resolve_active_map(state_path)
    except ValueError as exc:
        raise RuntimeError(f'active map binding is invalid: {exc}') from exc
    package = binding.package
    navigation_map = Path(package.asset_path('navigation_map'))
    localization_map = Path(package.asset_path('localization_map'))
    relocalization_assets = package.asset_path('relocalization_assets')
    map_id = package.map_id
    map_version = package.map_version
    generation = binding.generation

    hmi_runtime_dir = Path(DEFAULT_HMI_RUNTIME_ROOT) / map_id / map_version
    _write_hmi_runtime_config(
        hmi_runtime_dir,
        navigation_map,
        map_id,
        map_version,
        generation,
    )

    system_share = Path(get_package_share_directory('agt_system_bringup'))
    field_demo = system_share / 'launch' / 'rviz_field_demo.launch.py'
    map_manager_launch = Path(get_package_share_directory('agt_map_manager')) / 'launch' / 'map_manager.launch.py'
    # Upstream agt_robot_hmi deliberately installs its Qt application in bin/
    # and provides start.sh to set Qt/LD_LIBRARY_PATH safely. It is therefore
    # not a ROS libexec target and must not be started through launch_ros.Node.
    hmi_start = Path(get_package_prefix('agt_robot_hmi')) / 'bin' / 'start.sh'
    if not hmi_start.is_file():
        raise RuntimeError(f'agt_robot_hmi launcher does not exist: {hmi_start}')
    return [
        # The HMI uses services for all package lifecycle changes. It never
        # needs a filesystem path to discover, validate, activate, or publish.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(map_manager_launch))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(field_demo)),
            launch_arguments={
                'map': str(navigation_map),
                'global_map': str(localization_map),
                'relocalization_assets': relocalization_assets,
                'map_id': f'{map_id}_{map_version}',
                # A mission is operational data, not a mutable file in an
                # immutable Map Package. Keep it version-scoped and auditable.
                'mission_dir': str(Path(DEFAULT_MISSION_ROOT) / map_id / map_version),
                'launch_rviz': 'false',
            'auto_relocalize': LaunchConfiguration('auto_relocalize').perform(context),
            'enable_rtk': LaunchConfiguration('enable_rtk').perform(context),
            'enable_map_tracking': LaunchConfiguration('enable_map_tracking').perform(context),
            }.items()),
        # The HMI subscribes to the Nav2 /map topic and publishes PoseStamped
        # goals to /goal_pose. agt_rviz_patrol, already included above, queues
        # those targets for the existing inspection mission action.
        ExecuteProcess(
            cmd=[str(hmi_start)], name='agt_robot_hmi', output='screen',
            cwd=str(hmi_runtime_dir),
            additional_env={'AGT_HMI_CONFIG_DIR': str(hmi_runtime_dir)}),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('active_state_file', default_value=DEFAULT_ACTIVE_STATE),
        DeclareLaunchArgument('auto_relocalize', default_value='true'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('enable_map_tracking', default_value='false'),
        OpaqueFunction(function=_start_from_active_map),
    ])
