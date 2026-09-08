"""Start the field navigation stack and Qt HMI from the active workspace map."""

import json
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    if not state_path.is_file():
        raise RuntimeError(f'active map state does not exist: {state_path}')
    try:
        state = yaml.safe_load(state_path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RuntimeError(f'cannot read active map state {state_path}: {exc}') from exc

    navigation_map = Path(str(state.get('navigation_map_yaml', ''))).expanduser()
    localization_map = Path(str(state.get('localization_map_pcd', ''))).expanduser()
    relocalization_assets = str(state.get('relocalization_assets_path', '')).strip()
    map_id = str(state.get('map_id', '')).strip()
    map_version = str(state.get('map_version', '')).strip()
    try:
        generation = max(0, int(state.get('generation', 0)))
    except (TypeError, ValueError) as exc:
        raise RuntimeError('active map state has invalid generation') from exc
    if not navigation_map.is_file() or not localization_map.is_file() or not map_id or not map_version:
        raise RuntimeError('active map state is incomplete or points to missing navigation/localization assets')
    if relocalization_assets and not Path(relocalization_assets).is_dir():
        raise RuntimeError(f'active relocalization assets do not exist: {relocalization_assets}')

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
    # Upstream agt_robot_hmi deliberately installs its Qt application in bin/
    # and provides start.sh to set Qt/LD_LIBRARY_PATH safely. It is therefore
    # not a ROS libexec target and must not be started through launch_ros.Node.
    hmi_start = Path(get_package_prefix('agt_robot_hmi')) / 'bin' / 'start.sh'
    if not hmi_start.is_file():
        raise RuntimeError(f'agt_robot_hmi launcher does not exist: {hmi_start}')
    return [
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
        OpaqueFunction(function=_start_from_active_map),
    ])
