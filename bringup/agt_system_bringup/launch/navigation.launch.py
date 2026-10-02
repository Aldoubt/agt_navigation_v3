"""Nav2, the sole obstacle-cloud node and the guarded command chain."""

from pathlib import Path
from copy import deepcopy
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from agt_map_manager.map_catalog import resolve_map


CONFIG_FILES = (
    'robot.yaml', 'navigation.yaml', 'controller.yaml',
    'costmap.yaml', 'perception.yaml', 'safety.yaml',
)


def select_navigation_config(robot_profile, explicit_dir, default_dir):
    """Keep Bunker defaults, but never silently load them for the YHS chassis.

    The marker records a human field review; its presence is not itself a
    measurement, safety certification, or permission to release a blocked robot.
    """
    default = Path(default_dir).resolve()
    if robot_profile != 'yhs_v1':
        if explicit_dir:
            raise RuntimeError('nav_config_dir override is reserved for measured yhs_v1 configuration')
        return default
    if not explicit_dir:
        raise RuntimeError('YHS navigation requires explicit nav_config_dir with measured YHS footprint, limits and obstacles')
    selected = Path(explicit_dir).expanduser().resolve()
    if selected == default or not selected.is_dir():
        raise RuntimeError('YHS navigation may not use the canonical Bunker config directory')
    marker_path = selected / 'field_profile.yaml'
    try:
        marker = yaml.safe_load(marker_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'YHS navigation field review marker missing/invalid: {marker_path}') from exc
    if (not isinstance(marker, dict) or marker.get('robot_profile') != 'yhs_v1' or
            marker.get('field_verified') is not True or not marker.get('verified_by')):
        raise RuntimeError('YHS navigation config requires field_profile.yaml: '
                           'robot_profile: yhs_v1, field_verified: true, verified_by: <reviewer>')
    missing = [name for name in CONFIG_FILES if not (selected / name).is_file()]
    if missing:
        raise RuntimeError(f'YHS navigation config missing required YAML files: {missing}')
    return selected


def _deep_merge(target, source):
    for key, value in source.items():
        if key in target and isinstance(target[key], dict) and isinstance(value, dict):
            _deep_merge(target[key], value)
        else:
            target[key] = value


def _params(tree, *node_path):
    node = tree
    for key in node_path:
        node = node[key]
    return node['ros__parameters']


def _build_runtime_params(config_dir, local_obstacle_avoidance=True):
    merged = {}
    for name in CONFIG_FILES:
        with (config_dir / name).open(encoding='utf-8') as stream:
            _deep_merge(merged, yaml.safe_load(stream) or {})

    robot = _params(merged, 'agt_robot_config')
    # The global costmap is static-only. Recomputing the same route every
    # second made the grid planner alternate its near-robot segment and drove
    # the tracked chassis into a left/right pursuit cycle. Retain the current
    # path until the goal changes or Nav2 proves that path invalid.
    bt_share = Path(get_package_share_directory('nav2_bt_navigator'))
    _params(merged, 'bt_navigator')['default_nav_to_pose_bt_xml'] = str(
        bt_share / 'behavior_trees' /
        'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')
    for costmap in ('local_costmap', 'global_costmap'):
        params = _params(merged, costmap, costmap)
        params['footprint'] = robot['footprint']
        params['footprint_padding'] = robot['footprint_padding']

    if not local_obstacle_avoidance:
        # Trial mode: ignore live LiDAR obstacles in the controller costmap,
        # while retaining mapped walls and footprint inflation.
        local = _params(merged, 'local_costmap', 'local_costmap')
        global_ = _params(merged, 'global_costmap', 'global_costmap')
        local['plugins'] = ['static_layer', 'inflation_layer']
        local['static_layer'] = deepcopy(global_['static_layer'])
        local.pop('voxel_layer', None)

    limits = _params(merged, 'agt_motion_limits')
    controller = _params(merged, 'controller_server')
    controller.update({
        'min_x_velocity_threshold': 0.01,
        'min_y_velocity_threshold': 0.0,
        'min_theta_velocity_threshold': 0.01,
    })
    follow = controller['FollowPath']
    follow.update({
        'desired_linear_vel': limits['controller_cruise_mps'],
        'min_approach_linear_velocity': limits['controller_approach_mps'],
        'regulated_linear_scaling_min_speed': limits['controller_regulated_min_mps'],
        'rotate_to_heading_angular_vel': limits['rotate_to_heading_radps'],
        'max_angular_accel': limits['controller_bootstrap_angular_accel'],
    })

    behavior = _params(merged, 'behavior_server')
    behavior.update({
        'max_rotational_vel': limits['rotate_to_heading_radps'],
        'min_rotational_vel': limits['controller_regulated_min_mps'],
        'rotational_acc_lim': limits['angular_accel_radps2'],
    })
    smoother = _params(merged, 'velocity_smoother')
    smoother.update({
        'max_velocity': [limits['forward_mps'], 0.0, limits['angular_radps']],
        'min_velocity': [-limits['reverse_mps'], 0.0, -limits['angular_radps']],
        'max_accel': [limits['linear_accel_mps2'], 0.0, limits['angular_accel_radps2']],
        'max_decel': [-limits['linear_decel_mps2'], 0.0, -limits['angular_decel_radps2']],
    })
    guard = _params(merged, 'agt_cmd_vel_guard')
    guard.update({
        'max_linear_x': limits['forward_mps'],
        'max_reverse_x': limits['reverse_mps'],
        'max_angular_z': limits['angular_radps'],
        'max_linear_accel': limits['linear_accel_mps2'],
        'max_linear_decel': limits['linear_decel_mps2'],
        'max_angular_accel': limits['angular_accel_radps2'],
    })

    output = Path(tempfile.mkdtemp(prefix='agt_navigation_params_')) / 'runtime.yaml'
    output.write_text(yaml.safe_dump(merged, sort_keys=False), encoding='utf-8')
    return str(output)


def resolve_payload_interlock(mode, robot_config_spec, robot_profile):
    """Return True when chassis motion must wait for /agt/payload/drive_permission.

    'auto' follows the whole-robot config (a physically installed arm => True);
    'true' forces it on; 'false' is refused when the robot config requires it.
    """
    mode = (mode or 'auto').strip().lower()
    # Always resolved: an empty robot_config maps the legacy robot profile to
    # its single supported robot config (bunker_v1 -> bunker_inspection) and
    # fails for profiles without one, so no robot silently runs without it.
    import sys
    bringup = Path(get_package_share_directory('agt_robot_bringup'))
    sys.path.insert(0, str(bringup / 'tools'))
    import robot_config  # read-only config resolution; starts nothing
    required = robot_config.payload_interlock_required(robot_config_spec, robot_profile)
    if mode == 'auto':
        return required
    if mode == 'true':
        return True
    if mode == 'false':
        if required:
            raise RuntimeError(
                f'payload_interlock:=false conflicts with robot_config {robot_config_spec!r} '
                '(arm installed): the chassis interlock cannot be disabled for this robot')
        return False
    raise RuntimeError(f'payload_interlock must be auto|true|false, got {mode!r}')


def motion_guard_node_spec(backend):
    """Select exactly one guard process. ``python`` is the field rollback path."""
    selected = (backend or 'cpp').strip().lower()
    if selected == 'cpp':
        return 'agt_base_runtime', 'motion_guard'
    if selected == 'python':
        return 'agt_base_control', 'cmd_vel_guard'
    raise RuntimeError(f'motion_guard_backend must be cpp|python, got {backend!r}')


def base_adapter_node_spec(adapter):
    """Resolve the currently implemented hardware boundary, failing closed otherwise."""
    selected = (adapter or '').strip().lower()
    if selected == 'bunker':
        return 'agt_base_runtime', 'bunker_adapter'
    if selected == 'ackermann':
        raise RuntimeError('Ackermann conversion is software-only; no real driver adapter is enabled')
    if selected in ('yhs', 'yhs_tk_mid'):
        raise RuntimeError('YHS base adapter is BLOCKED pending vehicle protocol/kinematics audit')
    raise RuntimeError(f'unsupported base adapter {adapter!r}; expected bunker')


def _validate_base_adapter(adapter):
    base_adapter_node_spec(adapter)


def _include(package, launch_file, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def _launch_runtime(context):
    robot_profile = LaunchConfiguration('robot').perform(context)
    payload_interlock = resolve_payload_interlock(
        LaunchConfiguration('payload_interlock').perform(context),
        LaunchConfiguration('robot_config').perform(context), robot_profile)
    base_adapter = LaunchConfiguration('base_adapter').perform(context)
    if base_adapter == 'bunker' and robot_profile != 'bunker_v1':
        raise RuntimeError(
            f'Bunker adapter cannot serve robot profile {robot_profile!r}; refusing mismatched base')
    map_spec = LaunchConfiguration('map').perform(context)
    try:
        selected_map = resolve_map(map_spec, robot_profile, Path(
            LaunchConfiguration('map_registry').perform(context)))
    except ValueError as exc:
        raise RuntimeError(f'MAP_ERROR: {exc}') from exc
    map_path = selected_map.navigation_map

    share = Path(get_package_share_directory('agt_system_bringup'))
    local_obstacle_avoidance = (
        LaunchConfiguration('local_obstacle_avoidance').perform(context).lower() == 'true')
    config_dir = select_navigation_config(
        robot_profile, LaunchConfiguration('nav_config_dir').perform(context), share / 'config')
    params_file = _build_runtime_params(
        config_dir, local_obstacle_avoidance=local_obstacle_avoidance)
    motion_guard_backend = LaunchConfiguration('motion_guard_backend').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    motion_guard_node_spec(motion_guard_backend)
    _validate_base_adapter(base_adapter)
    return [
        _include('agt_pointcloud_preprocessor', 'local_perception.launch.py', {
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'rear_filter_enabled': LaunchConfiguration(
                'obstacle_rear_filter_enabled').perform(context),
            'rear_filter_center_deg': LaunchConfiguration(
                'obstacle_rear_filter_center_deg').perform(context),
            'rear_filter_width_deg': LaunchConfiguration(
                'obstacle_rear_filter_width_deg').perform(context),
            'rear_filter_min_range_m': LaunchConfiguration(
                'obstacle_rear_filter_min_range_m').perform(context),
            'rear_filter_max_range_m': LaunchConfiguration(
                'obstacle_rear_filter_max_range_m').perform(context),
            'statistics_output': LaunchConfiguration(
                'obstacle_statistics_output').perform(context),
            'debug_log_interval_sec': LaunchConfiguration(
                'obstacle_debug_log_interval_sec').perform(context),
            'debug_base_cloud_enabled': LaunchConfiguration(
                'obstacle_debug_base_cloud_enabled').perform(context),
            'debug_base_cloud_topic': LaunchConfiguration(
                'obstacle_debug_base_cloud_topic').perform(context),
        }),
        _include('agt_nav2_bringup', 'nav2.launch.py', {
            'map': str(map_path),
            'nav2_params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart').perform(context),
        }),
        _include('agt_navigation_supervisor', 'navigation_supervisor.launch.py', {
            'robot_profile': robot_profile,
            'map_id': selected_map.map_id,
            'map_version': selected_map.map_version,
            'map_valid': 'true',
            'use_sim_time': use_sim_time,
        }),
        _include('agt_navigation_capability', 'navigation_capability.launch.py', {
            'robot_profile': robot_profile,
            'map_id': selected_map.map_id,
            'map_version': selected_map.map_version,
            'require_payload_drive_permission': str(payload_interlock).lower(),
            'use_sim_time': use_sim_time,
        }),
        _include('agt_base_runtime', 'motion_guard.launch.py', {
            'backend': motion_guard_backend,
            'params_file': params_file,
            'require_payload_drive_permission': str(payload_interlock).lower(),
        }),
        _include('agt_base_runtime', 'base_adapter.launch.py', {
            'adapter': base_adapter,
        }),
        # Map-frame path editing has its own atomic launch owner.
        _include('agt_rviz_patrol', 'path_tool.launch.py', {
            'map_id': selected_map.map_id,
            'map_version': selected_map.map_version,
            'use_sim_time': use_sim_time,
        }),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='auto',
                              description='auto, active, latest, map_id or map_id/version'),
        DeclareLaunchArgument('robot', default_value='bunker_v1'),
        DeclareLaunchArgument('nav_config_dir', default_value='',
                              description='YHS only: dedicated measured Nav2 config directory + field_profile.yaml'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='whole-robot config id/dir; decides payload_interlock'),
        DeclareLaunchArgument(
            'base_adapter',
            default_value=EnvironmentVariable('AGT_BASE_ADAPTER', default_value='bunker'),
            description='bunker only; selected from the validated whole-robot config'),
        DeclareLaunchArgument('payload_interlock', default_value='auto',
                              description='auto (from robot_config) | true | false; '
                                          'true = hold chassis unless the arm grants drive permission'),
        DeclareLaunchArgument(
            'motion_guard_backend',
            default_value=EnvironmentVariable('AGT_MOTION_GUARD_BACKEND', default_value='cpp'),
            description='cpp (default) or python (rollback); starts one command guard'),
        DeclareLaunchArgument('map_registry',
                              default_value=EnvironmentVariable('AGT_MAP_REGISTRY', default_value='')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('local_obstacle_avoidance', default_value='true',
                              description='Set false for a static-map-only local costmap trial.'),
        DeclareLaunchArgument(
            'obstacle_rear_filter_enabled', default_value='false',
            description='Trial-only short-range rear pole mask on obstacle marking.'),
        DeclareLaunchArgument('obstacle_rear_filter_center_deg', default_value='180.0'),
        DeclareLaunchArgument('obstacle_rear_filter_width_deg', default_value='70.0'),
        DeclareLaunchArgument('obstacle_rear_filter_min_range_m', default_value='0.5'),
        DeclareLaunchArgument('obstacle_rear_filter_max_range_m', default_value='1.0'),
        DeclareLaunchArgument(
            'obstacle_statistics_output', default_value='',
            description='Diagnostic: path for the cumulative preprocessor filter statistics YAML.'),
        DeclareLaunchArgument(
            'obstacle_debug_log_interval_sec', default_value='0.0',
            description='Diagnostic: cumulative filter-statistics log interval; 0 disables it.'),
        DeclareLaunchArgument(
            'obstacle_debug_base_cloud_enabled', default_value='false',
            description='Diagnostic: publish accepted obstacle points in base_link for audit/RViz.'),
        DeclareLaunchArgument(
            'obstacle_debug_base_cloud_topic', default_value='/agt/debug/points_obstacles_base',
            description='Diagnostic: topic for the base_link audit cloud.'),
        OpaqueFunction(function=_launch_runtime),
    ])
