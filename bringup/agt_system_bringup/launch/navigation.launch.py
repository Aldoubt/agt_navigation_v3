"""Nav2, the sole obstacle-cloud node and the guarded command chain."""

from pathlib import Path
from copy import deepcopy
import math
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


def _read_yaml(path, label):
    try:
        value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'{label} missing or invalid: {path}') from exc
    if not isinstance(value, dict):
        raise RuntimeError(f'{label} must contain a YAML mapping: {path}')
    return value


def _robot_profile_path(robot_profile, robot_profiles_dir=None):
    if robot_profiles_dir is None:
        description_share = Path(get_package_share_directory('agt_robot_description'))
        robot_profiles_dir = description_share / 'config' / 'robot_profiles'
    path = Path(robot_profiles_dir) / f'{robot_profile}.yaml'
    if not path.is_file():
        raise RuntimeError(f'robot profile does not exist: {path}')
    profile = _read_yaml(path, 'robot profile')
    if profile.get('robot_id') != robot_profile:
        raise RuntimeError(f'robot profile id mismatch in {path}')
    return profile


def _same_numbers(left, right, tolerance=1.0e-8):
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return (len(left) == len(right) and
                all(_same_numbers(a, b, tolerance) for a, b in zip(left, right)))
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)
    return left == right


def _positive_finite(value):
    return (isinstance(value, (int, float)) and math.isfinite(float(value)) and
            float(value) > 0.0)


def _profile_nav_configs(config_dir):
    return {
        name: _read_yaml(Path(config_dir) / name, f'Nav2 config {name}')
        for name in CONFIG_FILES
    }


def _validate_tracked_profile(config_dir, robot_profile, manifest):
    """Assert that the canonical Bunker files still implement the frozen baseline."""
    base = robot_profile['base']
    if (base.get('kinematics') != 'skid_steer' or base.get('rotate_in_place') is not True or
            base.get('publish_odom_tf') is not False or base.get('control_rate_hz') != 50):
        raise RuntimeError('bunker_v1 Robot Profile must remain skid_steer with in-place rotation')
    expected = {
        'schema_version': 1,
        'profile_id': 'bunker_tracked',
        'robot_profile': 'bunker_v1',
        'kinematics': 'skid_steer',
        'parameter_files': {
            'robot': 'robot.yaml',
            'navigation': 'navigation.yaml',
            'controller': 'controller.yaml',
            'costmap': 'costmap.yaml',
            'perception': 'perception.yaml',
            'safety': 'safety.yaml',
        },
        'planner': {'plugin': 'nav2_smac_planner/SmacPlanner2D'},
        'controller': {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'rotate_to_heading': True,
            'allow_reversing': False,
        },
        'recovery': {'spin': True, 'backup': True},
        'minimum_turning_radius_source': None,
        'footprint_source': 'robot.yaml',
        'motion_limits_source': 'safety.yaml',
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise RuntimeError(f'tracked/skid navigation profile has invalid {key!r}')

    configs = _profile_nav_configs(config_dir)
    nav = configs['navigation.yaml']
    controller = configs['controller.yaml']
    robot_yaml = configs['robot.yaml']
    safety = configs['safety.yaml']
    planner_plugin = _params(nav, 'planner_server')['GridBased'].get('plugin')
    follow = _params(controller, 'controller_server')['FollowPath']
    behaviors = _params(nav, 'behavior_server').get('behavior_plugins', [])
    if planner_plugin != manifest['planner']['plugin']:
        raise RuntimeError('tracked/skid planner no longer matches its profile manifest')
    if (follow.get('plugin') != manifest['controller']['plugin'] or
            follow.get('use_rotate_to_heading') is not True or
            follow.get('allow_reversing') is not False):
        raise RuntimeError('tracked/skid controller no longer matches its profile manifest')
    if (('spin' in behaviors) is not manifest['recovery']['spin'] or
            ('backup' in behaviors) is not manifest['recovery']['backup']):
        raise RuntimeError('tracked/skid recovery behaviors no longer match their profile manifest')

    robot_params = _params(robot_yaml, 'agt_robot_config')
    declared_footprint = yaml.safe_load(robot_params['footprint'])
    if not _same_numbers(declared_footprint, robot_profile['geometry']['footprint']):
        raise RuntimeError('Bunker Nav2 footprint differs from the Robot Profile footprint')
    if not math.isclose(float(robot_params['footprint_padding']),
                        float(robot_profile['geometry']['safety_margin']), abs_tol=1e-8):
        raise RuntimeError('Bunker Nav2 footprint padding differs from the Robot Profile margin')

    limits = _params(safety, 'agt_motion_limits')
    if (not math.isclose(float(limits['forward_mps']),
                         float(robot_profile['motion']['max_linear_velocity']), abs_tol=1e-8) or
            not math.isclose(float(limits['angular_radps']),
                             float(robot_profile['motion']['max_angular_velocity']), abs_tol=1e-8)):
        raise RuntimeError('Bunker Nav2 limits differ from the Robot Profile motion limits')
    rates = (
        _params(controller, 'controller_server')['controller_frequency'],
        _params(safety, 'velocity_smoother')['smoothing_frequency'],
        _params(safety, 'agt_cmd_vel_guard')['publish_rate_hz'],
    )
    if any(not _positive_finite(rate) or float(rate) < 50.0 for rate in rates):
        raise RuntimeError('Bunker controller, smoother and motion guard must preserve the 50 Hz chain')


def _validate_ackermann_nav_semantics(config_dir, robot_profile, manifest):
    """Validate Ackermann planner/controller contracts against measured profile fields.

    This checks a config bundle only. Selection for a live launch additionally
    requires ``field_verified: true`` and a non-empty reviewer in its manifest.
    """
    base = robot_profile.get('base', {})
    geometry = robot_profile.get('geometry', {})
    motion = robot_profile.get('motion', {})
    ack = base.get('ackermann', {})
    if (base.get('kinematics') != 'ackermann' or base.get('rotate_in_place') is not False or
            not isinstance(ack, dict)):
        raise RuntimeError('Ackermann Robot Profile must declare ackermann kinematics and rotate_in_place: false')
    for key in ('wheelbase_m', 'steering_angle_min_rad', 'steering_angle_max_rad',
                'max_steering_rate_radps', 'minimum_turning_radius_m'):
        if not isinstance(ack.get(key), (int, float)) or not math.isfinite(float(ack[key])):
            raise RuntimeError(f'Ackermann Robot Profile is missing measured {key}')
    if (ack['wheelbase_m'] <= 0.0 or ack['steering_angle_min_rad'] >= 0.0 or
            ack['steering_angle_max_rad'] <= 0.0 or ack['max_steering_rate_radps'] <= 0.0 or
            ack['minimum_turning_radius_m'] <= 0.0 or
            ack['steering_angle_min_rad'] <= -math.pi / 2.0 or
            ack['steering_angle_max_rad'] >= math.pi / 2.0):
        raise RuntimeError('Ackermann Robot Profile contains invalid steering geometry')
    max_steering_angle = max(abs(float(ack['steering_angle_min_rad'])),
                             abs(float(ack['steering_angle_max_rad'])))
    kinematic_radius = float(ack['wheelbase_m']) / math.tan(max_steering_angle)
    if float(ack['minimum_turning_radius_m']) + 1.0e-8 < kinematic_radius:
        raise RuntimeError('Ackermann minimum turning radius is tighter than its wheelbase/steering limits')
    if not _positive_finite(base.get('control_rate_hz')) or float(base['control_rate_hz']) < 50.0:
        raise RuntimeError('Ackermann Robot Profile must preserve the 50 Hz command chain')
    for key in ('width', 'length'):
        if not _positive_finite(geometry.get(key)):
            raise RuntimeError(f'Ackermann Robot Profile requires positive measured geometry.{key}')
    margin = geometry.get('safety_margin')
    if not isinstance(margin, (int, float)) or not math.isfinite(float(margin)) or float(margin) < 0.0:
        raise RuntimeError('Ackermann Robot Profile requires a finite non-negative geometry.safety_margin')

    required_manifest = {
        'schema_version': 1,
        'robot_profile': robot_profile.get('robot_id'),
        'kinematics': 'ackermann',
        'planner': {
            'plugin': 'nav2_smac_planner/SmacPlannerHybrid',
            'motion_model_for_search': 'DUBIN',
        },
        'controller': {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'rotate_to_heading': False,
            'allow_reversing': False,
        },
        'recovery': {'spin': False, 'backup': False},
        'minimum_turning_radius_source': 'robot_profile.base.ackermann.minimum_turning_radius_m',
        'footprint_source': 'robot_profile.geometry.footprint',
        'motion_limits_source': 'safety.yaml',
    }
    for key, value in required_manifest.items():
        if manifest.get(key) != value:
            raise RuntimeError(f'Ackermann navigation profile has invalid {key!r}')

    configs = _profile_nav_configs(config_dir)
    nav = configs['navigation.yaml']
    controller = configs['controller.yaml']
    robot_yaml = configs['robot.yaml']
    costmap = configs['costmap.yaml']
    safety = configs['safety.yaml']
    planner = _params(nav, 'planner_server')['GridBased']
    follow = _params(controller, 'controller_server')['FollowPath']
    behaviors = _params(nav, 'behavior_server').get('behavior_plugins', [])
    if planner.get('plugin') != required_manifest['planner']['plugin']:
        raise RuntimeError('Ackermann requires the Smac Hybrid planner')
    if planner.get('motion_model_for_search') != 'DUBIN':
        raise RuntimeError('Ackermann profile with reversing disabled requires the DUBIN motion model')
    turning_radius = float(ack['minimum_turning_radius_m'])
    if not math.isclose(float(planner.get('minimum_turning_radius', math.nan)),
                        turning_radius, rel_tol=0.0, abs_tol=1e-8):
        raise RuntimeError('Smac Hybrid minimum_turning_radius must equal the Robot Profile value')
    global_costmap = _params(costmap, 'global_costmap', 'global_costmap')
    resolution = global_costmap.get('resolution')
    if not _positive_finite(resolution):
        raise RuntimeError('Ackermann global costmap resolution must be finite and positive')
    if turning_radius < float(resolution):
        raise RuntimeError('Ackermann minimum turning radius must not be below the global costmap resolution')

    if (follow.get('plugin') != required_manifest['controller']['plugin'] or
            follow.get('use_rotate_to_heading') is not False or
            follow.get('allow_reversing') is not False):
        raise RuntimeError('Ackermann RPP must disable rotate_to_heading and reversing')
    if 'spin' in behaviors or 'backup' in behaviors:
        raise RuntimeError('Ackermann recovery must not request spin or backup motion')
    controller_params = _params(controller, 'controller_server')
    if not controller_params.get('progress_checker_plugin'):
        raise RuntimeError('Ackermann controller requires an explicit progress checker')
    progress = controller_params.get('progress_checker', {})
    if not progress.get('plugin'):
        raise RuntimeError('Ackermann progress checker requires an explicit plugin')
    for key in ('required_movement_radius', 'required_movement_angle', 'movement_time_allowance'):
        value = progress.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0.0:
            raise RuntimeError(f'Ackermann progress checker requires positive {key}')
    for key in ('lookahead_dist', 'min_lookahead_dist', 'max_lookahead_dist',
                'max_allowed_time_to_collision_up_to_carrot'):
        value = follow.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0.0:
            raise RuntimeError(f'Ackermann RPP requires positive {key}')
    if (follow.get('use_velocity_scaled_lookahead_dist') is True and
            float(follow['min_lookahead_dist']) > float(follow['max_lookahead_dist'])):
        raise RuntimeError('Ackermann RPP minimum lookahead must not exceed maximum lookahead')

    robot_params = _params(robot_yaml, 'agt_robot_config')
    declared_footprint = yaml.safe_load(robot_params['footprint'])
    if not isinstance(geometry.get('footprint'), list) or not _same_numbers(
            declared_footprint, geometry['footprint']):
        raise RuntimeError('Ackermann Nav2 footprint must match the Robot Profile geometry')
    if not math.isclose(float(robot_params['footprint_padding']),
                        float(geometry['safety_margin']), abs_tol=1e-8):
        raise RuntimeError('Ackermann Nav2 footprint padding must match the Robot Profile safety margin')
    if (robot_params.get('base_frame') != robot_profile.get('frames', {}).get('base') or
            robot_params.get('rotation_frame') != robot_profile.get('frames', {}).get('footprint')):
        raise RuntimeError('Ackermann Nav2 base/footprint frames must match the Robot Profile')

    limits = _params(safety, 'agt_motion_limits')
    forward = float(limits['forward_mps'])
    reverse = float(limits['reverse_mps'])
    angular = float(limits['angular_radps'])
    profile_linear = float(motion.get('max_linear_velocity', math.nan))
    profile_angular = float(motion.get('max_angular_velocity', math.nan))
    if (not all(math.isfinite(value) for value in
                (forward, reverse, angular, profile_linear, profile_angular)) or
            forward <= 0.0 or reverse != 0.0 or angular <= 0.0):
        raise RuntimeError('Ackermann Nav2 limits must be finite, positive, and have reverse disabled')
    if (forward > profile_linear or angular > profile_angular or
            angular > forward / turning_radius):
        raise RuntimeError('Ackermann Nav2 limits exceed the Robot Profile speed/turning envelope')
    cruise = float(limits['controller_cruise_mps'])
    approach = float(limits['controller_approach_mps'])
    regulated_min = float(limits['controller_regulated_min_mps'])
    if not (0.0 <= approach <= cruise <= forward and 0.0 <= regulated_min <= cruise):
        raise RuntimeError('Ackermann controller speeds must fit the profile forward-speed limits')
    for name in ('linear_accel_mps2', 'linear_decel_mps2',
                 'angular_accel_radps2', 'angular_decel_radps2'):
        if not _positive_finite(limits.get(name)):
            raise RuntimeError(f'Ackermann motion limit {name} must be positive')
    rates = (
        _params(controller, 'controller_server').get('controller_frequency'),
        _params(safety, 'velocity_smoother').get('smoothing_frequency'),
        _params(safety, 'agt_cmd_vel_guard').get('publish_rate_hz'),
    )
    if any(not _positive_finite(rate) or float(rate) < 50.0 for rate in rates):
        raise RuntimeError('Ackermann controller, smoother and motion guard must preserve the 50 Hz command chain')


def _validate_generic_tracked_profile(config_dir, profile, manifest):
    """Resolve measured YHS/custom planar Twist profiles without Bunker geometry."""
    base, geometry, motion = profile['base'], profile['geometry'], profile['motion']
    if (base.get('kinematics') not in ('skid_steer', 'differential') or
            base.get('command_interface') != 'twist' or
            base.get('publish_odom_tf') is not False or
            base.get('rotate_in_place') is not True or
            not _positive_finite(base.get('control_rate_hz')) or base['control_rate_hz'] < 50):
        raise RuntimeError('tracked profile requires verified planar Twist, rotation and a 50 Hz chain')
    for key in ('width', 'length'):
        if not _positive_finite(geometry.get(key)):
            raise RuntimeError(f'tracked profile requires measured geometry.{key}')
    for key in ('max_linear_velocity', 'max_angular_velocity'):
        if not _positive_finite(motion.get(key)):
            raise RuntimeError(f'tracked profile requires measured motion.{key}')
    footprint = geometry.get('footprint')
    if (not isinstance(footprint, list) or len(footprint) < 3 or
            any(len(p) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p)
                for p in footprint)):
        raise RuntimeError('tracked profile requires a finite measured footprint polygon')
    margin = geometry.get('safety_margin')
    if not isinstance(margin, (int, float)) or not math.isfinite(margin) or margin < 0:
        raise RuntimeError('tracked profile requires a measured nonnegative safety margin')
    planner_plugin = 'nav2_smac_planner/SmacPlanner2D'
    rpp_plugin = 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    if (manifest.get('schema_version') != 1 or
            manifest.get('planner', {}).get('plugin') != planner_plugin or
            manifest.get('controller') != {'plugin': rpp_plugin, 'rotate_to_heading': True,
                                          'allow_reversing': False}):
        raise RuntimeError('tracked profile requires Smac2D and forward RPP with rotate-to-heading')
    configs = _profile_nav_configs(config_dir)
    nav, control = configs['navigation.yaml'], configs['controller.yaml']
    follow = _params(control, 'controller_server')['FollowPath']
    if (_params(nav, 'planner_server')['GridBased'].get('plugin') != planner_plugin or
            follow.get('plugin') != rpp_plugin or follow.get('use_rotate_to_heading') is not True or
            follow.get('allow_reversing') is not False):
        raise RuntimeError('tracked Nav2 algorithms differ from their manifest')
    robot = _params(configs['robot.yaml'], 'agt_robot_config')
    if (not _same_numbers(yaml.safe_load(robot['footprint']), footprint) or
            not _same_numbers(robot['footprint_padding'], margin)):
        raise RuntimeError('tracked Nav2 footprint and margin must equal the selected Robot Profile')
    local = _params(configs['costmap.yaml'], 'local_costmap', 'local_costmap')
    if local.get('global_frame') != 'odom' or local.get('rolling_window') is not True:
        raise RuntimeError('tracked local costmap must be a rolling odom-frame costmap')
    if local.get('robot_base_frame') != profile['frames']['base']:
        raise RuntimeError('tracked costmap base frame differs from the Robot Profile')
    limits = _params(configs['safety.yaml'], 'agt_motion_limits')
    if (not _same_numbers(limits['forward_mps'], motion['max_linear_velocity']) or
            not _same_numbers(limits['angular_radps'], motion['max_angular_velocity'])):
        raise RuntimeError('tracked velocity limits differ from the Robot Profile')
    for key in ('linear_accel_mps2', 'linear_decel_mps2', 'angular_accel_radps2',
                'angular_decel_radps2'):
        if not _positive_finite(limits.get(key)):
            raise RuntimeError(f'tracked profile requires measured {key}')
    rates = (_params(control, 'controller_server')['controller_frequency'],
             _params(configs['safety.yaml'], 'velocity_smoother')['smoothing_frequency'],
             _params(configs['safety.yaml'], 'agt_cmd_vel_guard')['publish_rate_hz'])
    if any(not _positive_finite(r) or r < 50 for r in rates):
        raise RuntimeError('tracked controller, smoother and guard must run at least 50 Hz')


def select_navigation_config(robot_profile, explicit_dir, default_dir, robot_profiles_dir=None):
    """Select a kinematics-matched Nav2 profile and fail closed before field use."""
    default = Path(default_dir).resolve()
    selected_robot = (robot_profile or 'bunker_v1').strip()
    profile = _robot_profile_path(selected_robot, robot_profiles_dir)
    kinematics = profile.get('base', {}).get('kinematics')

    if selected_robot == 'bunker_v1':
        if explicit_dir:
            raise RuntimeError('nav_config_dir override is reserved for a reviewed non-baseline profile')
        if kinematics != 'skid_steer':
            raise RuntimeError('bunker_v1 Robot Profile must remain skid_steer')
        manifest = _read_yaml(default / 'navigation_profiles' / 'bunker_v1.yaml',
                              'tracked/skid navigation profile')
        _validate_tracked_profile(default, profile, manifest)
        return default

    if not explicit_dir:
        raise RuntimeError(f'{selected_robot} requires an explicit measured nav_config_dir')
    selected = Path(explicit_dir).expanduser().resolve()
    if selected == default or not selected.is_dir():
        raise RuntimeError(f'{selected_robot} may not use the canonical Bunker config directory')
    missing = [name for name in CONFIG_FILES if not (selected / name).is_file()]
    if missing:
        raise RuntimeError(f'Nav2 profile missing required YAML files: {missing}')
    manifest = _read_yaml(selected / 'navigation_profile.yaml', 'navigation profile manifest')
    if manifest.get('robot_profile') != selected_robot:
        raise RuntimeError('navigation profile robot_profile does not match the selected Robot Profile')
    if manifest.get('kinematics') != kinematics:
        raise RuntimeError('navigation profile kinematics do not match the selected Robot Profile')
    if manifest.get('field_verified') is not True or not str(manifest.get('verified_by', '')).strip():
        raise RuntimeError('non-baseline Nav2 profile requires measured field_verified: true and verified_by')
    if kinematics == 'ackermann':
        _validate_ackermann_nav_semantics(selected, profile, manifest)
        return selected
    if kinematics in ('skid_steer', 'differential'):
        _validate_generic_tracked_profile(selected, profile, manifest)
        return selected
    raise RuntimeError(
        f'Nav2 profile for kinematics {kinematics!r} is not implemented/verified; refusing Bunker fallback')


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
    })
    if follow.get('use_rotate_to_heading', False):
        follow.update({
            'rotate_to_heading_angular_vel': limits['rotate_to_heading_radps'],
            'max_angular_accel': limits['controller_bootstrap_angular_accel'],
        })

    behavior = _params(merged, 'behavior_server')
    if 'spin' in behavior.get('behavior_plugins', []):
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
    if selected in ('tracked', 'yhs', 'custom'):
        return 'agt_base_runtime', 'tracked_twist_adapter'
    if selected == 'ackermann':
        raise RuntimeError('Ackermann conversion is software-only; no real driver adapter is enabled')
    raise RuntimeError(f'unsupported base adapter {adapter!r}; expected bunker|tracked|yhs|custom')


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
            'params_file': LaunchConfiguration('base_adapter_params_file').perform(context),
            'robot_profile': robot_profile,
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
                          description='Reviewed measured non-baseline Nav2 profile directory'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='whole-robot config id/dir; decides payload_interlock'),
        DeclareLaunchArgument(
            'base_adapter',
            default_value=EnvironmentVariable('AGT_BASE_ADAPTER', default_value='bunker'),
            description='bunker|tracked|yhs|custom; generic adapters require measured driver parameters'),
        DeclareLaunchArgument('base_adapter_params_file', default_value=''),
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
