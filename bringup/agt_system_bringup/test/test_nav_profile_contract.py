"""Offline Nav2 profile contract tests; Ackermann dimensions are test-only."""
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]
DEFAULT_CONFIG = REPOSITORY / 'config'
DESCRIPTION_PROFILES = REPOSITORY.parent / 'agt_robot_description' / 'config' / 'robot_profiles'
LAUNCH = PACKAGE / 'launch' / 'navigation.launch.py'


def load_launch():
    spec = importlib.util.spec_from_file_location('agt_test_navigation_profile_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def write_yaml(path, value):
    Path(path).write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')


def params(tree, *node_path):
    node = tree
    for key in node_path:
        node = node[key]
    return node['ros__parameters']


def test_bunker_profile_resolves_to_the_existing_tracked_baseline():
    mod = load_launch()
    selected = mod.select_navigation_config(
        'bunker_v1', '', DEFAULT_CONFIG)
    assert selected == DEFAULT_CONFIG.resolve()

    nav = read_yaml(selected / 'navigation.yaml')
    controller = read_yaml(selected / 'controller.yaml')
    limits = read_yaml(selected / 'safety.yaml')
    assert params(nav, 'planner_server')['GridBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert 'spin' in params(nav, 'behavior_server')['behavior_plugins']
    assert 'backup' in params(nav, 'behavior_server')['behavior_plugins']
    assert params(controller, 'controller_server')['FollowPath']['use_rotate_to_heading'] is True
    assert params(controller, 'controller_server')['FollowPath']['allow_reversing'] is False
    safety = params(limits, 'agt_motion_limits')
    assert safety['forward_mps'] == 0.55
    assert safety['reverse_mps'] == 0.20
    assert safety['angular_radps'] == 0.65
    assert safety['controller_cruise_mps'] == 0.40
    assert safety['controller_approach_mps'] == 0.05
    assert safety['controller_regulated_min_mps'] == 0.10
    assert params(controller, 'controller_server')['controller_frequency'] == 50.0
    assert params(limits, 'velocity_smoother')['smoothing_frequency'] == 50.0
    assert params(limits, 'agt_cmd_vel_guard')['publish_rate_hz'] == 50.0


def test_bunker_baseline_fails_if_the_canonical_planner_changes(tmp_path):
    mod = load_launch()
    config = tmp_path / 'config'
    config.mkdir()
    for name in mod.CONFIG_FILES:
        (config / name).write_bytes((DEFAULT_CONFIG / name).read_bytes())
    profiles = config / 'navigation_profiles'
    profiles.mkdir()
    (profiles / 'bunker_v1.yaml').write_bytes(
        (DEFAULT_CONFIG / 'navigation_profiles' / 'bunker_v1.yaml').read_bytes())
    nav = read_yaml(config / 'navigation.yaml')
    params(nav, 'planner_server')['GridBased']['plugin'] = 'nav2_smac_planner/SmacPlannerHybrid'
    write_yaml(config / 'navigation.yaml', nav)

    with pytest.raises(RuntimeError, match='planner no longer matches'):
        mod.select_navigation_config(
            'bunker_v1', '', config, robot_profiles_dir=DESCRIPTION_PROFILES)


def make_ackermann_test_bundle(tmp_path):
    """Build a disposable fixture. These synthetic dimensions are not robot data."""
    mod = load_launch()
    config = tmp_path / 'ackermann_test_fixture'
    config.mkdir()
    for name in mod.CONFIG_FILES:
        (config / name).write_bytes((DEFAULT_CONFIG / name).read_bytes())

    # Test-only geometry: wheelbase 1.0 m, steering bounds +/-0.5 rad, a
    # conservative 2.0 m minimum radius, and an explicitly synthetic footprint.
    profile_dir = tmp_path / 'robot_profiles'
    profile_dir.mkdir()
    profile = {
        'robot_id': 'ackermann_test_fixture',
        'base': {
            'kinematics': 'ackermann',
            'rotate_in_place': False,
            'control_rate_hz': 50,
            'ackermann': {
                'wheelbase_m': 1.0,
                'steering_angle_min_rad': -0.5,
                'steering_angle_max_rad': 0.5,
                'max_steering_rate_radps': 10.0,
                'minimum_turning_radius_m': 2.0,
            },
        },
        'frames': {'base': 'base_link', 'footprint': 'base_footprint'},
        'geometry': {
            'width': 0.8,
            'length': 1.2,
            'footprint': [[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]],
            'safety_margin': 0.04,
        },
        'motion': {'max_linear_velocity': 1.0, 'max_angular_velocity': 0.4},
    }
    write_yaml(profile_dir / 'ackermann_test_fixture.yaml', profile)

    manifest = {
        'schema_version': 1,
        'profile_id': 'ackermann_test_fixture_only',
        'robot_profile': 'ackermann_test_fixture',
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
        # This remains false: the fixture must never qualify for field startup.
        'field_verified': False,
    }
    write_yaml(config / 'navigation_profile.yaml', manifest)

    nav = read_yaml(config / 'navigation.yaml')
    planner = params(nav, 'planner_server')['GridBased']
    planner.update({
        'plugin': 'nav2_smac_planner/SmacPlannerHybrid',
        'motion_model_for_search': 'DUBIN',
        'minimum_turning_radius': 2.0,
    })
    behavior = params(nav, 'behavior_server')
    behavior['behavior_plugins'] = ['wait']
    behavior.pop('spin', None)
    behavior.pop('backup', None)
    write_yaml(config / 'navigation.yaml', nav)

    controller = read_yaml(config / 'controller.yaml')
    controller_params = params(controller, 'controller_server')
    controller_params['FollowPath'].update({
        'use_rotate_to_heading': False,
        'allow_reversing': False,
        # Test-only RPP settings; never a field calibration.
        'lookahead_dist': 0.5,
        'min_lookahead_dist': 0.4,
        'max_lookahead_dist': 0.8,
        'max_allowed_time_to_collision_up_to_carrot': 0.7,
        'regulated_linear_scaling_min_radius': 2.0,
    })
    controller_params['progress_checker'].update({
        'required_movement_radius': 0.02,
        'required_movement_angle': 0.04,
        'movement_time_allowance': 3.0,
    })
    write_yaml(config / 'controller.yaml', controller)

    robot = read_yaml(config / 'robot.yaml')
    robot_params = params(robot, 'agt_robot_config')
    robot_params['footprint'] = yaml.safe_dump(profile['geometry']['footprint'], default_flow_style=True).strip()
    robot_params['footprint_padding'] = profile['geometry']['safety_margin']
    write_yaml(config / 'robot.yaml', robot)

    safety = read_yaml(config / 'safety.yaml')
    limits = params(safety, 'agt_motion_limits')
    limits.update({
        'forward_mps': 0.8,
        'reverse_mps': 0.0,
        'angular_radps': 0.3,
        'controller_cruise_mps': 0.6,
        'controller_approach_mps': 0.05,
        'controller_regulated_min_mps': 0.1,
        # Synthetic limits make fan-out assertions distinct from Bunker.
        'linear_accel_mps2': 0.3,
        'linear_decel_mps2': 0.5,
        'angular_accel_radps2': 0.2,
        'angular_decel_radps2': 0.25,
    })
    limits.pop('rotate_to_heading_radps', None)
    limits.pop('controller_bootstrap_angular_accel', None)
    write_yaml(config / 'safety.yaml', safety)
    return mod, config, profile_dir, profile


def test_ackermann_bundle_is_kinematically_consistent_but_not_field_enabled(tmp_path, monkeypatch):
    mod, config, profile_dir, _ = make_ackermann_test_bundle(tmp_path)
    profile = mod._robot_profile_path('ackermann_test_fixture', profile_dir)
    manifest = read_yaml(config / 'navigation_profile.yaml')
    mod._validate_ackermann_nav_semantics(config, profile, manifest)

    # A valid software fixture cannot be promoted to a live profile by the resolver.
    with pytest.raises(RuntimeError, match='field_verified: true'):
        mod.select_navigation_config(
            'ackermann_test_fixture', config, DEFAULT_CONFIG, robot_profiles_dir=profile_dir)

    # The builder still derives the test bundle's limits and footprint without
    # requiring rotate-in-place-only tuning for Ackermann.
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda _package: str(tmp_path / 'nav2'))
    params_path = Path(mod._build_runtime_params(config))
    runtime = read_yaml(params_path)
    shutil_dir = params_path.parent
    try:
        smoother = params(runtime, 'velocity_smoother')
        guard = params(runtime, 'agt_cmd_vel_guard')
        controller = params(runtime, 'controller_server')['FollowPath']
        assert smoother['max_velocity'] == [0.8, 0.0, 0.3]
        assert smoother['max_accel'] == [0.3, 0.0, 0.2]
        assert guard['max_linear_x'] == 0.8
        assert guard['max_linear_accel'] == 0.3
        assert controller['desired_linear_vel'] == 0.6
        assert controller['use_rotate_to_heading'] is False
        assert params(runtime, 'global_costmap', 'global_costmap')['footprint'] == robot_footprint(config)
    finally:
        params_path.unlink(missing_ok=True)
        shutil_dir.rmdir()


def robot_footprint(config):
    return params(read_yaml(config / 'robot.yaml'), 'agt_robot_config')['footprint']


@pytest.mark.parametrize('invalid_case', [
    'rotate', 'spin', 'radius', 'footprint', 'nonfinite_accel', 'slow_smoother'])
def test_ackermann_contract_rejects_inconsistent_nav2_assumptions(tmp_path, invalid_case):
    mod, config, profile_dir, _ = make_ackermann_test_bundle(tmp_path)
    profile = mod._robot_profile_path('ackermann_test_fixture', profile_dir)
    if invalid_case == 'rotate':
        controller = read_yaml(config / 'controller.yaml')
        params(controller, 'controller_server')['FollowPath']['use_rotate_to_heading'] = True
        write_yaml(config / 'controller.yaml', controller)
        expected = 'disable rotate_to_heading'
    elif invalid_case == 'spin':
        nav = read_yaml(config / 'navigation.yaml')
        params(nav, 'behavior_server')['behavior_plugins'].append('spin')
        write_yaml(config / 'navigation.yaml', nav)
        expected = 'must not request spin'
    elif invalid_case == 'radius':
        nav = read_yaml(config / 'navigation.yaml')
        params(nav, 'planner_server')['GridBased']['minimum_turning_radius'] = 2.1
        write_yaml(config / 'navigation.yaml', nav)
        expected = 'must equal the Robot Profile'
    else:
        if invalid_case == 'footprint':
            robot = read_yaml(config / 'robot.yaml')
            params(robot, 'agt_robot_config')['footprint_padding'] = 0.1
            write_yaml(config / 'robot.yaml', robot)
            expected = 'footprint padding must match'
        elif invalid_case == 'nonfinite_accel':
            safety = read_yaml(config / 'safety.yaml')
            params(safety, 'agt_motion_limits')['angular_accel_radps2'] = float('inf')
            write_yaml(config / 'safety.yaml', safety)
            expected = 'angular_accel_radps2 must be positive'
        else:
            safety = read_yaml(config / 'safety.yaml')
            params(safety, 'velocity_smoother')['smoothing_frequency'] = 49.0
            write_yaml(config / 'safety.yaml', safety)
            expected = 'must preserve the 50 Hz command chain'

    with pytest.raises(RuntimeError, match=expected):
        mod._validate_ackermann_nav_semantics(
            config, profile, read_yaml(config / 'navigation_profile.yaml'))


def test_ackermann_schema_documents_required_fields_without_a_physical_profile():
    schema = json.loads((DEFAULT_CONFIG / 'navigation_profiles' / 'ackermann.schema.json').read_text())
    assert schema['properties']['planner']['properties']['plugin']['const'] == \
        'nav2_smac_planner/SmacPlannerHybrid'
    assert schema['properties']['controller']['properties']['rotate_to_heading']['const'] is False
    assert 'minimum_turning_radius_source' in schema['required']
    assert not list(DEFAULT_CONFIG.glob('*ackermann*.yaml'))
