from pathlib import Path
import ast
import importlib.util

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_local_obstacle_trial_keeps_static_map_and_disables_live_layer():
    launch_file = ROOT / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('agt_navigation_trial_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config_dir = ROOT.parents[1] / 'config'

    normal = yaml.safe_load(Path(module._build_runtime_params(config_dir)).read_text())
    trial = yaml.safe_load(Path(module._build_runtime_params(
        config_dir, local_obstacle_avoidance=False)).read_text())
    normal_local = normal['local_costmap']['local_costmap']['ros__parameters']
    trial_local = trial['local_costmap']['local_costmap']['ros__parameters']
    assert normal_local['plugins'] == ['voxel_layer', 'inflation_layer']
    assert trial_local['plugins'] == ['static_layer', 'inflation_layer']
    assert 'voxel_layer' not in trial_local
    assert trial_local['static_layer'] == trial['global_costmap'][
        'global_costmap']['ros__parameters']['static_layer']


def test_navigation_top_level_launch_files_exclude_mission():
    names = sorted(path.name for path in (ROOT / 'launch').glob('*.launch.py'))
    assert names == [
        'debug.launch.py',
        'hardware.launch.py',
        'initialization_view.launch.py',
        'localization.launch.py',
        'navigation.launch.py',
        'system.launch.py',
    ]


def _has_node_action(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
               node.func.id == 'Node' for node in ast.walk(tree))


def _node_specs(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    specs = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                node.func.id == 'Node'):
            continue
        fields = {}
        for keyword in node.keywords:
            if isinstance(keyword.value, ast.Constant) and isinstance(keyword.value.value, str):
                fields[keyword.arg] = keyword.value.value
        specs.append((fields.get('package'), fields.get('executable'), fields.get('name')))
    return specs


def test_atomic_launch_entries_and_aggregator_node_ownership():
    repo = ROOT.parents[1]
    platform = repo.parent / 'agt_robot_platform' / 'agt_robot_bringup' / 'launch'
    description = repo.parent / 'agt_robot_description' / 'launch'
    atomics = [
        description / 'description.launch.py',
        platform / 'lidar.launch.py',
        platform / 'base.launch.py',
        repo / 'navigation/nav2/agt_navigation_runtime/launch/lio.launch.py',
        repo / 'cleaning/agt_pointcloud_preprocessor/launch/local_perception.launch.py',
        repo / 'navigation/localization/agt_global_relocalization/launch/global_relocalization.launch.py',
        repo / 'navigation/localization/agt_localization_manager/launch/localization_manager.launch.py',
        repo / 'platform/agt_base_runtime/launch/motion_guard.launch.py',
        repo / 'navigation/nav2/agt_nav2_bringup/launch/nav2.launch.py',
        repo / 'navigation/nav2/agt_rviz_patrol/launch/rviz.launch.py',
    ]
    assert all(path.is_file() for path in atomics)

    # Convenience layers compose atomic owners. A direct Node action here would
    # silently create a second owner when the component is launched separately.
    aggregators = [
        ROOT / 'launch/hardware.launch.py',
        ROOT / 'launch/localization.launch.py',
        ROOT / 'launch/navigation.launch.py',
        ROOT / 'launch/system.launch.py',
        platform / 'robot_hardware.launch.py',
        platform / 'sensors.launch.py',
    ]
    assert all(not _has_node_action(path) for path in aggregators)

    local = (repo / 'bringup/agt_system_bringup/launch/localization.launch.py').read_text()
    navigation = (repo / 'bringup/agt_system_bringup/launch/navigation.launch.py').read_text()
    system = (repo / 'bringup/agt_system_bringup/launch/system.launch.py').read_text()
    assert "'agt_navigation_runtime', 'lio.launch.py'" in local
    assert "'agt_localization_manager', 'localization_manager.launch.py'" in local
    assert "'agt_pointcloud_preprocessor', 'local_perception.launch.py'" in navigation
    assert "'agt_base_runtime', 'motion_guard.launch.py'" in navigation
    assert "'agt_nav2_bringup', 'nav2.launch.py'" in navigation
    assert "'agt_robot_bringup'), 'launch'" not in system
    assert "'hardware.launch.py'" in system
    assert "'localization.launch.py'" in system
    assert "'navigation.launch.py'" in system


def test_singleton_node_definitions_have_atomic_owners():
    repo = ROOT.parents[1]
    description = repo.parent / 'agt_robot_description/launch/description.launch.py'
    manager = repo / 'navigation/localization/agt_localization_manager/launch/localization_manager.launch.py'
    guard_cpp = repo / 'platform/agt_base_runtime/launch/motion_guard.launch.py'
    guard_py = repo / 'bringup/agt_base_control/launch/cmd_vel_guard.launch.py'
    nav2 = repo / 'navigation/nav2/agt_nav2_bringup/launch/nav2.launch.py'
    lidar_helper = repo.parent / 'agt_robot_platform/agt_robot_bringup/tools/robot_hardware_components.py'

    assert "package='robot_state_publisher'" in description.read_text()
    assert "package='agt_localization_manager'" in manager.read_text()
    assert "executable='motion_guard'" in guard_cpp.read_text()
    assert "executable='cmd_vel_guard'" in guard_py.read_text()
    assert "package='nav2_map_server'" in nav2.read_text()
    helper = lidar_helper.read_text()
    assert "package='livox_ros_driver2'" in helper
    assert "'bunker_base', 'bunker_base.launch.py'" in helper
    assert "'publish_odom_tf': 'false'" in helper
    assert 'YHS base startup is BLOCKED' in helper

    # Parse the production launch trees and count concrete Node declarations.
    # Simulation, offline fixtures and the explicit path-tool offline helper are
    # separate test/utility graphs and are intentionally excluded.
    launch_roots = [
        repo / 'bringup/agt_system_bringup/launch',
        repo / 'bringup/agt_base_control/launch',
        repo / 'cleaning/agt_pointcloud_preprocessor/launch',
        repo / 'navigation/localization/agt_localization_manager/launch',
        repo / 'navigation/localization/agt_global_relocalization/launch',
        repo / 'navigation/nav2/agt_navigation_runtime/launch',
        repo / 'navigation/nav2/agt_nav2_bringup/launch',
        repo / 'navigation/nav2/agt_rviz_patrol/launch',
        repo / 'platform/agt_base_runtime/launch',
        repo.parent / 'agt_robot_description/launch',
        repo.parent / 'agt_robot_platform/agt_robot_bringup/launch',
    ]
    sources = [path for root in launch_roots for path in root.glob('*.launch.py')
               if path.name not in ('path_tool_offline.launch.py',
                                    'mq4_planner_fixture.launch.py')]
    specs = [spec for path in sources for spec in _node_specs(path)]
    assert specs.count(('robot_state_publisher', 'robot_state_publisher',
                        'robot_state_publisher')) == 1
    assert specs.count(('agt_localization_manager', 'localization_manager',
                        'agt_localization_manager')) == 1
    assert specs.count(('agt_pointcloud_preprocessor', 'agt_pointcloud_preprocessor',
                        'agt_pointcloud_preprocessor')) == 1
    assert specs.count(('nav2_map_server', 'map_server', 'map_server')) == 1
    assert specs.count(('agt_base_runtime', 'motion_guard', 'agt_cmd_vel_guard')) == 1
    assert specs.count(('agt_base_control', 'cmd_vel_guard', 'agt_cmd_vel_guard')) == 1

    lio_selector = (repo / 'navigation/nav2/agt_navigation_runtime/launch/lio.launch.py').read_text()
    assert "if backend == 'fastlio2'" in lio_selector
    assert "elif backend == 'batch_lio'" in lio_selector


def test_navigation_starts_one_motion_guard_with_cpp_default_and_python_rollback():
    launch_text = (ROOT / 'launch' / 'navigation.launch.py').read_text(encoding='utf-8')
    assert "'motion_guard_backend'," in launch_text
    assert "EnvironmentVariable('AGT_MOTION_GUARD_BACKEND', default_value='cpp')" in launch_text
    assert "'agt_base_runtime', 'motion_guard.launch.py'" in launch_text
    assert "'agt_base_runtime', 'base_adapter.launch.py'" in launch_text
    assert not _has_node_action(ROOT / 'launch/navigation.launch.py')
    assert "return 'agt_base_runtime', 'bunker_adapter'" in launch_text
    assert "return 'agt_base_runtime', 'motion_guard'" in launch_text
    assert "return 'agt_base_control', 'cmd_vel_guard'" in launch_text

    guard_launch = (ROOT.parents[1] / 'platform/agt_base_runtime/launch/motion_guard.launch.py').read_text()
    assert "DeclareLaunchArgument('backend', default_value='cpp'" in guard_launch
    assert "if backend == 'python'" in guard_launch

    python_guard = (ROOT.parents[1] / 'bringup/agt_base_control/agt_base_control/'
                    'cmd_vel_guard.py').read_text(encoding='utf-8')
    cpp_guard = (ROOT.parents[1] / 'platform/agt_base_runtime/src/motion_guard_node.cpp').read_text(
        encoding='utf-8')
    safety = yaml.safe_load((ROOT.parents[1] / 'config/safety.yaml').read_text(encoding='utf-8'))
    assert "'output_topic', '/agt/base/cmd_vel'" in python_guard
    assert '"output_topic", "/agt/base/cmd_vel"' in cpp_guard
    assert safety['agt_cmd_vel_guard']['ros__parameters']['output_topic'] == '/agt/base/cmd_vel'


def test_field_wrapper_validates_and_exports_motion_guard_backend():
    wrapper = (ROOT.parents[1] / 'scripts' / 'run_field_stack.sh').read_text(encoding='utf-8')
    assert 'export AGT_BASE_ADAPTER="$ROBOT_BASE_ADAPTER"' in wrapper
    assert '--motion-guard-backend' in wrapper
    assert 'export AGT_MOTION_GUARD_BACKEND="$MOTION_GUARD_BACKEND"' in wrapper
    assert "printf 'motion_guard_backend=%s\\n' \"$MOTION_GUARD_BACKEND\"" in wrapper


def test_navigation_and_inspection_modes_are_explicit():
    mission = (ROOT.parents[2] / 'agt_mission' / 'agt_mission_bringup' / 'launch' /
               'mission.launch.py').read_text(encoding='utf-8')
    hardware = (ROOT.parents[2] / 'agt_robot_platform' / 'agt_robot_bringup' / 'tools' /
                'robot_hardware_components.py').read_text(encoding='utf-8')
    assert "'enable_legacy_inspection', default_value='false'" in mission
    assert "if value('enable_legacy_inspection').lower() == 'true':" in mission
    assert "'agt_navigation_runtime'," not in mission.split("if value('enable_legacy_inspection')")[0]
    # Device defaults moved to the whole-robot config (single source); the Bunker
    # inspection robot still enables the camera gimbal by default.
    assert 'enable_camera_gimbal' in hardware
    bunker = yaml.safe_load((ROOT.parents[2] / 'agt_robot_platform' / 'agt_robot_bringup' /
                             'config' / 'robots' / 'bunker_inspection' / 'robot.yaml')
                            .read_text(encoding='utf-8'))
    assert bunker['payloads']['camera_gimbal']['enabled'] is True


def test_six_config_sources_are_installed():
    cmake = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    for name in (
        'robot.yaml', 'navigation.yaml', 'controller.yaml',
        'costmap.yaml', 'perception.yaml', 'safety.yaml',
    ):
        assert f'config/{name}' in cmake


def test_geometry_motion_and_obstacle_sources_are_not_duplicated():
    config_root = ROOT.parents[1] / 'config'
    configs = {
        path.name: yaml.safe_load(path.read_text(encoding='utf-8'))
        for path in config_root.glob('*.yaml')
    }
    assert sorted(configs) == [
        'controller.yaml', 'costmap.yaml', 'navigation.yaml',
        'perception.yaml', 'robot.yaml', 'safety.yaml',
    ]
    assert 'footprint' in configs['robot.yaml']['agt_robot_config']['ros__parameters']
    assert 'agt_motion_limits' in configs['safety.yaml']
    assert '/agt/navigation/points_obstacles' == (
        configs['perception.yaml']['agt_pointcloud_preprocessor']['ros__parameters'][
            'output_topic'])
    perception = configs['perception.yaml']['agt_pointcloud_preprocessor']['ros__parameters']
    for filter_name in ('self_filter', 'rear_filter', 'ground_filter', 'voxel'):
        assert isinstance(perception[filter_name]['enabled'], bool)
    assert perception['self_filter']['enabled'] is True
    assert perception['ground_filter']['enabled'] is True
    assert perception['ground_filter']['mode'] == 'radial_slope'
    assert perception['voxel']['enabled'] is True
    # Rear masking is an explicit field-trial launch option, never a hidden
    # default that removes real obstacles behind the robot.
    assert perception['rear_filter']['enabled'] is False

    serialized = {name: yaml.safe_dump(data) for name, data in configs.items()}
    assert [name for name, text in serialized.items() if 'footprint:' in text] == [
        'robot.yaml']
    assert [name for name, text in serialized.items() if 'forward_mps:' in text] == [
        'safety.yaml']
    assert [name for name, text in serialized.items()
            if 'output_topic: /agt/navigation/points_obstacles' in text] == [
                'perception.yaml']


def test_costmap_clearance_and_dynamic_clearing_contract():
    config_root = ROOT.parents[1] / 'config'
    costmap = yaml.safe_load((config_root / 'costmap.yaml').read_text(encoding='utf-8'))
    perception = yaml.safe_load(
        (config_root / 'perception.yaml').read_text(encoding='utf-8'))
    controller = yaml.safe_load(
        (config_root / 'controller.yaml').read_text(encoding='utf-8'))

    local = costmap['local_costmap']['local_costmap']['ros__parameters']
    global_ = costmap['global_costmap']['global_costmap']['ros__parameters']
    voxel = perception['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    follow = controller['controller_server']['ros__parameters']['FollowPath']

    assert local['inflation_layer']['inflation_radius'] >= 0.85
    assert global_['inflation_layer']['inflation_radius'] >= 1.0
    assert follow['cost_scaling_dist'] <= local['inflation_layer']['inflation_radius']
    assert follow['inflation_cost_scaling_factor'] == (
        local['inflation_layer']['cost_scaling_factor'])

    assert set(voxel['observation_sources'].split()) == {'lidar3d_mark', 'lidar3d_clear'}
    assert voxel['lidar3d_mark']['topic'] == '/agt/navigation/points_obstacles'
    assert voxel['lidar3d_mark']['marking'] is True
    assert voxel['lidar3d_mark']['clearing'] is False
    assert voxel['lidar3d_clear']['topic'] == '/agt/livox/points'
    assert voxel['lidar3d_clear']['marking'] is False
    assert voxel['lidar3d_clear']['clearing'] is True
    assert voxel['lidar3d_clear']['min_obstacle_height'] < 0.0


def test_tracked_chassis_controller_uses_stable_path_and_lag_aware_preview():
    repo = ROOT.parents[1]
    controller = yaml.safe_load((repo / 'config/controller.yaml').read_text())
    safety = yaml.safe_load((repo / 'config/safety.yaml').read_text())
    follow = controller['controller_server']['ros__parameters']['FollowPath']
    limits = safety['agt_motion_limits']['ros__parameters']

    assert follow['use_velocity_scaled_lookahead_dist'] is True
    assert follow['use_interpolation'] is True
    assert follow['min_lookahead_dist'] >= 0.65
    assert follow['lookahead_time'] >= 2.0
    assert follow['regulated_linear_scaling_min_radius'] >= 1.2
    assert limits['controller_cruise_mps'] <= 0.40

    bt_name = 'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml'
    system_launch = (ROOT / 'launch/navigation.launch.py').read_text()
    nav2_launch = (
        repo / 'navigation/nav2/agt_nav2_bringup/launch/nav2.launch.py'
    ).read_text()
    assert bt_name in system_launch
    assert bt_name in nav2_launch


def test_rear_pole_trial_keeps_short_range_marking_only_mask():
    repo = ROOT.parents[1]
    perception = yaml.safe_load((repo / 'config/perception.yaml').read_text())
    params = perception['agt_pointcloud_preprocessor']['ros__parameters']
    assert params['rear_filter'] == {
        'enabled': False, 'center_deg': 180.0, 'width_deg': 70.0,
        'min_range_m': 0.5, 'max_range_m': 1.0,
    }
    launch = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "'obstacle_rear_filter_enabled', default_value='false'" in launch
    assert "'obstacle_rear_filter_enabled').perform(context)" in launch
    atomic_perception = (repo / 'cleaning/agt_pointcloud_preprocessor/launch/local_perception.launch.py').read_text()
    assert "'rear_filter.enabled': ParameterValue(" in atomic_perception
    source = (repo / 'cleaning/agt_pointcloud_preprocessor/src/obstacle_cloud_node.cpp').read_text()
    assert 'if (rear_enabled_ && inside_rear_sector(p_base))' in source
    assert 'std::hypot(p.x(), p.y())' in source
    assert 'normalize_angle(bearing - rear_center_rad_)' in source
    assert '++statistics_.rear_removed;' in source
    assert 'rear_sector_enabled:' in source
    # Marking is filtered; clearing still receives the unmasked raw PointCloud2 branch.
    voxel = perception['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    assert voxel['lidar3d_clear']['topic'] == '/agt/livox/points'
    assert voxel['lidar3d_mark']['topic'] == params['output_topic']
    lio = (repo / 'navigation/nav2/agt_navigation_runtime/launch/fastlio_navigation_lio.launch.py').read_text()
    assert "default_value='/livox/lidar'" in lio
    assert 'points_obstacles' not in lio
