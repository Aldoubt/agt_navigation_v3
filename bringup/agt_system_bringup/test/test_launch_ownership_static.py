from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_navigation_top_level_launch_files_exclude_mission():
    names = sorted(path.name for path in (ROOT / 'launch').glob('*.launch.py'))
    assert names == [
        'debug.launch.py',
        'hardware.launch.py',
        'localization.launch.py',
        'navigation.launch.py',
    ]


def test_unique_owner_inclusions():
    launch_text = {
        path.name: path.read_text(encoding='utf-8')
        for path in (ROOT / 'launch').glob('*.launch.py')
    }
    all_text = '\n'.join(launch_text.values())
    assert all_text.count("'agt_localization_manager', 'localization_manager.launch.py'") == 1
    assert all_text.count("package='agt_pointcloud_preprocessor'") == 1
    assert "FindPackageShare('agt_robot_bringup')" in launch_text['hardware.launch.py']
    assert 'livox_ros_driver2' not in launch_text['hardware.launch.py']
    assert 'bunker_base' not in launch_text['hardware.launch.py']
    assert 'agt_robot_description' not in launch_text['localization.launch.py']
    assert 'agt_robot_description' not in launch_text['navigation.launch.py']
    assert "agt_localization_manager" not in launch_text['debug.launch.py']

    robot_owner = ROOT.parents[2] / 'agt_robot_platform' / 'agt_robot_bringup' / 'launch' / 'robot_hardware.launch.py'
    owner_text = robot_owner.read_text(encoding='utf-8')
    assert owner_text.count("'agt_robot_description', 'display.launch.py'") == 1
    assert owner_text.count("'livox_ros_driver2', 'msg_MID360_launch.py'") == 1
    assert owner_text.count("'bunker_base', 'bunker_base.launch.py'") == 1


def test_navigation_and_inspection_modes_are_explicit():
    mission = (ROOT.parents[2] / 'agt_mission' / 'agt_mission_bringup' / 'launch' /
               'mission.launch.py').read_text(encoding='utf-8')
    hardware = (ROOT.parents[2] / 'agt_robot_platform' / 'agt_robot_bringup' / 'launch' /
                'robot_hardware.launch.py').read_text(encoding='utf-8')
    assert "'enable_legacy_inspection', default_value='false'" in mission
    assert "if value('enable_legacy_inspection').lower() == 'true':" in mission
    assert "'agt_navigation_runtime'," not in mission.split("if value('enable_legacy_inspection')")[0]
    assert "'enable_camera_gimbal', default_value='true'" in hardware


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
        repo / 'navigation/nav2/agt_nav2_bringup/launch/navigation.launch.py'
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
    assert "'rear_filter.enabled': observation['rear_filter_enabled']" in launch
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
