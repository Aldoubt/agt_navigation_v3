"""Initialization policy/orchestration regression; no robot drivers or Nav2."""
import importlib.util
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from agt_robot_interfaces.msg import LocalizationStatus
from agt_global_relocalization.initialization_policy import validate_mode, validate_seed, validate_refinement
from agt_global_relocalization.initialization_relocalization import InitializationRelocalization

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('mode', ['auto', 'auto_then_manual', 'manual'])
def test_modes(mode):
    assert validate_mode(mode) == mode


@pytest.mark.parametrize('mode', ['', 'both', 'AUTO', 'manual_then_auto'])
def test_invalid_modes(mode):
    with pytest.raises(ValueError):
        validate_mode(mode)


def seed():
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.orientation.w = 1.0
    return msg


@pytest.mark.parametrize('problem', ['wrong_frame', 'empty_frame', 'nan', 'zero_q'])
def test_invalid_seed_rejected(problem):
    msg = seed()
    if problem == 'wrong_frame': msg.header.frame_id = 'odom'
    if problem == 'empty_frame': msg.header.frame_id = ''
    if problem == 'nan': msg.pose.pose.position.x = math.nan
    if problem == 'zero_q': msg.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError):
        validate_seed(msg.pose.pose, msg.header.frame_id, 'map')


def pose():
    return dict(x=0., y=0., z=0., qx=0., qy=0., qz=0., qw=1.)


@pytest.mark.parametrize('problem', ['nan_score', 'nan_pose', 'translation', 'yaw', 'overlap'])
def test_bad_refinement_is_rejected(problem):
    initial, refined = pose(), pose()
    fitness, overlap = .1, .8
    if problem == 'nan_score': fitness = math.nan
    if problem == 'nan_pose': refined['x'] = math.inf
    if problem == 'translation': refined['x'] = 2.01
    if problem == 'yaw': refined.update(qz=1., qw=0.)
    if problem == 'overlap': overlap = 1.1
    with pytest.raises(ValueError):
        validate_refinement(initial, refined, fitness, overlap, 2., 30.)


def test_small_refinement_is_allowed():
    refined = pose()
    refined['x'] = .1
    validate_refinement(pose(), refined, .2, .8, 2., 30.)


def fake_controller(**kwargs):
    messages = []
    values = dict(mode='auto_then_manual', busy=False, awaiting_ack=False, initialized=False,
                  pending_request=True, auto_timer=None, manual_enabled=False, clouds=[1, 2],
                  _near_hint_attempted=False,
                  phase='AUTO_SEARCH', status=lambda *a: messages.append(a))
    values.update(kwargs)
    return SimpleNamespace(**values), messages


def test_fallback_cancels_pending_auto_and_clears_old_query():
    node, messages = fake_controller()
    response = InitializationRelocalization.enter_manual(node, None, Trigger.Response())
    assert response.success
    assert not node.pending_request and node.manual_enabled and not node.clouds
    assert node.phase == 'WAIT_MANUAL_INITIAL_POSE'
    assert messages[-1][0] == 'WAIT_MANUAL_INITIAL_POSE'


@pytest.mark.parametrize('updates', [dict(mode='auto'), dict(busy=True), dict(awaiting_ack=True), dict(initialized=True)])
def test_fallback_cannot_override_live_or_accepted_work(updates):
    node, _ = fake_controller(**updates)
    response = InitializationRelocalization.enter_manual(node, None, Trigger.Response())
    assert not response.success
    assert not node.manual_enabled


def test_manual_recovery_does_not_start_bbs():
    node, _ = fake_controller(manual_enabled=True, initialized=True)
    InitializationRelocalization.on_request(node, None)
    assert not node.initialized and not node.pending_request
    assert node.phase == 'WAIT_MANUAL_INITIAL_POSE'


@pytest.mark.parametrize('state,valid,fresh,accepted', [(3, True, True, True), (3, False, True, False), (3, True, False, False), (2, True, True, False)])
def test_manager_ack_required(state, valid, fresh, accepted):
    node, _ = fake_controller(awaiting_ack=True)
    msg = LocalizationStatus()
    msg.state, msg.global_correction_valid, msg.local_odom_fresh = state, valid, fresh
    InitializationRelocalization.on_localization(node, msg)
    assert node.initialized is accepted


def test_unsolicited_seed_after_initialization_is_ignored():
    node, messages = fake_controller(manual_enabled=True, initialized=True)
    InitializationRelocalization.on_manual_seed(node, seed())
    assert messages[-1][0] == 'MANUAL_SEED_REJECTED'


@pytest.mark.parametrize('mode,auto_result,expected,code', [
    ('auto', 0, 'AUTO\n', 0), ('auto', 1, 'AUTO\n', 1),
    ('manual', 0, 'Invalid localization mode', 2),
])
def test_field_shell_dispatch_is_auto_only(mode, auto_result, expected, code):
    helper = ROOT / 'scripts/localization_initialization.sh'
    script = f'''source "{helper}"
LOCALIZATION_MODE={mode}
relocalize_until_ready() {{ echo AUTO; return {auto_result}; }}
initialize_localization
'''
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == code
    assert expected in result.stdout + result.stderr
    if mode == 'manual': assert 'AUTO' not in result.stdout


def test_staged_start_never_launches_navigation_before_initialization():
    source = (ROOT / 'scripts/run_field_stack.sh').read_text()
    assert 'LOCALIZATION_MODE=auto' in source
    assert 'start_child hardware' not in source
    assert source.index('if ! initialize_localization; then') < source.index('start_child navigation')
    assert source.index('if ! initialize_localization; then') < source.index('start_child navigation')
    helper = (ROOT / 'scripts/localization_initialization.sh').read_text()
    assert 'relocalize_until_ready relocalize' in helper
    assert 'wait_for_manual_initialization' not in helper
    assert 'Use RViz 2D Pose Estimate on /initialpose' not in source
    assert '/initialpose' not in helper


def test_legacy_initialization_view_is_not_reachable_from_v1_field_startup():
    text = (ROOT / 'bringup/agt_system_bringup/launch/initialization_view.launch.py').read_text()
    for forbidden in ('controller_server', 'planner_server', 'static_transform_publisher', 'bt_navigator'):
        assert forbidden not in text
    assert "'/agt/initialization/map'" in text
    rviz = (ROOT / 'bringup/agt_system_bringup/config/initialization.rviz').read_text()
    assert 'rviz_default_plugins/SetInitialPose' in rviz
    assert '/initialpose' in rviz
    field = (ROOT / 'scripts/run_field_stack.sh').read_text()
    assert 'initialization_view.launch.py' not in field


def test_v1_selects_only_global_relocalizer(tmp_path, monkeypatch):
    from launch import LaunchContext
    path = ROOT / 'bringup/agt_system_bringup/launch/localization.launch.py'
    spec = importlib.util.spec_from_file_location('mode_localization', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '_include', lambda pkg, file, arguments=None, condition=None: (pkg, file, arguments))
    monkeypatch.setattr(module, 'freeze_fastlio_config', lambda *a: 'frozen-fast.yaml')
    pcd = tmp_path / 'map.pcd'
    pcd.write_text('fixture')
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(global_map=str(pcd), relocalization_assets='', map_id='test', map_version='v1',
        localization_mode='auto', lio_backend='fastlio2', use_sim_time='false', lidar_topic='/livox/lidar', imu_topic='/livox/imu',
        fastlio_config='unused', auto_relocalize='true', query_capture_dir=''))
    actions = module._localization_nodes(ctx)
    relocalizers = [a for a in actions if a[0] == 'agt_global_relocalization']
    assert len(relocalizers) == 1
    assert relocalizers[0][2]['relocalization_executable'] == 'global_relocalization'
    assert relocalizers[0][2]['selected_map_id'] == 'test'
    assert relocalizers[0][2]['selected_map_version'] == 'v1'
    assert relocalizers[0][2]['auto_request'] == 'true'


@pytest.mark.parametrize('mode', ['auto_then_manual', 'manual'])
def test_v1_system_localization_rejects_seeded_modes_before_starting_nodes(mode, tmp_path):
    from launch import LaunchContext
    path = ROOT / 'bringup/agt_system_bringup/launch/localization.launch.py'
    spec = importlib.util.spec_from_file_location('mode_localization_reject', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pcd = tmp_path / 'map.pcd'
    pcd.write_text('fixture')
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(
        global_map=str(pcd), relocalization_assets='', map_id='test', map_version='v1',
        localization_mode=mode))
    with pytest.raises(RuntimeError, match='automatic global relocalization'):
        module._localization_nodes(ctx)

@pytest.mark.parametrize('problem', ['zero_stamp', 'stale_seed', 'future_seed', 'stale_scan', 'moving'])
def test_manual_callback_rejects_before_native_work(problem):
    node, messages = fake_controller(manual_enabled=True)
    msg = seed()
    msg.header.stamp.sec = 100
    scan = seed()
    scan.header.stamp.sec = 100
    node.clouds = [scan]
    params = dict(map_frame='map', manual_seed_max_age_sec=30., manual_scan_max_age_sec=.5)
    node.get_parameter = lambda key: SimpleNamespace(value=params[key])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
    node._request_readiness = lambda: (problem != 'moving', 'WAIT_STATIONARY', 'must remain stationary')
    if problem == 'zero_stamp': msg.header.stamp.sec = 0
    if problem == 'stale_seed': msg.header.stamp.sec = 1
    if problem == 'future_seed': msg.header.stamp.sec = 101
    if problem == 'stale_scan': scan.header.stamp.sec = 99
    InitializationRelocalization.on_manual_seed(node, msg)
    assert messages[-1][0] == 'MANUAL_SEED_REJECTED'


def test_auto_helper_failure_does_not_release_nav_gate_or_fallback():
    helper = ROOT / 'scripts/localization_initialization.sh'
    script = f'''source "{helper}"
LOCALIZATION_MODE=auto
relocalize_until_ready() {{ echo AUTO; return 1; }}
initialize_localization
'''
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == 1
    assert 'AUTO' in result.stdout
    assert 'MANUAL' not in result.stdout and '/initialpose' not in result.stdout


def test_invalid_localization_mode_fails_closed_before_global_search():
    helper = ROOT / 'scripts/localization_initialization.sh'
    script = f'''source "{helper}"
LOCALIZATION_MODE=manual
relocalize_until_ready() {{ echo GLOBAL_SHOULD_NOT_RUN; }}
initialize_localization
'''
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == 2
    assert 'SHOULD_NOT_RUN' not in result.stdout
