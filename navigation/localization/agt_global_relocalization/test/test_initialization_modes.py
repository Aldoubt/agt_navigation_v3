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
    ('auto_then_manual', 0, 'AUTO\n', 0),
    ('auto_then_manual', 1, 'MANUAL', 0), ('manual', 0, 'MANUAL', 0),
])
def test_shell_dispatch(mode, auto_result, expected, code):
    helper = ROOT / 'scripts/localization_initialization.sh'
    script = f'''source "{helper}"
LOCALIZATION_MODE={mode}
relocalize_until_ready() {{ echo AUTO; return {auto_result}; }}
wait_for_manual_initialization() {{ echo MANUAL; }}
initialize_localization
'''
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == code
    assert expected in result.stdout
    if mode == 'manual': assert 'AUTO' not in result.stdout
    if mode == 'auto' or auto_result == 0 and mode != 'manual': assert 'MANUAL' not in result.stdout


def test_staged_start_never_launches_navigation_before_initialization():
    source = (ROOT / 'scripts/run_field_stack.sh').read_text()
    assert 'LOCALIZATION_MODE=auto' in source
    assert source.index('if ! initialize_localization; then') < source.index('start_child navigation')
    helper = (ROOT / 'scripts/localization_initialization.sh').read_text()
    assert 'wait_for_localization_settle 8' in helper
    assert 'stop_process_group initialization_view' in helper
    assert 'wait_for_manual_initialization' in helper


def test_initialization_view_has_no_navigation_control_or_identity_tf():
    text = (ROOT / 'bringup/agt_system_bringup/launch/initialization_view.launch.py').read_text()
    for forbidden in ('controller_server', 'planner_server', 'static_transform_publisher', 'bt_navigator'):
        assert forbidden not in text
    assert "'/agt/initialization/map'" in text
    rviz = (ROOT / 'bringup/agt_system_bringup/config/initialization.rviz').read_text()
    assert 'rviz_default_plugins/SetInitialPose' in rviz
    assert '/initialpose' in rviz


@pytest.mark.parametrize('mode,exe', [('auto','global_relocalization'), ('auto_then_manual','initialization_relocalization'), ('manual','initialization_relocalization')])
def test_one_relocalizer_selected(mode, exe, tmp_path, monkeypatch):
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
        localization_mode=mode, lio_backend='fastlio2', use_sim_time='false', lidar_topic='/livox/lidar', imu_topic='/livox/imu',
        fastlio_config='unused', auto_relocalize='true', query_capture_dir=''))
    actions = module._localization_nodes(ctx)
    relocalizers = [a for a in actions if a[0] == 'agt_global_relocalization']
    assert len(relocalizers) == 1
    assert relocalizers[0][2]['relocalization_executable'] == exe
    if mode != 'auto': assert relocalizers[0][2]['auto_request'] == 'false'

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


@pytest.mark.parametrize('accepted', [False, True])
def test_waiting_helper_does_not_release_nav_gate_on_rejection_and_is_cancellable(tmp_path, accepted):
    import signal
    import time
    helper = ROOT / 'scripts/localization_initialization.sh'
    script = f'''source "{helper}"
trap 'exit 130' INT TERM
RUN_DIR="{tmp_path}"
NAV_MAP=unused
ENABLE_RVIZ=false
CHILD_PIDS=()
CHILD_LABELS=()
wait_for_service() {{ return 0; }}
ros2() {{ echo 'success=True'; }}
timeout() {{ shift; "$@"; }}
start_child() {{ CHILD_PIDS+=("$$"); CHILD_LABELS+=("$1"); }}
wait_for_topic() {{ return 0; }}
wait_for_localization_settle() {{ sleep .05; return {0 if accepted else 1}; }}
stop_process_group() {{ echo VIEW_STOPPED; }}
wait_for_manual_initialization && echo PASS_NAV_GATE
'''
    process = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        if not accepted:
            time.sleep(.3)
            assert process.poll() is None
            process.send_signal(signal.SIGINT)
        out, err = process.communicate(timeout=5)
        assert '[WAIT_MANUAL_INITIAL_POSE]' in out
        assert ('PASS_NAV_GATE' in out) is accepted
        if accepted:
            assert 'VIEW_STOPPED' in out
            assert process.returncode == 0
        else:
            assert process.returncode == 130
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
