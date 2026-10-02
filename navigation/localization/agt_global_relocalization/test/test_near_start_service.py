"""Isolated callbacks only: no ROS node, driver, native GICP or moving goal."""
from types import SimpleNamespace

import pytest
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time
from agt_robot_interfaces.msg import LocalizationStatus

import agt_global_relocalization.initialization_relocalization as impl
from agt_global_relocalization.start_hint import StartHintError


def fake_node(monkeypatch):
    events, seeds = [], []
    params = dict(start_hint_file='/reviewed/only.yaml', selected_map_id='test',
                  selected_map_version='v1', manual_scan_max_age_sec=0.5,
                  map_frame='map')
    node = SimpleNamespace(mode='auto_then_manual', manual_enabled=False,
                           busy=False, pending_request=False, awaiting_ack=False,
                           initialized=False, _near_hint_attempted=False,
                           phase='WAIT_AUTO_REQUEST',
                           clouds=[SimpleNamespace(header=SimpleNamespace(
                               stamp=Time(sec=100, nanosec=0)))])
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_100_000_000))
    node.resolve_map_inputs = lambda: ('/test/global_map.pcd', '', '', '', 0)
    node._request_readiness = lambda: (True, 'QUERY_READY', 'stationary complete query')
    node._query_pose_to_base_pose = lambda pose, frame: dict(pose, x=pose['x'] + 0.3)
    node.status = lambda *args: events.append(args)

    def run(seed):
        seeds.append(seed)
        node.awaiting_ack = True  # parent run_seeded_once sets this after pose publication
    node.run_seeded_once = run
    pose = dict(x=4.0, y=5.0, z=0.1, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    loaded = []
    def validate(path, **kw):
        loaded.append((path, kw))
        return pose
    monkeypatch.setattr(impl, 'load_start_hint', validate)
    return node, params, events, seeds, loaded


def call(node):
    return impl.InitializationRelocalization.relocalize_near_start(
        node, Trigger.Request(), Trigger.Response())


def test_missing_reviewed_pose_skips_near_start(monkeypatch):
    node, params, _events, seeds, _loaded = fake_node(monkeypatch)
    params['start_hint_file'] = ''
    answer = call(node)
    assert not answer.success and answer.message.startswith('SKIPPED:')
    assert not seeds and not node._near_hint_attempted


def test_map_switch_is_config_error_without_any_pose_publication(monkeypatch):
    node, _params, _events, seeds, _loaded = fake_node(monkeypatch)
    node.resolve_map_inputs = lambda: ('/test/global_map.pcd', '', 'other', 'v1', 5)
    answer = call(node)
    assert not answer.success and answer.message.startswith('CONFIG_ERROR:')
    assert not seeds and not node._near_hint_attempted


def test_changed_pcd_hint_is_rejected_at_point_of_use(monkeypatch):
    node, _params, _events, seeds, _loaded = fake_node(monkeypatch)
    def invalid(*args, **kwargs):
        raise StartHintError('PCD SHA mismatch')
    monkeypatch.setattr(impl, 'load_start_hint', invalid)
    answer = call(node)
    assert not answer.success and 'CONFIG_ERROR:' in answer.message
    assert not seeds and not node._near_hint_attempted


@pytest.mark.parametrize('readiness,now_ns', [
    ((False, 'WAIT_STATIONARY', 'robot moving'), 100_100_000_000),
    ((True, 'QUERY_READY', 'stationary'), 102_000_000_000),
])
def test_near_start_requires_complete_stationary_fresh_query(monkeypatch, readiness, now_ns):
    node, _params, _events, seeds, _loaded = fake_node(monkeypatch)
    node._request_readiness = lambda: readiness
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now_ns))
    answer = call(node)
    assert not answer.success and answer.message.startswith('QUERY_NOT_READY:')
    assert not seeds and not node._near_hint_attempted


def test_reviewed_body_pose_converts_to_base_and_needs_manager_ack(monkeypatch):
    node, _params, events, seeds, loaded = fake_node(monkeypatch)
    answer = call(node)
    assert answer.success and answer.message.startswith('POSE_PROPOSED:')
    assert node._near_hint_attempted and not node.initialized
    assert len(seeds) == 1 and seeds[0].header.frame_id == 'map'
    assert seeds[0].pose.pose.position.x == pytest.approx(4.3)
    assert loaded[0][1] == {'map_id': 'test', 'map_version': 'v1',
                            'map_pcd': '/test/global_map.pcd'}
    assert events[-1][0] == 'NEAR_START_GICP'
    msg = LocalizationStatus()
    msg.state = LocalizationStatus.STATE_LOCALIZED
    msg.global_correction_valid = True
    msg.local_odom_fresh = False
    impl.InitializationRelocalization.on_localization(node, msg)
    assert not node.initialized
    msg.local_odom_fresh = True
    impl.InitializationRelocalization.on_localization(node, msg)
    assert node.initialized and node.phase == 'INITIALIZED'


def test_failed_local_gicp_clears_query_for_global_fallback(monkeypatch):
    node, _params, events, _seeds, _loaded = fake_node(monkeypatch)
    def failure(seed):
        raise RuntimeError('synthetic local GICP rejection')
    node.run_seeded_once = failure
    answer = call(node)
    assert not answer.success and answer.message.startswith('NEAR_START_FAILED:')
    assert node._near_hint_attempted and not node.busy and not node.awaiting_ack
    assert not node.clouds and events[-1][0] == 'NEAR_START_REJECTED'
    assert not call(node).success  # A reviewed hint is still single-use.
