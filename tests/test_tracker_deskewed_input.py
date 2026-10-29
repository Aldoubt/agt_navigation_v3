"""Tracker input contracts, using in-memory messages and no ROS context.

Only numpy and pytest are required. ROS imports are scoped fakes; the actual
tracker callbacks and query assembly are loaded from this checkout. These tests
do not initialize ROS, create a node, spawn the native matcher, or touch hardware.
"""

from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


@pytest.fixture
def tracker_module(monkeypatch):
    def install(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    class Time:
        def __init__(self, *, nanoseconds=0):
            self.nanoseconds = nanoseconds

        @classmethod
        def from_msg(cls, msg):
            return cls(nanoseconds=msg.sec * 1_000_000_000 + msg.nanosec)

        def to_msg(self):
            return stamp(self.nanoseconds)

    time_module = install('rclpy.time', Time=Time)
    install('rclpy', time=time_module)
    install('rclpy.duration', Duration=lambda **kwargs: SimpleNamespace(**kwargs))
    install('rclpy.node', Node=object)
    for package, names in (
        ('geometry_msgs', ['PoseWithCovarianceStamped']),
        ('nav_msgs', ['Odometry']),
        ('sensor_msgs', ['PointCloud2']),
        ('std_msgs', ['String']),
    ):
        messages = install(package + '.msg', **{name: type(name, (), {}) for name in names})
        install(package, msg=messages)
    def pose_message():
        return SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()),
            covariance=[0.] * 36))

    sys.modules['geometry_msgs.msg'].PoseWithCovarianceStamped = pose_message
    point_cloud = install('sensor_msgs_py.point_cloud2',
                          read_points=lambda msg, **kwargs: iter(msg.points))
    install('sensor_msgs_py', point_cloud2=point_cloud)
    install('tf2_ros', Buffer=object, TransformListener=object,
            TransformException=type('TransformException', (Exception,), {}))
    messages = install('agt_robot_interfaces.msg', LocalizationStatus=SimpleNamespace(
        STATE_LOCALIZED=2, STATE_DEGRADED=4))
    install('agt_robot_interfaces', msg=messages)
    source = (Path(__file__).resolve().parents[1] / 'navigation/localization/'
              'agt_map_tracker/agt_map_tracker/map_tracker.py')
    spec = importlib.util.spec_from_file_location('_tracker_deskew_contract_test', source)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def pose_matrix(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                              np.sin(pitch), np.cos(yaw), np.sin(yaw))
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def apply(matrix, points):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def odom(module, ns, matrix):
    position, quaternion = module.matrix_pose(matrix)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns), frame_id='odom'),
        child_frame_id='base_link',
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(**dict(zip(('x', 'y', 'z'), position))),
            orientation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), quaternion))))))


def cloud(ns, points, frame='odom'):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns), frame_id=frame),
                           points=np.asarray(points, dtype=float))


def transform(module, matrix=None, ns=10_000_000_000):
    position, quaternion = module.matrix_pose(np.eye(4) if matrix is None else matrix)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(**dict(zip(('x', 'y', 'z'), position))),
            rotation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), quaternion)))))


@pytest.fixture
def tracker(tracker_module):
    node = object.__new__(tracker_module.MapTracker)
    parameters = {
        'scan_input_mode': 'odom_deskewed', 'scan_topic': '/fastlio2/world_cloud',
        'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
        'odom_match_max_skew_sec': .10, 'cloud_pairing_max_lag_sec': .20,
        'max_reference_cloud_age_sec': .20, 'max_future_cloud_sec': .05,
        'tf_timeout_sec': .10, 'query_accumulate_clouds': 3,
        'query_max_span_sec': .30,
        'query_voxel_leaf_m': 0., 'min_query_points': 1,
        'local_map_radius_xy_m': 20., 'local_map_half_height_m': 5.,
        'global_map': '', 'degraded_after_rejects': 2, 'recovery_after_rejects': 4,
        'max_points': 150000, 'save_debug_cloud': False, 'native_timeout_sec': 8.,
        'max_debug_failure_samples': 20,
        'reject_degenerate_hessian': False, 'max_hessian_condition_number': 1.e10,
        'max_fitness': .50, 'min_overlap': .40,
        'max_translation_innovation_m': .50, 'max_yaw_innovation_deg': 5.,
        'apply_correction': True,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None,
                                              warn=lambda *a, **k: None)
    node._odom = deque(maxlen=6000)
    node._clouds = deque(maxlen=20)
    node._pending_clouds = deque(maxlen=20)
    node._latest_input_stamp_ns = 0
    node._last_cloud_stamp_ns = 0
    node._last_attempted_stamp_ns = 0
    node._cloud_time_sync_errors = node._cloud_tf_errors = 0
    node._attempt_id = node._consecutive_rejects = 0
    node._localized = node._correction_valid = True
    node._last_run_ns = 0
    node._debug_failure_count = 0
    node._debug_run_directory = None
    calls, statuses, poses = [], [], []
    clock = SimpleNamespace(nanoseconds=None)

    def now():
        ns = clock.nanoseconds
        if ns is None:
            ns = (node._clouds[-1].stamp_ns if node._clouds else 10_000_000_000) + 10_000_000
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp(ns))

    node.get_clock = lambda: SimpleNamespace(now=now)

    def lookup(*args, **kwargs):
        calls.append((args, kwargs))
        return transform(tracker_module)

    node._tf = SimpleNamespace(lookup_transform=lookup)
    node._pose_pub = SimpleNamespace(publish=poses.append)
    node._publish_status = lambda state, **fields: statuses.append({'state': state, **fields})
    return SimpleNamespace(node=node, module=tracker_module, parameters=parameters,
                           tf_calls=calls, statuses=statuses, poses=poses, clock=clock)


@pytest.mark.parametrize('cloud_first', [False, True])
def test_exact_pair_handles_both_arrival_orders_and_nonzero_extrinsics(tracker, cloud_first):
    node, module = tracker.node, tracker.module
    ns = 10_000_000_000
    # Independently construct the world points and canonical base pose from a
    # tilted sensor/body and a nonidentity mounting transform.
    odom_body = pose_matrix((62., -31., 4.), (.18, -.12, .75))
    body_base = pose_matrix((.25, -.14, .48), (-.08, .30, -.20))
    body_points = np.array([[2., 1., .4], [4., -2., 1.2], [-1., 3., -.6]])
    canonical = odom_body @ body_base
    expected = apply(np.linalg.inv(body_base), body_points)
    scan = cloud(ns, apply(odom_body, body_points))
    position = odom(module, ns, canonical)
    callbacks = [(node._on_cloud, scan), (node._on_odom, position)]
    if not cloud_first:
        callbacks.reverse()
    callbacks[0][0](callbacks[0][1])
    assert not node._clouds
    callbacks[1][0](callbacks[1][1])
    assert len(node._clouds) == 1
    sample = node._clouds[-1]
    np.testing.assert_allclose(sample.points_base, expected, atol=1e-12)
    np.testing.assert_allclose(sample.odom_pose, canonical, atol=1e-12)
    assert sample.stamp_ns == sample.odom_stamp_ns == ns
    assert sample.source_frame == 'odom'
    assert not node._pending_clouds
    assert not tracker.tf_calls, 'world-to-base must use the paired canonical pose, not live TF'
    assert node._cloud_time_sync_errors == node._consecutive_rejects == 0


def test_range_filter_is_in_base_coordinates_far_from_odom_origin(tracker):
    ns = 10_000_000_000
    canonical = pose_matrix((101., -72., 13.), (.25, -.3, .9))
    points_base = np.array([[1., 2., 3.], [29., 0., 0.], [.1, 0., 0.], [31., 0., 0.]])
    tracker.node._on_odom(odom(tracker.module, ns, canonical))
    tracker.node._on_cloud(cloud(ns, apply(canonical, points_base)))
    np.testing.assert_allclose(tracker.node._clouds[-1].points_base, points_base[:2], atol=1e-12)


@pytest.mark.parametrize('offset_ns', [-1, 1, 50_000_000])
def test_nearby_odom_is_never_substituted_for_exact_pair(tracker, offset_ns):
    node, ns = tracker.node, 10_000_000_000
    node._on_cloud(cloud(ns, [[1., 2., 0.]]))
    node._on_odom(odom(tracker.module, ns + offset_ns, pose_matrix((10., 0., 0.))))
    assert not node._clouds
    assert len(node._pending_clouds) == 1
    assert node._cloud_time_sync_errors == 0
    node._on_odom(odom(tracker.module, ns, np.eye(4)))
    np.testing.assert_allclose(node._clouds[-1].points_base, [[1., 2., 0.]])
    assert node._clouds[-1].odom_stamp_ns == ns


def test_unmatched_cloud_expires_by_sensor_watermark_without_gicp_reject(tracker):
    node, ns = tracker.node, 10_000_000_000
    node._consecutive_rejects = 2
    node._on_cloud(cloud(ns, [[1., 0., 0.]]))
    # Merely retrying a drain cannot expire a cloud while sensor time is still.
    for _ in range(4):
        node._drain_pending_clouds()
    assert len(node._pending_clouds) == 1
    node._on_odom(odom(tracker.module, ns + 200_000_001, np.eye(4)))
    assert not node._pending_clouds
    assert not node._clouds
    assert node._cloud_time_sync_errors == 1
    assert node._attempt_id == 0
    assert node._consecutive_rejects == 2
    assert not tracker.statuses


def test_exact_sample_is_used_before_watermark_expiry(tracker):
    node, ns = tracker.node, 10_000_000_000
    node._pending_clouds.append(cloud(ns, [[1., 0., 0.]]))
    node._odom.append(tracker.module.OdomSample(ns, np.eye(4)))
    node._latest_input_stamp_ns = ns + 2_000_000_000
    node._drain_pending_clouds()
    assert len(node._clouds) == 1
    assert not node._pending_clouds
    assert node._cloud_time_sync_errors == 0


@pytest.mark.parametrize('wrong_frame', ['map', 'body', 'base_link'])
def test_world_mode_rejects_wrong_frame_without_tf_or_gicp(tracker, wrong_frame):
    node, ns = tracker.node, 10_000_000_000
    node._on_odom(odom(tracker.module, ns, np.eye(4)))
    node._on_cloud(cloud(ns, [[1., 0., 0.]], wrong_frame))
    assert not node._clouds
    assert not node._pending_clouds
    assert not tracker.tf_calls
    assert node._consecutive_rejects == node._attempt_id == 0


def test_pending_queue_is_bounded_without_advancing_gicp_state(tracker):
    node = tracker.node
    for index in range(45):
        node._on_cloud(cloud(10_000_000_000 + index * 1_000_000, [[1., 0., 0.]]))
    assert 0 < len(node._pending_clouds) <= 20
    assert node._consecutive_rejects == node._attempt_id == 0
    assert not node._clouds


def test_duplicate_and_late_clouds_cannot_roll_reference_backward(tracker):
    node, first, second = tracker.node, 10_000_000_000, 10_100_000_000
    node._on_cloud(cloud(first, [[1., 0., 0.]]))
    node._on_odom(odom(tracker.module, second, np.eye(4)))
    node._on_cloud(cloud(second, [[2., 0., 0.]]))
    assert node._clouds[-1].stamp_ns == second
    # A previously pending older scan becomes pairable only after the new scan.
    node._on_odom(odom(tracker.module, first, np.eye(4)))
    node._on_cloud(cloud(second, [[99., 0., 0.]]))
    node._on_cloud(cloud(first, [[1., 0., 0.]]))
    assert [sample.stamp_ns for sample in node._clouds] == [second]
    assert node._consecutive_rejects == node._attempt_id == 0


def test_two_moving_frames_aggregate_same_static_scene_in_latest_base(tracker, monkeypatch):
    node = tracker.node
    world_points = np.array([[8., 2., .5], [9., 1., 1.], [7., -2., .2]])
    first_pose = pose_matrix((4., 0., 0.), (.05, -.1, .4))
    last_pose = pose_matrix((5., .5, .1), (-.1, .1, .65))
    for ns, pose in [(10_000_000_000, first_pose), (10_100_000_000, last_pose)]:
        node._on_odom(odom(tracker.module, ns, pose))
        node._on_cloud(cloud(ns, world_points))
    assembled = []

    def capture_query(points, leaf):
        assembled.append(points.copy())
        return points

    monkeypatch.setattr(node, '_voxel', capture_query)
    node._track()  # Stops at the missing-map check, before native execution.
    expected = apply(np.linalg.inv(last_pose), world_points)
    np.testing.assert_allclose(assembled[0], np.concatenate([expected, expected]), atol=1e-12)
    assert tracker.statuses[-1]['reason'] == 'global_map_missing'


def test_sensor_mode_retains_actual_nearest_odom_stamp_in_attempt_diagnostics(tracker):
    node, ns, actual_ns = tracker.node, 10_000_000_000, 9_980_000_000
    tracker.parameters['scan_input_mode'] = 'sensor'
    tracker.parameters['scan_topic'] = '/agt/livox/points'
    node._on_odom(odom(tracker.module, actual_ns, np.eye(4)))
    node._on_cloud(cloud(ns, [[1., 2., 0.]], 'livox_frame'))
    sample = node._clouds[-1]
    assert sample.stamp_ns == ns
    assert sample.odom_stamp_ns == actual_ns
    assert sample.source_frame == 'livox_frame'
    node._track()  # Capture the real per-attempt fields without native execution.
    fields = tracker.statuses[-1]
    assert fields['reference_cloud_stamp_ns'] == ns
    assert fields['odom_pose_source_stamp_ns'] == actual_ns
    assert fields['odom_pose_source_stamp_ns'] != fields['reference_cloud_stamp_ns']


def test_cloud_sample_keeps_three_argument_constructor_compatible(tracker_module):
    sample = tracker_module.CloudSample(10, np.zeros((1, 3)), np.eye(4))
    assert sample.stamp_ns == 10
    assert hasattr(sample, 'odom_stamp_ns')
    assert hasattr(sample, 'source_frame')


def fake_native(tracker, tmp_path, monkeypatch, *, success=True,
                result_overrides=None, stdout=None, exception=None):
    """Exercise the complete decision path while replacing process creation."""
    map_path = tmp_path / 'map.pcd'
    map_path.write_text('test fixture; never read by a native process\n')
    tracker.parameters['global_map'] = str(map_path)
    calls = []
    result = {
        'success': success, 'message': 'GICP did not converge',
        'fitness': .01, 'overlap': 1., 'map_points': 2000,
        'hessian_eigenvalues': [1., 2., 3., 4., 5., 6.],
        'hessian_condition_number': 6., 'hessian_degenerate': False,
        'x': 0., 'y': 0., 'z': 0., 'qx': 0., 'qy': 0., 'qz': 0., 'qw': 1.,
    }
    result.update(result_overrides or {})

    def run(*args, **kwargs):
        calls.append((args, kwargs))
        command = args[0]
        if '--debug-crop' in command:
            crop = Path(command[command.index('--debug-crop') + 1])
            crop.write_text('stub native crop\n')
        if exception is not None:
            raise exception
        return SimpleNamespace(returncode=0 if success else 1,
                               stdout=json.dumps(result) if stdout is None else stdout, stderr='')

    monkeypatch.setattr(tracker.module.subprocess, 'run', run)
    return calls


@pytest.mark.parametrize('success', [False, True])
def test_deskewed_reference_is_attempted_only_once_after_native_decision(
        tracker, tmp_path, monkeypatch, success):
    node, ns = tracker.node, 10_000_000_000
    calls = fake_native(tracker, tmp_path, monkeypatch, success=success)
    node._on_odom(odom(tracker.module, ns, np.eye(4)))
    node._on_cloud(cloud(ns, [[1., 2., 0.]]))
    node._track()
    assert len(calls) == node._attempt_id == 1
    assert node._last_attempted_stamp_ns == ns
    assert tracker.statuses[-1]['decision'] == ('ACCEPT' if success else 'REJECT')
    rejects, poses = node._consecutive_rejects, len(tracker.poses)
    for _ in range(3):
        node._track()
        assert tracker.statuses[-1]['state'] == 'WAIT_QUERY'
        assert tracker.statuses[-1].get('decision') != 'REJECT'
    assert len(calls) == node._attempt_id == 1
    assert node._consecutive_rejects == rejects
    assert len(tracker.poses) == poses == int(success)


@pytest.mark.parametrize('age_ns', [200_000_001, -50_000_001])
def test_stale_or_future_reference_waits_without_native_pose_or_reject(
        tracker, tmp_path, monkeypatch, age_ns):
    node, ns = tracker.node, 10_000_000_000
    calls = fake_native(tracker, tmp_path, monkeypatch)
    node._consecutive_rejects = 2
    node._on_odom(odom(tracker.module, ns, np.eye(4)))
    node._on_cloud(cloud(ns, [[1., 2., 0.]]))
    tracker.clock.nanoseconds = ns + age_ns
    node._track()
    assert tracker.statuses[-1]['state'] == 'WAIT_QUERY'
    assert tracker.statuses[-1].get('decision') != 'REJECT'
    assert node._attempt_id == node._last_attempted_stamp_ns == 0
    assert node._consecutive_rejects == 2
    assert not calls
    assert not tracker.poses


@pytest.mark.parametrize('age_ns', [200_000_000, -50_000_000])
def test_reference_exactly_at_freshness_boundary_can_attempt(
        tracker, tmp_path, monkeypatch, age_ns):
    node, ns = tracker.node, 10_000_000_000
    calls = fake_native(tracker, tmp_path, monkeypatch)
    node._on_odom(odom(tracker.module, ns, np.eye(4)))
    node._on_cloud(cloud(ns, [[1., 2., 0.]]))
    tracker.clock.nanoseconds = ns + age_ns
    node._track()
    assert len(calls) == node._attempt_id == 1
    assert tracker.statuses[-1]['decision'] == 'ACCEPT'


def test_newer_fresh_reference_can_attempt_after_previous_reference(tracker, tmp_path, monkeypatch):
    calls = fake_native(tracker, tmp_path, monkeypatch)
    for index, ns in enumerate([10_000_000_000, 10_100_000_000], start=1):
        tracker.node._on_odom(odom(tracker.module, ns, np.eye(4)))
        tracker.node._on_cloud(cloud(ns, [[1., 2., 0.]]))
        tracker.node._track()
        assert len(calls) == tracker.node._attempt_id == index
        assert tracker.node._last_attempted_stamp_ns == ns
    assert len(tracker.poses) == 2


def test_sensor_mode_retains_previous_repeat_and_age_behavior(tracker, tmp_path, monkeypatch):
    calls = fake_native(tracker, tmp_path, monkeypatch, success=False)
    tracker.parameters['scan_input_mode'] = 'sensor'
    ns = 10_000_000_000
    tracker.node._on_odom(odom(tracker.module, ns, np.eye(4)))
    tracker.node._on_cloud(cloud(ns, [[1., 2., 0.]], 'livox_frame'))
    tracker.clock.nanoseconds = ns + 2_000_000_000
    for _ in range(2):
        tracker.node._track()
    assert len(calls) == tracker.node._attempt_id == tracker.node._consecutive_rejects == 2


@pytest.mark.parametrize('after_crop', [False, True])
@pytest.mark.parametrize('prior_rejects', [0, 1, 2, 3])
def test_sparse_input_is_not_a_pose_rejection_or_recovery_request(
        tracker, tmp_path, monkeypatch, after_crop, prior_rejects):
    node = tracker.node
    calls = fake_native(tracker, tmp_path, monkeypatch)
    tracker.parameters.update(min_query_points=2, query_accumulate_clouds=1)
    node._consecutive_rejects = prior_rejects
    points = [[1., 0., 0.], [21., 0., 0.]] if after_crop else [[1., 0., 0.]]
    for index in range(6):
        ns = 10_000_000_000 + index * 100_000_000
        node._on_odom(odom(tracker.module, ns, np.eye(4)))
        node._on_cloud(cloud(ns, points))
        node._track()
        status = tracker.statuses[-1]
        assert status['state'] == 'INPUT_INSUFFICIENT'
        assert status['decision'] == 'NOT_ATTEMPTED'
        assert status['reason_codes'] == ['MIN_QUERY_POINTS']
        assert status['reason'] == ('min_query_points_after_map_crop' if after_crop
                                    else 'min_query_points')
        assert status['consecutive_rejects'] == prior_rejects
        assert status['query_points'] == 1
        assert status['min_query_points'] == 2
        assert status['odom_pose_source_stamp_ns'] == ns
        assert status['odom_cloud_skew_sec'] == 0.
        if after_crop:
            assert status['query_points_before_map_crop'] == 2
        assert node._consecutive_rejects == prior_rejects
    assert node._attempt_id == 6
    assert not calls and not tracker.poses


def test_valid_registration_after_sparse_input_restores_normal_acceptance(
        tracker, tmp_path, monkeypatch):
    node = tracker.node
    calls = fake_native(tracker, tmp_path, monkeypatch)
    tracker.parameters.update(min_query_points=2, query_accumulate_clouds=1)
    node._consecutive_rejects = 1
    for index, points in enumerate(([[1., 0., 0.]], [[1., 0., 0.], [2., 0., 0.]])):
        ns = 10_000_000_000 + index * 100_000_000
        node._on_odom(odom(tracker.module, ns, np.eye(4)))
        node._on_cloud(cloud(ns, points))
        node._track()
    assert len(calls) == len(tracker.poses) == 1
    assert tracker.statuses[-1]['state'] == 'TRACKING_OK'
    assert tracker.statuses[-1]['decision'] == 'ACCEPT'
    assert node._consecutive_rejects == 0


def test_sparse_input_cannot_erase_real_registration_failure_history(
        tracker, tmp_path, monkeypatch):
    node = tracker.node
    calls = fake_native(tracker, tmp_path, monkeypatch, success=False)
    tracker.parameters.update(min_query_points=2, query_accumulate_clouds=1)
    points = [[1., 0., 0.], [2., 0., 0.]]
    # The fourth actual rejection must still trigger the unchanged recovery
    # policy despite insufficient input between real registration attempts.
    for index in range(8):
        ns = 10_000_000_000 + index * 100_000_000
        node._on_odom(odom(tracker.module, ns, np.eye(4)))
        node._on_cloud(cloud(ns, points if index % 2 else points[:1]))
        node._track()
    assert len(calls) == node._consecutive_rejects == 4
    assert tracker.statuses[-1]['state'] == 'RECOVERY_REQUIRED'
    assert tracker.statuses[-1]['decision'] == 'REJECT'
    assert not tracker.poses


def test_pending_only_input_waits_with_sync_diagnostics(tracker, tmp_path, monkeypatch):
    calls = fake_native(tracker, tmp_path, monkeypatch)
    tracker.node._on_cloud(cloud(10_000_000_000, [[1., 2., 0.]]))
    tracker.node._track()
    status = tracker.statuses[-1]
    assert status['state'] == 'WAIT_QUERY'
    assert status['pending_cloud_count'] == 1
    assert status['cloud_time_sync_error_count'] == 0
    assert not calls and not tracker.poses
    assert tracker.node._attempt_id == tracker.node._consecutive_rejects == 0


@pytest.mark.parametrize('mode, stamps, expected_indices', [
    ('odom_deskewed', [9_000_000_000, 9_100_000_000, 10_000_000_000], [2]),
    ('odom_deskewed', [9_800_000_000, 9_900_000_000, 10_000_000_000], [0, 1, 2]),
    ('odom_deskewed', [9_700_000_000, 9_800_000_000, 10_000_000_000], [0, 1, 2]),
    ('odom_deskewed', [9_699_999_999, 9_800_000_000, 10_000_000_000], [1, 2]),
    ('sensor', [9_000_000_000, 9_100_000_000, 10_000_000_000], [0, 1, 2]),
])
def test_query_span_omits_pre_gap_clouds_and_reports_only_selected_timing(
        tracker, monkeypatch, mode, stamps, expected_indices):
    tracker.parameters['scan_input_mode'] = mode
    for index, ns in enumerate(stamps):
        tracker.node._on_odom(odom(tracker.module, ns, np.eye(4)))
        tracker.node._on_cloud(cloud(ns, [[index + 1., 1., 0.]]))
    assembled = []

    def capture(points, leaf):
        assembled.append(points.copy())
        return points

    monkeypatch.setattr(tracker.node, '_voxel', capture)
    tracker.node._track()
    np.testing.assert_allclose(assembled[0], [[index + 1., 1., 0.] for index in expected_indices])
    status = tracker.statuses[-1]
    assert status['reason'] == 'global_map_missing'
    assert [item['cloud_stamp_ns'] for item in status['query_cloud_timing']] == [
        stamps[index] for index in expected_indices]
    assert [item['odom_stamp_ns'] for item in status['query_cloud_timing']] == [
        stamps[index] for index in expected_indices]


def enable_debug(tracker, tmp_path):
    root = tmp_path / 'debug'
    tracker.parameters.update(save_debug_cloud=True, debug_directory=str(root),
                              apply_correction=True)
    return root


def receive_and_track(tracker, ns):
    tracker.node._on_odom(odom(tracker.module, ns, np.eye(4)))
    tracker.node._on_cloud(cloud(ns, [[1., 2., 0.]]))
    tracker.node._track()


@pytest.mark.parametrize('prior_failure', [False, True])
def test_success_cleans_pending_debug_without_erasing_previous_failure(
        tracker, tmp_path, monkeypatch, prior_failure):
    root = enable_debug(tracker, tmp_path)
    retained_before = {}
    if prior_failure:
        fake_native(tracker, tmp_path, monkeypatch, success=False)
        receive_and_track(tracker, 9_900_000_000)
        retained_before = {path.relative_to(root): path.read_bytes()
                           for path in root.rglob('*') if path.is_file()}
    calls = fake_native(tracker, tmp_path, monkeypatch)
    receive_and_track(tracker, 10_000_000_000)
    assert len(calls) == len(tracker.poses) == 1
    assert tracker.statuses[-1]['decision'] == 'ACCEPT'
    assert tracker.statuses[-1]['correction_applied'] is True
    run = tracker.node._debug_run_directory
    assert run.parent == root and run.name.startswith('run_')
    assert not list(root.rglob('.pending_*'))
    assert len(list(run.glob('attempt_*'))) == int(prior_failure)
    assert tracker.node._debug_failure_count == int(prior_failure)
    assert {path.relative_to(root): path.read_bytes() for path in root.rglob('*')
            if path.is_file()} == retained_before


@pytest.mark.parametrize('case, expected_code', [
    ('nonconverged', 'GICP_NOT_CONVERGED'),
    ('innovation', 'INNOVATION_HIGH'),
    ('quality', 'FITNESS_LOW'),
    ('malformed_json', 'BACKEND_PROTOCOL_ERROR'),
    ('timeout', 'BACKEND_PROTOCOL_ERROR'),
])
def test_debug_failure_retains_query_seed_crop_and_result(
        tracker, tmp_path, monkeypatch, case, expected_code):
    root = enable_debug(tracker, tmp_path)
    options = {
        'nonconverged': {'success': False},
        'innovation': {'result_overrides': {'x': .80}},
        'quality': {'result_overrides': {'fitness': .80}},
        'malformed_json': {'stdout': 'native output was not JSON'},
        'timeout': {'exception': tracker.module.subprocess.TimeoutExpired('stub', 8.)},
    }
    calls = fake_native(tracker, tmp_path, monkeypatch, **options[case])
    receive_and_track(tracker, 10_000_000_000)
    assert len(calls) == 1
    assert tracker.statuses[-1]['decision'] == 'REJECT'
    assert expected_code in tracker.statuses[-1]['reason_codes']
    assert not tracker.poses
    runs = list(root.glob('run_*'))
    assert len(runs) == 1
    artifacts = list(runs[0].glob('attempt_*'))
    assert len(artifacts) == 1
    assert not list(root.rglob('.pending_*'))
    assert {path.name for path in artifacts[0].iterdir()} == {
        'query_cloud.pcd', 'initial_pose.yaml', 'registration_result.yaml', 'local_map_crop.pcd'}
    query = (artifacts[0] / 'query_cloud.pcd').read_text()
    assert 'POINTS 1' in query and '1.000000 2.000000 0.000000' in query
    seed = json.loads((artifacts[0] / 'initial_pose.yaml').read_text())
    result = json.loads((artifacts[0] / 'registration_result.yaml').read_text())
    assert seed['reference_cloud_stamp_ns'] == result['reference_cloud_stamp_ns'] == 10_000_000_000
    assert result['odom_pose_source_stamp_ns'] == 10_000_000_000
    assert result['attempt_id'] == 1
    if case in {'malformed_json', 'timeout'}:
        assert result['backend_error']
    else:
        assert 'backend' in result


def test_debug_failure_ring_keeps_only_twenty_most_recent_failures(tracker, tmp_path, monkeypatch):
    root = enable_debug(tracker, tmp_path)
    calls = fake_native(tracker, tmp_path, monkeypatch, success=False)
    for index in range(22):
        receive_and_track(tracker, 10_000_000_000 + index * 100_000_000)
    assert len(calls) == tracker.node._debug_failure_count == 22
    runs = list(root.glob('run_*'))
    assert len(runs) == 1
    artifacts = list(runs[0].glob('attempt_*'))
    assert len(artifacts) == 20
    assert not list(root.rglob('.pending_*'))
    recorded = [json.loads((path / 'registration_result.yaml').read_text())
                for path in artifacts]
    assert sorted(item['attempt_id'] for item in recorded) == list(range(3, 23))
    assert all((path / 'query_cloud.pcd').is_file() for path in artifacts)


def test_new_debug_session_uses_distinct_directory_and_preserves_previous_run(
        tracker, tmp_path, monkeypatch):
    root = enable_debug(tracker, tmp_path)
    fake_native(tracker, tmp_path, monkeypatch, success=False)
    receive_and_track(tracker, 10_000_000_000)
    first_run = tracker.node._debug_run_directory
    saved = {path.relative_to(root): path.read_bytes()
             for path in first_run.rglob('*') if path.is_file()}
    # A second in-memory instance exercises a fresh process's zeroed debug state
    # without constructing a ROS node or starting a process.
    second_node = object.__new__(tracker.module.MapTracker)
    second_node.get_parameter = tracker.node.get_parameter
    second_node._debug_run_directory = None
    second_node._debug_failure_count = 0
    second_node._attempt_id = 1
    pending = second_node._debug_attempt_dir()
    (pending / 'registration_result.yaml').write_text('{"second_run": true}\n')
    second_node._retain_debug_artifact(pending)
    assert second_node._debug_run_directory != first_run
    assert len(list(root.glob('run_*'))) == 2
    assert not list(root.rglob('.pending_*'))
    assert {path.relative_to(root): path.read_bytes()
            for path in first_run.rglob('*') if path.is_file()} == saved
