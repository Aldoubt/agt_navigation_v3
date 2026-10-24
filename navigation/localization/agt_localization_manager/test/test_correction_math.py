import math
import json
from collections import deque
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from agt_localization_manager.localization_manager import (
    LocalizationManager,
    LocalizationState,
    _Pose3,
    _RecoveryStateMachine,
    _compose,
    _correction_delta,
    _inverse,
    _local_odom_loss_recoverable,
    _make_metrics_message,
    _slerp,
    _tracking_innovation_exceeds,
    _translation_delta,
    _yaw,
)


def test_pose_compose_inverse_round_trip():
    pose = _Pose3((1.2, -0.4, 0.7), (0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    identity = _compose(pose, _inverse(pose))
    assert max(abs(v) for v in identity.p) < 1e-6
    assert abs(_yaw(identity.q)) < 1e-6


def test_global_pose_input_generates_map_to_odom_correction():
    map_odom = _Pose3(
        (2.0, -1.0, 0.3),
        (0.0, 0.0, math.sin(0.2), math.cos(0.2)),
    )
    odom_base = _Pose3(
        (4.0, 0.5, 0.0),
        (0.0, 0.0, math.sin(-0.35), math.cos(-0.35)),
    )
    map_base = _compose(map_odom, odom_base)

    # This is the exact ownership contract used by LocalizationManager.
    recovered = _compose(map_base, _inverse(odom_base))
    assert _translation_delta(recovered, map_odom) < 1e-6
    assert abs(_yaw(recovered.q) - _yaw(map_odom.q)) < 1e-6


def test_tracking_failure_enters_lost_and_relocalizing_recovery():
    recovery = _RecoveryStateMachine(cooldown_sec=5.0)
    recovery.state = LocalizationState.TRACKING

    recovery.tracking_failure('tracker_failure', auto_request=True)
    assert recovery.state == LocalizationState.LOST
    assert recovery.try_request(10.0)
    assert recovery.state == LocalizationState.RELOCALIZING
    assert recovery.request_count == 1


def test_recovery_cooldown_blocks_duplicate_request_until_expired():
    recovery = _RecoveryStateMachine(cooldown_sec=5.0)

    recovery.tracking_failure('first_failure', auto_request=True)
    assert recovery.try_request(10.0)
    assert recovery.cooldown_remaining(12.0) == 3.0
    recovery.tracking_failure('second_failure', auto_request=True)
    assert not recovery.try_request(12.0)
    assert recovery.try_request(15.0)


def test_transient_local_odom_loss_can_recover_with_existing_global_correction():
    assert _local_odom_loss_recoverable(
        LocalizationState.LOST, 'local_odom_lost:1.01s', True)
    assert not _local_odom_loss_recoverable(
        LocalizationState.LOST, 'map_tracking_recovery_required', True)
    assert not _local_odom_loss_recoverable(
        LocalizationState.LOST, 'local_odom_lost:1.01s', False)


def test_metrics_after_global_pose_acceptance_reports_localized():
    metrics = _make_metrics_message(
        LocalizationState.LOCALIZED,
        'UNKNOWN',
        'global_pose_accepted',
        _Pose3((1.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
        0.0,
        0.0,
        0.0,
        0.0,
        0.05,
        0.02,
        None,
    )
    assert metrics.state == 'LOCALIZED'
    assert metrics.map_odom_x == 1.0
    assert metrics.global_position_std == 0.05


def test_metrics_reflects_tracking_innovation_gate_as_degraded():
    yaw_delta = math.radians(6.0)
    assert _tracking_innovation_exceeds(0.1, yaw_delta, 0.5, math.radians(5.0))
    metrics = _make_metrics_message(
        LocalizationState.DEGRADED,
        'DEGRADED',
        'tracking_suspect:innovation_gate',
        None,
        0.0,
        0.0,
        0.6,
        yaw_delta,
        0.1,
        0.02,
        None,
    )
    assert metrics.state == 'DEGRADED'
    assert metrics.tracking_innovation_translation == 0.6
    assert metrics.tracking_innovation_yaw == pytest.approx(yaw_delta)


def test_metrics_records_correction_translation_and_yaw_delta():
    previous = _Pose3((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    current = _Pose3(
        (2.0, 2.0, 0.0),
        (0.0, 0.0, math.sin(0.1), math.cos(0.1)),
    )
    translation, yaw_delta = _correction_delta(current, previous)
    assert translation == pytest.approx(1.0)
    assert yaw_delta == pytest.approx(0.2)


def test_metrics_without_global_correction_reports_wait_global():
    metrics = _make_metrics_message(
        LocalizationState.WAIT_GLOBAL,
        'UNKNOWN',
        'waiting_global_pose',
        None,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0e9,
        1.0e9,
        None,
    )
    assert metrics.state == 'WAIT_GLOBAL'
    assert metrics.map_odom_x == 0.0
    assert metrics.map_odom_yaw == 0.0


def test_slerp_halfway_has_bounded_translation_and_yaw():
    start = _Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    end = _Pose3((1.0, 0.0, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    middle = _slerp(start, end, 0.5)
    assert abs(middle.p[0] - 0.5) < 1e-6
    assert abs(_yaw(middle.q) - math.pi / 4) < 1e-6


def test_backend_rejection_leaves_relocalizing_for_wait_global():
    manager = object.__new__(LocalizationManager)
    manager._state = LocalizationState.RELOCALIZING
    manager._reason = 'relocalization_requested'
    manager._backend_debug_state = None
    manager._correction_current = None
    manager._recovery_pending = True

    msg = String()
    msg.data = json.dumps({'state': 'REJECTED', 'detail': 'small_gicp did not converge'})
    LocalizationManager._on_backend_status(manager, msg)

    assert manager._backend_debug_state == 'REJECTED'
    assert manager._state == LocalizationState.WAIT_GLOBAL
    assert manager._reason == (
        'global_relocalization_rejected:small_gicp did not converge')
    assert not manager._recovery_pending


def test_backend_collecting_remains_relocalizing_while_pending():
    manager = object.__new__(LocalizationManager)
    manager._state = LocalizationState.WAIT_GLOBAL
    manager._reason = 'waiting_global_pose'
    manager._backend_debug_state = None
    manager._correction_current = None
    manager._recovery_pending = False

    msg = String()
    msg.data = json.dumps({'state': 'COLLECTING', 'detail': '2/5 frames'})
    LocalizationManager._on_backend_status(manager, msg)

    assert manager._backend_debug_state == 'COLLECTING'
    assert manager._state == LocalizationState.RELOCALIZING
    assert manager._reason == 'global_relocalization:collecting'


def _global_candidate(frame='map', stamp_ns=1_000_000_000):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.04
    msg.pose.covariance[7] = 0.04
    msg.pose.covariance[14] = 0.04
    msg.pose.covariance[35] = math.radians(2.0) ** 2
    return msg


def _candidate_manager():
    values = {'map_frame': 'map', 'accept_zero_covariance': False,
              'max_global_position_std_m': 1.0, 'max_global_yaw_std_deg': 20.0,
              'global_match_max_skew_sec': 0.1}
    return SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))


def test_invalid_global_candidates_are_rejected_before_correction():
    manager = _candidate_manager()
    for msg, reason in (
            (_global_candidate(frame='odom'), 'global_pose_wrong_frame'),
            (_global_candidate(stamp_ns=0), 'global_pose_zero_stamp')):
        assert LocalizationManager._validate_global(manager, msg)[3] == reason
    missing = _global_candidate()
    missing.pose.covariance = [0.0] * 36
    assert LocalizationManager._validate_global(manager, missing)[3] == 'global_pose_missing_covariance'
    uncertain = _global_candidate()
    uncertain.pose.covariance[0] = 4.0
    assert LocalizationManager._validate_global(manager, uncertain)[3] == 'global_position_uncertainty_too_large'
    uncertain = _global_candidate()
    uncertain.pose.covariance[35] = math.radians(30.0) ** 2
    assert LocalizationManager._validate_global(manager, uncertain)[3] == 'global_yaw_uncertainty_too_large'


def _local_odom(stamp_ns):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_rejected_candidate_does_not_replace_existing_correction():
    manager = _candidate_manager()
    manager._last_global_std = (math.inf, math.inf)
    manager._reason = 'tracking'
    manager._correction_current = object()
    manager.get_logger = lambda: SimpleNamespace(warn=lambda text: None)
    previous = manager._correction_current
    LocalizationManager._on_global_pose(manager, _global_candidate(frame='odom'))
    assert manager._correction_current is previous
    assert manager._reason == 'global_pose_wrong_frame'


def test_global_candidate_without_time_aligned_odom_is_rejected():
    manager = _candidate_manager()
    manager._odom = deque([_local_odom(1_000_000_000)])
    assert LocalizationManager._nearest_odom(manager, 1_200_000_000) is None
    assert LocalizationManager._nearest_odom(manager, 1_050_000_000) is not None


def test_odom_regression_clears_window_and_requires_newer_valid_stamp():
    values = {'odom_frame': 'odom', 'base_frame': 'base_link', 'odom_buffer_sec': 30.0}
    manager = SimpleNamespace(
        _odom=deque(), _last_odom_rx_ns=0, _last_odom_stamp_ns=None,
        _observation_error=None, _reason='waiting_local_odom',
        odom_timestamp_error=LocalizationManager.odom_timestamp_error,
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123456)),
        get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None))
    LocalizationManager._on_odom(manager, _local_odom(2_000_000_000))
    assert len(manager._odom) == 1
    LocalizationManager._on_odom(manager, _local_odom(1_000_000_000))
    assert not manager._odom
    assert manager._last_odom_stamp_ns == 2_000_000_000
    assert manager._last_odom_rx_ns == 123456
    assert manager._observation_error == 'local_odom_timestamp_regression'
    LocalizationManager._on_odom(manager, _local_odom(2_000_000_000))
    assert not manager._odom
    LocalizationManager._on_odom(manager, _local_odom(3_000_000_000))
    assert len(manager._odom) == 1
    assert manager._observation_error is None
