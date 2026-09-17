import math

import pytest

from agt_localization_manager.localization_manager import (
    LocalizationState,
    _Pose3,
    _RecoveryStateMachine,
    _compose,
    _correction_delta,
    _inverse,
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
