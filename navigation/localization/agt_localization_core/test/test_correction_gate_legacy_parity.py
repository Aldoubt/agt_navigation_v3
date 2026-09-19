import math

import pytest

from agt_localization_core.correction_gate import (
    Innovation,
    exceeds_innovation_gate,
    smooth_correction,
    tracking_innovation,
)
from agt_localization_core.pose_math import Pose3
from legacy_source import legacy_manager


def _legacy_pose(pose):
    return legacy_manager._Pose3(pose.position, pose.quaternion)


@pytest.mark.parametrize(
    ('translation', 'yaw_deg', 'expected'),
    [(0.5, 5.0, False), (0.5001, 0.0, True), (0.1, 5.1, True)],
)
def test_innovation_gate_matches_legacy(translation, yaw_deg, expected):
    yaw_rad = math.radians(yaw_deg)
    core = exceeds_innovation_gate(Innovation(translation, yaw_rad), 0.5, math.radians(5.0))
    legacy = legacy_manager._tracking_innovation_exceeds(
        translation, yaw_rad, 0.5, math.radians(5.0))
    assert core is legacy is expected


def test_tracking_innovation_matches_legacy_measurement_geometry():
    correction = Pose3((1.0, 2.0, 0.0), (0.0, 0.0, math.sin(0.1), math.cos(0.1)))
    odom = Pose3((3.0, -1.0, 0.0), (0.0, 0.0, math.sin(-0.2), math.cos(-0.2)))
    measured = Pose3((4.3, 0.9, 0.0), (0.0, 0.0, math.sin(0.0), math.cos(0.0)))
    core = tracking_innovation(correction, odom, measured)

    predicted = legacy_manager._slerp(_legacy_pose(correction), _legacy_pose(correction), 0.0)
    predicted = legacy_manager._Pose3(predicted.p, predicted.q)
    # The legacy callback forms predicted_base = compose(correction, odom).
    predicted = legacy_manager._compose(predicted, _legacy_pose(odom))
    legacy_translation = legacy_manager._translation_delta(predicted, _legacy_pose(measured))
    legacy_yaw = abs(legacy_manager._angle_wrap(
        legacy_manager._yaw(_legacy_pose(measured).q) - legacy_manager._yaw(predicted.q)))
    assert core.translation_m == pytest.approx(legacy_translation)
    assert core.yaw_rad == pytest.approx(legacy_yaw)


def test_rate_limited_smoothing_matches_legacy_slerp_alpha():
    current = Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    target = Pose3((4.0, 0.0, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    dt = 0.1
    core = smooth_correction(
        current, target, dt,
        smoothing_enabled=True, tau_sec=3.0,
        max_linear_rate_mps=0.10, max_yaw_rate_radps=math.radians(2.0),
    )

    alpha = 1.0 - math.exp(-dt / 3.0)
    alpha = min(alpha, (0.10 * dt) / legacy_manager._translation_delta(
        _legacy_pose(current), _legacy_pose(target)))
    yaw_distance = abs(legacy_manager._angle_wrap(
        legacy_manager._yaw(_legacy_pose(target).q) - legacy_manager._yaw(_legacy_pose(current).q)))
    alpha = min(alpha, (math.radians(2.0) * dt) / yaw_distance)
    legacy = legacy_manager._slerp(_legacy_pose(current), _legacy_pose(target), alpha)
    assert core.position == pytest.approx(legacy.p)
    assert core.quaternion == pytest.approx(legacy.q)
