import math

import pytest

from agt_localization_core.pose_math import Pose3, compose, inverse, map_to_odom, yaw
from legacy_source import legacy_manager


def _legacy_pose(pose):
    return legacy_manager._Pose3(pose.position, pose.quaternion)


def test_map_to_odom_matches_legacy_compose_inverse():
    map_base = Pose3((5.2, -1.4, 0.3), (0.0, 0.0, math.sin(0.45), math.cos(0.45)))
    odom_base = Pose3((2.1, 3.0, -0.2), (0.0, 0.0, math.sin(-0.2), math.cos(-0.2)))

    core = map_to_odom(map_base, odom_base)
    legacy = legacy_manager._compose(
        _legacy_pose(map_base), legacy_manager._inverse(_legacy_pose(odom_base)))

    assert core.position == pytest.approx(legacy.p)
    assert core.quaternion == pytest.approx(legacy.q)
    assert yaw(core.quaternion) == pytest.approx(legacy_manager._yaw(legacy.q))


def test_compose_and_inverse_round_trip_matches_legacy():
    pose = Pose3((1.2, -0.4, 0.7), (0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    core = compose(pose, inverse(pose))
    legacy = legacy_manager._compose(_legacy_pose(pose), legacy_manager._inverse(_legacy_pose(pose)))
    assert core.position == pytest.approx(legacy.p)
    assert core.quaternion == pytest.approx(legacy.q)
