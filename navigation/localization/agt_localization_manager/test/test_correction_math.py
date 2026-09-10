import math

from agt_localization_manager.localization_manager import _Pose3, _compose, _inverse, _slerp, _yaw


def test_pose_compose_inverse_round_trip():
    pose = _Pose3((1.2, -0.4, 0.7), (0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    identity = _compose(pose, _inverse(pose))
    assert max(abs(v) for v in identity.p) < 1e-6
    assert abs(_yaw(identity.q)) < 1e-6


def test_slerp_halfway_has_bounded_translation_and_yaw():
    start = _Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    end = _Pose3((1.0, 0.0, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    middle = _slerp(start, end, 0.5)
    assert abs(middle.p[0] - 0.5) < 1e-6
    assert abs(_yaw(middle.q) - math.pi / 4) < 1e-6
