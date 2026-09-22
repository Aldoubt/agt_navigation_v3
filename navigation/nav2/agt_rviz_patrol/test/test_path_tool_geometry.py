import math

import pytest

from agt_rviz_patrol.path_tool import align_relative_pose, interpolate_polyline


def test_interpolate_polyline_preserves_corners_and_endpoints():
    samples = interpolate_polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.4)

    assert samples[0][:2] == pytest.approx((0.0, 0.0))
    assert samples[-1][:2] == pytest.approx((1.0, 1.0))
    assert sum(1 for sample in samples if sample[:2] == pytest.approx((1.0, 0.0))) == 1
    assert samples[-1][2] == pytest.approx(math.pi / 2.0)
    assert all(
        math.hypot(b[0] - a[0], b[1] - a[1]) <= 0.4 + 1.0e-9
        for a, b in zip(samples, samples[1:])
    )


def test_wheel_pose_is_rigidly_aligned_to_initial_map_pose():
    aligned = align_relative_pose(
        wheel_pose=(2.0, 1.0, math.pi / 2.0),
        wheel_origin=(1.0, 1.0, 0.0),
        map_origin=(10.0, 20.0, math.pi / 2.0),
    )

    assert aligned == pytest.approx((10.0, 21.0, math.pi))
