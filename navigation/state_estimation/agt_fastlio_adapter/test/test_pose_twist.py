import math

import pytest
from agt_fastlio_adapter.pose_twist import PoseTwistEstimator, pose_delta_twist
from agt_fastlio_adapter.frame_conversion import body_twist_to_base

I = (0.0, 0.0, 0.0, 1.0)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def test_first_pose_is_not_differentiated_from_the_origin():
    estimator = PoseTwistEstimator()
    assert estimator.update((20.0, 30.0, 0.0), I, 1_000_000_000) == ((0.0,)*3, (0.0,)*3)
    linear, angular = estimator.update((20.1, 30.0, 0.0), I, 1_100_000_000)
    assert linear == pytest.approx((1.0, 0.0, 0.0))
    assert angular == (0.0,)*3


def test_pose_rotation_fills_angular_velocity_missing_upstream():
    estimator = PoseTwistEstimator()
    estimator.update((0.0,)*3, I, 1_000_000_000)
    linear, angular = estimator.update((0.0,)*3, yaw(0.02), 1_100_000_000)
    assert linear == (0.0,)*3
    assert angular == pytest.approx((0.0, 0.0, 0.2))


def test_linear_velocity_is_in_current_base_axes():
    linear, _ = pose_delta_twist((0, 0, 0), yaw(math.pi/2),
                                (0, .1, 0), yaw(math.pi/2), .1)
    assert linear == pytest.approx((1.0, 0.0, 0.0), abs=1e-10)


def test_quaternion_sign_flip_does_not_create_a_rotation():
    _, angular = pose_delta_twist((0,)*3, I, (0,)*3, (0, 0, 0, -1), .1)
    assert angular == (0.0,)*3


def test_long_gap_resets_differentiation_baseline():
    estimator = PoseTwistEstimator()
    estimator.update((0,)*3, I, 1_000_000_000)
    assert estimator.update((5, 0, 0), yaw(.5), 3_000_000_000) == ((0.0,)*3, (0.0,)*3)
    linear, angular = estimator.update((5.1, 0, 0), yaw(.5), 3_100_000_000)
    assert math.hypot(*linear[:2]) == pytest.approx(1.0)
    assert angular == (0.0,)*3


@pytest.mark.parametrize('bad_stamp', [1_000_000_000, 900_000_000, 0])
def test_bad_timestamp_does_not_change_history(bad_stamp):
    estimator = PoseTwistEstimator()
    estimator.update((0,)*3, I, 1_000_000_000)
    previous = estimator.previous
    with pytest.raises(ValueError):
        estimator.update((20, 0, 0), I, bad_stamp)
    assert estimator.previous == previous


def test_nan_pose_rejected_before_history_changes():
    estimator = PoseTwistEstimator()
    with pytest.raises(ValueError):
        estimator.update((float('nan'), 0, 0), I, 1_000_000_000)
    assert estimator.previous is None


def test_twist_norm_is_bounded_without_changing_the_pose():
    estimator = PoseTwistEstimator(max_linear=2.0)
    estimator.update((0,)*3, I, 1_000_000_000)
    linear, _ = estimator.update((1.0, 0, 0), I, 1_100_000_000)
    assert linear == pytest.approx((2.0, 0.0, 0.0))
    assert estimator.previous[0] == (1.0, 0, 0)


def test_upstream_twist_reference_point_shift_includes_omega_cross_r():
    linear, angular = body_twist_to_base((1, 0, 0), (0, 0, 2), (1, 0, 0), I)
    assert linear == pytest.approx((1, 2, 0))
    assert angular == pytest.approx((0, 0, 2))


def test_invalid_twist_limits_fail_closed():
    with pytest.raises(ValueError):
        PoseTwistEstimator(max_dt=0.001)
