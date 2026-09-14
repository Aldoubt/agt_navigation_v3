from pathlib import Path

import pytest

from agt_batch_lio_adapter.extrinsics import (
    compose_transform,
    inverse_transform,
    load_batch_lio_body_to_lidar,
    matrix_to_quaternion_xyzw,
)


def test_checked_in_batch_config_is_internal_extrinsic_source():
    root = Path(__file__).resolve().parents[4]
    config = root / 'mapping/agt_mapping_bringup/config/batch_lio_mid360.yaml'
    translation, quaternion = load_batch_lio_body_to_lidar(config)
    assert translation == pytest.approx((0.011, 0.02329, -0.04412), abs=1e-12)
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)


def test_transform_inverse_round_trip():
    translation = (0.11, -0.22, 0.33)
    quaternion = matrix_to_quaternion_xyzw([
        0.0, -1.0, 0.0,
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0,
    ])
    inverse_t, inverse_q = inverse_transform(translation, quaternion)
    identity_t, identity_q = compose_transform(
        translation, quaternion, inverse_t, inverse_q)
    assert identity_t == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert identity_q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)


def test_body_to_base_is_composed_from_internal_and_physical_sources():
    # Synthetic frame chain:
    # T_body_lidar is the Batch-LIO internal extrinsic.
    # T_lidar_base is supplied by robot_description/TF.
    t_body_lidar = (0.01, 0.02, -0.04)
    q_body_lidar = (0.0, 0.0, 0.0, 1.0)
    t_lidar_base = (-0.40, 0.0, -0.70)
    q_lidar_base = (0.0, 0.0, 0.0, 1.0)
    t_body_base, q_body_base = compose_transform(
        t_body_lidar, q_body_lidar, t_lidar_base, q_lidar_base)
    assert t_body_base == pytest.approx((-0.39, 0.02, -0.74), abs=1e-12)
    assert q_body_base == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)
