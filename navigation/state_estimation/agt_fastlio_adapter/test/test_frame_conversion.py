import math

import pytest

from agt_fastlio_adapter.frame_conversion import body_vector_to_base, compose_body_to_base_pose


def test_body_to_base_translation_is_rotated_into_odom():
    # odom<-body is a +90 degree yaw rotation; its +X body lever arm is +Y in odom.
    root = math.sqrt(0.5)
    translation, quaternion = compose_body_to_base_pose(
        (10.0, 20.0, 0.0),
        (0.0, 0.0, root, root),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    assert translation == pytest.approx((10.0, 21.0, 0.0))
    assert quaternion == pytest.approx((0.0, 0.0, root, root))


def test_body_vector_is_expressed_in_base_axes():
    root = math.sqrt(0.5)
    # T_body_base is +90 degrees: body +Y becomes base +X.
    assert body_vector_to_base((0.0, 1.0, 0.0), (0.0, 0.0, root, root)) == pytest.approx((1.0, 0.0, 0.0))
