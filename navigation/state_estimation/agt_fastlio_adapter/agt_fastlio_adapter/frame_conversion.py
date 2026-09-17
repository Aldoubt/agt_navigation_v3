"""Frame conversion helpers for the FAST-LIO2 navigation adapter."""

from __future__ import annotations

from typing import Iterable

from agt_batch_lio_adapter.extrinsics import compose_transform, q_conj, q_mul, q_norm, rotate


def compose_body_to_base_pose(
    body_translation: Iterable[float],
    body_quaternion_xyzw: Iterable[float],
    body_to_base_translation: Iterable[float],
    body_to_base_quaternion_xyzw: Iterable[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose ``T_odom_base = T_odom_body * T_body_base``."""
    return compose_transform(
        body_translation,
        body_quaternion_xyzw,
        body_to_base_translation,
        body_to_base_quaternion_xyzw,
    )


def body_vector_to_base(vector: Iterable[float], body_to_base_quaternion_xyzw: Iterable[float]):
    """Express a vector from FAST-LIO body axes in base_link axes."""
    return rotate(q_conj(body_to_base_quaternion_xyzw), vector)


def compose_orientation(
    body_quaternion_xyzw: Iterable[float], body_to_base_quaternion_xyzw: Iterable[float]
) -> tuple[float, float, float, float]:
    return q_mul(q_norm(body_quaternion_xyzw), q_norm(body_to_base_quaternion_xyzw))
