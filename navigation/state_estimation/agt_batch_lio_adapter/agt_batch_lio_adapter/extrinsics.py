from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Tuple

import yaml


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def q_norm(values: Iterable[float]) -> Quaternion:
    q = tuple(float(v) for v in values)
    if len(q) != 4:
        raise ValueError('quaternion must contain four xyzw values')
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def q_mul(a: Iterable[float], b: Iterable[float]) -> Quaternion:
    ax, ay, az, aw = q_norm(a)
    bx, by, bz, bw = q_norm(b)
    return q_norm((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def q_conj(q: Iterable[float]) -> Quaternion:
    x, y, z, w = q_norm(q)
    return -x, -y, -z, w


def rotate(q: Iterable[float], v: Iterable[float]) -> Vector3:
    x, y, z = (float(x) for x in v)
    qx, qy, qz, qw = q_norm(q)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def compose_transform(
    t_ab: Iterable[float],
    q_ab: Iterable[float],
    t_bc: Iterable[float],
    q_bc: Iterable[float],
) -> tuple[Vector3, Quaternion]:
    """Compose T_ac = T_ab * T_bc using xyzw quaternions."""
    ta = tuple(float(v) for v in t_ab)
    tb = tuple(float(v) for v in t_bc)
    if len(ta) != 3 or len(tb) != 3:
        raise ValueError('translations must contain three values')
    rotated = rotate(q_ab, tb)
    return (
        (ta[0] + rotated[0], ta[1] + rotated[1], ta[2] + rotated[2]),
        q_mul(q_ab, q_bc),
    )


def inverse_transform(
    translation: Iterable[float],
    quaternion_xyzw: Iterable[float],
) -> tuple[Vector3, Quaternion]:
    t = tuple(float(v) for v in translation)
    if len(t) != 3:
        raise ValueError('translation must contain three values')
    iq = q_conj(quaternion_xyzw)
    return rotate(iq, (-t[0], -t[1], -t[2])), iq


def matrix_to_quaternion_xyzw(values: Iterable[float]) -> Quaternion:
    """Convert a row-major 3x3 rotation matrix into a normalized xyzw quaternion."""
    r = tuple(float(v) for v in values)
    if len(r) != 9:
        raise ValueError('rotation matrix must contain nine row-major values')
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = r
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return q_norm((qx, qy, qz, qw))


def load_batch_lio_body_to_lidar(config_path: str | Path) -> tuple[Vector3, Quaternion]:
    """Read Batch-LIO's exact runtime T_body_lidar from its parameter YAML.

    Functionhx/Batch-LIO applies:
        p_body = extrinsic_R * p_lidar + extrinsic_T
    so mapping.extrinsic_R/T are already T_body_lidar.
    """
    path = Path(config_path).expanduser()
    if not path.is_file():
        raise RuntimeError(f'Batch-LIO config not found: {path}')

    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    root = data.get('/**', data)
    params = root.get('ros__parameters', root) if isinstance(root, dict) else {}
    mapping = params.get('mapping', {}) if isinstance(params, dict) else {}
    if not isinstance(mapping, dict):
        raise RuntimeError(f'Batch-LIO config has no mapping section: {path}')

    translation = mapping.get('extrinsic_T')
    rotation = mapping.get('extrinsic_R')
    if not isinstance(translation, list) or len(translation) != 3:
        raise RuntimeError(f'Batch-LIO mapping.extrinsic_T must contain 3 values: {path}')
    if not isinstance(rotation, list) or len(rotation) != 9:
        raise RuntimeError(f'Batch-LIO mapping.extrinsic_R must contain 9 values: {path}')

    return (
        tuple(float(v) for v in translation),
        matrix_to_quaternion_xyzw(rotation),
    )


def transform_msg_to_tuple(transform) -> tuple[Vector3, Quaternion]:
    tr = transform.translation
    qr = transform.rotation
    return (
        (float(tr.x), float(tr.y), float(tr.z)),
        q_norm((float(qr.x), float(qr.y), float(qr.z), float(qr.w))),
    )


def load_lio_body_to_lidar(config_path: str | Path) -> tuple[Vector3, Quaternion]:
    """Read T_body_lidar from Batch ROS YAML or FAST-LIO2 flat r_il/t_il YAML.

    Both are forward transforms: p_body = R * p_lidar + t. Never invert signs
    merely because a different frontend was selected. Keep the Batch loader
    unchanged for existing callers and use the active frontend's own calibration.
    """
    path = Path(config_path).expanduser()
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'Expected a LIO configuration mapping: {path}')
    if 'r_il' not in data and 't_il' not in data:
        return load_batch_lio_body_to_lidar(path)
    rotation, translation = data.get('r_il'), data.get('t_il')
    if not isinstance(rotation, list) or len(rotation) != 9:
        raise RuntimeError(f'FAST-LIO2 r_il must contain 9 values: {path}')
    if not isinstance(translation, list) or len(translation) != 3:
        raise RuntimeError(f'FAST-LIO2 t_il must contain 3 values: {path}')
    rotation = [float(value) for value in rotation]
    translation = tuple(float(value) for value in translation)
    if not all(math.isfinite(value) for value in rotation + list(translation)):
        raise RuntimeError('LIO calibration must be finite')
    # Reject a reflection/scaling matrix rather than normalizing it into a pose.
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[3*i+k] * rotation[3*j+k] for k in range(3))
            if abs(dot - float(i == j)) > 1.0e-5:
                raise RuntimeError('FAST-LIO2 r_il must be an orthonormal rotation')
    a, b, c, d, e, f, g, h, i = rotation
    determinant = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g)
    if abs(determinant - 1.0) > 1.0e-5:
        raise RuntimeError('FAST-LIO2 r_il must be a proper rotation')
    return translation, matrix_to_quaternion_xyzw(rotation)
