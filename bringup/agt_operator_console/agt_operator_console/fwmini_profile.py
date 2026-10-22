"""FW-mini preview profile I/O and isolated RobotModel generation."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import yaml


def default_profile_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('agt_operator_console')) / 'config' / 'fwmini_swerve_preview.yaml'
    except (ImportError, LookupError):
        return Path(__file__).resolve().parent.parent / 'config' / 'fwmini_swerve_preview.yaml'


def default_candidate_path() -> Path:
    ros_home = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))).expanduser()
    return ros_home / 'agt_operator_console' / 'fwmini_swerve_candidate.yaml'


def load_profile(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot read FW-mini profile {path}: {exc}') from exc
    if not isinstance(data, dict) or data.get('profile_id') != 'fwmini_swerve_preview':
        raise ValueError('Expected FW-mini profile_id: fwmini_swerve_preview')
    body = data.get('body')
    if not isinstance(body, dict):
        raise ValueError('Profile is missing body geometry')
    for key in ('length_m', 'width_m', 'height_m'):
        value = body.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value <= 0:
            raise ValueError(f'body.{key} must be a positive finite value')
    mount = data.get('lidar_mount')
    if not isinstance(mount, dict):
        raise ValueError('Profile is missing lidar_mount')
    return data


def optional_vector(values: list[str]) -> list[float | None]:
    if len(values) != 3:
        raise ValueError('Each transform vector must have exactly three values')
    result: list[float | None] = []
    for raw in values:
        text = raw.strip()
        if not text:
            result.append(None)
            continue
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f'Invalid transform value: {text!r}') from exc
        if not math.isfinite(value):
            raise ValueError('Transform values must be finite')
        result.append(value)
    return result


def has_complete_lidar_transform(profile: dict[str, Any]) -> bool:
    mount = profile.get('lidar_mount', {})
    xyz = mount.get('xyz_m')
    rpy = mount.get('rpy_rad')
    return (
        isinstance(xyz, list) and isinstance(rpy, list) and len(xyz) == len(rpy) == 3
        and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in xyz + rpy)
    )


def save_candidate(
    source: dict[str, Any], xyz: list[float | None], rpy: list[float | None], destination: Path,
) -> Path:
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError('XYZ and RPY each require three entries')
    candidate = yaml.safe_load(yaml.safe_dump(source, sort_keys=False))
    candidate['profile_status'] = 'DRAFT_PREVIEW_ONLY'
    candidate['field_verified'] = False
    candidate['lidar_mount']['xyz_m'] = xyz
    candidate['lidar_mount']['rpy_rad'] = rpy
    candidate['lidar_mount']['measured'] = all(v is not None for v in xyz + rpy)
    candidate['lidar_mount']['status'] = (
        'CANDIDATE_REQUIRES_INDEPENDENT_CHECK'
        if candidate['lidar_mount']['measured'] else 'NEEDS_MANUAL_MEASUREMENT'
    )
    destination = Path(destination).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(candidate, sort_keys=False), encoding='utf-8')
    return destination


def build_preview_urdf(profile: dict[str, Any]) -> str:
    """Build a body-only preview URDF; add the lidar frame only when all 6 values exist."""
    body = profile['body']
    length, width, height = (float(body[k]) for k in ('length_m', 'width_m', 'height_m'))
    # FW-mini's source xacro places the base_link origin at the bottom face;
    # preserve that convention for this isolated visualization.
    links = [
        '<link name="base_footprint"/>',
        '<link name="base_link">'
        f'<visual><origin xyz="0 0 {height / 2:.9g}"/><geometry>'
        f'<box size="{length:.9g} {width:.9g} {height:.9g}"/></geometry>'
        '<material name="fwmini_preview_blue"><color rgba="0.15 0.55 0.85 0.8"/>'
        '</material></visual>'
        f'<collision><origin xyz="0 0 {height / 2:.9g}"/><geometry>'
        f'<box size="{length:.9g} {width:.9g} {height:.9g}"/>'
        '</geometry></collision></link>',
    ]
    joints = [
        '<joint name="preview_footprint_to_base" type="fixed">'
        '<parent link="base_footprint"/><child link="base_link"/>'
        f'<origin xyz="0 0 {float(profile.get("base_footprint_to_base_z", 0.0)):.9g}" rpy="0 0 0"/></joint>',
    ]
    if has_complete_lidar_transform(profile):
        mount = profile['lidar_mount']
        xyz = ' '.join(f'{float(v):.12g}' for v in mount['xyz_m'])
        rpy = ' '.join(f'{float(v):.12g}' for v in mount['rpy_rad'])
        links.append('<link name="livox_frame"/>')
        joints.append(
            '<joint name="preview_lidar_mount" type="fixed">'
            '<parent link="base_link"/><child link="livox_frame"/>'
            f'<origin xyz="{xyz}" rpy="{rpy}"/></joint>'
        )
    return '<robot name="fwmini_profile_preview">' + ''.join(links + joints) + '</robot>'
