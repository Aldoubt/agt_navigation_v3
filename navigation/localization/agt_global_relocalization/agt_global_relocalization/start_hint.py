"""Reviewed, version-bound near-start pose. Never infer a pose from grid origin.

The hint is separate from immutable Map Packages. A human must confirm the
T_map_body coordinate/frame against the exact localization PCD before it can
be used as a bounded local GICP seed; absence means skip this optional stage.
"""
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import yaml


class StartHintError(ValueError):
    """Untrusted or mismatched pose; do not publish a localization result."""


def _digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_start_hint(path: str | Path, *, map_id: str, map_version: str,
                    map_pcd: str | Path) -> dict[str, float]:
    """Validate the map identity, review and complete SE(3) pose before use."""
    hint_file = Path(path).expanduser()
    if not hint_file.is_file():
        raise StartHintError(f'start hint does not exist: {hint_file}')
    try:
        data = yaml.safe_load(hint_file.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise StartHintError(f'unreadable start hint: {exc}') from exc
    if not isinstance(data, dict) or type(data.get('schema_version')) is not int \
            or data['schema_version'] != 1:
        raise StartHintError('start hint schema_version must be 1')
    if not map_id or not map_version or data.get('map_id') != map_id \
            or data.get('map_version') != map_version:
        raise StartHintError('start hint map_id/map_version does not match selected map')
    if data.get('frame_id') != 'map' or data.get('pose_semantics') != 'T_map_body':
        raise StartHintError('start hint must be a map-frame T_map_body pose, not grid origin or base_link')
    if data.get('approved_for_near_search') is not True \
            or not str(data.get('reviewed_by') or '').strip() \
            or not str(data.get('source') or '').strip():
        raise StartHintError('start hint requires an explicit reviewer, source and approval')
    pcd = Path(map_pcd).expanduser()
    if not pcd.is_file():
        raise StartHintError(f'localization PCD does not exist: {pcd}')
    recorded = str(data.get('localization_map_sha256') or '').lower()
    if len(recorded) != 64 or any(c not in '0123456789abcdef' for c in recorded) \
            or recorded != _digest(pcd):
        raise StartHintError('start hint localization PCD SHA-256 does not match')
    source_pose = data.get('pose')
    fields = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    if not isinstance(source_pose, dict) or set(source_pose) != set(fields):
        raise StartHintError('start hint requires exactly x,y,z,qx,qy,qz,qw')
    try:
        pose = {key: float(source_pose[key]) for key in fields}
    except (TypeError, ValueError) as exc:
        raise StartHintError('start hint pose values must be numeric') from exc
    if any(not math.isfinite(value) for value in pose.values()):
        raise StartHintError('start hint pose must be finite')
    qnorm = math.sqrt(sum(pose[key] ** 2 for key in ('qx', 'qy', 'qz', 'qw')))
    if not 0.99 <= qnorm <= 1.01:
        raise StartHintError('start hint quaternion must be unit length')
    return pose


def main() -> None:
    parser = argparse.ArgumentParser(description='Validate a reviewed near-start map pose')
    parser.add_argument('--hint', required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--map-pcd', required=True)
    args = parser.parse_args()
    try:
        load_start_hint(args.hint, map_id=args.map_id,
                        map_version=args.map_version, map_pcd=args.map_pcd)
    except StartHintError as exc:
        parser.exit(2, f'INVALID_START_HINT: {exc}\n')
    print(f'VALID_START_HINT {args.map_id}/{args.map_version}: {args.hint}')


if __name__ == '__main__':
    main()
