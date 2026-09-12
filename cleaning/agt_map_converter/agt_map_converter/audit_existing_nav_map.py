"""Evidence-only trajectory audit for an existing immutable Nav2 map."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

import numpy as np
import yaml

from .pcd_to_nav_map import (
    load_trajectory_poses,
    trajectory_conflict_regions,
    trajectory_swept_mask,
    write_pgm,
)
from .validate_nav_map import read_pgm_header, validate


def _load_map_image(path: Path) -> np.ndarray:
    width, height, payload = read_pgm_header(path)
    return np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: Path, label: str) -> dict:
    if not path.is_file():
        raise ValueError(f'{label} missing: {path}')
    value = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a YAML mapping: {path}')
    return value


def _verify_provenance(map_dir: Path, trajectory_path: Path) -> dict:
    """Require the recorded trajectory and localization PCD to match provenance."""
    package_dir = map_dir.parent
    converter = _load_yaml(map_dir / 'converter_metadata.yaml', 'converter metadata')
    package = _load_yaml(package_dir / 'metadata.yaml', 'map package metadata')
    expected_trajectory = Path(str(converter.get('trajectory_poses', ''))).expanduser()
    if not expected_trajectory.is_file():
        raise ValueError('converter metadata trajectory poses are missing')
    if trajectory_path.resolve() != expected_trajectory.resolve():
        raise ValueError('trajectory path does not match converter metadata provenance')
    poses = load_trajectory_poses(trajectory_path)
    if converter.get('trajectory_pose_count') != len(poses):
        raise ValueError('trajectory pose count disagrees with converter metadata')
    if not package.get('map_id') or not package.get('map_version') or package.get('frame_id') != 'map':
        raise ValueError('map package metadata lacks map_id/map_version/frame_id=map')

    assets = package.get('assets') or {}
    localization = assets.get('localization_map') or {}
    expected_localization_sha = localization.get('sha256')
    package_localization = package_dir / str(localization.get('path', ''))
    converter_source = Path(str(converter.get('source_pcd', ''))).expanduser()
    if not expected_localization_sha or not package_localization.is_file() or not converter_source.is_file():
        raise ValueError('localization PCD provenance is incomplete')
    if _sha256(package_localization) != expected_localization_sha:
        raise ValueError('package localization PCD does not match metadata hash')
    if _sha256(converter_source) != expected_localization_sha:
        raise ValueError('converter source PCD does not match package localization PCD generation')

    # If Polar Context provenance is present, require it to name this same
    # formal poses.txt.  Its source PCD is separately bound above by SHA256.
    polar = package_dir / 'localization' / 'relocalization' / 'polar_context.yaml'
    if polar.is_file():
        polar_data = _load_yaml(polar, 'Polar Context metadata')
        source_dir = Path(str(polar_data.get('source_map_dir', ''))).expanduser()
        source_poses = source_dir / str(polar_data.get('source_poses', ''))
        if not source_poses.is_file() or source_poses.resolve() != trajectory_path.resolve():
            raise ValueError('Polar Context trajectory provenance does not match audit trajectory')

    return {
        'map_id': package['map_id'],
        'map_version': package['map_version'],
        'frame_id': package['frame_id'],
        'trajectory_poses': str(trajectory_path),
        'trajectory_pose_count': len(poses),
        'converter_source_pcd': str(converter_source),
        'localization_pcd_sha256': expected_localization_sha,
        'trajectory_front_m': float(converter.get('trajectory_front_m', 0.40)),
        'trajectory_rear_m': float(converter.get('trajectory_rear_m', 0.72)),
        'trajectory_half_width_m': float(converter.get('trajectory_half_width_m', 0.46)),
        'poses': poses,
    }


def audit_existing_navigation_map(map_directory: Path, trajectory_poses: Path, output: Path) -> dict:
    """Audit an existing map without mutating any input asset."""
    map_directory = map_directory.expanduser().resolve()
    trajectory_poses = trajectory_poses.expanduser().resolve()
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f'audit output already exists: {output}')
    errors = validate(map_directory)
    if errors:
        raise ValueError('map validation failed: ' + '; '.join(errors))
    nav = _load_yaml(map_directory / 'map.yaml', 'map.yaml')
    provenance = _verify_provenance(map_directory, trajectory_poses)
    image = _load_map_image(map_directory / str(nav['image']))
    grid = np.flipud(image)
    known_values = (grid == 0) | (grid == 205) | (grid == 254)
    if not bool(known_values.all()):
        raise ValueError('map.pgm contains values outside the trinary 0/205/254 convention')

    resolution = float(nav['resolution'])
    origin = [float(v) for v in nav['origin']]
    swept = trajectory_swept_mask(
        grid.shape, origin, resolution, provenance.pop('poses'),
        provenance['trajectory_front_m'], provenance['trajectory_rear_m'],
        provenance['trajectory_half_width_m'])
    occupied = swept & (grid == 0)
    unknown = swept & (grid == 205)
    free = swept & (grid == 254)
    swept_count = int(swept.sum())
    if swept_count == 0:
        raise ValueError('trajectory projection does not intersect the existing map grid')
    regions = trajectory_conflict_regions(occupied, origin, resolution)
    for region in regions:
        region['centroid_map'] = region.pop('centroid_m')

    summary = {
        'trajectory_swept_cells': swept_count,
        'trajectory_free_cells': int(free.sum()),
        'trajectory_unknown_cells': int(unknown.sum()),
        'trajectory_unknown_ratio': float(unknown.sum() / swept_count),
        'trajectory_occupied_cells': int(occupied.sum()),
        'trajectory_occupied_ratio': float(occupied.sum() / swept_count),
        'occupied_conflict_region_count': len(regions),
    }
    # Every swept cell must have exactly one trinary classification.
    if summary['trajectory_free_cells'] + summary['trajectory_unknown_cells'] + summary['trajectory_occupied_cells'] != swept_count:
        raise ValueError('trajectory corridor classification is inconsistent')
    status = 'PASS' if summary['trajectory_occupied_cells'] == 0 else 'REVIEW'
    evidence = {
        'format_version': 1,
        'audit_kind': 'existing_navigation_map_evidence_only',
        'acceptance_status': status,
        'acceptance_reason': (
            'no trajectory occupied conflicts' if status == 'PASS'
            else 'trajectory occupied conflicts require human review as fixed obstacle or suspected artifact'),
        'unknown_corridor_note': 'Unknown corridor cells are diagnostic only and do not independently fail this gate.',
        'provenance': provenance,
        'map': {
            'map_directory': str(map_directory),
            'map_yaml': str(map_directory / 'map.yaml'),
            'map_pgm': str(map_directory / str(nav['image'])),
            'resolution_m': resolution,
            'origin': origin,
            'grid_shape': [int(grid.shape[0]), int(grid.shape[1])],
        },
        'summary': summary,
        'occupied_conflict_regions': regions,
        'artifacts': {
            'trajectory_occupied_conflicts_pgm': 'trajectory_occupied_conflicts.pgm',
            'trajectory_unknown_corridor_pgm': 'trajectory_unknown_corridor.pgm',
            'legend': {'0': 'selected diagnostic cells', '254': 'all other cells'},
        },
    }
    output.mkdir(parents=True)
    write_pgm(output / 'trajectory_occupied_conflicts.pgm', np.where(np.flipud(occupied), 0, 254))
    write_pgm(output / 'trajectory_unknown_corridor.pgm', np.where(np.flipud(unknown), 0, 254))
    (output / 'existing_map_audit.yaml').write_text(
        yaml.safe_dump(evidence, sort_keys=False), encoding='utf-8')
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evidence-only trajectory audit for an existing Nav2 PGM.')
    parser.add_argument('map_directory', help='Existing navigation directory containing map.yaml/map.pgm')
    parser.add_argument('--trajectory-poses', required=True, help='Same-generation formal poses.txt')
    parser.add_argument('--output', required=True, help='New directory for audit artifacts; input map is never modified')
    args = parser.parse_args(argv)
    try:
        result = audit_existing_navigation_map(
            Path(args.map_directory), Path(args.trajectory_poses), Path(args.output))
    except (OSError, ValueError) as exc:
        print(f'MAP EXISTING AUDIT FAIL: {exc}', file=sys.stderr)
        raise SystemExit(2)
    print(f"MAP EXISTING AUDIT {result['acceptance_status']}")


if __name__ == '__main__':
    main()
