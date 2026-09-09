"""Reproducible PCD-to-Map-Package publication pipeline."""

from __future__ import annotations

import argparse
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import yaml

from agt_map_converter.pcd_to_nav_map import generate_navigation_map
from agt_map_converter.validate_nav_map import validate as validate_navigation_map

from .create_map_package import build_package
from .map_package import sha256_file, validate_package


_CONVERTER_DEFAULTS = {
    'resolution': 0.10,
    'margin': 1.0,
    'min_points': 2,
    'max_step': 0.22,
    'max_slope_deg': 20.0,
    'trajectory_front_m': 0.40,
    'trajectory_rear_m': 0.72,
    'trajectory_half_width_m': 0.46,
}


def load_pipeline_config(path: Path) -> Dict[str, Any]:
    path = path.expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'pipeline config is unreadable: {exc}') from exc
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('pipeline config requires schema_version: 1')
    converter = data.get('converter', {})
    if not isinstance(converter, dict):
        raise ValueError('pipeline converter must be a mapping')
    unknown = set(converter) - set(_CONVERTER_DEFAULTS)
    if unknown:
        raise ValueError(f'pipeline converter has unsupported keys: {sorted(unknown)}')

    normalized = dict(_CONVERTER_DEFAULTS)
    for key, default in _CONVERTER_DEFAULTS.items():
        value = converter.get(key, default)
        try:
            normalized[key] = int(value) if key == 'min_points' else float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'pipeline converter {key} is invalid') from exc
    if (normalized['resolution'] <= 0 or normalized['margin'] < 0
            or normalized['min_points'] < 1
            or any(normalized[key] < 0 for key in (
                'trajectory_front_m', 'trajectory_rear_m', 'trajectory_half_width_m'))):
        raise ValueError('pipeline converter contains invalid bounds')
    return {'schema_version': 1, 'converter': normalized}


def generate_map_package(
    *,
    map_root: Path,
    map_id: str,
    map_version: str,
    source_pcd: Path,
    pipeline_config: Path,
    relocalization_assets_dir: Path | None = None,
    trajectory_poses: Path | None = None,
    rtk_origin: Path | None = None,
    preview: Path | None = None,
) -> Path:
    """Generate, validate and atomically publish one immutable Map Package."""
    source_pcd = source_pcd.expanduser().resolve()
    config = load_pipeline_config(pipeline_config)
    if not source_pcd.is_file() or source_pcd.suffix.lower() != '.pcd':
        raise ValueError(f'source_pcd must be an existing .pcd file: {source_pcd}')

    with tempfile.TemporaryDirectory(prefix='agt_map_pipeline_') as temp_dir:
        staging = Path(temp_dir)
        navigation = staging / 'navigation'
        converter_metadata = generate_navigation_map(
            source_pcd,
            navigation,
            trajectory_poses_path=trajectory_poses,
            **config['converter'],
        )
        validation_errors = validate_navigation_map(navigation)
        if validation_errors:
            raise RuntimeError(
                'generated navigation map failed validation: ' + '; '.join(validation_errors))

        pipeline_record = {
            **config,
            'pipeline': 'agt_map_manager/map_pipeline',
            'source_pcd': str(source_pcd),
            'source_pcd_sha256': sha256_file(source_pcd),
            'trajectory_poses': str(trajectory_poses.resolve()) if trajectory_poses else '',
        }
        pipeline_path = staging / 'pipeline.yaml'
        pipeline_path.write_text(
            yaml.safe_dump(pipeline_record, sort_keys=False), encoding='utf-8')

        quality_path = staging / 'quality_report.yaml'
        quality_path.write_text(yaml.safe_dump({
            'schema_version': 1,
            'status': 'pass',
            'validated_utc': datetime.now(timezone.utc).isoformat(),
            'checks': {
                'navigation_map': 'pass',
                'coordinate_frame': 'map',
                'localization_navigation_generation': 'same_package',
            },
            'converter': converter_metadata,
        }, sort_keys=False), encoding='utf-8')

        package = build_package(
            map_root=map_root,
            map_id=map_id,
            map_version=map_version,
            source_pcd=source_pcd,
            navigation_dir=navigation,
            relocalization_assets_dir=relocalization_assets_dir,
            rtk_origin=rtk_origin,
            preview=preview,
            generation_pipeline=pipeline_path,
            quality_report=quality_path,
        )
    info = validate_package(package / 'metadata.yaml', verify_hashes=True)
    if not info.valid:
        raise RuntimeError(f'published map package failed validation: {info.reason}')
    return package


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Generate, validate and publish an immutable AGT Map Package.')
    parser.add_argument('--map-root', default='/home/yangxuan/ros2_ws/agt_data/maps')
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--source-pcd', required=True)
    parser.add_argument('--pipeline-config', required=True)
    parser.add_argument('--relocalization-assets-dir')
    parser.add_argument('--trajectory-poses')
    parser.add_argument('--rtk-origin')
    parser.add_argument('--preview')
    args = parser.parse_args(argv)
    try:
        package = generate_map_package(
            map_root=Path(args.map_root),
            map_id=args.map_id,
            map_version=args.map_version,
            source_pcd=Path(args.source_pcd),
            pipeline_config=Path(args.pipeline_config),
            relocalization_assets_dir=(
                Path(args.relocalization_assets_dir)
                if args.relocalization_assets_dir else None),
            trajectory_poses=Path(args.trajectory_poses) if args.trajectory_poses else None,
            rtk_origin=Path(args.rtk_origin) if args.rtk_origin else None,
            preview=Path(args.preview) if args.preview else None,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    print(f'Generated validated Map Package: {package}')
