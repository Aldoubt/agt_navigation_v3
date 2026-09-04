from __future__ import annotations

import argparse
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .map_package import sha256_file, sha256_tree, validate_package


NAV_REQUIRED = ('map.yaml', 'map.pgm')
NAV_OPTIONAL = (
    'elevation.pgm',
    'slope.pgm',
    'roughness.pgm',
    'obstacle.pgm',
    'converter_metadata.yaml',
)


def _safe_component(value: str, field: str) -> str:
    value = value.strip()
    if not value or value in ('.', '..') or '/' in value or '\\' in value:
        raise ValueError(f'{field} must be a non-empty single path component')
    return value


def _copy_file(src: Path, dst: Path) -> None:
    if not src.is_file():
        raise FileNotFoundError(src)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _validate_relocalization_source(path: Path) -> None:
    required = (
        path / 'relocalization_assets.yaml',
        path / 'global_map_downsampled.pcd',
        path / 'voxelmaps_coords' / 'voxel_params.txt',
    )
    for required_path in required:
        if not required_path.is_file():
            raise ValueError(f'relocalization assets missing {required_path.relative_to(path)}')
    if not list((path / 'voxelmaps_coords').glob('*.pcd')):
        raise ValueError('relocalization assets missing voxelmaps_coords/*.pcd')


def build_package(
    map_root: Path,
    map_id: str,
    map_version: str,
    source_pcd: Path,
    navigation_dir: Path,
    rtk_origin: Path | None = None,
    preview: Path | None = None,
    relocalization_assets_dir: Path | None = None,
) -> Path:
    map_id = _safe_component(map_id, 'map_id')
    map_version = _safe_component(map_version, 'map_version')
    map_root = map_root.expanduser().resolve()
    source_pcd = source_pcd.expanduser().resolve()
    navigation_dir = navigation_dir.expanduser().resolve()
    if source_pcd.suffix.lower() != '.pcd' or not source_pcd.is_file():
        raise ValueError(f'source_pcd must be an existing .pcd file: {source_pcd}')
    if not navigation_dir.is_dir():
        raise ValueError(f'navigation_dir must exist: {navigation_dir}')
    for filename in NAV_REQUIRED:
        if not (navigation_dir / filename).is_file():
            raise ValueError(f'navigation_dir is missing required {filename}')

    if relocalization_assets_dir is not None:
        relocalization_assets_dir = relocalization_assets_dir.expanduser().resolve()
        if not relocalization_assets_dir.is_dir():
            raise ValueError(f'relocalization_assets_dir must exist: {relocalization_assets_dir}')
        _validate_relocalization_source(relocalization_assets_dir)

    map_root.mkdir(parents=True, exist_ok=True)
    destination_parent = map_root / map_id
    destination_parent.mkdir(parents=True, exist_ok=True)
    destination = destination_parent / map_version
    if destination.exists():
        raise FileExistsError(
            f'map package already exists: {destination}; create a new map_version instead of overwriting')

    staging = Path(tempfile.mkdtemp(
        prefix=f'.{map_version}.staging.', dir=str(destination_parent)))
    try:
        localization_dst = staging / 'localization' / 'global_map.pcd'
        _copy_file(source_pcd, localization_dst)

        nav_dst = staging / 'navigation'
        nav_dst.mkdir(parents=True, exist_ok=True)
        for filename in NAV_REQUIRED + NAV_OPTIONAL:
            src = navigation_dir / filename
            if src.is_file():
                _copy_file(src, nav_dst / filename)

        assets = {
            'localization_map': {
                'path': 'localization/global_map.pcd',
                'sha256': sha256_file(localization_dst),
            },
            'navigation_map': {
                'path': 'navigation/map.yaml',
                'sha256': sha256_file(nav_dst / 'map.yaml'),
            },
        }

        if relocalization_assets_dir is not None:
            relocalization_dst = staging / 'localization' / 'relocalization'
            shutil.copytree(relocalization_assets_dir, relocalization_dst)
            assets['relocalization_assets'] = {
                'path': 'localization/relocalization',
                'sha256': sha256_tree(relocalization_dst),
            }

        layer_names = {
            'elevation': 'elevation.pgm',
            'slope': 'slope.pgm',
            'roughness': 'roughness.pgm',
            'obstacle': 'obstacle.pgm',
        }
        for asset_name, filename in layer_names.items():
            path = nav_dst / filename
            if path.is_file():
                assets[asset_name] = {
                    'path': f'navigation/{filename}',
                    'sha256': sha256_file(path),
                }

        if rtk_origin is not None:
            rtk_origin = rtk_origin.expanduser().resolve()
            rtk_dst = staging / 'rtk' / 'origin.yaml'
            _copy_file(rtk_origin, rtk_dst)
            assets['rtk_origin'] = {
                'path': 'rtk/origin.yaml',
                'sha256': sha256_file(rtk_dst),
            }

        if preview is not None:
            preview = preview.expanduser().resolve()
            preview_dst = staging / 'preview.png'
            _copy_file(preview, preview_dst)
            assets['preview'] = {
                'path': 'preview.png',
                'sha256': sha256_file(preview_dst),
            }

        metadata = {
            'schema_version': 1,
            'map_id': map_id,
            'map_version': map_version,
            'frame_id': 'map',
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'generator': 'agt_map_manager/create_map_package',
            'assets': assets,
        }
        (staging / 'metadata.yaml').write_text(
            yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')

        validation = validate_package(staging / 'metadata.yaml', verify_hashes=True)
        if not validation.valid:
            raise RuntimeError(f'staged package failed self-validation: {validation.reason}')

        # Atomic directory publication on the same filesystem. Until this rename
        # succeeds, Map Manager recursive discovery cannot see metadata.yaml at
        # the final map_id/map_version path.
        os.rename(staging, destination)
        return destination
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Atomically build a versioned AGT Map Package from a source PCD and converter output.')
    parser.add_argument('--map-root', default='~/.ros/agt_maps')
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--source-pcd', required=True)
    parser.add_argument('--navigation-dir', required=True)
    parser.add_argument('--relocalization-assets-dir')
    parser.add_argument('--rtk-origin')
    parser.add_argument('--preview')
    args = parser.parse_args(argv)

    destination = build_package(
        map_root=Path(args.map_root),
        map_id=args.map_id,
        map_version=args.map_version,
        source_pcd=Path(args.source_pcd),
        navigation_dir=Path(args.navigation_dir),
        relocalization_assets_dir=(
            Path(args.relocalization_assets_dir) if args.relocalization_assets_dir else None),
        rtk_origin=Path(args.rtk_origin) if args.rtk_origin else None,
        preview=Path(args.preview) if args.preview else None,
    )
    print(f'Created validated Map Package: {destination}')


if __name__ == '__main__':
    main()
