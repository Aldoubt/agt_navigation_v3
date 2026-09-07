"""Publish an HMI-edited Nav2 map as a new immutable AGT Map Package."""

from __future__ import annotations

import argparse
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .create_map_package import NAV_OPTIONAL, build_package
from .map_package import PackageInfo, validate_package


DEFAULT_MAP_ROOT = Path('/home/yangxuan/ros2_ws/agt_data/maps')
DEFAULT_ACTIVE_STATE = DEFAULT_MAP_ROOT / 'active_map.yaml'


def _atomic_write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_path, path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def _read_generation(path: Path) -> int:
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        return max(0, int(data.get('generation', 0)))
    except (OSError, ValueError, TypeError, yaml.YAMLError):
        return 0


def _activate(info: PackageInfo, state_file: Path) -> None:
    if not info.valid:
        raise ValueError(f'cannot activate invalid package: {info.reason}')
    generation = _read_generation(state_file) + 1
    _atomic_write_yaml(state_file, {
        'schema_version': 1,
        'generation': generation,
        'map_id': info.map_id,
        'map_version': info.map_version,
        'package_path': str(info.package_path),
        'metadata_path': str(info.metadata_path),
        'navigation_map_yaml': info.asset_path('navigation_map'),
        'localization_map_pcd': info.asset_path('localization_map'),
        'relocalization_assets_path': info.asset_path('relocalization_assets'),
        'rtk_origin_yaml': info.asset_path('rtk_origin'),
    })


def _copy_hmi_navigation(edited_map_yaml: Path, staging_navigation: Path, base: PackageInfo) -> None:
    data = yaml.safe_load(edited_map_yaml.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise ValueError('edited map YAML must be a mapping')
    image_name = str(data.get('image', '')).strip()
    if not image_name:
        raise ValueError('edited map YAML is missing image')
    image_path = (edited_map_yaml.parent / image_name).resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f'edited map image does not exist: {image_path}')

    staging_navigation.mkdir(parents=True, exist_ok=True)
    data['image'] = 'map.pgm'
    (staging_navigation / 'map.yaml').write_text(
        yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    shutil.copy2(image_path, staging_navigation / 'map.pgm')

    # Preserve baseline provenance that still describes the unedited map build.
    # The HMI edit itself is recorded separately below.
    for name in NAV_OPTIONAL:
        source = base.package_path / 'navigation' / name
        if source.is_file() and name not in {'hmi_edit_metadata.yaml', 'map.topology'}:
            shutil.copy2(source, staging_navigation / name)

    source_topology = edited_map_yaml.with_suffix('.topology')
    if source_topology.is_file():
        shutil.copy2(source_topology, staging_navigation / 'map.topology')

    edit_metadata = {
        'generator': 'agt_map_manager/promote_hmi_navigation_edit',
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'base_map_id': base.map_id,
        'base_map_version': base.map_version,
        'edited_map_yaml': str(edited_map_yaml.resolve()),
        'topology_imported': source_topology.is_file(),
        'note': (
            'HMI edit changes only the navigation occupancy/topology asset. '
            'The localization PCD and relocalization assets are copied unchanged.'),
    }
    (staging_navigation / 'hmi_edit_metadata.yaml').write_text(
        yaml.safe_dump(edit_metadata, sort_keys=False), encoding='utf-8')


def promote(
    base_metadata: Path,
    edited_map_yaml: Path,
    map_root: Path,
    map_id: str,
    map_version: str,
    activate: bool,
    active_state_file: Path,
) -> Path:
    base = validate_package(base_metadata, verify_hashes=True)
    if not base.valid:
        raise ValueError(f'base Map Package is invalid: {base.reason}')
    edited_map_yaml = edited_map_yaml.expanduser().resolve()
    if not edited_map_yaml.is_file():
        raise FileNotFoundError(edited_map_yaml)

    with tempfile.TemporaryDirectory(prefix='agt_hmi_navigation_edit_') as tmp:
        navigation = Path(tmp) / 'navigation'
        _copy_hmi_navigation(edited_map_yaml, navigation, base)
        destination = build_package(
            map_root=map_root,
            map_id=map_id,
            map_version=map_version,
            source_pcd=Path(base.asset_path('localization_map')),
            navigation_dir=navigation,
            relocalization_assets_dir=(
                Path(base.asset_path('relocalization_assets'))
                if base.asset_path('relocalization_assets') else None),
        )

    info = validate_package(destination / 'metadata.yaml', verify_hashes=True)
    if not info.valid:
        raise RuntimeError(f'new HMI Map Package failed validation: {info.reason}')
    if activate:
        _activate(info, active_state_file.expanduser().resolve())
    return destination


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Import an HMI-edited map as a new immutable AGT Map Package.')
    parser.add_argument('--base-metadata', required=True)
    parser.add_argument('--edited-map-yaml', required=True)
    parser.add_argument('--map-root', default=str(DEFAULT_MAP_ROOT))
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--activate', action='store_true')
    parser.add_argument('--active-state-file', default=str(DEFAULT_ACTIVE_STATE))
    args = parser.parse_args(argv)
    destination = promote(
        base_metadata=Path(args.base_metadata),
        edited_map_yaml=Path(args.edited_map_yaml),
        map_root=Path(args.map_root),
        map_id=args.map_id,
        map_version=args.map_version,
        activate=bool(args.activate),
        active_state_file=Path(args.active_state_file),
    )
    print(f'Created HMI-edited Map Package: {destination}')


if __name__ == '__main__':
    main()
