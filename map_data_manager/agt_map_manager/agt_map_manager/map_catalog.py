"""Registry backed map selection for a specific robot profile."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from .map_package import sha256_tree, validate_package


@dataclass(frozen=True)
class ResolvedMap:
    map_id: str
    map_version: str
    package_path: Path
    navigation_map: Path
    localization_map: Path
    relocalization_assets: Path


def read_registry(path: Path) -> dict:
    path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'map registry unreadable: {exc}') from exc
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('map registry requires schema_version: 1')
    if data.get('default_map') not in ('latest_validated', 'active'):
        raise ValueError('map registry default_map must be latest_validated or active')
    if not isinstance(data.get('maps'), dict):
        raise ValueError('map registry maps must be a mapping')
    return data


def resolve_map(spec: str, robot_profile: str, registry_path: Path) -> ResolvedMap:
    registry_path = Path(registry_path).expanduser().resolve()
    registry = read_registry(registry_path)
    spec = spec.strip() or 'auto'
    robot_profile = robot_profile.strip()
    if not robot_profile:
        raise ValueError('robot profile is required')

    default_id = str(registry.get('default_map_id', '')).strip()
    if spec in ('auto', 'active', 'latest'):
        if not default_id:
            raise ValueError('map registry default_map_id is required')
        map_id = default_id
        pointer = registry['default_map'] if spec == 'auto' else (
            'latest_validated' if spec == 'latest' else 'active')
        version = str(registry['maps'].get(map_id, {}).get(pointer, '')).strip()
    elif '/' in spec:
        parts = spec.split('/')
        if len(parts) != 2 or not all(parts):
            raise ValueError('map spec must be map_id/map_version')
        map_id, version = parts
    else:
        map_id = spec
        version = str(registry['maps'].get(map_id, {}).get('latest_validated', '')).strip()

    entry = registry['maps'].get(map_id)
    if not isinstance(entry, dict) or not version:
        raise ValueError(f'map not registered or pointer empty: {map_id}/{version}')
    versions = entry.get('versions')
    if not isinstance(versions, dict) or version not in versions:
        raise ValueError(f'map version not registered: {map_id}/{version}')
    record = versions[version]
    if not isinstance(record, dict) or record.get('status') != 'VALIDATED':
        raise ValueError(f'map version is not VALIDATED: {map_id}/{version}')
    package_path = (registry_path.parent / map_id / version).resolve()
    if package_path.parent != (registry_path.parent / map_id).resolve():
        raise ValueError('map version escapes registry root')
    expected_hash = record.get('package_sha256', '')
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ValueError('map registry package checksum missing')
    if sha256_tree(package_path) != expected_hash:
        raise ValueError('MAP_ERROR map package checksum mismatch')
    info = validate_package(package_path / 'metadata.yaml', verify_hashes=True)
    if not info.valid or info.map_id != map_id or info.map_version != version:
        raise ValueError(f'map package invalid: {info.reason}')
    metadata = yaml.safe_load(info.metadata_path.read_text(encoding='utf-8'))
    compatibility = metadata.get('compatibility') or {}
    profiles = compatibility.get('robot_profiles') or []
    navigation = metadata.get('navigation') or {}
    profile_nav = navigation.get(robot_profile) or {}
    if robot_profile not in profiles or not isinstance(profile_nav, dict):
        raise ValueError(f'MAP_ERROR incompatible map/robot: {map_id}/{version} {robot_profile}')
    expected_nav = (package_path / str(profile_nav.get('map', ''))).resolve()
    if expected_nav != Path(info.asset_path('navigation_map')):
        raise ValueError('MAP_ERROR navigation asset does not match robot profile')
    relocalization = info.asset_path('relocalization_assets')
    if not relocalization:
        raise ValueError('MAP_ERROR relocalization assets missing')
    return ResolvedMap(map_id, version, package_path, expected_nav,
                       Path(info.asset_path('localization_map')), Path(relocalization))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Resolve a validated map for one robot profile')
    parser.add_argument('--map', default='auto')
    parser.add_argument('--robot', default='bunker_v1')
    parser.add_argument('--registry', default=os.environ.get('AGT_MAP_REGISTRY', ''))
    args = parser.parse_args(argv)
    if not args.registry:
        parser.error('--registry or AGT_MAP_REGISTRY is required')
    try:
        result = resolve_map(args.map, args.robot, Path(args.registry))
    except ValueError as exc:
        parser.error(str(exc))
    for key, value in (
        ('map_id', result.map_id), ('map_version', result.map_version),
        ('map_root', result.package_path), ('navigation_map', result.navigation_map),
        ('localization_map', result.localization_map),
        ('relocalization_assets', result.relocalization_assets),
    ):
        print(f'{key}={value}')
