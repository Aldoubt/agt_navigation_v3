"""Validate the active-map pointer before binding navigation and localization."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import yaml

from .map_package import PackageInfo, validate_package


@dataclass(frozen=True)
class RuntimeMapBinding:
    generation: int
    package: PackageInfo


def resolve_active_map(
    active_state_file: Path,
    *,
    require_relocalization_assets: bool = True,
) -> RuntimeMapBinding:
    """Resolve one hash-verified package and reject stale pointer fields."""
    active_state_file = active_state_file.expanduser().resolve()
    try:
        state = yaml.safe_load(active_state_file.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'active map state is unreadable: {exc}') from exc
    if not isinstance(state, dict) or state.get('schema_version') != 1:
        raise ValueError('active map state requires schema_version: 1')
    try:
        generation = int(state.get('generation', 0))
    except (TypeError, ValueError) as exc:
        raise ValueError('active map state has invalid generation') from exc
    if generation < 1:
        raise ValueError('active map state generation must be >= 1')

    metadata_path = Path(str(state.get('metadata_path', ''))).expanduser()
    if not metadata_path.is_file():
        raise ValueError('active map state metadata_path is missing')
    package = validate_package(metadata_path, verify_hashes=True)
    if not package.valid:
        raise ValueError(f'active map package is invalid: {package.reason}')

    expected = {
        'map_id': package.map_id,
        'map_version': package.map_version,
        'package_path': str(package.package_path),
        'navigation_map_yaml': package.asset_path('navigation_map'),
        'localization_map_pcd': package.asset_path('localization_map'),
        'relocalization_assets_path': package.asset_path('relocalization_assets'),
    }
    for key, value in expected.items():
        if str(state.get(key, '')).strip() != value:
            raise ValueError(f'active map state {key} does not match validated metadata')
    if require_relocalization_assets and not expected['relocalization_assets_path']:
        raise ValueError('active map package is missing relocalization assets')
    return RuntimeMapBinding(generation=generation, package=package)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Validate the active AGT Map Package before starting runtime consumers.')
    parser.add_argument('--active-state-file', required=True)
    parser.add_argument('--allow-missing-relocalization-assets', action='store_true')
    args = parser.parse_args(argv)
    try:
        binding = resolve_active_map(
            Path(args.active_state_file),
            require_relocalization_assets=not args.allow_missing_relocalization_assets,
        )
    except ValueError as exc:
        parser.error(str(exc))
    package = binding.package
    print(
        f'Runtime binding valid: {package.map_id}/{package.map_version} '
        f'generation={binding.generation} package={package.package_path}')
