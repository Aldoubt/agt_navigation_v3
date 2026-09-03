from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import yaml


REQUIRED_ASSETS = ('localization_map', 'navigation_map')
KNOWN_ASSETS = (
    'localization_map',
    'navigation_map',
    'rtk_origin',
    'elevation',
    'slope',
    'roughness',
    'obstacle',
    'preview',
)


@dataclass(frozen=True)
class Asset:
    name: str
    path: Path
    sha256: str = ''


@dataclass(frozen=True)
class PackageInfo:
    metadata_path: Path
    package_path: Path
    map_id: str
    map_version: str
    frame_id: str
    valid: bool
    reason: str
    assets: Dict[str, Asset]

    @property
    def key(self) -> Tuple[str, str]:
        return self.map_id, self.map_version

    def asset_path(self, name: str) -> str:
        asset = self.assets.get(name)
        return str(asset.path) if asset else ''


def sha256_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _inside(root: Path, path: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _invalid(metadata_path: Path, package_path: Path, reason: str,
             map_id: str = '', map_version: str = '', frame_id: str = 'map',
             assets: Optional[Dict[str, Asset]] = None) -> PackageInfo:
    return PackageInfo(
        metadata_path=metadata_path,
        package_path=package_path,
        map_id=map_id,
        map_version=map_version,
        frame_id=frame_id,
        valid=False,
        reason=reason,
        assets=assets or {},
    )


def validate_package(metadata_path: Path, verify_hashes: bool = True) -> PackageInfo:
    metadata_path = metadata_path.expanduser().resolve()
    package_path = metadata_path.parent.resolve()

    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        return _invalid(metadata_path, package_path, f'metadata_read_failed:{exc}')

    if not isinstance(data, dict):
        return _invalid(metadata_path, package_path, 'metadata_must_be_mapping')

    try:
        schema_version = int(data.get('schema_version', -1))
    except (TypeError, ValueError):
        return _invalid(metadata_path, package_path, 'invalid_schema_version')
    if schema_version != 1:
        return _invalid(metadata_path, package_path, 'unsupported_schema_version')

    map_id = str(data.get('map_id', '')).strip()
    map_version = str(data.get('map_version', '')).strip()
    frame_id = str(data.get('frame_id', 'map')).strip() or 'map'
    if not map_id:
        return _invalid(metadata_path, package_path, 'missing_map_id')
    if not map_version:
        return _invalid(metadata_path, package_path, 'missing_map_version', map_id=map_id)
    if frame_id != 'map':
        return _invalid(
            metadata_path, package_path, 'frame_id_must_be_map',
            map_id=map_id, map_version=map_version, frame_id=frame_id)

    raw_assets = data.get('assets')
    if not isinstance(raw_assets, dict):
        return _invalid(
            metadata_path, package_path, 'assets_must_be_mapping',
            map_id=map_id, map_version=map_version, frame_id=frame_id)

    assets: Dict[str, Asset] = {}
    for name in KNOWN_ASSETS:
        entry = raw_assets.get(name)
        if entry is None:
            continue
        if isinstance(entry, str):
            rel_path = entry
            expected_hash = ''
        elif isinstance(entry, dict):
            rel_path = str(entry.get('path', '')).strip()
            expected_hash = str(entry.get('sha256', '')).strip().lower()
        else:
            return _invalid(
                metadata_path, package_path, f'asset_{name}_must_be_string_or_mapping',
                map_id, map_version, frame_id, assets)

        if not rel_path:
            return _invalid(
                metadata_path, package_path, f'asset_{name}_missing_path',
                map_id, map_version, frame_id, assets)
        candidate = (package_path / rel_path).resolve()
        if not _inside(package_path, candidate):
            return _invalid(
                metadata_path, package_path, f'asset_{name}_escapes_package_root',
                map_id, map_version, frame_id, assets)
        if not candidate.is_file():
            return _invalid(
                metadata_path, package_path, f'asset_{name}_missing:{rel_path}',
                map_id, map_version, frame_id, assets)
        if expected_hash:
            if len(expected_hash) != 64 or any(ch not in '0123456789abcdef' for ch in expected_hash):
                return _invalid(
                    metadata_path, package_path, f'asset_{name}_invalid_sha256',
                    map_id, map_version, frame_id, assets)
            if verify_hashes:
                actual = sha256_file(candidate)
                if actual != expected_hash:
                    return _invalid(
                        metadata_path, package_path,
                        f'asset_{name}_sha256_mismatch:{actual}',
                        map_id, map_version, frame_id, assets)
        assets[name] = Asset(name=name, path=candidate, sha256=expected_hash)

    for required in REQUIRED_ASSETS:
        if required not in assets:
            return _invalid(
                metadata_path, package_path, f'missing_required_asset:{required}',
                map_id, map_version, frame_id, assets)

    nav_yaml = assets['navigation_map'].path
    if nav_yaml.suffix.lower() not in ('.yaml', '.yml'):
        return _invalid(
            metadata_path, package_path, 'navigation_map_must_be_yaml',
            map_id, map_version, frame_id, assets)
    if assets['localization_map'].path.suffix.lower() != '.pcd':
        return _invalid(
            metadata_path, package_path, 'localization_map_must_be_pcd',
            map_id, map_version, frame_id, assets)

    return PackageInfo(
        metadata_path=metadata_path,
        package_path=package_path,
        map_id=map_id,
        map_version=map_version,
        frame_id=frame_id,
        valid=True,
        reason='valid',
        assets=assets,
    )


def discover_packages(root: Path, verify_hashes: bool = True) -> Iterable[PackageInfo]:
    root = root.expanduser().resolve()
    if not root.exists():
        return []

    try:
        metadata_files = sorted(root.rglob('metadata.yaml'))
    except OSError:
        return []

    packages = [validate_package(path, verify_hashes) for path in metadata_files]
    counts: Dict[Tuple[str, str], int] = {}
    for package in packages:
        if package.map_id and package.map_version:
            counts[package.key] = counts.get(package.key, 0) + 1

    result = []
    for package in packages:
        if package.map_id and package.map_version and counts.get(package.key, 0) > 1:
            result.append(PackageInfo(
                metadata_path=package.metadata_path,
                package_path=package.package_path,
                map_id=package.map_id,
                map_version=package.map_version,
                frame_id=package.frame_id,
                valid=False,
                reason='duplicate_map_id_and_version',
                assets=package.assets,
            ))
        else:
            result.append(package)
    return result
