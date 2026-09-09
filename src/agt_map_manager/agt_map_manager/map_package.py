from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import yaml


REQUIRED_ASSETS = ('localization_map', 'navigation_map')
KNOWN_ASSETS = (
    'localization_map',
    'relocalization_assets',
    'navigation_map',
    'rtk_origin',
    'elevation',
    'slope',
    'roughness',
    'obstacle',
    'preview',
    'generation_pipeline',
    'quality_report',
)
DIRECTORY_ASSETS = {'relocalization_assets'}


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


def sha256_tree(path: Path) -> str:
    """Hash a directory deterministically from relative paths and file bytes."""
    path = path.resolve()
    digest = hashlib.sha256()
    for file_path in sorted(p for p in path.rglob('*') if p.is_file()):
        rel = file_path.relative_to(path).as_posix().encode('utf-8')
        digest.update(len(rel).to_bytes(4, 'big'))
        digest.update(rel)
        file_hash = sha256_file(file_path).encode('ascii')
        digest.update(file_hash)
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


def _validate_relocalization_assets(path: Path) -> str:
    required_files = (
        path / 'relocalization_assets.yaml',
        path / 'global_map_downsampled.pcd',
        path / 'polar_context.db',
        path / 'polar_context.yaml',
        path / 'voxelmaps_coords' / 'voxel_params.txt',
    )
    for required in required_files:
        if not required.is_file():
            return f'relocalization_assets_missing:{required.relative_to(path).as_posix()}'
    voxel_pcds = sorted((path / 'voxelmaps_coords').glob('*.pcd'))
    if not voxel_pcds:
        return 'relocalization_assets_missing:voxelmaps_coords/*.pcd'
    return ''


def _validate_navigation_map(path: Path, package_path: Path) -> str:
    """Validate the minimum Nav2 runtime contract owned by a Map Package."""
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        return f'navigation_map_read_failed:{exc}'
    if not isinstance(data, dict):
        return 'navigation_map_must_be_mapping'

    image = data.get('image')
    if not isinstance(image, str) or not image.strip():
        return 'navigation_map_missing_image'
    image_path = (path.parent / image).resolve()
    if not _inside(package_path, image_path):
        return 'navigation_map_image_escapes_package_root'
    if not image_path.is_file():
        return f'navigation_map_image_missing:{image}'

    try:
        resolution = float(data.get('resolution', 0.0))
    except (TypeError, ValueError):
        return 'navigation_map_invalid_resolution'
    if resolution <= 0.0:
        return 'navigation_map_invalid_resolution'

    origin = data.get('origin')
    if not isinstance(origin, list) or len(origin) != 3:
        return 'navigation_map_invalid_origin'
    try:
        [float(value) for value in origin]
    except (TypeError, ValueError):
        return 'navigation_map_invalid_origin'

    try:
        from agt_map_converter.validate_nav_map import (
            UNKNOWN_OCCUPANCY_PROBABILITY,
            UNKNOWN_PGM_VALUE,
            read_pgm_header,
        )

        _, _, payload = read_pgm_header(image_path)
        if (
            UNKNOWN_PGM_VALUE in payload
            and int(data.get('negate', 0)) == 0
            and float(data.get('free_thresh', 0.25)) > UNKNOWN_OCCUPANCY_PROBABILITY
        ):
            return 'navigation_map_unknown_cells_interpreted_as_free'
    except (OSError, ValueError) as exc:
        return f'navigation_map_pgm_invalid:{exc}'
    return ''


def _validate_provenance_sections(data: Dict, assets: Dict[str, Asset]) -> str:
    """Keep generated-package provenance explicit without rejecting legacy packages."""
    generation = data.get('generation')
    if generation is not None:
        if not isinstance(generation, dict):
            return 'generation_must_be_mapping'
        if generation.get('pipeline_asset') != 'generation_pipeline':
            return 'generation_pipeline_asset_must_be_generation_pipeline'
        if 'generation_pipeline' not in assets:
            return 'generation_pipeline_asset_missing'
        source_hash = str(generation.get('source_pcd_sha256', '')).strip().lower()
        if len(source_hash) != 64 or any(ch not in '0123456789abcdef' for ch in source_hash):
            return 'generation_invalid_source_pcd_sha256'

    quality = data.get('quality')
    if quality is not None:
        if not isinstance(quality, dict):
            return 'quality_must_be_mapping'
        if quality.get('report_asset') != 'quality_report':
            return 'quality_report_asset_must_be_quality_report'
        if 'quality_report' not in assets:
            return 'quality_report_asset_missing'
        if quality.get('status') not in ('pass', 'fail'):
            return 'quality_status_must_be_pass_or_fail'
    return ''


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

        if name in DIRECTORY_ASSETS:
            if not candidate.is_dir():
                return _invalid(
                    metadata_path, package_path, f'asset_{name}_missing_directory:{rel_path}',
                    map_id, map_version, frame_id, assets)
            internal_error = _validate_relocalization_assets(candidate)
            if internal_error:
                return _invalid(
                    metadata_path, package_path, internal_error,
                    map_id, map_version, frame_id, assets)
        elif not candidate.is_file():
            return _invalid(
                metadata_path, package_path, f'asset_{name}_missing:{rel_path}',
                map_id, map_version, frame_id, assets)

        if expected_hash:
            if len(expected_hash) != 64 or any(ch not in '0123456789abcdef' for ch in expected_hash):
                return _invalid(
                    metadata_path, package_path, f'asset_{name}_invalid_sha256',
                    map_id, map_version, frame_id, assets)
            if verify_hashes:
                actual = sha256_tree(candidate) if name in DIRECTORY_ASSETS else sha256_file(candidate)
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

    nav_error = _validate_navigation_map(nav_yaml, package_path)
    if nav_error:
        return _invalid(
            metadata_path, package_path, nav_error,
            map_id, map_version, frame_id, assets)

    provenance_error = _validate_provenance_sections(data, assets)
    if provenance_error:
        return _invalid(
            metadata_path, package_path, provenance_error,
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
