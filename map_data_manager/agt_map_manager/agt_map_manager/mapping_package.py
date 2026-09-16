"""Read-only loader and integrity validator for mapping-owned Map Packages."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
from typing import Any

import yaml


REQUIRED_FILES = ('map.pcd', 'poses.txt', 'poses_timed.txt', 'calibration.yaml', 'metadata.yaml', 'manifest.yaml', 'checksums.sha256')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_mapping_package(root: str | Path | None = None, site: str = '', version: str = '') -> Path:
    base = Path(root or os.environ.get('AGT_MAP_ROOT', '~/ros2_ws/maps')).expanduser().resolve()
    package = base / site / version
    if not package.is_dir():
        raise FileNotFoundError(f'mapping Map Package does not exist: {package}')
    missing = [name for name in REQUIRED_FILES if not (package / name).is_file()]
    if not (package / 'patches').is_dir():
        missing.append('patches/')
    if missing:
        raise ValueError(f'mapping Map Package is incomplete; missing: {", ".join(missing)}')

    manifest: dict[str, Any] = yaml.safe_load((package / 'manifest.yaml').read_text(encoding='utf-8')) or {}
    if manifest.get('schema_version') != 1 or manifest.get('package_kind') != 'mapping_source':
        raise ValueError('unsupported mapping Map Package manifest')
    checksums = manifest.get('checksums') or {}
    if not isinstance(checksums, dict):
        raise ValueError('manifest checksums must be a mapping')
    for relative, expected in checksums.items():
        path = package / str(relative)
        if not path.is_file() or _sha256(path) != str(expected):
            raise ValueError(f'checksum mismatch: {relative}')

    metadata: dict[str, Any] = yaml.safe_load((package / 'metadata.yaml').read_text(encoding='utf-8')) or {}
    if metadata.get('backend') != 'PGO' or (metadata.get('backend_status') or {}).get('optimized') is not True:
        raise ValueError('mapping Map Package is not an optimized PGO package')
    if metadata.get('pose_semantics') != 'T_map_body' or metadata.get('patches_frame') != 'body':
        raise ValueError('mapping Map Package frame contract is invalid')
    return package


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description='Validate and load a mapping-owned Map Package')
    parser.add_argument('--map-root', default=None)
    parser.add_argument('--site', required=True)
    parser.add_argument('--version', required=True)
    args = parser.parse_args()
    print(load_mapping_package(args.map_root, args.site, args.version))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
