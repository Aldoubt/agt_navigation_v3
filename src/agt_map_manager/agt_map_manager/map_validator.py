"""Lifecycle validation report for a complete navigation Map Package."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml

from .map_package import validate_package


def validate_map_package(metadata_path: Path) -> Dict[str, Any]:
    metadata_path = Path(metadata_path).expanduser().resolve()
    info = validate_package(metadata_path, verify_hashes=True)
    checks = {
        'navigation_map': False,
        'localization_map': False,
        'polar_context': False,
        'bbs_assets': False,
        'hash': bool(info.valid),
        'frame_id': info.frame_id == 'map',
        'version_consistency': bool(info.map_id and info.map_version),
    }
    if info.valid:
        nav = Path(info.asset_path('navigation_map'))
        loc = Path(info.asset_path('localization_map'))
        assets = Path(info.asset_path('relocalization_assets'))
        checks['navigation_map'] = nav.is_file() and (nav.parent / 'map.pgm').is_file()
        checks['localization_map'] = loc.is_file()
        checks['polar_context'] = (assets / 'polar_context.db').is_file() and (assets / 'polar_context.yaml').is_file()
        checks['bbs_assets'] = (assets / 'global_map_downsampled.pcd').is_file() and (assets / 'voxelmaps_coords').is_dir() and any((assets / 'voxelmaps_coords').glob('*.pcd'))
    passed = all(checks.values())
    return {
        'result': 'PASS' if passed else 'FAIL',
        'map_id': info.map_id,
        'map_version': info.map_version,
        'metadata_path': str(metadata_path),
        'reason': '' if passed else (info.reason or 'required_asset_missing'),
        'checks': {key: 'PASS' if value else 'FAIL' for key, value in checks.items()},
    }


def write_validation_report(metadata_path: Path, report: Dict[str, Any]) -> Path:
    path = Path(metadata_path).parent / 'validation_report.yaml'
    path.write_text(yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
    return path
