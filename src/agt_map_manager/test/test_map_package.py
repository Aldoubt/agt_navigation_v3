import hashlib
from pathlib import Path

import yaml

from agt_map_manager.map_package import discover_packages, validate_package


def _write_package(root: Path, map_id='site_a', version='v1', nav_path=None):
    package = root / map_id / version
    (package / 'localization').mkdir(parents=True)
    (package / 'navigation').mkdir(parents=True)
    pcd = package / 'localization' / 'global_map.pcd'
    nav = package / 'navigation' / 'map.yaml'
    pcd.write_bytes(b'pcd-test')
    nav.write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n', encoding='utf-8')
    data = {
        'schema_version': 1,
        'map_id': map_id,
        'map_version': version,
        'frame_id': 'map',
        'assets': {
            'localization_map': {'path': 'localization/global_map.pcd'},
            'navigation_map': {'path': nav_path or 'navigation/map.yaml'},
        },
    }
    metadata = package / 'metadata.yaml'
    metadata.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return package, metadata, pcd, nav


def test_valid_package(tmp_path):
    _, metadata, _, _ = _write_package(tmp_path)
    info = validate_package(metadata)
    assert info.valid
    assert info.map_id == 'site_a'
    assert info.map_version == 'v1'


def test_invalid_schema_is_rejected_not_raised(tmp_path):
    _, metadata, _, _ = _write_package(tmp_path)
    data = yaml.safe_load(metadata.read_text(encoding='utf-8'))
    data['schema_version'] = 'not-a-number'
    metadata.write_text(yaml.safe_dump(data), encoding='utf-8')
    info = validate_package(metadata)
    assert not info.valid
    assert info.reason == 'invalid_schema_version'


def test_asset_path_cannot_escape_package(tmp_path):
    _, metadata, _, _ = _write_package(tmp_path, nav_path='../../outside.yaml')
    outside = tmp_path / 'site_a' / 'outside.yaml'
    outside.write_text('bad', encoding='utf-8')
    info = validate_package(metadata)
    assert not info.valid
    assert 'escapes_package_root' in info.reason


def test_declared_hash_mismatch_is_rejected(tmp_path):
    _, metadata, pcd, _ = _write_package(tmp_path)
    data = yaml.safe_load(metadata.read_text(encoding='utf-8'))
    data['assets']['localization_map']['sha256'] = hashlib.sha256(b'other').hexdigest()
    metadata.write_text(yaml.safe_dump(data), encoding='utf-8')
    info = validate_package(metadata, verify_hashes=True)
    assert not info.valid
    assert 'sha256_mismatch' in info.reason
    assert pcd.exists()


def test_duplicate_map_id_version_marks_both_invalid(tmp_path):
    _, metadata_a, _, _ = _write_package(tmp_path / 'root_a', map_id='same', version='v1')
    _, metadata_b, _, _ = _write_package(tmp_path / 'root_b', map_id='same', version='v1')
    assert metadata_a.exists() and metadata_b.exists()
    packages = list(discover_packages(tmp_path, verify_hashes=False))
    matches = [p for p in packages if p.key == ('same', 'v1')]
    assert len(matches) == 2
    assert all(not p.valid and p.reason == 'duplicate_map_id_and_version' for p in matches)
