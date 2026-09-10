from pathlib import Path

import yaml

from agt_map_manager.map_package import sha256_file, sha256_tree
from agt_map_manager.map_validator import validate_map_package


def _make_package(root: Path, *, polar=True):
    package = root / 'site' / 'v1'
    (package / 'navigation').mkdir(parents=True)
    (package / 'localization' / 'relocalization' / 'voxelmaps_coords').mkdir(parents=True)
    (package / 'navigation' / 'map.pgm').write_bytes(b'P5\n1 1\n255\n0')
    (package / 'navigation' / 'map.yaml').write_text('image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n', encoding='utf-8')
    (package / 'localization' / 'global_map.pcd').write_text('VERSION 0.7\n', encoding='utf-8')
    assets = package / 'localization' / 'relocalization'
    (assets / 'global_map_downsampled.pcd').write_text('VERSION 0.7\n', encoding='utf-8')
    (assets / 'relocalization_assets.yaml').write_text('downsampled_map: global_map_downsampled.pcd\n', encoding='utf-8')
    (assets / 'voxelmaps_coords' / 'voxel_params.txt').write_text('0.5\n', encoding='utf-8')
    (assets / 'voxelmaps_coords' / '0.pcd').write_text('VERSION 0.7\n', encoding='utf-8')
    if polar:
        (assets / 'polar_context.db').write_bytes(b'db')
        (assets / 'polar_context.yaml').write_text('schema: 1\n', encoding='utf-8')
    metadata = {
        'schema_version': 1, 'map_id': 'site', 'map_version': 'v1', 'frame_id': 'map',
        'assets': {
            'localization_map': {'path': 'localization/global_map.pcd', 'sha256': sha256_file(package / 'localization' / 'global_map.pcd')},
            'navigation_map': {'path': 'navigation/map.yaml', 'sha256': sha256_file(package / 'navigation' / 'map.yaml')},
            'relocalization_assets': {'path': 'localization/relocalization', 'sha256': sha256_tree(assets)},
        },
    }
    (package / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    return package / 'metadata.yaml'


def test_validator_requires_polar_context_and_bbs_assets(tmp_path):
    assert validate_map_package(_make_package(tmp_path))['result'] == 'PASS'
    report = validate_map_package(_make_package(tmp_path / 'missing', polar=False))
    assert report['result'] == 'FAIL'
    assert report['checks']['polar_context'] == 'FAIL'
