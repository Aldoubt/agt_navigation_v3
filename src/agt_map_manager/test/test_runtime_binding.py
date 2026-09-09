from pathlib import Path

import pytest
import yaml

from agt_map_manager.create_map_package import build_package
from agt_map_manager.map_package import validate_package
from agt_map_manager.runtime_binding import resolve_active_map


def _write_relocalization_assets(root: Path) -> Path:
    root.mkdir(parents=True)
    (root / 'relocalization_assets.yaml').write_text('schema_version: 1\n', encoding='utf-8')
    (root / 'global_map_downsampled.pcd').write_bytes(b'pcd')
    voxel = root / 'voxelmaps_coords'
    voxel.mkdir()
    (voxel / 'voxel_params.txt').write_text('min_level_res 0.5\n', encoding='utf-8')
    (voxel / '0.pcd').write_bytes(b'pcd')
    return root


def _build_active_package(tmp_path: Path):
    source = tmp_path / 'source.pcd'
    source.write_bytes(b'pcd')
    navigation = tmp_path / 'navigation'
    navigation.mkdir()
    (navigation / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
    (navigation / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n', encoding='utf-8')
    package = build_package(
        map_root=tmp_path / 'maps',
        map_id='site_a',
        map_version='v1',
        source_pcd=source,
        navigation_dir=navigation,
        relocalization_assets_dir=_write_relocalization_assets(tmp_path / 'relocalization'),
    )
    info = validate_package(package / 'metadata.yaml', verify_hashes=True)
    assert info.valid
    state = {
        'schema_version': 1,
        'generation': 4,
        'map_id': info.map_id,
        'map_version': info.map_version,
        'package_path': str(info.package_path),
        'metadata_path': str(info.metadata_path),
        'navigation_map_yaml': info.asset_path('navigation_map'),
        'localization_map_pcd': info.asset_path('localization_map'),
        'relocalization_assets_path': info.asset_path('relocalization_assets'),
        'rtk_origin_yaml': '',
    }
    active = tmp_path / 'active_map.yaml'
    active.write_text(yaml.safe_dump(state, sort_keys=False), encoding='utf-8')
    return active


def test_runtime_binding_requires_one_hash_validated_generation(tmp_path: Path):
    binding = resolve_active_map(_build_active_package(tmp_path))
    assert binding.generation == 4
    assert binding.package.key == ('site_a', 'v1')


def test_runtime_binding_rejects_stale_pointer_paths(tmp_path: Path):
    active = _build_active_package(tmp_path)
    state = yaml.safe_load(active.read_text(encoding='utf-8'))
    state['navigation_map_yaml'] = '/tmp/not-the-package-map.yaml'
    active.write_text(yaml.safe_dump(state, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValueError, match='navigation_map_yaml'):
        resolve_active_map(active)
