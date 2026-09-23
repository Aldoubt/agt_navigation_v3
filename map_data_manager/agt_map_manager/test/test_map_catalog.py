from pathlib import Path

import pytest
import yaml

from agt_map_manager.map_catalog import resolve_map
from agt_map_manager.map_package import sha256_tree, validate_package
from test_map_package import _add_relocalization_assets, _write_package


def _registered_map(tmp_path: Path):
    package, metadata, _, nav = _write_package(tmp_path, map_id='orchard', version='v1')
    _add_relocalization_assets(package, metadata)
    profile_nav = package / 'navigation' / 'bunker_v1'
    profile_nav.mkdir()
    (package / 'navigation' / 'map.pgm').rename(profile_nav / 'map.pgm')
    nav.rename(profile_nav / 'map.yaml')
    data = yaml.safe_load(metadata.read_text(encoding='utf-8'))
    data['assets']['navigation_map']['path'] = 'navigation/bunker_v1/map.yaml'
    data['compatibility'] = {'robot_profiles': ['bunker_v1']}
    data['navigation'] = {'bunker_v1': {'map': 'navigation/bunker_v1/map.yaml'}}
    metadata.write_text(yaml.safe_dump(data), encoding='utf-8')
    assert validate_package(metadata).valid
    registry = tmp_path / 'registry.yaml'
    registry.write_text(yaml.safe_dump({
        'schema_version': 1,
        'default_map': 'latest_validated',
        'default_map_id': 'orchard',
        'maps': {'orchard': {
            'active': 'v1', 'latest_validated': 'v1',
            'versions': {'v1': {'status': 'VALIDATED',
                                'package_sha256': sha256_tree(package)}},
        }},
    }), encoding='utf-8')
    return package, registry


@pytest.mark.parametrize('spec', ['auto', 'active', 'latest', 'orchard', 'orchard/v1'])
def test_resolver_selection_modes(tmp_path, spec):
    package, registry = _registered_map(tmp_path)
    selected = resolve_map(spec, 'bunker_v1', registry)
    assert selected.package_path == package
    assert selected.navigation_map == package / 'navigation/bunker_v1/map.yaml'


def test_resolver_rejects_incompatible_robot(tmp_path):
    _, registry = _registered_map(tmp_path)
    with pytest.raises(ValueError, match='MAP_ERROR incompatible'):
        resolve_map('auto', 'another_robot', registry)


def test_resolver_rejects_tampered_package(tmp_path):
    package, registry = _registered_map(tmp_path)
    (package / 'navigation/bunker_v1/map.pgm').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='checksum mismatch'):
        resolve_map('auto', 'bunker_v1', registry)


def test_resolver_keeps_active_and_latest_distinct(tmp_path):
    _, registry = _registered_map(tmp_path)
    data = yaml.safe_load(registry.read_text(encoding='utf-8'))
    data['maps']['orchard']['active'] = ''
    registry.write_text(yaml.safe_dump(data), encoding='utf-8')
    assert resolve_map('auto', 'bunker_v1', registry).map_version == 'v1'
    with pytest.raises(ValueError, match='pointer empty'):
        resolve_map('active', 'bunker_v1', registry)
