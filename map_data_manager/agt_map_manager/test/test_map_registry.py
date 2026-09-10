from pathlib import Path

import yaml

from agt_map_manager.map_registry import MapRecord, MapRegistry, MapState


def test_registry_transitions_and_active_archives_previous(tmp_path: Path):
    registry = MapRegistry(tmp_path / 'map_registry.yaml')
    registry.records[('site', 'v1')] = MapRecord('site', 'v1', MapState.CANDIDATE, 'now', str(tmp_path / 'v1'))
    registry.records[('site', 'v2')] = MapRecord('site', 'v2', MapState.CANDIDATE, 'now', str(tmp_path / 'v2'))
    registry.save()

    registry.transition('site', 'v1', MapState.VALIDATING)
    registry.transition('site', 'v1', MapState.VALIDATED)
    registry.transition('site', 'v2', MapState.VALIDATING)
    registry.transition('site', 'v2', MapState.VALIDATED)
    registry.set_active('site', 'v1')
    registry.set_active('site', 'v2')

    assert registry.get('site', 'v1').status == MapState.ARCHIVED
    assert registry.get('site', 'v2').status == MapState.ACTIVE
    data = yaml.safe_load((tmp_path / 'map_registry.yaml').read_text())
    assert len(data['maps']) == 2


def test_active_cannot_be_discarded_by_state_machine(tmp_path: Path):
    registry = MapRegistry(tmp_path / 'map_registry.yaml')
    registry.records[('site', 'v1')] = MapRecord('site', 'v1', MapState.ACTIVE, 'now', str(tmp_path / 'v1'))
    try:
        registry.transition('site', 'v1', MapState.ARCHIVED)
    except ValueError:
        # Direct deletion is guarded by MapManager; archival remains allowed.
        assert False
