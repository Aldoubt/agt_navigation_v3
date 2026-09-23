"""Promotion fault injection; candidate and map paths are pytest temporary paths."""

from pathlib import Path

import pytest
import yaml

import agt_map_manager.map_promotion as promotion
from agt_map_manager.map_catalog import resolve_map
from agt_map_manager.map_package import validate_package
from test_map_package import _add_relocalization_assets, _write_package


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    experiments = tmp_path / 'experiments'
    map_root = tmp_path / 'maps'
    monkeypatch.setattr(promotion, 'EXPERIMENTS_ROOT', experiments)

    def candidate(version):
        path, metadata, _, _ = _write_package(
            experiments, map_id='orchard', version=version)
        _add_relocalization_assets(path, metadata)
        (path / 'candidate_state.yaml').write_text(
            'schema_version: 1\nstate: BUILT\n', encoding='utf-8')
        return path

    return map_root, candidate


def _registry(root: Path):
    return yaml.safe_load((root / 'registry.yaml').read_text(encoding='utf-8'))


def _state(candidate: Path):
    return yaml.safe_load((candidate / 'candidate_state.yaml').read_text(encoding='utf-8'))


def _assert_failed_activation_keeps_previous(root, candidate, previous_pointer):
    entry = _registry(root)['maps']['orchard']
    assert entry['active'] == 'v1'
    assert entry['latest_validated'] == 'v2'
    assert entry['versions']['v2']['status'] == 'VALIDATED'
    assert (root / 'active_map.yaml').read_bytes() == previous_pointer
    assert _state(candidate)['state'] == 'PROMOTED'
    assert 'error' in _state(candidate)
    assert validate_package(root / 'orchard/v2/metadata.yaml', verify_hashes=True).valid
    assert resolve_map('active', 'bunker_v1', root / 'registry.yaml').map_version == 'v1'
    assert resolve_map('auto', 'bunker_v1', root / 'registry.yaml').map_version == 'v2'


def test_promotion_and_activation_commit_together(sandbox):
    root, candidate = sandbox
    source = candidate('v1')
    published = promotion.promote_candidate(source, root, activate=True)
    assert published == root / 'orchard/v1'
    assert _registry(root)['maps']['orchard']['active'] == 'v1'
    assert _state(source)['state'] == 'PROMOTED'
    assert yaml.safe_load((root / 'active_map.yaml').read_text())['map_version'] == 'v1'


def test_activation_failure_does_not_reject_published_version(sandbox, monkeypatch):
    root, candidate = sandbox
    promotion.promote_candidate(candidate('v1'), root, activate=True)
    previous_pointer = (root / 'active_map.yaml').read_bytes()
    source = candidate('v2')

    def fail_before_write(*_args):
        raise OSError('injected activation failure')

    monkeypatch.setattr(promotion, '_activate', fail_before_write)
    with pytest.raises(OSError, match='injected activation failure'):
        promotion.promote_candidate(source, root, activate=True)
    _assert_failed_activation_keeps_previous(root, source, previous_pointer)


def test_activation_failure_after_pointer_write_restores_previous(sandbox, monkeypatch):
    root, candidate = sandbox
    promotion.promote_candidate(candidate('v1'), root, activate=True)
    previous_pointer = (root / 'active_map.yaml').read_bytes()
    source = candidate('v2')
    real_activate = promotion._activate

    def fail_after_write(info, state_path):
        real_activate(info, state_path)
        raise OSError('injected post-write failure')

    monkeypatch.setattr(promotion, '_activate', fail_after_write)
    with pytest.raises(OSError, match='injected post-write failure'):
        promotion.promote_candidate(source, root, activate=True)
    _assert_failed_activation_keeps_previous(root, source, previous_pointer)


def test_registry_activation_commit_failure_restores_pointer(sandbox, monkeypatch):
    root, candidate = sandbox
    promotion.promote_candidate(candidate('v1'), root, activate=True)
    previous_pointer = (root / 'active_map.yaml').read_bytes()
    source = candidate('v2')
    real_write = promotion._atomic_write_yaml

    def fail_second_registry_write(path, data):
        if path == root / 'registry.yaml' and data['maps']['orchard']['active'] == 'v2':
            raise OSError('injected active registry commit failure')
        real_write(path, data)

    monkeypatch.setattr(promotion, '_atomic_write_yaml', fail_second_registry_write)
    with pytest.raises(OSError, match='injected active registry commit failure'):
        promotion.promote_candidate(source, root, activate=True)
    _assert_failed_activation_keeps_previous(root, source, previous_pointer)


def test_initial_registry_failure_removes_unregistered_package(sandbox, monkeypatch):
    root, candidate = sandbox
    promotion.promote_candidate(candidate('v1'), root, activate=True)
    previous_pointer = (root / 'active_map.yaml').read_bytes()
    source = candidate('v2')
    real_write = promotion._atomic_write_yaml

    def fail_initial_registry_write(path, data):
        if path == root / 'registry.yaml' and 'v2' in data['maps']['orchard']['versions']:
            raise OSError('injected registry publish failure')
        real_write(path, data)

    monkeypatch.setattr(promotion, '_atomic_write_yaml', fail_initial_registry_write)
    with pytest.raises(OSError, match='injected registry publish failure'):
        promotion.promote_candidate(source, root, activate=True)
    assert not (root / 'orchard/v2').exists()
    assert _registry(root)['maps']['orchard']['active'] == 'v1'
    assert _registry(root)['maps']['orchard']['latest_validated'] == 'v1'
    assert (root / 'active_map.yaml').read_bytes() == previous_pointer
    assert _state(source)['state'] == 'REJECTED'


def test_promotion_without_activation_keeps_active_pointer_empty(sandbox):
    root, candidate = sandbox
    source = candidate('v1')
    published = promotion.promote_candidate(source, root, activate=False)
    assert validate_package(published / 'metadata.yaml', verify_hashes=True).valid
    assert _registry(root)['maps']['orchard']['latest_validated'] == 'v1'
    assert _registry(root)['maps']['orchard']['active'] == ''
    assert not (root / 'active_map.yaml').exists()
    assert _state(source)['state'] == 'PROMOTED'


def test_first_activation_failure_leaves_no_active_pointer(sandbox, monkeypatch):
    root, candidate = sandbox
    source = candidate('v1')

    def fail_before_write(*_args):
        raise OSError('injected first activation failure')

    monkeypatch.setattr(promotion, '_activate', fail_before_write)
    with pytest.raises(OSError, match='injected first activation failure'):
        promotion.promote_candidate(source, root, activate=True)
    assert _registry(root)['maps']['orchard']['latest_validated'] == 'v1'
    assert _registry(root)['maps']['orchard']['active'] == ''
    assert not (root / 'active_map.yaml').exists()
    assert _state(source)['state'] == 'PROMOTED'
    assert 'error' in _state(source)
    assert validate_package(root / 'orchard/v1/metadata.yaml', verify_hashes=True).valid
