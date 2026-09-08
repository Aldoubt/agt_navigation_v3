from pathlib import Path

import pytest
import yaml

from agt_map_manager.edit_session import (
    assert_navigation_maps_compatible,
    create_edit_session,
    load_edit_session,
    read_navigation_contract,
    set_edit_session_state,
    validate_edit_session_for_publish,
)
from agt_map_manager.map_package import validate_package


def _write_pgm(path: Path, width: int = 8, height: int = 6) -> None:
    path.write_bytes(
        f'P5\n# generated test map\n{width} {height}\n255\n'.encode('ascii')
        + bytes([254]) * width * height
    )


def _write_nav(directory: Path, *, width=8, height=6, resolution=0.1,
               origin=(-4.25, 7.5, 0.0), mode='trinary', negate=0,
               occupied_thresh=0.65, free_thresh=0.25) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    _write_pgm(directory / 'map.pgm', width=width, height=height)
    data = {
        'image': 'map.pgm',
        'mode': mode,
        'resolution': resolution,
        'origin': list(origin),
        'negate': negate,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
    }
    path = directory / 'map.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return path


def _write_package(root: Path):
    package = root / 'site_a' / 'v1'
    nav = _write_nav(package / 'navigation')
    localization = package / 'localization'
    localization.mkdir(parents=True)
    (localization / 'global_map.pcd').write_bytes(b'pcd-test')
    metadata = {
        'schema_version': 1,
        'map_id': 'site_a',
        'map_version': 'v1',
        'frame_id': 'map',
        'assets': {
            'localization_map': {'path': 'localization/global_map.pcd'},
            'navigation_map': {'path': 'navigation/map.yaml'},
        },
    }
    metadata_path = package / 'metadata.yaml'
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    return validate_package(metadata_path, verify_hashes=True), nav


def _edit_yaml(path: Path, key: str, value) -> None:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    data[key] = value
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def test_contract_fingerprint_is_stable(tmp_path):
    nav = _write_nav(tmp_path / 'nav')
    first = read_navigation_contract(nav)
    second = read_navigation_contract(nav)
    assert first.geometry_fingerprint == second.geometry_fingerprint
    assert first.contract_fingerprint == second.contract_fingerprint
    assert first.width == 8
    assert first.height == 6


def test_round_trip_contract_is_accepted(tmp_path):
    base = _write_nav(tmp_path / 'base')
    edited = _write_nav(tmp_path / 'edited')
    before, after = assert_navigation_maps_compatible(base, edited)
    assert before.contract_fingerprint == after.contract_fingerprint


@pytest.mark.parametrize(
    ('field', 'value', 'reason'),
    [
        ('resolution', 0.05, 'resolution'),
        ('origin', [-4.0, 7.5, 0.0], 'origin_x'),
        ('origin', [-4.25, 7.5, 0.01], 'origin_yaw'),
        ('mode', 'scale', 'mode'),
        ('occupied_thresh', 0.7, 'occupied_thresh'),
    ],
)
def test_contract_changes_are_rejected(tmp_path, field, value, reason):
    base = _write_nav(tmp_path / 'base')
    edited = _write_nav(tmp_path / 'edited')
    _edit_yaml(edited, field, value)
    with pytest.raises(ValueError) as exc:
        assert_navigation_maps_compatible(base, edited)
    assert reason in str(exc.value)


def test_resized_pgm_is_rejected(tmp_path):
    base = _write_nav(tmp_path / 'base')
    edited = _write_nav(tmp_path / 'edited')
    _write_pgm(edited.parent / 'map.pgm', width=9, height=6)
    with pytest.raises(ValueError, match='width'):
        assert_navigation_maps_compatible(base, edited)


def test_nonzero_origin_yaw_is_not_supported_even_without_base_comparison(tmp_path):
    nav = _write_nav(tmp_path / 'nav', origin=(0.0, 0.0, 0.1))
    with pytest.raises(ValueError, match='origin_yaw_not_supported'):
        read_navigation_contract(nav)


def test_edit_session_preserves_geometry_and_state(tmp_path):
    package, _ = _write_package(tmp_path / 'maps')
    assert package.valid
    edit_root = tmp_path / 'edits'
    session = create_edit_session(package, edit_root)
    assert session.state == 'open'
    assert session.navigation_map_yaml.is_file()
    assert (session.session_path / 'map.pgm').is_file()
    assert validate_edit_session_for_publish(edit_root, session.session_id).state == 'open'

    cancelled = set_edit_session_state(
        edit_root, session.session_id, 'cancelled', 'operator_cancelled')
    assert cancelled.state == 'cancelled'
    assert load_edit_session(edit_root, session.session_id).reason == 'operator_cancelled'
    with pytest.raises(ValueError, match='not_open'):
        validate_edit_session_for_publish(edit_root, session.session_id)


def test_session_detects_hmi_geometry_drift(tmp_path):
    package, _ = _write_package(tmp_path / 'maps')
    edit_root = tmp_path / 'edits'
    session = create_edit_session(package, edit_root)
    _edit_yaml(session.navigation_map_yaml, 'resolution', 0.2)
    with pytest.raises(ValueError, match='resolution'):
        validate_edit_session_for_publish(edit_root, session.session_id)
