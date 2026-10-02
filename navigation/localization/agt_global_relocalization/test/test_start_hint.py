"""Offline contract for a reviewed, immutable-map-bound near-start seed."""
import hashlib

import pytest
import yaml

from agt_global_relocalization.start_hint import StartHintError, load_start_hint


def hint_files(tmp_path):
    pcd = tmp_path / 'global_map.pcd'
    pcd.write_bytes(b'PCD under test; no ROS device needed')
    hint = tmp_path / 'start_hint.yaml'
    data = {
        'schema_version': 1,
        'map_id': 'bunker_mid360',
        'map_version': 'example-v1',
        'frame_id': 'map',
        'pose_semantics': 'T_map_body',
        'localization_map_sha256': hashlib.sha256(pcd.read_bytes()).hexdigest(),
        'source': 'manually verified mapping-start coordinate',
        'reviewed_by': 'human operator',
        'approved_for_near_search': True,
        'pose': {'x': 3.0, 'y': -2.0, 'z': 0.5,
                 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
    }
    hint.write_text(yaml.safe_dump(data))
    return hint, pcd, data


def validate(hint, pcd):
    return load_start_hint(hint, map_id='bunker_mid360',
                           map_version='example-v1', map_pcd=pcd)


def test_approved_hint_binds_selected_map_and_pcd(tmp_path):
    hint, pcd, data = hint_files(tmp_path)
    assert validate(hint, pcd) == data['pose']


@pytest.mark.parametrize('change', [
    {'approved_for_near_search': False},
    {'reviewed_by': ''},
    {'map_version': 'a-different-version'},
    {'frame_id': 'odom'},
    {'pose_semantics': 'T_map_base'},
    {'localization_map_sha256': '0' * 64},
    {'source': ''},
])
def test_unapproved_or_mismatched_hint_is_rejected(tmp_path, change):
    hint, pcd, data = hint_files(tmp_path)
    data.update(change)
    hint.write_text(yaml.safe_dump(data))
    with pytest.raises(StartHintError):
        validate(hint, pcd)


def test_missing_hint_never_infers_grid_origin(tmp_path):
    _hint, pcd, _data = hint_files(tmp_path)
    with pytest.raises(StartHintError, match='does not exist'):
        validate(tmp_path / 'missing.yaml', pcd)


def test_changed_pcd_is_rejected(tmp_path):
    hint, pcd, _data = hint_files(tmp_path)
    pcd.write_bytes(pcd.read_bytes() + b'changed')
    with pytest.raises(StartHintError, match='SHA-256'):
        validate(hint, pcd)


def test_bad_pose_and_quaternion_are_rejected(tmp_path):
    hint, pcd, data = hint_files(tmp_path)
    for key, value in [('x', float('nan')), ('qw', 0.5)]:
        bad = dict(data)
        bad['pose'] = dict(data['pose'])
        bad['pose'][key] = value
        hint.write_text(yaml.safe_dump(bad))
        with pytest.raises(StartHintError):
            validate(hint, pcd)
