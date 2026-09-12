import math
from pathlib import Path
import re

import pytest
import yaml

from agt_global_relocalization.global_relocalization import GlobalRelocalization


def _pose(x, y, z, qx, qy, qz, qw):
    return {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


def test_compose_inverse_round_trip_is_identity():
    body_to_base = _pose(
        -0.16403417, 0.02439982, -0.49511119,
        0.000477, -0.100267018, -0.001592, 0.994959177)
    identity = GlobalRelocalization.compose_pose(
        body_to_base, GlobalRelocalization.inverse_pose(body_to_base))
    assert identity['x'] == pytest.approx(0.0, abs=1e-12)
    assert identity['y'] == pytest.approx(0.0, abs=1e-12)
    assert identity['z'] == pytest.approx(0.0, abs=1e-12)
    assert identity['qx'] == pytest.approx(0.0, abs=1e-12)
    assert identity['qy'] == pytest.approx(0.0, abs=1e-12)
    assert identity['qz'] == pytest.approx(0.0, abs=1e-12)
    assert identity['qw'] == pytest.approx(1.0, abs=1e-12)


def test_map_body_to_base_conversion_preserves_se3_contract():
    # T_map_base = T_map_body * T_body_base.  This is the P2.18 publication
    # conversion after native GICP has aligned a body-frame query.
    map_to_body = _pose(-0.15, 0.03, -0.48, 0.0, 0.0, 0.0, 1.0)
    body_to_base = _pose(
        -0.16403417, 0.02439982, -0.49511119,
        0.000477, -0.100267018, -0.001592, 0.994959177)
    map_to_base = GlobalRelocalization.compose_pose(map_to_body, body_to_base)
    recovered_map_to_body = GlobalRelocalization.compose_pose(
        map_to_base, GlobalRelocalization.inverse_pose(body_to_base))
    for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
        assert recovered_map_to_body[key] == pytest.approx(map_to_body[key], abs=1e-12)


def test_mapping_body_livox_default_is_unit_identity_rotation():
    q = GlobalRelocalization._normalized_xyzw((0.0, 0.0, 0.0, 1.0), 'mapping')
    assert q == (0.0, 0.0, 0.0, 1.0)
    point = GlobalRelocalization._rotate_xyz(1.0, -2.0, 3.0, q)
    assert point == pytest.approx((1.0, -2.0, 3.0))


def test_mapping_body_is_the_formal_query_contract():
    mode, alias = GlobalRelocalization.normalize_query_frame_mode('mapping_body')
    assert mode == 'mapping_body'
    assert alias == ''


def test_acceptance_defaults_keep_manual_and_candidate_bbs_in_mapping_body():
    """A formal PGO replay must never start in an implicit mixed mode."""
    root = Path(__file__).resolve().parents[4]
    config = yaml.safe_load((root / 'navigation/localization/agt_global_relocalization/'
                             'config/global_relocalization.yaml').read_text(encoding='utf-8'))
    params = config['agt_global_relocalization']['ros__parameters']
    assert params['relocalization_query_frame_mode'] == 'mapping_body'
    assert params['bbs_query_frame_mode'] == 'mapping_body'

    for launch_path in (
        root / 'navigation/localization/agt_global_relocalization/launch/'
        'global_relocalization.launch.py',
        root / 'bringup/agt_system_bringup/launch/'
        'offline_relocalization_demo.launch.py',
        root / 'bringup/agt_system_bringup/launch/'
        'acceptance_offline_replay.launch.py',
    ):
        launch = launch_path.read_text(encoding='utf-8')
        assert "'bbs_query_frame_mode', default_value='mapping_body'" in launch


def test_manual_seed_mapping_body_contract_has_one_conversion_at_each_boundary():
    """Manual seed sends T_map_body to GICP and publishes one T_map_base conversion."""
    root = Path(__file__).resolve().parents[4]
    manual = (root / 'navigation/localization/agt_global_relocalization/'
              'agt_global_relocalization/manual_seed_relocalization.py').read_text(encoding='utf-8')
    assert 'T_map_body=T_map_base*T_base_body' in manual
    assert 'T_map_base; body mode composes T_map_body*T_body_base exactly once.' in manual
    assert "initial_query = self._seed_base_to_query_pose(initial_base_audit, query_mode)" in manual
    assert "backend_refined_base = self._query_pose_to_base_pose(" in manual


def test_base_link_is_retained_as_explicit_compatibility_mode():
    mode, alias = GlobalRelocalization.normalize_query_frame_mode('base_link')
    assert mode == 'base_link'
    assert alias == ''


def test_body_aligned_is_a_deprecated_mapping_body_alias():
    mode, alias = GlobalRelocalization.normalize_query_frame_mode('body_aligned')
    assert mode == 'mapping_body'
    assert alias == 'body_aligned'


def test_unknown_query_frame_mode_is_rejected():
    with pytest.raises(RuntimeError, match='mapping_body or base_link'):
        GlobalRelocalization.normalize_query_frame_mode('sensor_frame')


def test_candidate_bbs_has_explicit_non_mixing_frame_mode_contract():
    """Lock the base-link compatibility and mapping-body experiment boundaries."""
    root = Path(__file__).resolve().parents[4]
    source = (root / 'navigation/localization/agt_global_relocalization/'
              'agt_global_relocalization/global_relocalization.py').read_text(encoding='utf-8')
    native = (root / 'navigation/localization/agt_global_relocalization_native/src/'
              'candidate_bbs_gicp_localizer.cpp').read_text(encoding='utf-8')
    assert 'if mode != bbs_mode:' in source
    assert 'must match;' in source
    assert 'refusing to mix query and candidate pose frames' in source
    assert 'bbs_query_frame_mode=bbs_mode' in source
    assert 'if (o.bbs_query_frame_mode == "base_link")' in native
    assert 'std::string bbs_query_frame_mode{"mapping_body"};' in native
    assert 'T_map_base = T_map_body * T_body_base' in native
    assert 'const Eigen::Isometry3d T_body_base = T_base_body.inverse();' in native
    assert native.count('T_map_base = T_map_body * T_body_base') == 1
    assert '--bbs-query-frame-mode' in native

    config = yaml.safe_load((root / 'navigation/localization/agt_global_relocalization/'
                             'config/global_relocalization.yaml').read_text(encoding='utf-8'))
    params = config['agt_global_relocalization']['ros__parameters']
    body_to_base = _pose(
        *params['body_to_base_translation'],
        *params['body_to_base_quaternion_xyzw'])
    expected_base_to_body = GlobalRelocalization.inverse_pose(body_to_base)
    command = params['candidate_sdk_command']
    assert '--bbs-query-frame-mode {bbs_query_frame_mode}' in command
    values = {}
    for axis in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'):
        match = re.search(rf'--base-from-body-{axis}\s+([-+0-9.eE]+)', command)
        assert match, axis
        values[axis] = float(match.group(1))
    assert values['tx'] == pytest.approx(expected_base_to_body['x'], abs=1e-6)
    assert values['ty'] == pytest.approx(expected_base_to_body['y'], abs=1e-6)
    assert values['tz'] == pytest.approx(expected_base_to_body['z'], abs=1e-6)
    assert values['qx'] == pytest.approx(expected_base_to_body['qx'], abs=1e-6)
    assert values['qy'] == pytest.approx(expected_base_to_body['qy'], abs=1e-6)
    assert values['qz'] == pytest.approx(expected_base_to_body['qz'], abs=1e-6)
    assert values['qw'] == pytest.approx(expected_base_to_body['qw'], abs=1e-6)
