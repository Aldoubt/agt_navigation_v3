import math
from pathlib import Path
import pytest
import yaml

from agt_global_relocalization.global_relocalization import GlobalRelocalization


def _pose(x, y, z, qx, qy, qz, qw):
    return {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}


def test_compose_inverse_round_trip_is_identity():
    body_to_base = _pose(
        -0.31, 0.02, -0.72,
        0.0, 0.0, 0.0, 1.0)
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
        -0.31, 0.02, -0.72,
        0.0, 0.0, 0.0, 1.0)
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

    launch = (root / 'navigation/localization/agt_global_relocalization/launch/'
              'global_relocalization.launch.py').read_text(encoding='utf-8')
    assert "'bbs_query_frame_mode', default_value='mapping_body'" in launch


def test_stationary_filter_requires_a_full_window_and_rejects_one_spike():
    samples = [(0.01, 0.01), (0.02, 0.01), (0.18, 0.02), (0.02, 0.01)]
    assert GlobalRelocalization.robust_motion(samples, 5) is None

    samples.append((0.01, 0.01))
    linear, angular = GlobalRelocalization.robust_motion(samples, 5)
    assert linear == pytest.approx(0.02)
    assert angular == pytest.approx(0.01)


def test_relocalization_query_requires_every_configured_cloud():
    assert not GlobalRelocalization.has_complete_query(0, 5)
    assert not GlobalRelocalization.has_complete_query(1, 5)
    assert not GlobalRelocalization.has_complete_query(4, 5)
    assert GlobalRelocalization.has_complete_query(5, 5)
    assert GlobalRelocalization.has_complete_query(6, 5)


def test_field_stationary_authority_uses_bunker_wheel_odometry():
    root = Path(__file__).resolve().parents[4]
    config = yaml.safe_load((root / 'navigation/localization/agt_global_relocalization/'
                             'config/global_relocalization.yaml').read_text(encoding='utf-8'))
    params = config['agt_global_relocalization']['ros__parameters']
    assert params['stationary_odom_topic'] == '/wheel/odom'
    assert params['stationary_filter_window_samples'] >= 3
    assert params['stationary_hard_linear_threshold_mps'] > params[
        'stationary_linear_threshold_mps']


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
    assert 'body_to_base_translation' not in params
    assert 'body_to_base_quaternion_xyzw' not in params
    assert params['mount_lidar_frame'] == 'livox_frame'
    assert params['mount_base_frame'] == 'base_link'

    command = params['candidate_sdk_command']
    assert '--bbs-query-frame-mode {bbs_query_frame_mode}' in command
    placeholders = {
        'tx': 'base_from_body_tx',
        'ty': 'base_from_body_ty',
        'tz': 'base_from_body_tz',
        'qx': 'base_from_body_qx',
        'qy': 'base_from_body_qy',
        'qz': 'base_from_body_qz',
        'qw': 'base_from_body_qw',
    }
    for axis, placeholder in placeholders.items():
        assert f'--base-from-body-{axis} {{{placeholder}}}' in command

    assert 'load_lio_body_to_lidar' in source
    assert "self.get_parameter('body_to_base_calibration_file')" in source
    assert "self.get_parameter('batch_lio_config_file')" in source
    assert 'lookup_transform(' in source
    assert "self.get_parameter('mount_lidar_frame')" in source
    assert "self.get_parameter('mount_base_frame')" in source


def test_explicit_fastlio_calibration_takes_precedence_over_legacy_batch_argument(tmp_path):
    from types import SimpleNamespace
    from geometry_msgs.msg import TransformStamped
    config = tmp_path / 'fastlio.yaml'
    config.write_text(yaml.safe_dump({'t_il': [-.011, -.02329, .04412],
                                    'r_il': [1., 0, 0, 0, 1., 0, 0, 0, 1.]}))
    values = {'body_to_base_calibration_file': str(config),
              'batch_lio_config_file': '/must/not/be/read.yaml',
              'mount_lidar_frame': 'livox_frame', 'mount_base_frame': 'base_link',
              'tf_timeout_sec': .2}
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = -0.3
    transform.transform.translation.z = -0.8
    fake = SimpleNamespace(
        _body_to_base_cache=None,
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        get_logger=lambda: SimpleNamespace(info=lambda text: None),
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args, **kwargs: transform),
    )
    result = GlobalRelocalization.body_to_base_pose(fake)
    assert (result['x'], result['y'], result['z']) == pytest.approx((-.311, -.02329, -.75588))
