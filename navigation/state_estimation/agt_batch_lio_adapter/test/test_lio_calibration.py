import pytest
import yaml
from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar

IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def write(tmp_path, data):
    path = tmp_path / 'calibration.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_fastlio_uses_its_own_forward_transform_without_sign_inversion(tmp_path):
    path = write(tmp_path, {'r_il': IDENTITY, 't_il': [-.011, -.02329, .04412]})
    t, q = load_lio_body_to_lidar(path)
    assert t == pytest.approx((-.011, -.02329, .04412))
    assert q == pytest.approx((0, 0, 0, 1))


def test_legacy_batch_yaml_keeps_its_existing_transform(tmp_path):
    path = write(tmp_path, {'/**': {'ros__parameters': {'mapping': {
        'extrinsic_R': IDENTITY, 'extrinsic_T': [.011, .02329, -.04412]}}}})
    t, q = load_lio_body_to_lidar(path)
    assert t == pytest.approx((.011, .02329, -.04412))
    assert q == pytest.approx((0, 0, 0, 1))


@pytest.mark.parametrize('rotation,translation', [
    (IDENTITY[:-1], [0, 0, 0]),
    (IDENTITY, [0, 0]),
    (IDENTITY, [float('nan'), 0, 0]),
    ([-1., 0, 0, 0, 1., 0, 0, 0, 1.], [0, 0, 0]),
    ([2., 0, 0, 0, 1., 0, 0, 0, 1.], [0, 0, 0]),
])
def test_malformed_fastlio_calibration_is_rejected(tmp_path, rotation, translation):
    path = write(tmp_path, {'r_il': rotation, 't_il': translation})
    with pytest.raises(RuntimeError):
        load_lio_body_to_lidar(path)
