"""Backend selection and immutable FAST-LIO2 navigation configuration.

No ROS graph interaction. Called before constructing the selected LIO launch.
"""

from pathlib import Path
import tempfile
import yaml

from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar

LIO_BACKENDS = ('batch_lio', 'fastlio2')


def validate_backend(value):
    if value not in LIO_BACKENDS:
        raise ValueError(f'Unsupported lio_backend {value!r}; choose batch_lio or fastlio2')
    return value


def freeze_fastlio_config(config_file, lidar_topic, imu_topic, output_directory=None):
    """Snapshot the exact frontend calibration; override only raw input topics.

    World/body frames are a contract, not an arbitrary label replacement.
    Online extrinsic estimation would invalidate the adapter's static cache.
    """
    source = Path(config_file).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or 'r_il' not in data or 't_il' not in data:
        raise ValueError('FAST-LIO2 needs a flat runtime YAML with r_il and t_il')
    if data.get('world_frame') != 'odom' or data.get('body_frame') != 'body':
        raise ValueError('FAST-LIO2 navigation requires world_frame=odom, body_frame=body')
    if data.get('esti_il', False) is not False:
        raise ValueError('FAST-LIO2 navigation requires esti_il=false (frozen calibration)')
    load_lio_body_to_lidar(source)  # Validate numeric calibration before any node starts.
    if not str(lidar_topic).strip() or not str(imu_topic).strip():
        raise ValueError('raw LiDAR and IMU topics must not be empty')
    data['lidar_topic'], data['imu_topic'] = str(lidar_topic), str(imu_topic)
    directory = (Path(output_directory) if output_directory is not None
                 else Path(tempfile.mkdtemp(prefix='agt_fastlio2_')))
    directory.mkdir(parents=True, exist_ok=True)
    result = directory / 'fastlio2.yaml'
    result.write_text('# Frozen navigation input; source: ' + str(source) + '\n' +
                      yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return str(result)


def require_same_calibration(frontend_config, adapter_config):
    """Refuse a legacy override whose transform differs from the frontend."""
    actual_t, actual_q = load_lio_body_to_lidar(frontend_config)
    selected_t, selected_q = load_lio_body_to_lidar(adapter_config)
    translation_error = max(abs(a - b) for a, b in zip(actual_t, selected_t))
    # Quaternion sign does not change the represented orientation.
    rotation_error = 1.0 - abs(sum(a * b for a, b in zip(actual_q, selected_q)))
    if translation_error > 1.0e-8 or rotation_error > 1.0e-8:
        raise ValueError('body calibration does not match the active FAST-LIO2 r_il/t_il')
