"""Backend/config tests; launch actions are inspected, never executed."""
import importlib.util
from pathlib import Path
import os
import subprocess

import pytest
import yaml
from agt_navigation_runtime.lio_config import (
    freeze_fastlio_config, require_same_calibration, validate_backend,
)

ROOT = Path(__file__).resolve().parents[4]
RUNTIME = ROOT / 'navigation/nav2/agt_navigation_runtime'


def fast_config():
    return yaml.safe_load((RUNTIME / 'config/fastlio2_mid360_navigation.yaml').read_text())


def write(tmp_path, data, name='input.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize('backend', ['batch_lio', 'fastlio2'])
def test_supported_backend(backend):
    assert validate_backend(backend) == backend


@pytest.mark.parametrize('backend', ['', 'both', 'fastlio', 'batch_lio fastlio2'])
def test_unknown_or_multiple_backend_is_rejected(backend):
    with pytest.raises(ValueError):
        validate_backend(backend)


def test_frozen_fastlio_config_preserves_calibration_and_source(tmp_path):
    original = fast_config()
    source = write(tmp_path, original)
    before = source.read_bytes()
    path = freeze_fastlio_config(source, '/test/lidar', '/test/imu', tmp_path / 'snapshot')
    snapshot = yaml.safe_load(Path(path).read_text())
    assert source.read_bytes() == before
    assert snapshot['r_il'] == original['r_il']
    assert snapshot['t_il'] == original['t_il']
    assert snapshot['lidar_topic'] == '/test/lidar'
    assert snapshot['imu_topic'] == '/test/imu'
    require_same_calibration(source, path)


@pytest.mark.parametrize('key,value', [
    ('world_frame', 'map'), ('body_frame', 'base_link'), ('esti_il', True),
])
def test_unadaptable_fastlio_configuration_is_rejected(tmp_path, key, value):
    data = fast_config()
    data[key] = value
    source = write(tmp_path, data)
    with pytest.raises(ValueError):
        freeze_fastlio_config(source, '/livox/lidar', '/livox/imu', tmp_path / 'snapshot')


def test_batch_signs_cannot_be_silently_used_as_fastlio_calibration(tmp_path):
    source = write(tmp_path, fast_config())
    batch = RUNTIME / 'config/batch_lio_mid360.yaml'
    with pytest.raises(ValueError, match='does not match'):
        require_same_calibration(source, batch)


def load_localization_launch():
    path = ROOT / 'bringup/agt_system_bringup/launch/localization.launch.py'
    spec = importlib.util.spec_from_file_location('tested_localization_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('backend,expected', [
    ('batch_lio', 'navigation_lio.launch.py'),
    ('fastlio2', 'fastlio_navigation_lio.launch.py'),
])
def test_only_selected_lio_launch_is_constructed(tmp_path, monkeypatch, backend, expected):
    from launch import LaunchContext
    module = load_localization_launch()
    monkeypatch.setattr(module, '_include', lambda package, launch_file, arguments=None, condition=None:
                        (package, launch_file, arguments or {}, condition))
    original_freeze = module.freeze_fastlio_config
    monkeypatch.setattr(module, 'freeze_fastlio_config', lambda cfg, lidar, imu:
                        original_freeze(cfg, lidar, imu, tmp_path / 'snapshot'))
    global_map = tmp_path / 'map.pcd'
    global_map.write_text('')
    context = LaunchContext()
    context.launch_configurations.update({
        'global_map': str(global_map), 'relocalization_assets': str(tmp_path),
        'lio_backend': backend, 'use_sim_time': 'true',
        'batch_config': str(RUNTIME / 'config/batch_lio_mid360.yaml'),
        'fastlio_config': str(RUNTIME / 'config/fastlio2_mid360_navigation.yaml'),
        'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
        'auto_relocalize': 'false', 'enable_map_tracking': 'false',
    })
    actions = module._localization_nodes(context)
    frontends = [x for x in actions if x[0] == 'agt_navigation_runtime']
    assert len(frontends) == 1 and frontends[0][1] == expected
    assert sum(x[0] == 'agt_localization_manager' for x in actions) == 1
    relocalization = next(x for x in actions if x[0] == 'agt_global_relocalization')
    calibration = relocalization[2]['body_to_base_calibration_file']
    if backend == 'fastlio2':
        assert calibration == frontends[0][2]['body_to_base_calibration_file']
        require_same_calibration(frontends[0][2]['fastlio_config'], calibration)
    else:
        assert calibration == frontends[0][2]['batch_config']
    for action in actions:
        if action[0] in ('agt_navigation_runtime', 'agt_localization_manager', 'agt_global_relocalization'):
            assert action[2]['use_sim_time'] == 'true'


def test_fastlio_default_matches_mapping_body_query_calibration():
    data = fast_config()
    relocalization = yaml.safe_load((ROOT / 'navigation/localization/agt_global_relocalization/'
                                    'config/global_relocalization.yaml').read_text())
    params = relocalization['agt_global_relocalization']['ros__parameters']
    assert data['t_il'] == params['mapping_body_livox_translation']
    assert data['r_il'] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def test_adapter_launch_forwards_clock_and_does_not_force_legacy_calibration():
    source = (ROOT / 'navigation/state_estimation/agt_fastlio_adapter/launch/adapter.launch.py').read_text()
    assert "DeclareLaunchArgument('use_sim_time'" in source
    assert "ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)" in source
    source = (RUNTIME / 'launch/fastlio_navigation_lio.launch.py').read_text()
    assert 'require_same_calibration(runtime_config, calibration)' in source
    assert "'use_sim_time': use_sim_time" in source


@pytest.mark.parametrize('text, expected', [('false', False), ('true', True)])
def test_fastlio_launch_converts_resolved_clock_argument_to_bool(tmp_path, monkeypatch,
                                                                text, expected):
    from launch import LaunchContext
    from launch_ros.parameter_descriptions import ParameterValue

    path = RUNTIME / 'launch/fastlio_navigation_lio.launch.py'
    spec = importlib.util.spec_from_file_location('tested_fastlio_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'freeze_fastlio_config',
                        lambda config, lidar, imu: str(config))
    monkeypatch.setattr(module, 'require_same_calibration', lambda frontend, adapter: None)
    context = LaunchContext()
    config = RUNTIME / 'config/fastlio2_mid360_navigation.yaml'
    context.launch_configurations.update({
        'fastlio_config': str(config),
        'lidar_topic': '/livox/lidar',
        'imu_topic': '/livox/imu',
        'body_to_base_calibration_file': str(config),
        'use_sim_time': text,
    })

    node = module._lio_nodes(context)[0]
    parameter = next(value for value in node._Node__parameters[0].values()
                     if isinstance(value, ParameterValue))
    assert parameter.evaluate(context) is expected


@pytest.mark.parametrize('mode', ['navigation', 'inspection'])
@pytest.mark.parametrize('backend', ['batch_lio', 'fastlio2'])
def test_field_dry_run_selects_backend_without_creating_run_or_starting_nodes(tmp_path, mode, backend):
    ws = ROOT.parent.parent
    if not (ws / 'install/setup.bash').is_file() or not Path('/opt/ros/humble/setup.bash').is_file():
        pytest.skip('requires an existing sourced Humble workspace for the shell dry run')
    for name in ['localization/relocalization', 'navigation']:
        (tmp_path / name).mkdir(parents=True, exist_ok=True)
    (tmp_path / 'localization/global_map.pcd').write_text('')
    (tmp_path / 'navigation/map.yaml').write_text('{}')
    config = RUNTIME / 'config' / ('batch_lio_mid360.yaml' if backend == 'batch_lio'
                                  else 'fastlio2_mid360_navigation.yaml')
    env = dict(os.environ, AGT_FIELD_LOG_DIR=str(tmp_path / 'must_not_exist'))
    result = subprocess.run(['bash', str(ROOT / 'scripts/run_field_stack.sh'), '--mode', mode,
                             '--lio-backend', backend, '--lio-config', str(config),
                             '--map-root', str(tmp_path), '--dry-run'], env=env,
                            text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f'lio_backend={backend}' in result.stdout
    assert f'inspection_runtime={str(mode == "inspection").lower()}' in result.stdout
    assert '[START]' not in result.stdout
    assert not (tmp_path / 'must_not_exist').exists()


def test_runtime_check_rejects_simultaneous_backends(tmp_path):
    fake = tmp_path / 'ros2'
    fake.write_text('#!/bin/sh\nprintf "%s\\n" /agt_localization_manager /robot_state_publisher '
                    '/agt_pointcloud_preprocessor /agt_fastlio_adapter /fastlio2/lio_node '
                    '/agt_batch_lio_adapter /laserMapping\n')
    fake.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path) + ':' + os.environ.get('PATH', ''))
    result = subprocess.run(['bash', str(ROOT / 'scripts/check_runtime.sh'), '--lio-backend', 'fastlio2'],
                            env=env, text=True, capture_output=True, timeout=5)
    assert result.returncode != 0
    assert 'unselected backend' in result.stderr


def test_existing_stack_is_rejected_before_overwriting_its_lio_snapshot():
    source = (ROOT / 'scripts/run_field_stack.sh').read_text()
    assert source.index('Refusing to create duplicate owner') < source.index('cp -- "$LIO_CONFIG"')
    assert source.index('cp -- "$LIO_CONFIG"') < source.index('start_child hardware')
