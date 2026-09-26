"""Offline YHS Nav2 configuration isolation; never starts hardware."""
import importlib.util
from pathlib import Path
import shutil

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'navigation.launch.py'
DEFAULT_CONFIG = PACKAGE.parents[1] / 'config'


def load_launch():
    spec = importlib.util.spec_from_file_location('agt_test_yhs_navigation_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_yhs_nav_requires_dedicated_reviewed_configuration(tmp_path):
    mod = load_launch()
    assert mod.select_navigation_config('bunker_v1', '', DEFAULT_CONFIG) == DEFAULT_CONFIG.resolve()
    with pytest.raises(RuntimeError, match='reserved for measured yhs_v1'):
        mod.select_navigation_config('bunker_v1', str(tmp_path), DEFAULT_CONFIG)
    with pytest.raises(RuntimeError, match='requires explicit nav_config_dir'):
        mod.select_navigation_config('yhs_v1', '', DEFAULT_CONFIG)
    with pytest.raises(RuntimeError, match='canonical Bunker'):
        mod.select_navigation_config('yhs_v1', str(DEFAULT_CONFIG), DEFAULT_CONFIG)

    config = tmp_path / 'yhs_nav'
    config.mkdir()
    with pytest.raises(RuntimeError, match='field review marker'):
        mod.select_navigation_config('yhs_v1', str(config), DEFAULT_CONFIG)
    marker = config / 'field_profile.yaml'
    marker.write_text(yaml.safe_dump({'robot_profile': 'bunker_v1',
                                      'field_verified': True, 'verified_by': 'test'}))
    with pytest.raises(RuntimeError, match='field_profile.yaml'):
        mod.select_navigation_config('yhs_v1', str(config), DEFAULT_CONFIG)
    marker.write_text(yaml.safe_dump({'robot_profile': 'yhs_v1',
                                      'field_verified': True, 'verified_by': 'test fixture only'}))
    with pytest.raises(RuntimeError, match='missing required YAML'):
        mod.select_navigation_config('yhs_v1', str(config), DEFAULT_CONFIG)
    for name in mod.CONFIG_FILES:
        shutil.copy2(DEFAULT_CONFIG / name, config / name)
    assert mod.select_navigation_config('yhs_v1', str(config), DEFAULT_CONFIG) == config.resolve()
    # Unit fixture copies Bunker files solely to test the file gate; it is NOT
    # a valid YHS calibration. Field measurements and review remain mandatory.
