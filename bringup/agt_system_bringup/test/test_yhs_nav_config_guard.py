"""YHS Nav2 is fail-closed until its vehicle protocol/kinematics audit closes."""
import importlib.util
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'navigation.launch.py'
DEFAULT_CONFIG = PACKAGE.parents[1] / 'config'


def load_launch():
    spec = importlib.util.spec_from_file_location('agt_test_yhs_navigation_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('robot_profile', ['yhs_v1', 'yhs_tk_mid'])
def test_yhs_nav_is_blocked_even_when_an_explicit_config_directory_exists(tmp_path, robot_profile):
    mod = load_launch()
    fake_config = tmp_path / 'unverified_yhs_config'
    fake_config.mkdir()
    with pytest.raises(RuntimeError, match='YHS navigation remains BLOCKED'):
        mod.select_navigation_config(robot_profile, str(fake_config), DEFAULT_CONFIG)


def test_yhs_nav_does_not_fall_back_to_bunker_defaults():
    mod = load_launch()
    with pytest.raises(RuntimeError, match='YHS navigation remains BLOCKED'):
        mod.select_navigation_config('yhs_v1', '', DEFAULT_CONFIG)
