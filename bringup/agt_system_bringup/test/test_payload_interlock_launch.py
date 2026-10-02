"""Launch-level payload interlock resolution (no nodes are started)."""
import importlib.util
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory


def _load(package, name):
    path = Path(get_package_share_directory(package)) / 'launch' / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


NAV = _load('agt_system_bringup', 'navigation.launch.py')
MISSION = _load('agt_mission_bringup', 'mission.launch.py')


@pytest.mark.parametrize('mode,config,expected', [
    ('auto', '', False),                       # legacy Bunker launch unchanged
    ('auto', 'bunker_inspection', False),      # no arm installed
    ('true', '', True),
    ('true', 'bunker_inspection', True),
    ('false', 'bunker_inspection', False),
])
def test_navigation_interlock_modes(mode, config, expected):
    assert NAV.resolve_payload_interlock(mode, config, 'bunker_v1') is expected


def test_navigation_invalid_mode():
    with pytest.raises(RuntimeError, match='auto|true|false'):
        NAV.resolve_payload_interlock('maybe', '', 'bunker_v1')


def test_navigation_yhs_blocked_never_falls_back():
    with pytest.raises(Exception, match='BLOCKED'):
        NAV.resolve_payload_interlock('auto', 'yhs_harvesting', '')


def test_motion_guard_backend_selects_one_process_and_keeps_python_rollback():
    assert NAV.motion_guard_node_spec('cpp') == ('agt_base_runtime', 'motion_guard')
    assert NAV.motion_guard_node_spec('python') == ('agt_base_control', 'cmd_vel_guard')
    with pytest.raises(RuntimeError, match=r'cpp\|python'):
        NAV.motion_guard_node_spec('both')


def _value(**kw):
    defaults = {'payload_interlock': 'auto', 'enable_arm': 'false'}
    defaults.update(kw)
    return lambda name: defaults[name]


def test_mission_enable_arm_forces_interlock():
    assert MISSION._interlock(_value(enable_arm='true')) == 'true'
    assert MISSION._interlock(_value(enable_arm='true', payload_interlock='auto')) == 'true'
    with pytest.raises(RuntimeError, match='not allowed'):
        MISSION._interlock(_value(enable_arm='true', payload_interlock='false'))
    assert MISSION._interlock(_value()) == 'auto'
