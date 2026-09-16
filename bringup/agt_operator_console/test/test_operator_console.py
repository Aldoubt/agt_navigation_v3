from pathlib import Path

from agt_operator_console.operator_console import (
    OperatorConsole,
    parse_map_list,
    map_list_command,
    map_editor_command,
    map_selection_command,
)
from agt_operator_console.profile import Profile


def _profile() -> Profile:
    return Profile(
        drivers=(), checks=(), mode_commands={'navigation': ('echo', 'navigation')},
        mode_preflight_commands={},
        map_root=Path('/maps'), pipeline_config=Path('/pipeline.yaml'),
    )


def test_map_selection_is_exact_and_uses_profile_root():
    command = map_selection_command(_profile(), 'site_a', 'v002-approved')
    assert command[:4] == ('ros2', 'run', 'agt_map_manager', 'select_map_package')
    assert command[-4:] == ('--map-id', 'site_a', '--map-version', 'v002-approved')
    assert '/maps' in command


def test_map_list_command_and_parser_use_exact_versions():
    assert map_list_command(_profile())[-2:] == ('--map-root', '/maps')
    assert parse_map_list('site_a/v001\tvalid\t/maps/a\nsite_b/v002\tvalid\t/maps/b\n') == [
        ('site_a', 'v001'), ('site_b', 'v002')]


def test_map_editor_command_binds_one_exact_package():
    command = map_editor_command(_profile(), 'site_a', 'v003-edited')
    assert command[:4] == ('ros2', 'launch', 'agt_system_bringup', 'hmi_map_editor.launch.py')
    assert 'map_id:=site_a' in command and 'map_version:=v003-edited' in command


def test_navigation_never_starts_a_sensor_session(monkeypatch):
    console = OperatorConsole(_profile())
    calls = []
    monkeypatch.setattr(
        console,
        'start_sensors',
        lambda: (_ for _ in ()).throw(AssertionError('navigation must not start sensors')),
    )
    monkeypatch.setattr(console, 'preflight', lambda mode: True)
    monkeypatch.setattr('agt_operator_console.operator_console.subprocess.run', lambda *args, **kwargs: calls.append(args[0]))
    monkeypatch.setattr(console, '_run_child', lambda *args, **kwargs: 'stopped')
    assert console.navigation(map_id='site_a', map_version='v001') == 0
    assert calls[0][:4] == ('ros2', 'run', 'agt_map_manager', 'select_map_package')
    assert calls[1][:4] == ('ros2', 'run', 'agt_map_manager', 'validate_active_map')
