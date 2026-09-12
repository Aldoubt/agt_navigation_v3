from pathlib import Path


def test_offline_replay_audit_is_optional_and_no_hardware_motion_chain_is_declared():
    root = Path(__file__).resolve().parents[1]
    launch = (root / 'launch' / 'acceptance_offline_replay.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument(\n            'enable_replay_audit', default_value='false'" in launch
    assert "package='agt_operator_console', executable='replay_audit'" in launch
    for forbidden in (
        "package='bunker_base'", "package='agt_base_control'",
        "package='agt_capability_camera'", "cmd_vel_guard.launch.py",
    ):
        assert forbidden not in launch


def test_offline_relocalization_sublaunch_declares_no_hardware_driver():
    root = Path(__file__).resolve().parents[1]
    launch = (root / 'launch' / 'offline_relocalization_demo.launch.py').read_text(encoding='utf-8')
    for forbidden in ("package='bunker_base'", "package='agt_base_control'", "package='agt_capability_camera'"):
        assert forbidden not in launch
