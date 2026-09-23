"""Static checks only: never call a hardware or navigation launch function."""
import ast
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]


def test_field_script_preserves_navigation_and_inspection_modes():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    assert 'MODE=navigation' in text
    assert 'navigation)\n    ENABLE_INSPECTION=false' in text
    assert 'inspection)\n    ENABLE_INSPECTION=true' in text
    assert 'enable_camera_gimbal:="$ENABLE_INSPECTION"' in text
    assert 'NAV_LAUNCH=navigation.launch.py' in text
    assert 'NAV_PACKAGE=agt_mission_bringup' in text
    assert 'NAV_LAUNCH=mission.launch.py' in text
    assert 'MISSION_ARGS+=("enable_legacy_inspection:=true")' in text
    assert 'ros2 launch "$NAV_PACKAGE" "$NAV_LAUNCH"' in text
    assert '-p require_camera:="$ENABLE_INSPECTION"' in text


def test_field_script_stops_complete_launch_process_groups():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    assert 'setsid "$@" >"$logfile" 2>&1 &' in text
    assert 'kill -INT -- "-$pid"' in text
    assert 'kill -TERM -- "-$pid"' in text
    assert 'kill -KILL -- "-$pid"' in text


def test_field_script_retries_transient_global_relocalization_once():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    assert 'for attempt in 1 2' in text
    assert '${log_prefix}_service_${attempt}.txt' in text
    assert 'relocalize_until_ready relocalize' in text


def test_field_script_revalidates_localization_after_nav2_startup():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    nav_ready=text.index('if [[ "$navigation_ready" != true ]]')
    debug_start=text.index("if [[ \"$ENABLE_RVIZ\" == true ]]", nav_ready)
    runtime_check=text.index('"$SCRIPT_DIR/check_runtime.sh"')
    post_startup_check=text.index('ensure_localization_ready after_startup_checks')
    tf_check=text.index('"$SCRIPT_DIR/check_tf.sh"')
    final_check=text.index('ensure_localization_ready after_tf_check')
    preflight=text.index('ros2 run agt_navigation_runtime demo_preflight')
    assert nav_ready < debug_start < runtime_check < post_startup_check
    assert post_startup_check < tf_check < final_check < preflight
    assert 'wait_for_node /agt_navigation_debug_rviz 30' in text[debug_start:runtime_check]


def test_post_startup_check_preserves_an_accepted_global_correction():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    start=text.index('ensure_localization_ready()')
    end=text.index('\n}\n\nstop_process_group()', start) + 2
    function=text[start:end]
    assert 'wait_for_localization_settle 15' in function
    assert "grep -Eq '^global_correction_valid: true$'" in function
    assert 'preserving the valid global correction' in function
    assert 'refusing a second startup relocalization' in function
    assert 'relocalize_until_ready' not in function


def test_field_script_retries_transient_nav2_lifecycle_bringup_once():
    text=(ROOT/'scripts/run_field_stack.sh').read_text()
    assert 'navigation_ready=false' in text
    assert 'navigation_attempt_${attempt}.log' in text
    assert 'restarting navigation only' in text


def test_legacy_inspection_is_only_in_explicit_compatibility_mission_launch():
    navigation=(ROOT/'bringup/agt_system_bringup/launch/navigation.launch.py').read_text()
    mission=(ROOT.parent/'agt_mission/agt_mission_bringup/launch/mission.launch.py').read_text()
    ast.parse(navigation)
    ast.parse(mission)
    assert "'rviz_patrol.launch.py'" not in navigation
    assert "'runtime.launch.py'" not in navigation
    assert "'navigation.launch.py'" in mission
    assert "executable='mission_runtime_v4'" in mission
    assert "DeclareLaunchArgument('enable_legacy_inspection', default_value='false')" in mission
    assert "if value('enable_legacy_inspection').lower() == 'true':" in mission
    assert "'runtime.launch.py'" in mission and "'rviz_patrol.launch.py'" in mission


def test_production_runtime_no_longer_uses_asyncio_sleep():
    source=(ROOT/'navigation/nav2/agt_navigation_runtime/agt_navigation_runtime/mission_runtime.py').read_text()
    assert 'asyncio.sleep' not in source
    assert 'wait_until_stationary' in source
    assert 'self._waiter.sleep' in source


def test_camera_view_and_stop_gate_defaults_are_unchanged():
    import yaml
    config=yaml.safe_load((ROOT/'navigation/nav2/agt_navigation_runtime/config/runtime.yaml').read_text())['mission_runtime']['ros__parameters']
    assert config['stationary_hold_sec']==0.8
    assert config['stationary_timeout_sec']==8.0
    assert config['stationary_linear_threshold_mps']==0.03
    assert config['stationary_pose_linear_threshold_mps']==0.04
    preset=yaml.safe_load((ROOT/'navigation/nav2/agt_rviz_patrol/config/front_sky_three_views.yaml').read_text())
    assert len(preset['views'])==3
    assert all(v['required'] and v['save_image'] for v in preset['views'])
