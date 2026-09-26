"""run_field_stack.sh robot-config regression through the REAL script.

The script is executed with exported bash functions that replace `setsid`
(used by start_child for every ros2 launch) and `ros2 topic/node`. Every launch
is intercepted: argv is recorded and, for hardware.launch.py, the launch plan is
computed with agt_robot_bringup/hardware_launch_plan.py (no process spawned).
The script then fails its first topic wait and shuts down. No driver, motion
command or navigation goal can be started by this test.
Requires a sourced Humble workspace with agt_robot_bringup built.
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'run_field_stack.sh'
WS = SCRIPT.parents[3]
PLAN = WS / 'install/agt_robot_bringup/lib/agt_robot_bringup/hardware_launch_plan.py'
ROBOTS = WS / 'install/agt_robot_bringup/share/agt_robot_bringup/config/robots'

if not PLAN.is_file() or not Path('/opt/ros/humble/setup.bash').is_file():
    pytest.skip('needs Humble + built agt_robot_bringup', allow_module_level=True)

SHIM = r'''
setsid() {
  if [[ "$1" == ros2 && "$2" == launch ]]; then
    printf '%s\n' "$@" > "$AGT_TEST_OUT/$3__$4.argv"
    if [[ "$4" == hardware.launch.py ]]; then
      shift 4
      python3 "$AGT_TEST_PLAN" "$@" > "$AGT_TEST_OUT/hardware_plan.json" 2>&1
    fi
    exit 0
  fi
  echo "test shim refused setsid $*" >&2
  exit 99
}
ros2() {
  case "$1" in
    topic) return 1 ;;
    node) return 0 ;;
  esac
  command ros2 "$@"
}
export -f setsid ros2
exec bash "$AGT_TEST_SCRIPT" "$@"
'''

DESC = 'agt_robot_description/launch/display.launch.py'
MID = 'livox_ros_driver2/launch_ROS2/msg_MID360_launch.py'
BUNKER = 'bunker_base/launch/bunker_base.launch.py'
RTK = 'agt_asensing_driver/launch/asensing.launch.py'
CAM = 'autolabor_c1_bringup/launch/autolabor_c1.launch.py'


def run_stack(tmp_path, *args):
    out = tmp_path / 'out'
    out.mkdir()
    env = dict(os.environ, AGT_TEST_OUT=str(out), AGT_TEST_PLAN=str(PLAN),
               AGT_TEST_SCRIPT=str(SCRIPT), AGT_FIELD_LOG_DIR=str(tmp_path / 'run'))
    proc = subprocess.run(['bash', '-c', SHIM, 'shim', '--map', 'auto', *args],
                          env=env, capture_output=True, text=True, timeout=180)
    plan_file = out / 'hardware_plan.json'
    plan = json.loads(plan_file.read_text()) if plan_file.is_file() else None
    launches = sorted(p.name for p in out.glob('*.argv'))
    hw_argv = (out / 'agt_system_bringup__hardware.launch.py.argv')
    argv = hw_argv.read_text().split('\n') if hw_argv.is_file() else []
    return proc, plan, launches, argv


def test_legacy_navigation_camera_off(tmp_path):
    proc, plan, launches, argv = run_stack(tmp_path, '--mode', 'navigation')
    assert launches == ['agt_system_bringup__hardware.launch.py.argv'], proc.stderr
    assert plan['ok'], plan
    assert plan['includes'] == [DESC, MID, BUNKER]
    snapshot = tmp_path / 'run' / 'robot_config' / 'bunker_inspection'
    assert f'robot_config:={snapshot}' in argv
    assert (snapshot / 'robot.yaml').is_file()
    assert plan['launch_args']['bunker_can_port'] == 'can0'


def test_explicit_config_navigation_follows_yaml(tmp_path):
    proc, plan, _, argv = run_stack(tmp_path, '--mode', 'navigation',
                                    '--robot-config', 'bunker_inspection')
    assert plan['ok'], (plan, proc.stderr)
    assert plan['includes'] == [DESC, MID, BUNKER, CAM]   # YAML: camera enabled
    assert not any(a.startswith('enable_camera_gimbal') for a in argv)


def test_inspection_rtk(tmp_path):
    proc, plan, _, argv = run_stack(tmp_path, '--mode', 'inspection', '--rtk')
    assert plan['ok'], (plan, proc.stderr)
    assert plan['includes'] == [DESC, MID, BUNKER, RTK, CAM]
    assert 'enable_camera_gimbal:=true' in argv and 'enable_rtk:=true' in argv


def test_external_config_path_is_the_one_launched(tmp_path):
    # Review P1: preflight and launch must use the same validated config.
    ext = tmp_path / 'ext' / 'bunker_inspection'
    shutil.copytree(ROBOTS / 'bunker_inspection', ext)
    dev = yaml.safe_load((ext / 'devices.yaml').read_text())
    dev['base']['can_port'] = 'can9'
    (ext / 'devices.yaml').write_text(yaml.safe_dump(dev))
    proc, plan, _, _ = run_stack(tmp_path, '--mode', 'navigation',
                                 '--robot-config', str(ext / 'robot.yaml'))
    assert plan['ok'], (plan, proc.stderr)
    assert plan['launch_args']['bunker_can_port'] == 'can9'


def test_external_config_new_id_launches(tmp_path):
    ext = tmp_path / 'ext' / 'bunker_lab'
    shutil.copytree(ROBOTS / 'bunker_inspection', ext)
    for name in ('robot.yaml', 'devices.yaml'):
        data = yaml.safe_load((ext / name).read_text())
        data['robot_id'] = 'bunker_lab'
        (ext / name).write_text(yaml.safe_dump(data))
    proc, plan, _, _ = run_stack(tmp_path, '--mode', 'navigation', '--robot-config', str(ext))
    assert plan and plan['ok'], (plan, proc.stderr)


@pytest.mark.parametrize('args', [
    ('--mode', 'navigation', '--robot-config', 'yhs_harvesting'),
    ('--mode', 'navigation', '--robot', 'yhs_v1', '--robot-config', 'bunker_inspection'),
])
def test_rejected_configs_start_nothing(tmp_path, args):
    proc, plan, launches, _ = run_stack(tmp_path, *args)
    assert proc.returncode == 2
    assert launches == [] and plan is None
    assert 'nothing was started' in proc.stderr


def test_yhs_network_and_nav_flags_cannot_be_applied_to_bunker(tmp_path):
    # Wrong-robot flags fail before any hardware launch, even if files exist.
    cfg = tmp_path / 'yhs.json'
    cfg.write_text('{}')
    proc, plan, launches, _ = run_stack(tmp_path, '--mode', 'navigation',
                                        '--robot-config', 'bunker_inspection',
                                        '--yhs-livox-config', str(cfg),
                                        '--yhs-nav-config-dir', str(tmp_path))
    assert proc.returncode == 2
    assert plan is None and launches == []
    assert 'only valid for robot_config yhs_harvesting' in proc.stderr


def test_yhs_stays_blocked_with_explicit_livox_json(tmp_path):
    # The network path is never a bypass for missing gear, YHS map and geometry.
    cfg = tmp_path / 'yhs.json'
    cfg.write_text('{}')
    proc, plan, launches, _ = run_stack(tmp_path, '--mode', 'navigation',
                                        '--robot-config', 'yhs_harvesting', '--robot', 'yhs_v1',
                                        '--map', 'yhs_mid360/field-v1',
                                        '--yhs-livox-config', str(cfg))
    assert proc.returncode == 2
    assert plan is None and launches == []
    assert 'BLOCKED' in proc.stderr
