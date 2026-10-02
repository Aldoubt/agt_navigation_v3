"""No-device contracts for the independent hardware and navigation commands.

Both commands are executed ONLY with --dry-run. The launch plan is computed
in pure Python; no ros2 launch/driver/goal is ever invoked by these tests.
"""
import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
WS = ROOT.parents[1]
NAV = ROOT / 'scripts/run_field_stack.sh'
HARDWARE = ROOT / 'scripts/run_bunker_hardware.sh'
PLAN = WS / 'install/agt_robot_bringup/lib/agt_robot_bringup/hardware_launch_plan.py'
ROBOTS = WS / 'install/agt_robot_bringup/share/agt_robot_bringup/config/robots'

if not PLAN.is_file() or not Path('/opt/ros/humble/setup.bash').is_file():
    pytest.skip('needs Humble + built agt_robot_bringup', allow_module_level=True)

DESC = 'agt_robot_description/launch/display.launch.py'
MID = 'livox_ros_driver2/launch_ROS2/msg_MID360_launch.py'
BUNKER = 'bunker_base/launch/bunker_base.launch.py'
RTK = 'agt_asensing_driver/launch/asensing.launch.py'
CAM = 'autolabor_c1_bringup/launch/autolabor_c1.launch.py'


def hardware_dry_run(*args):
    proc = subprocess.run(['bash', str(HARDWARE), '--dry-run', *args],
                          capture_output=True, text=True, timeout=30)
    if proc.returncode:
        return proc, None, []
    command = next(line.partition('DRY_RUN: ')[2] for line in proc.stdout.splitlines()
                   if line.startswith('DRY_RUN: '))
    argv = shlex.split(command)
    assert argv[:4] == ['ros2', 'launch', 'agt_robot_bringup', 'robot_hardware.launch.py']
    plan_proc = subprocess.run(['python3', str(PLAN), *argv[4:]],
                               capture_output=True, text=True, timeout=20)
    assert plan_proc.returncode == 0, (plan_proc.stdout, plan_proc.stderr)
    return proc, json.loads(plan_proc.stdout), argv


def nav_dry_run(*args):
    return subprocess.run(['bash', str(NAV), '--dry-run', '--map', 'auto', *args],
                          env=dict(os.environ),
                          capture_output=True, text=True, timeout=30)


def test_default_hardware_is_single_bunker_lidar_model_owner_without_camera():
    proc, plan, argv = hardware_dry_run()
    assert proc.returncode == 0, proc.stderr
    assert plan['ok'] and plan['includes'] == [DESC, MID, BUNKER]
    assert plan['launch_args']['bunker_can_port'] == 'can0'
    assert 'enable_camera_gimbal:=false' in argv
    nav = nav_dry_run('--mode', 'navigation')
    assert nav.returncode == 0, nav.stderr
    assert 'hardware_owner=external' in nav.stdout
    assert 'initialization_source=automatic_global_relocalization' in nav.stdout
    assert 'localization_mode=auto' in nav.stdout
    assert 'camera_gimbal=false' in nav.stdout
    assert 'hardware_launch_args=' not in nav.stdout


def test_field_stack_exposes_cpp_guard_and_python_rollback(monkeypatch):
    monkeypatch.delenv('AGT_MOTION_GUARD_BACKEND', raising=False)
    default = nav_dry_run('--mode', 'navigation')
    assert default.returncode == 0, default.stderr
    assert 'motion_guard_backend=cpp' in default.stdout

    rollback = nav_dry_run('--mode', 'inspection', '--motion-guard-backend', 'python')
    assert rollback.returncode == 0, rollback.stderr
    assert 'motion_guard_backend=python' in rollback.stdout
    assert 'hardware_owner=external' in rollback.stdout


def test_field_stack_rejects_unknown_motion_guard_before_startup():
    proc = nav_dry_run('--mode', 'navigation', '--motion-guard-backend', 'bunker')
    assert proc.returncode == 2
    assert 'expected cpp or python' in proc.stderr
    assert 'hardware_owner=external' not in proc.stdout


def test_explicit_nav_config_never_enables_camera_driver_itself():
    nav = nav_dry_run('--mode', 'navigation', '--robot-config', 'bunker_inspection')
    assert nav.returncode == 0, nav.stderr
    assert 'camera_gimbal=true' in nav.stdout  # config expectation, not a new launch
    assert 'hardware_owner=external' in nav.stdout
    proc, plan, _ = hardware_dry_run('--robot-config', 'bunker_inspection')
    assert proc.returncode == 0 and plan['includes'] == [DESC, MID, BUNKER]


def test_inspection_c1_and_rtk_belong_only_to_hardware_command():
    proc, plan, argv = hardware_dry_run('--camera', '--rtk')
    assert proc.returncode == 0, proc.stderr
    assert plan['ok'] and plan['includes'] == [DESC, MID, BUNKER, RTK, CAM]
    assert 'enable_camera_gimbal:=true' in argv and 'enable_rtk:=true' in argv
    nav = nav_dry_run('--mode', 'inspection', '--rtk')
    assert nav.returncode == 0, nav.stderr
    assert 'inspection_runtime=true' in nav.stdout and 'camera_gimbal=true' in nav.stdout
    assert 'hardware_owner=external' in nav.stdout


def test_explicit_external_config_port_is_the_one_resolved(tmp_path):
    ext = tmp_path / 'bunker_inspection'
    shutil.copytree(ROBOTS / 'bunker_inspection', ext)
    dev = yaml.safe_load((ext / 'devices.yaml').read_text())
    dev['base']['can_port'] = 'can9'
    (ext / 'devices.yaml').write_text(yaml.safe_dump(dev))
    proc, plan, argv = hardware_dry_run('--robot-config', str(ext / 'robot.yaml'))
    assert proc.returncode == 0 and plan['ok'], (proc.stderr, plan)
    assert plan['launch_args']['bunker_can_port'] == 'can9'
    assert f'robot_config:={ext / "robot.yaml"}' in argv
    nav = nav_dry_run('--mode', 'navigation', '--robot-config', str(ext / 'robot.yaml'))
    assert nav.returncode == 0 and f'robot_config_dir={ext}' in nav.stdout


@pytest.mark.parametrize('bad', [
    ('--robot-config', 'yhs_harvesting'),
    ('--robot-config', 'bunker_inspection', '--robot', 'yhs_v1'),
])
def test_rejected_nav_configs_never_start_hardware(bad):
    proc = nav_dry_run('--mode', 'navigation', *bad)
    assert proc.returncode == 2 and 'nothing was started' in proc.stderr


def test_wrong_robot_hardware_config_is_blocked():
    proc, plan, argv = hardware_dry_run('--robot-config', 'yhs_harvesting')
    assert proc.returncode != 0 and plan is None and not argv
    assert 'BLOCKED' in proc.stderr


def test_v1_field_stack_rejects_manual_seed_modes_before_any_ros_graph_or_driver():
    proc = nav_dry_run('--mode', 'navigation', '--localization-mode', 'manual')
    assert proc.returncode == 2
    assert 'only auto' in proc.stderr
    assert 'hardware_owner=external' not in proc.stdout

    hint = nav_dry_run('--mode', 'navigation', '--start-hint', '/tmp/not-used.yaml')
    assert hint.returncode == 2
    assert 'Unknown option: --start-hint' in hint.stderr
    assert 'hardware_owner=external' not in hint.stdout
