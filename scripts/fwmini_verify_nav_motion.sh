#!/usr/bin/env bash
# Read-only ABI and Nav2 progress checks for an ALREADY running FW-mini stack.
# No services called, no velocity/goal published, no enable state changes.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/install/nav2_trial/setup.bash"
echo "=== FW-mini runtime CtrlCmd ABI ==="
pid="$(pgrep -f '^/home/yangxuan/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/install/nav2_trial/fwmini_chassis_adapter/lib/fwmini_chassis_adapter/fwmini_chassis_adapter_node' | head -1 || true)"
if [[ -z "$pid" ]]; then echo "FAIL: chassis adapter not running"; exit 2; fi
echo "adapter_pid=$pid"
name="libyhs_can_interfaces__rosidl_typesupport_cpp.so"
correct="$HOME/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/install/nav2_trial/yhs_can_interfaces/lib/$name"
if ! grep -Fq "$correct" "/proc/$pid/maps"; then
  echo "FAIL: adapter is using wrong CtrlCmd ABI. Restart ORIGINAL FW-mini Nav2 launch."
  grep "yhs_can_interfaces/lib/$name" "/proc/$pid/maps" | head -1 || true
  exit 3
fi
echo "PASS: FW-mini vendor-compatible typesupport library loaded"
echo "=== Nav2 motion progress ==="
actual="$(timeout 5 ros2 param get /controller_server progress_checker.plugin 2>&1 || true)"
echo "$actual"
if [[ "$actual" != *'nav2_controller::PoseProgressChecker'* ]]; then
  echo "FAIL: old Nav2 controller still running. Restart ORIGINAL nav launch."
  exit 4
fi
echo "PASS: PoseProgressChecker active"
echo "=== software motion enable (READ ONLY) ==="
timeout 5 ros2 param get /fwmini_chassis_adapter initial_motion_enabled || true
echo "=== health snapshot (READ ONLY) ==="
timeout 5 ros2 topic echo --once /navigation/health | grep -E 'state:|status:|imu_alive:|odom_alive:|localized:|nav2_active:' || true
echo "=== READY FOR OPERATOR-CONTROLLED SHORT ROUTE TEST ==="
