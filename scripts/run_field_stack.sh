#!/usr/bin/env bash

# One-terminal field orchestrator. It preserves the four top-level launch
# architecture and adds only readiness gates plus ordered shutdown.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
WS_ROOT=$(cd -- "$REPO_ROOT/../.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
set -euo pipefail

MAP_ROOT=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v005-confirmed-keepout
MAP_ID=bunker_mid360_v005_confirmed_keepout
MODE=navigation
ENABLE_RTK=false
ENABLE_RVIZ=false
DRY_RUN=false
STAMP=$(date +%Y%m%d_%H%M%S)
RUN_DIR=${AGT_FIELD_LOG_DIR:-"$HOME/.ros/agt_field_stack/$STAMP"}

# Diagnostic observation pass-through. The defaults reproduce an ordinary field run
# exactly; they exist so evidence can be collected without editing canonical configs.
OBSTACLE_STATS=""
OBSTACLE_DEBUG_BASE_CLOUD=false
OBSTACLE_LOG_INTERVAL=0.0
RVIZ_CONFIG=""

usage() {
  cat <<'EOF'
Usage: scripts/run_field_stack.sh [options]

Options:
  --mode MODE       Runtime mode: navigation (default) or inspection.
                    navigation: Nav2 only, camera and capture task disabled.
                    inspection: stop at each queued point, capture three views,
                    save images and metadata, then continue/return home.
  --map-root PATH   Map package root.
  --map-id ID       Runtime map identifier.
  --rviz            Start debug.launch.py after all checks pass.
  --rviz-config PATH
                    RViz config for --rviz (absolute path; use the diagnostics config
                    from the source tree when the package has not been rebuilt).
  --rtk             Enable RTK hardware input (metadata only).
  --dry-run         Validate mode/map inputs and print the resolved stack only.
  --obstacle-stats PATH
                    Write cumulative obstacle-filter statistics to PATH on clean exit.
  --obstacle-debug-base-cloud
                    Publish accepted obstacle points in base_link for audit/RViz.
  --obstacle-log-interval SEC
                    Periodic cumulative filter-statistics log interval; 0 disables it.
  -h, --help        Show this help.

The script starts the existing four-entry architecture in dependency order.
Press Ctrl+C once for ordered shutdown. Rosbag recording remains a separate
acceptance action so it can be stopped and finalized before powering off.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE=${2:?--mode requires navigation or inspection}
      shift 2
      ;;
    --map-root)
      MAP_ROOT=${2:?--map-root requires a path}
      shift 2
      ;;
    --map-id)
      MAP_ID=${2:?--map-id requires an id}
      shift 2
      ;;
    --rviz)
      ENABLE_RVIZ=true
      shift
      ;;
    --rviz-config)
      RVIZ_CONFIG=${2:?--rviz-config requires a path}
      shift 2
      ;;
    --rtk)
      ENABLE_RTK=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --obstacle-stats)
      OBSTACLE_STATS=${2:?--obstacle-stats requires a path}
      shift 2
      ;;
    --obstacle-debug-base-cloud)
      OBSTACLE_DEBUG_BASE_CLOUD=true
      shift
      ;;
    --obstacle-log-interval)
      OBSTACLE_LOG_INTERVAL=${2:?--obstacle-log-interval requires seconds}
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  navigation)
    ENABLE_INSPECTION=false
    ;;
  inspection)
    ENABLE_INSPECTION=true
    ;;
  *)
    printf 'Invalid --mode %s; expected navigation or inspection\n' "$MODE" >&2
    exit 2
    ;;
esac

if [[ -z "$RVIZ_CONFIG" ]]; then
  if [[ "$ENABLE_INSPECTION" == true ]]; then
    RVIZ_CONFIG="$WS_ROOT/install/agt_rviz_patrol/share/agt_rviz_patrol/config/agt_rviz_demo.rviz"
  else
    RVIZ_CONFIG=/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
  fi
fi

GLOBAL_MAP="$MAP_ROOT/localization/global_map.pcd"
RELOCALIZATION_ASSETS="$MAP_ROOT/localization/relocalization"
NAV_MAP="$MAP_ROOT/navigation/map.yaml"
for required in "$GLOBAL_MAP" "$RELOCALIZATION_ASSETS" "$NAV_MAP"; do
  if [[ ! -e "$required" ]]; then
    printf 'Missing map asset: %s\n' "$required" >&2
    exit 2
  fi
done

if [[ "$DRY_RUN" == true ]]; then
  printf 'mode=%s\n' "$MODE"
  printf 'map_root=%s\n' "$MAP_ROOT"
  printf 'map_id=%s\n' "$MAP_ID"
  printf 'navigation_map=%s\n' "$NAV_MAP"
  printf 'localization_map=%s\n' "$GLOBAL_MAP"
  printf 'relocalization_assets=%s\n' "$RELOCALIZATION_ASSETS"
  printf 'camera_gimbal=%s\n' "$ENABLE_INSPECTION"
  printf 'inspection_runtime=%s\n' "$ENABLE_INSPECTION"
  printf 'rviz_config=%s\n' "$RVIZ_CONFIG"
  exit 0
fi

mkdir -p -- "$RUN_DIR"
declare -a CHILD_PIDS=()
declare -a CHILD_LABELS=()

start_child() {
  local label=$1
  shift
  local logfile="$RUN_DIR/$label.log"
  printf '[START] %-12s log=%s\n' "$label" "$logfile"
  "$@" >"$logfile" 2>&1 &
  CHILD_PIDS+=("$!")
  CHILD_LABELS+=("$label")
}

tail_failure() {
  local label=$1
  local logfile="$RUN_DIR/$label.log"
  printf '\nLast output from %s:\n' "$label" >&2
  tail -60 "$logfile" >&2 || true
}

wait_for_topic() {
  local label=$1
  local topic=$2
  local timeout_sec=$3
  local deadline=$((SECONDS + timeout_sec))
  printf '[WAIT]  %-12s topic=%s\n' "$label" "$topic"
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      printf '[PASS]  %-12s topic=%s\n' "$label" "$topic"
      return 0
    fi
    sleep 1
  done
  printf '[FAIL]  %-12s no message on %s within %ss\n' \
    "$label" "$topic" "$timeout_sec" >&2
  return 1
}

wait_for_service() {
  local service=$1
  local timeout_sec=$2
  local deadline=$((SECONDS + timeout_sec))
  printf '[WAIT]  service=%s\n' "$service"
  while (( SECONDS < deadline )); do
    if ros2 service list --no-daemon --spin-time 1 --include-hidden-services 2>/dev/null \
        | grep -Fxq "$service"; then
      printf '[PASS]  service=%s\n' "$service"
      return 0
    fi
    sleep 1
  done
  printf '[FAIL]  missing service %s after %ss\n' "$service" "$timeout_sec" >&2
  return 1
}

wait_for_adapter() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))
  local consecutive=0
  printf '[WAIT]  Batch-LIO adapter FRESH\n'
  while (( SECONDS < deadline )); do
    local output
    output=$(timeout 3 ros2 topic echo /agt/odometry/adapter_status --once 2>/dev/null || true)
    if grep -Eq '^state: 0$' <<<"$output"; then
      consecutive=$((consecutive + 1))
      if (( consecutive >= 3 )); then
        printf '[PASS]  Batch-LIO adapter FRESH for three samples\n'
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 1
  done
  printf '[FAIL]  Batch-LIO adapter did not remain FRESH\n' >&2
  return 1
}

wait_for_localized() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))
  printf '[WAIT]  global localization\n'
  while (( SECONDS < deadline )); do
    local output
    output=$(timeout 3 ros2 topic echo /agt/localization/status --once 2>/dev/null || true)
    if grep -Eq '^state: 3$' <<<"$output" \
        && grep -Eq '^local_odom_fresh: true$' <<<"$output" \
        && grep -Eq '^global_correction_valid: true$' <<<"$output"; then
      printf '[PASS]  localization state=LOCALIZED\n'
      return 0
    fi
    if grep -Eq '^reason: global_relocalization_(failed|rejected):' <<<"$output"; then
      printf '%s\n' "$output" >&2
      printf '[FAIL]  global relocalization reached a terminal failure\n' >&2
      return 1
    fi
    sleep 1
  done
  printf '[FAIL]  localization did not become ready within %ss\n' "$timeout_sec" >&2
  return 1
}

cleanup() {
  local original_status=$?
  trap - EXIT INT TERM
  printf '\n[STOP] ordered stack shutdown\n'
  local i pid label
  for ((i=${#CHILD_PIDS[@]}-1; i>=0; i--)); do
    pid=${CHILD_PIDS[$i]}
    label=${CHILD_LABELS[$i]}
    if kill -0 "$pid" 2>/dev/null; then
      printf '[STOP] %-12s pid=%s\n' "$label" "$pid"
      kill -INT "$pid" 2>/dev/null || true
      local child_deadline=$((SECONDS + 15))
      while kill -0 "$pid" 2>/dev/null && (( SECONDS < child_deadline )); do
        sleep 1
      done
      if kill -0 "$pid" 2>/dev/null; then
        printf '[STOP] %-12s did not exit after SIGINT; sending SIGTERM\n' "$label" >&2
        kill -TERM "$pid" 2>/dev/null || true
      fi
    fi
    wait "$pid" 2>/dev/null || true
  done
  printf '[STOP] complete; logs=%s\n' "$RUN_DIR"
  exit "$original_status"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

existing_nodes=$(ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)
for owner in /robot_state_publisher /agt_localization_manager /agt_pointcloud_preprocessor; do
  if grep -Fxq "$owner" <<<"$existing_nodes"; then
    printf 'Refusing to create duplicate owner; node already exists: %s\n' "$owner" >&2
    exit 1
  fi
done

start_child hardware \
  ros2 launch agt_system_bringup hardware.launch.py \
  bunker_can_port:=can0 enable_rtk:="$ENABLE_RTK" \
  enable_camera_gimbal:="$ENABLE_INSPECTION"
wait_for_topic MID360 /livox/lidar 45 || { tail_failure hardware; exit 1; }
wait_for_topic MID360-IMU /livox/imu 30 || { tail_failure hardware; exit 1; }
wait_for_topic Bunker /wheel/odom 30 || { tail_failure hardware; exit 1; }

start_child localization \
  ros2 launch agt_system_bringup localization.launch.py \
  global_map:="$GLOBAL_MAP" \
  relocalization_assets:="$RELOCALIZATION_ASSETS" \
  auto_relocalize:=false
wait_for_service /agt/localization/relocalize 45 || { tail_failure localization; exit 1; }
wait_for_adapter 60 || { tail_failure localization; exit 1; }

printf '[CALL]  global relocalization (request waits for a complete stationary query)\n'
ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger '{}' \
  >"$RUN_DIR/relocalize_service.txt" 2>&1
wait_for_localized 60 || { tail_failure localization; exit 1; }

declare -a NAV_OBS_ARGS=()
if [[ -n "$OBSTACLE_STATS" ]]; then
  NAV_OBS_ARGS+=("obstacle_statistics_output:=$OBSTACLE_STATS")
fi
if [[ "$OBSTACLE_DEBUG_BASE_CLOUD" == true ]]; then
  NAV_OBS_ARGS+=("obstacle_debug_base_cloud_enabled:=true")
fi
if [[ "$OBSTACLE_LOG_INTERVAL" != "0.0" ]]; then
  NAV_OBS_ARGS+=("obstacle_debug_log_interval_sec:=$OBSTACLE_LOG_INTERVAL")
fi

start_child navigation \
  ros2 launch agt_system_bringup navigation.launch.py \
  map:="$NAV_MAP" map_id:="$MAP_ID" \
  enable_inspection:="$ENABLE_INSPECTION" \
  ${NAV_OBS_ARGS[@]+"${NAV_OBS_ARGS[@]}"}
wait_for_service /navigate_to_pose/_action/send_goal 60 \
  || { tail_failure navigation; exit 1; }

"$SCRIPT_DIR/check_runtime.sh" | tee "$RUN_DIR/check_runtime.txt"
"$SCRIPT_DIR/check_tf.sh" | tee "$RUN_DIR/check_tf.txt"
"$SCRIPT_DIR/check_topics.sh" | tee "$RUN_DIR/check_topics.txt"
ros2 run agt_navigation_runtime demo_preflight --ros-args \
  -p require_camera:="$ENABLE_INSPECTION" \
  | tee "$RUN_DIR/demo_preflight.txt"

if [[ "$ENABLE_RVIZ" == true ]]; then
  start_child debug \
    ros2 launch agt_system_bringup debug.launch.py \
    run_preflight:=false rviz_config:="$RVIZ_CONFIG"
fi

printf '\n[READY] Hardware, localization and Nav2 passed preflight.\n'
printf '[READY] Mode: %s\n' "$MODE"
if [[ "$ENABLE_INSPECTION" == true ]]; then
  printf '[READY] Inspection: queue RViz points, then call /agt/rviz_patrol/start.\n'
  printf '[READY] Records: ~/.ros/agt_inspection_records/\n'
else
  printf '[READY] Pure navigation: camera and inspection mission nodes are disabled.\n'
fi
printf '[READY] Logs: %s\n' "$RUN_DIR"
if [[ -n "$OBSTACLE_STATS" ]]; then
  printf '[READY] Obstacle-filter statistics will be written on clean exit: %s\n' "$OBSTACLE_STATS"
fi
printf '[READY] Start recording in another shell only when needed; stop it with Ctrl+C before power-off.\n'
printf '[READY] Press Ctrl+C here for ordered stack shutdown.\n'

while true; do
  for i in "${!CHILD_PIDS[@]}"; do
    pid=${CHILD_PIDS[$i]}
    if ! kill -0 "$pid" 2>/dev/null; then
      label=${CHILD_LABELS[$i]}
      printf '[FAIL] child exited unexpectedly: %s (pid=%s)\n' "$label" "$pid" >&2
      tail_failure "$label"
      exit 1
    fi
  done
  sleep 2
done
