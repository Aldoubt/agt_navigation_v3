#!/usr/bin/env bash

# One-terminal field orchestrator. It preserves the four top-level launch
# architecture and adds only readiness gates plus ordered shutdown.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
WS_ROOT=$(cd -- "$REPO_ROOT/../.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
set -euo pipefail

MAP_SPEC=auto
ROBOT_PROFILE=bunker_v1
MAP_REGISTRY=${AGT_MAP_REGISTRY:-"$WS_ROOT/maps/registry.yaml"}
MAP_ROOT_OVERRIDE=""
MAP_ID_OVERRIDE=""
MODE=navigation
LIO_BACKEND=batch_lio
LIO_CONFIG=""
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
REAR_POINTCLOUD_MASK=false
RVIZ_CONFIG=""

usage() {
  cat <<'EOF'
Usage: scripts/run_field_stack.sh [options]

Options:
  --mode MODE       Runtime mode: navigation (default) or inspection.
                    navigation: Nav2 only, camera and capture task disabled.
                    inspection: stop at each queued point, capture three views,
                    save images and metadata, then continue/return home.
  --lio-backend NAME Local odometry: batch_lio (default) or fastlio2; mutually exclusive.
  --lio-config PATH  Optional runtime YAML for the selected LIO backend.
  --map SPEC        auto (default), active, latest, map_id or map_id/version.
  --robot PROFILE   Robot profile (default: bunker_v1).
  --map-registry PATH
                    Validated map registry (default: <workspace>/maps/registry.yaml).
  --map-root PATH   Legacy map package selection; must be in the V4 registry.
  --map-id ID       Legacy map identifier; must be in the V4 registry.
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
  --rear-pointcloud-mask
                    Trial mask for the rear-mounted pole: base_link bearing
                    180 +/- 35 deg, planar range 0.5-1.0 m. Obstacle marking only.
  -h, --help        Show this help.

The script starts the staged runtime in dependency order.
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
    --lio-backend)
      LIO_BACKEND=${2:?--lio-backend requires batch_lio or fastlio2}
      shift 2
      ;;
    --lio-config)
      LIO_CONFIG=${2:?--lio-config requires a path}
      shift 2
      ;;
    --map-root)
      MAP_ROOT_OVERRIDE=${2:?--map-root requires a path}
      shift 2
      ;;
    --map-id)
      MAP_ID_OVERRIDE=${2:?--map-id requires an id}
      shift 2
      ;;
    --map)
      MAP_SPEC=${2:?--map requires a spec}
      shift 2
      ;;
    --robot)
      ROBOT_PROFILE=${2:?--robot requires a profile}
      shift 2
      ;;
    --map-registry)
      MAP_REGISTRY=${2:?--map-registry requires a path}
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
    --rear-pointcloud-mask)
      REAR_POINTCLOUD_MASK=true
      shift
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

case "$LIO_BACKEND" in
  batch_lio)
    LIO_CONFIG_NAME=batch_lio_mid360.yaml
    LIO_CONFIG_ARGUMENT=batch_config
    LIO_RAW_ODOM=/aft_mapped_to_init
    ;;
  fastlio2)
    LIO_CONFIG_NAME=fastlio2_mid360_navigation.yaml
    LIO_CONFIG_ARGUMENT=fastlio_config
    LIO_RAW_ODOM=/fastlio2/lio_odom
    ;;
  *)
    printf 'Invalid --lio-backend %s; expected batch_lio or fastlio2\n' "$LIO_BACKEND" >&2
    exit 2
    ;;
esac
if [[ -z "$LIO_CONFIG" ]]; then
  LIO_CONFIG="$WS_ROOT/install/agt_navigation_runtime/share/agt_navigation_runtime/config/$LIO_CONFIG_NAME"
fi
if [[ ! -f "$LIO_CONFIG" ]]; then
  printf 'Missing LIO configuration: %s\n' "$LIO_CONFIG" >&2
  exit 2
fi

if [[ -z "$RVIZ_CONFIG" ]]; then
  if [[ "$ENABLE_INSPECTION" == true ]]; then
    RVIZ_CONFIG="$WS_ROOT/install/agt_rviz_patrol/share/agt_rviz_patrol/config/agt_rviz_demo.rviz"
  else
    RVIZ_CONFIG="$WS_ROOT/install/agt_rviz_patrol/share/agt_rviz_patrol/config/agt_rviz_costmap_diagnostics.rviz"
  fi
fi

if [[ -n "$MAP_ROOT_OVERRIDE" ]]; then
  if [[ ! -f "$MAP_ROOT_OVERRIDE/metadata.yaml" ]]; then
    printf 'Missing metadata.yaml in --map-root: %s\n' "$MAP_ROOT_OVERRIDE" >&2
    exit 2
  fi
  MAP_SPEC=$(python3 - "$MAP_ROOT_OVERRIDE/metadata.yaml" <<'PY'
import sys
import yaml
data = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
print(f"{data['map_id']}/{data['map_version']}")
PY
)
fi
if [[ -n "$MAP_ID_OVERRIDE" && -z "$MAP_ROOT_OVERRIDE" ]]; then
  MAP_SPEC="$MAP_ID_OVERRIDE"
fi
if [[ -n "$MAP_ID_OVERRIDE" && "$MAP_ID_OVERRIDE" != "${MAP_SPEC%%/*}" ]]; then
  printf 'Conflicting --map-id and --map-root\n' >&2
  exit 2
fi
MAP_SELECTION=$(ros2 run agt_map_manager resolve_map --map "$MAP_SPEC" --robot "$ROBOT_PROFILE" --registry "$MAP_REGISTRY") || exit 2
while IFS='=' read -r key value; do
  case "$key" in
    map_root) MAP_ROOT=$value ;;
    map_id) MAP_ID=$value ;;
    map_version) MAP_VERSION=$value ;;
    navigation_map) NAV_MAP=$value ;;
    localization_map) GLOBAL_MAP=$value ;;
    relocalization_assets) RELOCALIZATION_ASSETS=$value ;;
  esac
done <<< "$MAP_SELECTION"
for required in "$GLOBAL_MAP" "$RELOCALIZATION_ASSETS" "$NAV_MAP"; do
  if [[ ! -e "$required" ]]; then
    printf 'Missing map asset: %s\n' "$required" >&2
    exit 2
  fi
done

if [[ "$DRY_RUN" == true ]]; then
  printf 'mode=%s\n' "$MODE"
  printf 'lio_backend=%s\n' "$LIO_BACKEND"
  printf 'lio_config=%s\n' "$LIO_CONFIG"
  printf 'lio_raw_odometry=%s\n' "$LIO_RAW_ODOM"
  printf 'rear_pointcloud_mask=%s\n' "$REAR_POINTCLOUD_MASK"
  if [[ "$REAR_POINTCLOUD_MASK" == true ]]; then
    printf 'rear_pointcloud_mask_sector=center:180deg,width:70deg,range:0.5-1.0m\n'
  fi
  printf 'map_root=%s\n' "$MAP_ROOT"
  printf 'map_registry=%s\n' "$MAP_REGISTRY"
  printf 'map_spec=%s\nrobot_profile=%s\n' "$MAP_SPEC" "$ROBOT_PROFILE"
  printf 'map_id=%s\n' "$MAP_ID"
  printf 'map_version=%s\n' "$MAP_VERSION"
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
  # Give every top-level launch its own process group. A ros2 launch wrapper
  # can exit before all of its nodes; cleanup must still be able to signal the
  # complete group instead of leaving hardware/localization owners behind.
  setsid "$@" >"$logfile" 2>&1 &
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

wait_for_node() {
  local node=$1
  local timeout_sec=$2
  local deadline=$((SECONDS + timeout_sec))
  printf '[WAIT]  node=%s\n' "$node"
  while (( SECONDS < deadline )); do
    if ros2 node list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq "$node"; then
      printf '[PASS]  node=%s\n' "$node"
      return 0
    fi
    sleep 1
  done
  printf '[FAIL]  missing node %s after %ss\n' "$node" "$timeout_sec" >&2
  return 1
}

wait_for_adapter() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))
  local consecutive=0
  printf '[WAIT]  %s adapter FRESH\n' "$LIO_BACKEND"
  while (( SECONDS < deadline )); do
    local output
    output=$(timeout 3 ros2 topic echo /agt/odometry/adapter_status --once 2>/dev/null || true)
    if grep -Eq '^state: 0$' <<<"$output"; then
      consecutive=$((consecutive + 1))
      if (( consecutive >= 3 )); then
        printf '[PASS]  %s adapter FRESH for three samples\n' "$LIO_BACKEND"
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 1
  done
  printf '[FAIL]  %s adapter did not remain FRESH\n' "$LIO_BACKEND" >&2
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

wait_for_localization_settle() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))
  local consecutive=0
  printf '[WAIT]  localization settle (preserve accepted global correction)\n'
  while (( SECONDS < deadline )); do
    local output
    output=$(timeout 3 ros2 topic echo /agt/localization/status --once 2>/dev/null || true)
    if grep -Eq '^state: 3$' <<<"$output" \
        && grep -Eq '^local_odom_fresh: true$' <<<"$output" \
        && grep -Eq '^global_correction_valid: true$' <<<"$output"; then
      consecutive=$((consecutive + 1))
      if (( consecutive >= 3 )); then
        printf '[PASS]  localization remained LOCALIZED for three samples\n'
        return 0
      fi
    else
      consecutive=0
    fi
    sleep 0.5
  done
  return 1
}

relocalize_until_ready() {
  local log_prefix=$1
  local attempt
  for attempt in 1 2; do
    printf '[CALL]  global relocalization attempt %s/2 (keep robot stationary)\n' "$attempt"
    if ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger '{}' \
        >"$RUN_DIR/${log_prefix}_service_${attempt}.txt" 2>&1 \
        && wait_for_localized 60; then
      return 0
    fi
    if (( attempt < 2 )); then
      printf '[RETRY] global relocalization did not converge; waiting for a fresh stationary query\n'
      sleep 3
    fi
  done
  return 1
}

ensure_localization_ready() {
  local phase=$1
  printf '[VERIFY] localization %s\n' "$phase"
  # A brief FAST-LIO scheduling delay can publish DEGRADED for one sample while
  # the already accepted map->odom correction remains valid. Give that state a
  # short grace period; a manual relocalization invalidates the good correction
  # immediately and must never be used merely to cure stale local odometry.
  if wait_for_localization_settle 15; then
    return 0
  fi

  local output
  output=$(timeout 3 ros2 topic echo /agt/localization/status --once 2>/dev/null || true)
  printf '%s\n' "$output" >&2
  if grep -Eq '^global_correction_valid: true$' <<<"$output"; then
    printf '[FAIL] localization did not settle %s; preserving the valid global correction\n' \
      "$phase" >&2
  else
    printf '[FAIL] global correction is invalid %s; refusing a second startup relocalization\n' \
      "$phase" >&2
  fi
  return 1
}

stop_process_group() {
  local label=$1
  local pid=$2
  if ! kill -0 -- "-$pid" 2>/dev/null; then
    return 0
  fi

  printf '[STOP] %-12s process_group=%s\n' "$label" "$pid"
  kill -INT -- "-$pid" 2>/dev/null || true
  local child_deadline=$((SECONDS + 15))
  while kill -0 -- "-$pid" 2>/dev/null && (( SECONDS < child_deadline )); do
    sleep 1
  done

  if kill -0 -- "-$pid" 2>/dev/null; then
    printf '[STOP] %-12s group did not exit after SIGINT; sending SIGTERM\n' "$label" >&2
    kill -TERM -- "-$pid" 2>/dev/null || true
    child_deadline=$((SECONDS + 5))
    while kill -0 -- "-$pid" 2>/dev/null && (( SECONDS < child_deadline )); do
      sleep 1
    done
  fi

  if kill -0 -- "-$pid" 2>/dev/null; then
    printf '[STOP] %-12s group did not exit after SIGTERM; sending SIGKILL\n' "$label" >&2
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local original_status=$?
  trap - EXIT INT TERM
  printf '\n[STOP] ordered stack shutdown\n'
  local i pid label
  for ((i=${#CHILD_PIDS[@]}-1; i>=0; i--)); do
    pid=${CHILD_PIDS[$i]}
    label=${CHILD_LABELS[$i]}
    stop_process_group "$label" "$pid"
    wait "$pid" 2>/dev/null || true
  done
  printf '[STOP] complete; logs=%s\n' "$RUN_DIR"
  exit "$original_status"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

existing_nodes=$(ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)
for owner in /robot_state_publisher /agt_localization_manager /agt_pointcloud_preprocessor \
    /agt_batch_lio_adapter /agt_fastlio_adapter /laserMapping /batch_lio /fastlio2/lio_node; do
  if grep -Fxq "$owner" <<<"$existing_nodes"; then
    printf 'Refusing to create duplicate owner; node already exists: %s\n' "$owner" >&2
    exit 1
  fi
done

# Preserve the exact LIO input for later A/B analysis; both backend paths use it.
cp -- "$LIO_CONFIG" "$RUN_DIR/lio_config_input.yaml"
{
  printf 'mode=%s\nlio_backend=%s\nlio_config_source=%s\n' "$MODE" "$LIO_BACKEND" "$LIO_CONFIG"
  printf 'lio_raw_odometry=%s\n' "$LIO_RAW_ODOM"
  printf 'rear_pointcloud_mask=%s\n' "$REAR_POINTCLOUD_MASK"
  if [[ "$REAR_POINTCLOUD_MASK" == true ]]; then
    printf 'rear_pointcloud_mask_sector=center:180deg,width:70deg,range:0.5-1.0m\n'
  fi
  sha256sum -- "$RUN_DIR/lio_config_input.yaml"
} >"$RUN_DIR/launch_selection.txt"
declare -a LIO_LAUNCH_ARGS=(
  "lio_backend:=$LIO_BACKEND"
  "$LIO_CONFIG_ARGUMENT:=$RUN_DIR/lio_config_input.yaml"
)

start_child hardware \
  ros2 launch agt_system_bringup hardware.launch.py \
  robot:="$ROBOT_PROFILE" \
  bunker_can_port:=can0 enable_rtk:="$ENABLE_RTK" \
  enable_camera_gimbal:="$ENABLE_INSPECTION"
wait_for_topic MID360 /livox/lidar 45 || { tail_failure hardware; exit 1; }
wait_for_topic MID360-IMU /livox/imu 30 || { tail_failure hardware; exit 1; }
wait_for_topic Bunker /wheel/odom 30 || { tail_failure hardware; exit 1; }

start_child localization \
  ros2 launch agt_system_bringup localization.launch.py \
  global_map:="$GLOBAL_MAP" \
  map_id:="$MAP_ID" map_version:="$MAP_VERSION" \
  relocalization_assets:="$RELOCALIZATION_ASSETS" \
  auto_relocalize:=false \
  "${LIO_LAUNCH_ARGS[@]}"
wait_for_service /agt/localization/relocalize 45 || { tail_failure localization; exit 1; }
wait_for_adapter 60 || { tail_failure localization; exit 1; }
if ! relocalize_until_ready relocalize; then
  tail_failure localization
  exit 1
fi

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
if [[ "$REAR_POINTCLOUD_MASK" == true ]]; then
  NAV_OBS_ARGS+=(
    "obstacle_rear_filter_enabled:=true"
    "obstacle_rear_filter_center_deg:=180.0"
    "obstacle_rear_filter_width_deg:=70.0"
    "obstacle_rear_filter_min_range_m:=0.5"
    "obstacle_rear_filter_max_range_m:=1.0"
  )
fi

navigation_ready=false
NAV_PACKAGE=agt_system_bringup
NAV_LAUNCH=navigation.launch.py
declare -a MISSION_ARGS=()
if [[ "$ENABLE_INSPECTION" == true ]]; then
  NAV_PACKAGE=agt_mission_bringup
  NAV_LAUNCH=mission.launch.py
  MISSION_ARGS+=("enable_legacy_inspection:=true")
fi
for attempt in 1 2; do
  start_child navigation \
    ros2 launch "$NAV_PACKAGE" "$NAV_LAUNCH" \
    map:="$MAP_SPEC" robot:="$ROBOT_PROFILE" map_registry:="$MAP_REGISTRY" \
    ${MISSION_ARGS[@]+"${MISSION_ARGS[@]}"} \
    ${NAV_OBS_ARGS[@]+"${NAV_OBS_ARGS[@]}"}
  if wait_for_service /navigate_to_pose/_action/send_goal 60; then
    navigation_ready=true
    break
  fi

  tail_failure navigation
  if (( attempt < 2 )); then
    navigation_index=$((${#CHILD_PIDS[@]} - 1))
    navigation_pid=${CHILD_PIDS[$navigation_index]}
    stop_process_group navigation "$navigation_pid"
    wait "$navigation_pid" 2>/dev/null || true
    mv -- "$RUN_DIR/navigation.log" "$RUN_DIR/navigation_attempt_${attempt}.log"
    printf '[RETRY] Nav2 lifecycle bringup did not complete; restarting navigation only\n'
    sleep 2
  fi
done
if [[ "$navigation_ready" != true ]]; then
  exit 1
fi

if [[ "$ENABLE_RVIZ" == true ]]; then
  rviz_xdg_data_dirs=${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}
  start_child debug \
    env -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE \
    -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX \
    -u GTK_IM_MODULE_FILE -u GTK_PATH -u LOCPATH -u SNAP \
    -u SNAP_LIBRARY_PATH -u XDG_DATA_HOME \
    XDG_DATA_DIRS="$rviz_xdg_data_dirs" \
    ros2 launch agt_system_bringup debug.launch.py \
    run_preflight:=false rviz_config:="$RVIZ_CONFIG"
  wait_for_node /agt_navigation_debug_rviz 30 || { tail_failure debug; exit 1; }
fi

"$SCRIPT_DIR/check_runtime.sh" --lio-backend "$LIO_BACKEND" | tee "$RUN_DIR/check_runtime.txt"
"$SCRIPT_DIR/check_topics.sh" | tee "$RUN_DIR/check_topics.txt"
ensure_localization_ready after_startup_checks || { tail_failure localization; exit 1; }
"$SCRIPT_DIR/check_tf.sh" | tee "$RUN_DIR/check_tf.txt"
# TF inspection itself must not leave a sticky LOST state immediately before
# the formal preflight and operator control are enabled.
ensure_localization_ready after_tf_check || { tail_failure localization; exit 1; }
ros2 run agt_navigation_runtime demo_preflight --ros-args \
  -p require_camera:="$ENABLE_INSPECTION" \
  | tee "$RUN_DIR/demo_preflight.txt"
ros2 run agt_navigation_supervisor wait_navigation_ready --timeout 45 --samples 3 \
  | tee "$RUN_DIR/navigation_health_gate.txt"

printf '\n[READY] Hardware, localization and Nav2 passed preflight.\n'
printf '[READY] Mode: %s\n' "$MODE"
printf '[READY] LIO backend: %s (raw odometry: %s)\n' "$LIO_BACKEND" "$LIO_RAW_ODOM"
if [[ "$REAR_POINTCLOUD_MASK" == true ]]; then
  printf '[READY] Rear pole mask: ON (180 +/- 35 deg, 0.5-1.0 m; obstacle marking only)\n'
else
  printf '[READY] Rear pole mask: OFF\n'
fi
if [[ "$ENABLE_INSPECTION" == true ]]; then
  printf '[READY] Inspection: queue RViz points, then call /agt/rviz_patrol/start.\n'
  printf '[READY] Records: ~/.ros/agt_inspection_records/\n'
else
  printf '[READY] Pure navigation: camera and inspection mission nodes are disabled.\n'
fi
printf '[READY] Paths: red=global plan, blue=LIO actual, green=wheel-relative.\n'
printf '[READY] Draw route with RViz Publish Point, then call /agt/path_tool/start.\n'
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
