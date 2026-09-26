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
ROBOT_PROFILE_EXPLICIT=""
ROBOT_CONFIG=""
MAP_REGISTRY=${AGT_MAP_REGISTRY:-"$WS_ROOT/maps/registry.yaml"}
MAP_ROOT_OVERRIDE=""
MAP_ID_OVERRIDE=""
MODE=navigation
LOCALIZATION_MODE=auto
LIO_BACKEND=fastlio2
LIO_CONFIG=""
YHS_LIVOX_CONFIG=""
YHS_NAV_CONFIG_DIR=""
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
LOCAL_OBSTACLE_AVOIDANCE=true
RVIZ_CONFIG=""

usage() {
  cat <<'EOF'
Usage: scripts/run_field_stack.sh [options]

Options:
  --mode MODE       Runtime mode: navigation (default) or inspection.
                    navigation: Nav2 only, camera and capture task disabled.
                    inspection: stop at each queued point, capture three views,
                    save images and metadata, then continue/return home.
  --lio-backend NAME Local odometry: fastlio2 (default) or batch_lio (explicit opt-in); mutually exclusive.
  --lio-config PATH  Optional runtime YAML for the selected LIO backend.
  --localization-mode MODE
                    auto (default): two automatic attempts, then exit on failure.
                    auto_then_manual: two attempts, then wait for RViz /initialpose.
                    manual: skip global search, wait for operator seed + local GICP.
                    Waiting keeps sensors/LIO alive but never starts Nav2.
                    Add --rviz for the map-only initialization window.
  --map SPEC        auto (default), active, latest, map_id or map_id/version.
  --robot PROFILE   Legacy robot profile (default: bunker_v1 -> robot config
                    bunker_inspection). Must match --robot-config if both are given.
  --robot-config ID|PATH
                    Whole-robot combination from agt_robot_bringup/config/robots
                    (bunker_inspection; yhs_harvesting is BLOCKED and refuses to start).
                    Stop any running stack before switching robots (no hot switch).
  --yhs-livox-config PATH
                    Required when yhs_harvesting is eventually field-validated:
                    independently generated YHS MID360 IP JSON; never Bunker default.
                    Requires devices.yaml navigation_lidar.driver_mode=mapping_custom.
  --yhs-nav-config-dir PATH
                    Required for YHS: reviewed YHS-only robot footprint, Nav2
                    parameters and safety limits; never Bunker defaults.
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
  --no-local-obstacle-avoidance
                    Trial mode: local costmap uses the static map and inflation,
                    without live LiDAR obstacle marking or clearing.
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
    --localization-mode)
      LOCALIZATION_MODE=${2:?--localization-mode requires auto, auto_then_manual or manual}
      shift 2
      ;;
    --lio-config)
      LIO_CONFIG=${2:?--lio-config requires a path}
      shift 2
      ;;
    --yhs-livox-config)
      YHS_LIVOX_CONFIG=${2:?--yhs-livox-config requires a verified YHS JSON path}
      shift 2
      ;;
    --yhs-nav-config-dir)
      YHS_NAV_CONFIG_DIR=${2:?--yhs-nav-config-dir requires a measured YHS Nav2 config directory}
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
      ROBOT_PROFILE_EXPLICIT=$ROBOT_PROFILE
      shift 2
      ;;
    --robot-config)
      ROBOT_CONFIG=${2:?--robot-config requires an id or robot.yaml path}
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
    --no-local-obstacle-avoidance)
      LOCAL_OBSTACLE_AVOIDANCE=false
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
if [[ "$ENABLE_INSPECTION" == true && "$LOCAL_OBSTACLE_AVOIDANCE" == false ]]; then
  printf '%s\n' '--no-local-obstacle-avoidance is supported only in navigation mode' >&2
  exit 2
fi

case "$LOCALIZATION_MODE" in
  auto|auto_then_manual|manual) ;;
  *) printf 'Invalid --localization-mode: %s\n' "$LOCALIZATION_MODE" >&2; exit 2 ;;
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
# Whole-robot combination (agt_robot_bringup/config/robots). Fails closed: a
# blocked or conflicting config never falls back to another robot.
ROBOT_CONFIG_RESOLVER="$WS_ROOT/install/agt_robot_bringup/lib/agt_robot_bringup/robot_config.py"
if [[ ! -f "$ROBOT_CONFIG_RESOLVER" ]]; then
  printf 'Missing robot config resolver: %s\n' "$ROBOT_CONFIG_RESOLVER" >&2
  printf 'Rebuild: colcon build --packages-select agt_robot_bringup --symlink-install\n' >&2
  exit 2
fi
# Device policy (single place):
#  * The whole-robot YAML decides which devices start.
#  * --mode inspection REQUIRES the camera: forced on, rejected if not installed.
#  * --mode navigation with an explicit --robot-config: YAML decides the camera.
#  * --mode navigation without --robot-config (legacy commands): camera off,
#    exactly as before the robot config existed.
#  * --rtk forces RTK on (metadata only); otherwise YAML decides.
# The same override set is used for the preflight resolution and the launch.
declare -a HW_OVERRIDES=()
declare -a HW_LAUNCH_ARGS=()
if [[ "$ENABLE_INSPECTION" == true ]]; then
  HW_OVERRIDES+=(--set enable_camera_gimbal=true)
  HW_LAUNCH_ARGS+=(enable_camera_gimbal:=true)
  CAMERA_POLICY=inspection_requires_camera
elif [[ -z "$ROBOT_CONFIG" ]]; then
  HW_OVERRIDES+=(--set enable_camera_gimbal=false)
  HW_LAUNCH_ARGS+=(enable_camera_gimbal:=false)
  CAMERA_POLICY=legacy_navigation_camera_off
else
  CAMERA_POLICY=robot_config
fi
if [[ "$ENABLE_RTK" == true ]]; then
  HW_OVERRIDES+=(--set enable_rtk=true)
  HW_LAUNCH_ARGS+=(enable_rtk:=true)
fi
if ! ROBOT_CONFIG_SELECTION=$(python3 "$ROBOT_CONFIG_RESOLVER" resolve \
    --robot-config "$ROBOT_CONFIG" --robot "$ROBOT_PROFILE_EXPLICIT" "${HW_OVERRIDES[@]}"); then
  printf 'Robot config rejected; nothing was started. Resolve the error above; stop any running stack before switching robots.\n' >&2
  exit 2
fi
while IFS='=' read -r key value; do
  case "$key" in
    robot_config) ROBOT_CONFIG_ID=$value ;;
    robot_config_dir) ROBOT_CONFIG_DIR=$value ;;
    enable_camera_gimbal) RESOLVED_CAMERA=$value ;;
    enable_rtk) RESOLVED_RTK=$value ;;
    robot_profile) ROBOT_PROFILE=$value ;;
    base_adapter) ROBOT_BASE_ADAPTER=$value ;;
    mid360_driver_mode) RESOLVED_MID360_DRIVER_MODE=$value ;;
    reserved_payloads) ROBOT_RESERVED_PAYLOADS=$value ;;
    payload_interlock) ROBOT_PAYLOAD_INTERLOCK=$value ;;
  esac
done <<< "$ROBOT_CONFIG_SELECTION"

# Navigation must use the same independently measured YHS MID360 network
# configuration as YHS mapping. Never silently inherit the vendor/Bunker JSON.
# The whole-robot resolver above still blocks YHS until geometry, gear and
# other field checks are complete; this flag is NOT an unblock override.
if [[ "$ROBOT_CONFIG_ID" == yhs_harvesting ]]; then
  if [[ -z "$YHS_LIVOX_CONFIG" ]]; then
    printf 'YHS navigation requires --yhs-livox-config with the verified YHS MID360 IP JSON\n' >&2
    exit 2
  fi
  if [[ "${RESOLVED_MID360_DRIVER_MODE:-}" != mapping_custom ]]; then
    printf 'YHS devices.yaml must select navigation_lidar.driver_mode=mapping_custom; refusing vendor/Bunker default\n' >&2
    exit 2
  fi
  if [[ ! -f "$YHS_LIVOX_CONFIG" ]]; then
    printf 'YHS MID360 JSON missing: %s\n' "$YHS_LIVOX_CONFIG" >&2
    exit 2
  fi
  YHS_LIVOX_CONFIG=$(realpath -- "$YHS_LIVOX_CONFIG")
  if ! python3 - "$YHS_LIVOX_CONFIG" <<'PY_YHS_LIVOX'
import ipaddress
import json
import sys
with open(sys.argv[1], encoding='utf-8') as stream:
    config = json.load(stream)
net = config['MID360']['host_net_info']
keys = ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')
hosts = {str(ipaddress.IPv4Address(net[key])) for key in keys}
if len(hosts) != 1:
    raise ValueError('YHS MID360 JSON host IPs differ across streams')
lidars = config['lidar_configs']
if not isinstance(lidars, list) or len(lidars) != 1:
    raise ValueError('YHS MID360 JSON requires exactly one LiDAR IP')
lidar = str(ipaddress.IPv4Address(lidars[0]['ip']))
if lidar in hosts:
    raise ValueError('YHS MID360 JSON must use different host and LiDAR IPs')
PY_YHS_LIVOX
  then
    printf 'Invalid YHS MID360 JSON; regenerate with measured host/lidar IPs\n' >&2
    exit 2
  fi
  if [[ -z "$YHS_NAV_CONFIG_DIR" || ! -d "$YHS_NAV_CONFIG_DIR" ]]; then
    printf 'YHS navigation requires --yhs-nav-config-dir with measured and reviewed Nav2 YAML\n' >&2
    exit 2
  fi
  YHS_NAV_CONFIG_DIR=$(realpath -- "$YHS_NAV_CONFIG_DIR")
  if [[ "$YHS_NAV_CONFIG_DIR" == "$WS_ROOT/install/agt_system_bringup/share/agt_system_bringup/config" ||
        "$YHS_NAV_CONFIG_DIR" == "$WS_ROOT/src/agt_navigation_v3/config" ]]; then
    printf 'YHS Nav2 config must be dedicated; refusing canonical Bunker directory\n' >&2
    exit 2
  fi
  if ! python3 - "$YHS_NAV_CONFIG_DIR" <<'PY_YHS_NAV'
import pathlib
import sys
import yaml
root = pathlib.Path(sys.argv[1])
marker = yaml.safe_load((root / 'field_profile.yaml').read_text(encoding='utf-8'))
if (not isinstance(marker, dict) or marker.get('robot_profile') != 'yhs_v1' or
        marker.get('field_verified') is not True or not marker.get('verified_by')):
    raise ValueError('YHS field_profile.yaml must attest reviewed YHS measurements')
required = ('robot.yaml', 'navigation.yaml', 'controller.yaml', 'costmap.yaml',
            'perception.yaml', 'safety.yaml')
missing = [name for name in required if not (root / name).is_file()]
if missing:
    raise ValueError(f'YHS Nav2 config missing: {missing}')
PY_YHS_NAV
  then
    printf 'Invalid YHS Nav2 config directory; no hardware was started\n' >&2
    exit 2
  fi
  if [[ "$MAP_SPEC" != */* ]]; then
    printf 'YHS navigation requires explicit --map map_id/version; refusing auto, active or Bunker default\n' >&2
    exit 2
  fi
  HW_LAUNCH_ARGS+=("mapping_livox_config:=$YHS_LIVOX_CONFIG")
elif [[ -n "$YHS_LIVOX_CONFIG" || -n "$YHS_NAV_CONFIG_DIR" ]]; then
  printf '%s\n' 'YHS-specific network/Nav2 flags are only valid for robot_config yhs_harvesting' >&2
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
  printf 'localization_mode=%s\n' "$LOCALIZATION_MODE"
  printf 'lio_backend=%s\n' "$LIO_BACKEND"
  printf 'lio_config=%s\n' "$LIO_CONFIG"
  if [[ "$ROBOT_CONFIG_ID" == yhs_harvesting ]]; then
    printf 'yhs_livox_config=%s\nyhs_nav_config_dir=%s\n' "$YHS_LIVOX_CONFIG" "$YHS_NAV_CONFIG_DIR"
  fi
  printf 'lio_raw_odometry=%s\n' "$LIO_RAW_ODOM"
  printf 'initialization_mode=%s\n' "$LOCALIZATION_MODE"
  printf 'rear_pointcloud_mask=%s\n' "$REAR_POINTCLOUD_MASK"
  printf 'local_obstacle_avoidance=%s\n' "$LOCAL_OBSTACLE_AVOIDANCE"
  if [[ "$REAR_POINTCLOUD_MASK" == true ]]; then
    printf 'rear_pointcloud_mask_sector=center:180deg,width:70deg,range:0.5-1.0m\n'
  fi
  printf 'map_root=%s\n' "$MAP_ROOT"
  printf 'map_registry=%s\n' "$MAP_REGISTRY"
  printf 'map_spec=%s\nrobot_profile=%s\n' "$MAP_SPEC" "$ROBOT_PROFILE"
  printf 'robot_config=%s\nbase_adapter=%s\nreserved_payloads=%s\n' \
    "$ROBOT_CONFIG_ID" "$ROBOT_BASE_ADAPTER" "$ROBOT_RESERVED_PAYLOADS"
  printf '%s\n' "$ROBOT_CONFIG_SELECTION" | sed -n 's/^\(enable_[a-z0-9_]*=\)/hardware_\1/p'
  printf 'map_id=%s\n' "$MAP_ID"
  printf 'map_version=%s\n' "$MAP_VERSION"
  printf 'navigation_map=%s\n' "$NAV_MAP"
  printf 'localization_map=%s\n' "$GLOBAL_MAP"
  printf 'relocalization_assets=%s\n' "$RELOCALIZATION_ASSETS"
  printf 'camera_gimbal=%s\n' "$RESOLVED_CAMERA"
  printf 'camera_policy=%s\nrtk=%s\nrobot_config_dir=%s\n' "$CAMERA_POLICY" "$RESOLVED_RTK" "$ROBOT_CONFIG_DIR"
  printf 'hardware_launch_args=%s\n' "${HW_LAUNCH_ARGS[*]}"
  printf 'inspection_runtime=%s\n' "$ENABLE_INSPECTION"
  printf 'payload_interlock=%s\n' "${ROBOT_PAYLOAD_INTERLOCK:-false}"
  printf 'rviz_config=%s\n' "$RVIZ_CONFIG"
  exit 0
fi

# After moving the hardware package to agt_robot_platform, an old symlink in
# install/ can still make the package discoverable while its launch file is gone.
ROBOT_HARDWARE_LAUNCH="$WS_ROOT/install/agt_robot_bringup/share/agt_robot_bringup/launch/robot_hardware.launch.py"
if [[ ! -f "$ROBOT_HARDWARE_LAUNCH" ]]; then
  printf 'Missing robot hardware launch: %s\n' "$ROBOT_HARDWARE_LAUNCH" >&2
  printf 'Rebuild the moved package: cd %s && source /opt/ros/humble/setup.bash && colcon build --packages-select agt_robot_bringup --symlink-install --cmake-clean-cache\n' "$WS_ROOT" >&2
  exit 2
fi

mkdir -p -- "$RUN_DIR"
# Launch exactly the robot config that passed preflight: snapshot the resolved
# directory (not the ID, which could resolve elsewhere) into this run's log dir.
ROBOT_CONFIG_SNAPSHOT="$RUN_DIR/robot_config/$ROBOT_CONFIG_ID"
mkdir -p -- "$RUN_DIR/robot_config"
cp -a -- "$ROBOT_CONFIG_DIR" "$ROBOT_CONFIG_SNAPSHOT"
printf '%s\n' "$ROBOT_CONFIG_SELECTION" > "$RUN_DIR/robot_config/resolved.txt"
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
  local owner=${4:-}
  local deadline=$((SECONDS + timeout_sec))
  local i
  printf '[WAIT]  %-12s topic=%s\n' "$label" "$topic"
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      printf '[PASS]  %-12s topic=%s\n' "$label" "$topic"
      return 0
    fi
    for ((i=0; i<${#CHILD_LABELS[@]}; i++)); do
      if [[ "${CHILD_LABELS[$i]}" == "$owner" ]] \
          && ! kill -0 -- "-${CHILD_PIDS[$i]}" 2>/dev/null; then
        printf '[FAIL]  %-12s %s launch exited before %s became ready\n' \
          "$label" "$owner" "$topic" >&2
        return 1
      fi
    done
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
    # ros2 topic echo YAML-quotes reasons containing ': ', as backend errors do.
    if grep -Eq "^reason: ['\"]?global_relocalization_(failed|rejected):" <<<"$output"; then
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

source "$SCRIPT_DIR/localization_initialization.sh"

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
  printf 'initialization_mode=%s\n' "$LOCALIZATION_MODE"
  printf 'rear_pointcloud_mask=%s\n' "$REAR_POINTCLOUD_MASK"
  printf 'local_obstacle_avoidance=%s\n' "$LOCAL_OBSTACLE_AVOIDANCE"
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
  robot_config:="$ROBOT_CONFIG_SNAPSHOT" robot:="$ROBOT_PROFILE" \
  "${HW_LAUNCH_ARGS[@]}"
wait_for_topic MID360 /livox/lidar 45 hardware || { tail_failure hardware; exit 1; }
wait_for_topic MID360-IMU /livox/imu 30 hardware || { tail_failure hardware; exit 1; }
# Wheel odometry is recorded for diagnostics; LIO is the navigation authority.

start_child localization \
  ros2 launch agt_system_bringup localization.launch.py \
  global_map:="$GLOBAL_MAP" \
  map_id:="$MAP_ID" map_version:="$MAP_VERSION" \
  relocalization_assets:="$RELOCALIZATION_ASSETS" \
  query_capture_dir:="$RUN_DIR/relocalization_queries" \
  auto_relocalize:=false localization_mode:="$LOCALIZATION_MODE" \
  "${LIO_LAUNCH_ARGS[@]}"
wait_for_service /agt/localization/relocalize 45 || { tail_failure localization; exit 1; }
wait_for_adapter 60 || { tail_failure localization; exit 1; }
if ! initialize_localization; then
  tail_failure localization
  exit 1
fi

declare -a NAV_OBS_ARGS=()
NAV_OBS_ARGS+=("local_obstacle_avoidance:=$LOCAL_OBSTACLE_AVOIDANCE")
if [[ "$ROBOT_CONFIG_ID" == yhs_harvesting ]]; then
  NAV_OBS_ARGS+=("nav_config_dir:=$YHS_NAV_CONFIG_DIR")
fi
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
    robot_config:="$ROBOT_CONFIG_SNAPSHOT" payload_interlock:="${ROBOT_PAYLOAD_INTERLOCK:-auto}" \
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
printf '[READY] Mode: %s; initialization: %s\n' "$MODE" "$LOCALIZATION_MODE"
printf '[READY] LIO backend: %s (raw odometry: %s)\n' "$LIO_BACKEND" "$LIO_RAW_ODOM"
if [[ "$LOCAL_OBSTACLE_AVOIDANCE" == false ]]; then
  printf '[READY] Local costmap: static map + inflation; live LiDAR obstacles OFF\n'
fi
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
