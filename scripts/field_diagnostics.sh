#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${REPO_ROOT}/config/field_hardware.env"

PASS=0
WARN=0
FAIL=0

ok()   { echo "PASS  $*"; PASS=$((PASS+1)); }
warn() { echo "WARN  $*"; WARN=$((WARN+1)); }
fail() { echo "FAIL  $*"; FAIL=$((FAIL+1)); }

have() { command -v "$1" >/dev/null 2>&1; }

echo "=== AGT field diagnostics ==="
echo "MID360=${AGT_MID360_IP}  CAN=${AGT_CAN_IFACE}@${AGT_CAN_BITRATE}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  ok "ROS 2 Humble found"
else
  fail "ROS 2 Humble not found at /opt/ros/humble/setup.bash"
fi

# Best-effort source the closest workspace install when invoked from <ws>/src/repo.
WS_ROOT="$(realpath -m "${REPO_ROOT}/../..")"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WS_ROOT}/install/setup.bash"
  ok "workspace overlay sourced: ${WS_ROOT}/install/setup.bash"
else
  warn "workspace overlay not found at ${WS_ROOT}/install/setup.bash"
fi

echo
echo "--- MID360 network ---"
if have ip; then
  route_out="$(ip route get "${AGT_MID360_IP}" 2>&1 || true)"
  if [[ -n "${route_out}" && "${route_out}" != *"unreachable"* ]]; then
    ok "route to MID360: ${route_out}"
  else
    fail "no route to MID360 ${AGT_MID360_IP}"
  fi
else
  fail "iproute2 command 'ip' is missing"
fi

if have ping; then
  if ping -c 1 -W 1 "${AGT_MID360_IP}" >/dev/null 2>&1; then
    ok "MID360 replies to ping at ${AGT_MID360_IP}"
  else
    fail "MID360 ping failed at ${AGT_MID360_IP}; fix Ethernet/subnet before ROS debugging"
  fi
else
  warn "ping command missing"
fi

echo
echo "--- Bunker SocketCAN ---"
if ip link show "${AGT_CAN_IFACE}" >/dev/null 2>&1; then
  details="$(ip -details link show "${AGT_CAN_IFACE}" 2>&1 || true)"
  echo "${details}"
  if grep -q "bitrate ${AGT_CAN_BITRATE}" <<<"${details}"; then
    ok "${AGT_CAN_IFACE} bitrate=${AGT_CAN_BITRATE}"
  else
    fail "${AGT_CAN_IFACE} is not configured for bitrate ${AGT_CAN_BITRATE}"
  fi
  if grep -Eq '<[^>]*UP[^>]*>' <<<"${details}"; then
    ok "${AGT_CAN_IFACE} is UP"
  else
    fail "${AGT_CAN_IFACE} exists but is not UP"
  fi
else
  fail "CAN interface ${AGT_CAN_IFACE} does not exist"
fi

if have systemctl; then
  if systemctl is-active --quiet agt-bunker-can.service 2>/dev/null; then
    ok "agt-bunker-can.service active"
  else
    warn "agt-bunker-can.service not active; install with scripts/install_bunker_can_service.sh"
  fi
fi

if have candump; then
  warn "CAN traffic is not consumed automatically by diagnostics; use 'candump ${AGT_CAN_IFACE}' while chassis is powered to inspect frames"
else
  warn "can-utils/candump missing"
fi

echo
echo "--- ROS graph ---"
if ! have ros2; then
  fail "ros2 command unavailable"
else
  topic_list="$(ros2 topic list -t 2>/dev/null || true)"
  action_list="$(ros2 action list -t 2>/dev/null || true)"
  service_list="$(ros2 service list -t 2>/dev/null || true)"

  check_topic() {
    local topic="$1"
    local required="$2"
    if grep -qE "^${topic//\//\/}([[:space:]]|$)" <<<"${topic_list}"; then
      ok "topic present: ${topic}"
    elif [[ "${required}" == "required" ]]; then
      fail "topic missing: ${topic}"
    else
      warn "topic missing: ${topic}"
    fi
  }
  check_action() {
    local action="$1"
    local required="$2"
    if grep -qE "^${action//\//\/}([[:space:]]|$)" <<<"${action_list}"; then
      ok "action present: ${action}"
    elif [[ "${required}" == "required" ]]; then
      fail "action missing: ${action}"
    else
      warn "action missing: ${action}"
    fi
  }
  check_service() {
    local service="$1"
    local required="$2"
    if grep -qE "^${service//\//\/}([[:space:]]|$)" <<<"${service_list}"; then
      ok "service present: ${service}"
    elif [[ "${required}" == "required" ]]; then
      fail "service missing: ${service}"
    else
      warn "service missing: ${service}"
    fi
  }

  check_topic /livox/lidar optional
  check_topic /livox/imu optional
  check_topic /agt/livox/points optional
  check_topic /agt/odometry/local optional
  check_topic /wheel/odom optional
  check_topic /agt/navigation/points_obstacles optional
  check_topic /map optional
  check_topic /plan optional
  check_topic /global_costmap/costmap optional
  check_topic /local_costmap/costmap optional
  check_topic /ins/navsatfix optional
  check_topic /ins/status optional

  check_action /navigate_to_pose optional
  check_action /camera_gimbal/acquire_view optional
  check_service /agt/localization/relocalize optional
  check_service /agt/rviz_patrol/start optional

  if have timeout && ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
    tf_out="$(timeout 2s ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if grep -q "Translation:" <<<"${tf_out}"; then
      ok "TF available: map -> base_link"
    else
      warn "TF not currently available: map -> base_link (expected before navigation, not necessarily before relocalization)"
    fi
  fi
fi

echo
echo "--- Summary ---"
echo "PASS=${PASS} WARN=${WARN} FAIL=${FAIL}"
if [[ "${FAIL}" -ne 0 ]]; then
  echo "FIELD DIAGNOSTICS: FAIL"
  exit 2
fi
echo "FIELD DIAGNOSTICS: PASS/WARN"
