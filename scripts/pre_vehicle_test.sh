#!/usr/bin/env bash
set -u
PASS=0; FAIL=0; WARN=0
ok(){ echo "PASS: $*"; PASS=$((PASS+1)); }
fail(){ echo "FAIL: $*"; FAIL=$((FAIL+1)); }
warn(){ echo "WARN: $*"; WARN=$((WARN+1)); }
have(){ command -v "$1" >/dev/null 2>&1; }

echo 'AGT Navigation V3 pre-vehicle test'
if have ip; then
  ip route get 192.168.1.117 >/dev/null 2>&1 && ok 'MID360 route 192.168.1.117' || fail 'MID360 route 192.168.1.117'
else fail 'ip command unavailable'; fi
if have ping; then ping -c 1 -W 1 192.168.1.117 >/dev/null 2>&1 && ok 'MID360 ping' || fail 'MID360 ping'; else warn 'ping unavailable'; fi

if have ip && ip link show can0 >/dev/null 2>&1; then
  details="$(ip -details link show can0 2>/dev/null)"
  grep -Eq 'bitrate 500000([^0-9]|$)' <<<"$details" && ok 'can0 bitrate 500000' || fail 'can0 bitrate is not 500000'
  grep -Eq '<[^>]*UP[^>]*>' <<<"$details" && ok 'can0 UP' || fail 'can0 is not UP'
else fail 'can0 missing'; fi

if ! have ros2; then fail 'ros2 unavailable'; else
  topics="$(ros2 topic list 2>/dev/null)"
  # Topics follow agt_robot_bringup/config/robot_topics.yaml and the FAST-LIO2 adapter output.
  for t in /livox/lidar /livox/imu /agt/odometry/local /agt/localization/status; do
    grep -Fxq "$t" <<<"$topics" && ok "topic $t" || fail "topic $t missing"
  done
  actions="$(ros2 action list 2>/dev/null)"
  grep -Eq '^/navigate_to_pose([[:space:]]|$)' <<<"$actions" && ok 'action NavigateToPose' || fail 'action NavigateToPose missing'
  if grep -Eq '^/acquire_view([[:space:]]|$)|^/camera_gimbal/acquire_view([[:space:]]|$)' <<<"$actions"; then ok 'action AcquireView'; else fail 'action AcquireView missing'; fi
  if have timeout && ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
    tf="$(timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    grep -q 'Translation:' <<<"$tf" && ok 'TF map -> base_link' || fail 'TF map -> base_link missing'
    timeout 3 ros2 run tf2_ros tf2_echo odom base_link >/dev/null 2>&1 && ok 'TF odom -> base_link' || fail 'TF odom -> base_link missing'
  else warn 'TF check unavailable'; fi
fi
echo "SUMMARY PASS=$PASS WARN=$WARN FAIL=$FAIL"
[[ $FAIL -eq 0 ]] && exit 0 || exit 2
