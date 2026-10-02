#!/usr/bin/env bash
# Sourced by run_field_stack.sh after its lifecycle helpers. No execution here.

wait_for_manual_initialization() {
  local response
  wait_for_service /agt/relocalization/enter_manual 20 || return 1
  response=$(timeout 30 ros2 service call /agt/relocalization/enter_manual \
    std_srvs/srv/Trigger '{}' 2>&1) || { printf '%s\n' "$response" >&2; return 1; }
  printf '%s\n' "$response" > "$RUN_DIR/enter_manual.txt"
  if ! grep -Eq 'success=True|success: true' <<<"$response"; then
    # A late successful auto result must not be discarded to force fallback.
    wait_for_localization_settle 10 && return 0
    printf '[FAIL] could not arm manual initialization: %s\n' "$response" >&2
    return 1
  fi

  start_child initialization_view \
    env -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE \
    -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX \
    -u GTK_IM_MODULE_FILE -u GTK_PATH -u LOCPATH -u SNAP \
    -u SNAP_LIBRARY_PATH -u XDG_DATA_HOME \
    XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}" \
    ros2 launch agt_system_bringup initialization_view.launch.py \
    map:="$NAV_MAP" rviz:="$ENABLE_RVIZ"
  local view_index=$((${#CHILD_PIDS[@]} - 1))
  local view_pid=${CHILD_PIDS[$view_index]}
  # Subscribe explicitly with latched-map QoS, even if the server published
  # before this reader started. No Nav2 map server is running at this stage.
  if ! timeout 20 ros2 topic echo /agt/initialization/map nav_msgs/msg/OccupancyGrid \
      --qos-durability transient_local --qos-reliability reliable --once --field info.width \
      > "$RUN_DIR/initialization_map_width.txt" 2>&1; then
    tail_failure initialization_view
    return 1
  fi
  if [[ "$ENABLE_RVIZ" == true ]]; then
    wait_for_node /agt_initialization_rviz 30 || { tail_failure initialization_view; return 1; }
  fi
  printf '[WAIT_MANUAL_INITIAL_POSE] Nav2 NOT started. Keep stationary.\n'
  printf '[WAIT_MANUAL_INITIAL_POSE] Use RViz 2D Pose Estimate on /initialpose, frame=map.\n'
  if [[ "$ENABLE_RVIZ" != true ]]; then
    printf '[WAIT_MANUAL_INITIAL_POSE] No local RViz requested; use your remote RViz with map topic /agt/initialization/map.\n'
  fi
  printf '[WAIT_MANUAL_INITIAL_POSE] Rejected matches remain waiting; Ctrl+C shuts down this stack.\n'
  while true; do
    # If one of our staged processes has died, do not wait forever or start Nav2.
    local pid
    for pid in "${CHILD_PIDS[@]}"; do
      if ! kill -0 "$pid" 2>/dev/null; then
        printf '[FAIL] staged process exited during manual initialization\n' >&2
        return 1
      fi
    done
    if wait_for_localization_settle 8; then
      break
    fi
  done
  stop_process_group initialization_view "$view_pid"
  wait "$view_pid" 2>/dev/null || true
  # It was the last staged process, so removing it keeps arrays contiguous.
  unset 'CHILD_PIDS[view_index]' 'CHILD_LABELS[view_index]'
}

# The optional local seed is single-use, never an inferred grid/PCD origin.
# A successful service response means only "GICP pose proposed"; the manager
# must still accept global correction with fresh local odometry before Nav2.
try_reviewed_start_hint() {
  if [[ "$START_HINT_STATUS" != VALIDATED ]]; then
    printf '[NEAR_START] SKIPPED (no version/PCD-bound reviewed start hint)\n'
    return 1
  fi
  printf '[NEAR_START] Validated map hint; waiting for stationary, fresh scan\n'
  wait_for_service /agt/localization/relocalize_near_start 20 || return 2
  local attempt response
  for attempt in $(seq 1 25); do
    response=$(timeout 30 ros2 service call /agt/localization/relocalize_near_start \
      std_srvs/srv/Trigger '{}' 2>&1) || {
      printf '%s\n' "$response" > "$RUN_DIR/near_start_${attempt}.txt"
      printf '[NEAR_START] service failed; falling back to full-map candidates\n' >&2
      return 1
    }
    printf '%s\n' "$response" > "$RUN_DIR/near_start_${attempt}.txt"
    if grep -Eq 'success=True|success: true' <<<"$response"; then
      if wait_for_localization_settle 30; then
        printf '[NEAR_START] manager accepted; Nav2 may start after further checks\n'
        return 0
      fi
      printf '[NEAR_START] proposed pose not accepted; trying global search\n' >&2
      return 1
    fi
    if grep -q 'CONFIG_ERROR:' <<<"$response"; then
      printf '[NEAR_START] hint or active map changed; aborting startup instead of falling back\n' >&2
      return 2
    fi
    if grep -q 'QUERY_NOT_READY:' <<<"$response"; then
      sleep 1
      continue
    fi
    printf '[NEAR_START] local GICP rejected; trying global search\n' >&2
    return 1
  done
  printf '[NEAR_START] no stationary fresh scan; trying global search\n' >&2
  return 1
}

initialize_localization() {
  case "$LOCALIZATION_MODE" in
    auto)
      relocalize_until_ready relocalize
      ;;
    auto_then_manual)
      local near_result=0
      try_reviewed_start_hint || near_result=$?
      if (( near_result == 0 )); then
        return 0
      fi
      if (( near_result == 2 )); then
        return 2
      fi
      if relocalize_until_ready relocalize; then
        return 0
      fi
      printf '[FALLBACK] all automatic localization failed; entering manual initialization\n'
      wait_for_manual_initialization
      ;;
    manual)
      wait_for_manual_initialization
      ;;
    *)
      printf 'Invalid localization mode: %s\n' "$LOCALIZATION_MODE" >&2
      return 2
      ;;
  esac
}
