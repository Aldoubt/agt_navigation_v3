#!/usr/bin/env bash
set -euo pipefail

LIO_BACKEND=""
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != --lio-backend ]]; then
    printf 'Usage: check_runtime.sh [--lio-backend batch_lio|fastlio2]\n' >&2
    exit 2
  fi
  LIO_BACKEND=$2
  case "$LIO_BACKEND" in
    batch_lio|fastlio2) ;;
    *) printf 'Invalid LIO backend: %s\n' "$LIO_BACKEND" >&2; exit 2 ;;
  esac
fi

EXPECTED_NODES=(
  /agt_localization_manager
  /robot_state_publisher
  /agt_pointcloud_preprocessor
)

FORBIDDEN_NODES=(/agt_obstacle_cloud_preprocessor /obstacle_cloud)
if [[ "$LIO_BACKEND" == batch_lio ]]; then
  EXPECTED_NODES+=(/agt_batch_lio_adapter /laserMapping)
  FORBIDDEN_NODES+=(/agt_fastlio_adapter /fastlio2/lio_node /batch_lio)
elif [[ "$LIO_BACKEND" == fastlio2 ]]; then
  EXPECTED_NODES+=(/agt_fastlio_adapter /fastlio2/lio_node)
  FORBIDDEN_NODES+=(/agt_batch_lio_adapter /laserMapping /batch_lio)
fi

# Query DDS directly. Discovery may return a partial graph for several seconds
# while Nav2 and RViz start. Require two consecutive complete snapshots before
# judging ownership, so one transient snapshot cannot shut down a healthy stack.
NODES=()
query_error=""
stable=0
node_count() {
  local expected=$1 count=0 node
  for node in "${NODES[@]}"; do
    [[ "$node" == "$expected" ]] && count=$((count + 1))
  done
  printf '%s\n' "$count"
}

printf '[WAIT]  runtime owners in ROS graph\n'
for attempt in 1 2 3 4 5; do
  if graph=$(ros2 node list --no-daemon --spin-time 2 2>&1); then
    mapfile -t NODES < <(printf '%s\n' "$graph" | sed '/^[[:space:]]*$/d')
    query_error=""
  else
    NODES=()
    query_error=$graph
  fi

  complete=true
  for expected in "${EXPECTED_NODES[@]}"; do
    [[ $(node_count "$expected") == 1 ]] || complete=false
  done
  forbidden_seen=false
  for retired in "${FORBIDDEN_NODES[@]}"; do
    if [[ $(node_count "$retired") != 0 ]]; then
      forbidden_seen=true
      complete=false
    fi
  done
  # A seen duplicate owner is a real safety violation; retry only missing owners.
  [[ "$forbidden_seen" == true ]] && break
  if [[ "$complete" == true && ${#NODES[@]} -gt 0 ]]; then
    stable=$((stable + 1))
    (( stable >= 2 )) && break
  else
    stable=0
  fi
  (( attempt < 5 )) && sleep 1
done
if [[ ${#NODES[@]} -eq 0 ]]; then
  printf 'FAIL ROS graph query: %s\n' "${query_error:-node list returned an empty graph}" >&2
  exit 1
fi
failed=0
if (( stable < 2 )); then
  printf 'FAIL runtime owners were not observed in two consecutive ROS graph snapshots\n' >&2
  failed=1
fi

for expected in "${EXPECTED_NODES[@]}"; do
  count=$(node_count "$expected")
  if [[ "$count" -eq 1 ]]; then
    printf 'PASS node %-34s count=1\n' "$expected"
  else
    printf 'FAIL node %-34s count=%s (expected 1)\n' "$expected" "$count" >&2
    failed=1
  fi
done

for retired in "${FORBIDDEN_NODES[@]}"; do
  count=$(node_count "$retired")
  if [[ "$count" -ne 0 ]]; then
    printf 'FAIL retired or unselected backend node still running: %s count=%s\n' "$retired" "$count" >&2
    failed=1
  fi
done

if [[ "$failed" -ne 0 ]]; then
  printf '\nCurrent ROS graph:\n' >&2
  printf '%s\n' "${NODES[@]}" >&2
  exit 1
fi

printf 'PASS runtime owner uniqueness\n'
