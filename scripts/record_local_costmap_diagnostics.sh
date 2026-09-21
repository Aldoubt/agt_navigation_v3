#!/usr/bin/env bash
# Local-costmap forensics: scenario recorder.
#
# Purpose: answer "why is the local costmap dirty" from recorded evidence instead of a
# screenshot. The decisive quantity is the distribution of /local_costmap/costmap_raw cell
# values (0 free / 1-252 inflated / 253 inscribed / 254 lethal / 255 unknown) plus the cost
# carried inside /local_costmap/published_footprint.
#
# This script is read-only with respect to the robot: it starts `ros2 bag record` and waits
# for the operator. It publishes no goal and no velocity command, and it changes no
# navigation parameter. All motion is performed by the operator through the normal stack.
#
# Enable the observation switches in the terminal that starts the stack (defaults stay
# unchanged when the arguments are omitted):
#
#   OUT="$HOME/ros2_ws/experiments/diagnostics/local_costmap_$(date +%Y%m%d_%H%M%S)"
#   ros2 launch agt_system_bringup navigation.launch.py map:=<navigation/map.yaml> \
#     obstacle_statistics_output:="$OUT/filter_statistics.yaml" \
#     obstacle_debug_base_cloud_enabled:=true \
#     obstacle_debug_log_interval_sec:=1.0
#
# Usage:
#   bash src/agt_navigation_v3/scripts/record_local_costmap_diagnostics.sh --out DIR
#   bash src/agt_navigation_v3/scripts/record_local_costmap_diagnostics.sh --segments s1,s3 \
#     --non-interactive --segment-seconds 30
#   bash src/agt_navigation_v3/scripts/record_local_costmap_diagnostics.sh --list
#
# Then analyse the result with:
#   python3 src/agt_navigation_v3/tools/local_costmap_forensics.py --root DIR

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

OUT=""
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-$HOME/ros2_ws/install/setup.bash}"
SEGMENTS="s1,s2,s3,s4,s5"
INTERACTIVE=1
SEGMENT_SECONDS=25
SKIP_SETUPS=0

REQUIRED_TOPICS=(
  /local_costmap/costmap_raw
  /local_costmap/costmap
  /local_costmap/published_footprint
)

CANDIDATE_TOPICS=(
  /tf
  /tf_static
  /map
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/costmap_raw
  /local_costmap/published_footprint
  /global_costmap/costmap
  /agt/livox/points
  /agt/navigation/points_obstacles
  /agt/debug/points_obstacles_base
  /agt/odometry/local
  /wheel/odom
  /agt/localization/status
  /plan
  /local_plan
)

declare -A SEG_TITLE SEG_ACTION SEG_CUES

SEG_TITLE[s1]="起点狭窄处静止"
SEG_ACTION[s1]="把车停在起点通道内、方向摆正，人退到激光视野外，全程保持静止。"
SEG_CUES[s1]="3:开始静止观测"

SEG_TITLE[s2]="开阔处静止"
SEG_ACTION[s2]="把车开到起点前方约 10 m 的开阔处停稳，人退到视野外，保持静止。"
SEG_CUES[s2]="3:开始静止观测"

SEG_TITLE[s3]="后方有人（站 10 s 后离开）"
SEG_ACTION[s3]="车保持静止；按提示让人站到车后约 2 m 并保持不动，随后离开视野并继续录满。"
SEG_CUES[s3]="3:请人走到车后约 2 m 站立不动;13:请人离开车后视野（不要再靠近）"

SEG_TITLE[s4]="直线行进后停稳"
SEG_ACTION[s4]="以当前允许的安全速度直线行进不少于 15 m，然后停稳；全程不靠近墙体。"
SEG_CUES[s4]="3:开始直线行进;18:停车并保持静止"

SEG_TITLE[s5]="原地重定位（可选）"
SEG_ACTION[s5]="车保持静止，在操作端触发一次 /agt/localization/relocalize，等待状态收敛后结束。"
SEG_CUES[s5]="3:触发一次全局重定位"

usage() {
  sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

log()  { printf '[record-diagnostics] %s\n' "$*"; }
warn() { printf '[record-diagnostics][WARN] %s\n' "$*" >&2; }
die()  { printf '[record-diagnostics][ERROR] %s\n' "$*" >&2; exit 1; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    --out) OUT="${2:-}"; shift 2 ;;
    --segments) SEGMENTS="${2:-}"; shift 2 ;;
    --segment-seconds) SEGMENT_SECONDS="${2:-}"; shift 2 ;;
    --ros-setup) ROS_SETUP="${2:-}"; shift 2 ;;
    --ws-setup) WS_SETUP="${2:-}"; shift 2 ;;
    --non-interactive) INTERACTIVE=0; shift ;;
    --skip-setups) SKIP_SETUPS=1; shift ;;
    --list) printf '%s\n' "s1 s2 s3 s4 s5"; exit 0 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

if [ -z "$OUT" ]; then
  OUT="$HOME/ros2_ws/experiments/diagnostics/local_costmap_$(date +%Y%m%d_%H%M%S)"
fi
OUT="${OUT/#\~/$HOME}"

if [ "$SKIP_SETUPS" -eq 0 ]; then
  [ -f "$ROS_SETUP" ] || die "ROS setup not found: $ROS_SETUP (pass --ros-setup or --skip-setups)"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [ -f "$WS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
  else
    warn "workspace setup not found: $WS_SETUP (continuing with the ROS setup only)"
  fi
fi

command -v ros2 >/dev/null 2>&1 || die "ros2 is not available in this shell"

mkdir -p "$OUT"
AVAILABLE_SPACE_KB="$(df -Pk "$OUT" | awk 'NR==2 {print $4}')"
if [ "${AVAILABLE_SPACE_KB:-0}" -lt 2097152 ]; then
  warn "less than 2 GiB free under $OUT (available: $((AVAILABLE_SPACE_KB / 1024)) MiB)"
fi

log "collecting topic inventory"
TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"

topic_present() {
  printf '%s\n' "$TOPIC_LIST" | grep -Fxq -- "$1"
}

MISSING_REQUIRED=0
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! topic_present "$topic"; then
    warn "required topic is not present: $topic"
    MISSING_REQUIRED=1
  fi
done
if [ "$MISSING_REQUIRED" -ne 0 ]; then
  die "start the navigation stack (map + localization + navigation) before recording"
fi

RECORD_TOPICS=()
SKIPPED_TOPICS=()
for topic in "${CANDIDATE_TOPICS[@]}"; do
  if topic_present "$topic"; then
    RECORD_TOPICS+=("$topic")
  else
    SKIPPED_TOPICS+=("$topic")
  fi
done

log "recording ${#RECORD_TOPICS[@]} topics into $OUT"
if [ "${#SKIPPED_TOPICS[@]}" -gt 0 ]; then
  log "skipping topics that are not published: ${SKIPPED_TOPICS[*]}"
fi

GIT_HEAD="$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"

write_scenario_yaml() {
  local meta_json="$1"
  META_JSON="$meta_json" python3 - "$OUT" "$GIT_HEAD" <<'PY'
import json, os, sys
from pathlib import Path

out = Path(sys.argv[1])
payload = json.loads(os.environ["META_JSON"])
payload["git_head"] = sys.argv[2]
payload["out_dir"] = str(out)
target = out / "scenario.yaml"
lines = [
    "# Generated by scripts/record_local_costmap_diagnostics.sh",
    f"generated_at_epoch: {payload['generated_at_epoch']}",
    f"git_head: {payload['git_head']}",
    f"out_dir: {payload['out_dir']}",
    f"interactive: {str(payload['interactive']).lower()}",
    f"segment_seconds: {payload['segment_seconds']}",
    "topics_recorded:",
]
for topic in payload["topics_recorded"]:
    lines.append(f"  - {topic}")
lines.append("topics_skipped:")
for topic in payload["topics_skipped"]:
    lines.append(f"  - {topic}")
lines.append("scenarios:")
for scenario in payload["scenarios"]:
    lines.append(f"  - name: {scenario['name']}")
    lines.append(f"    title: \"{scenario['title']}\"")
    lines.append(f"    action: \"{scenario['action']}\"")
    lines.append(f"    bag_dir: {scenario['bag_dir']}")
    lines.append(f"    started_epoch: {scenario['started_epoch']}")
    lines.append(f"    ended_epoch: {scenario['ended_epoch']}")
    lines.append(f"    metadata_complete: {str(scenario['metadata_complete']).lower()}")
    lines.append("    cues:")
    for cue in scenario["cues"]:
        lines.append(f"      - epoch: {cue['epoch']}")
        lines.append(f"        note: \"{cue['note']}\"")
target.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[record-diagnostics] wrote {target}")
PY
}

run_segment() {
  local name="$1"
  local title="${SEG_TITLE[$name]}"
  local action="${SEG_ACTION[$name]}"
  local cues="${SEG_CUES[$name]:-}"
  local bag_dir="$OUT/${name}_${title// /_}"
  local cue_json="[]"
  local started ended metadata_ok="false"

  printf '\n=== %s: %s ===\n%s\n' "$name" "$title" "$action"

  if [ "$INTERACTIVE" -eq 1 ]; then
    read -r -p "准备好后按 Enter 开始录制（该段最长 ${SEGMENT_SECONDS} s，可随时按 Enter 提前结束）... " _ || true
  else
    log "non-interactive mode: recording ${SEGMENT_SECONDS} s"
  fi

  started="$(date +%s.%N)"
  ros2 bag record --output "$bag_dir" --storage sqlite3 "${RECORD_TOPICS[@]}" \
    > "$OUT/${name}_record.log" 2>&1 &
  local record_pid=$!

  local cue_entries="[]"
  if [ -n "$cues" ]; then
    cue_entries="$(
      printf '%s' "$cues" | tr ';' '\n' | while IFS=: read -r at note; do
        [ -n "$at" ] || continue
        (
          sleep "$at"
          printf '\n[CUE %s] %s\n' "$at" "$note"
        ) &
        printf '%s\n' "$(date +%s.%N)	$note"
      done
    )"
  fi

  if [ "$INTERACTIVE" -eq 1 ]; then
    read -r -p "录制中。按 Enter 结束该段... " _ || true
  else
    sleep "$SEGMENT_SECONDS"
  fi

  kill -INT "$record_pid" 2>/dev/null || true
  wait "$record_pid" 2>/dev/null || true
  ended="$(date +%s.%N)"

  if [ -f "$bag_dir/metadata.yaml" ]; then
    metadata_ok="true"
  else
    warn "$name bag has no metadata.yaml; treat it as incomplete evidence"
  fi
  log "$name done: $bag_dir"

  CUE_ENTRIES="$cue_entries" python3 - "$name" "$title" "$action" "$bag_dir" \
    "$started" "$ended" "$metadata_ok" <<'PY'
import json, os, sys

name, title, action, bag_dir, started, ended, metadata_ok = sys.argv[1:8]
cues = []
raw = os.environ.get("CUE_ENTRIES", "").strip()
if raw:
    rows = raw.split("\n")
    for index in range(0, len(rows), 2):
        epoch = rows[index].split("\t")[0].strip()
        note = rows[index + 1].strip() if index + 1 < len(rows) else ""
        if epoch:
            cues.append({"epoch": float(epoch), "note": note})
print(json.dumps({
    "name": name,
    "title": title,
    "action": action,
    "bag_dir": bag_dir,
    "started_epoch": float(started),
    "ended_epoch": float(ended),
    "metadata_complete": metadata_ok == "true",
    "cues": cues,
}))
PY
}

SCENARIO_JSON="[]"
IFS=',' read -r -a SELECTED <<< "$SEGMENTS"
for name in "${SELECTED[@]}"; do
  name="$(printf '%s' "$name" | tr -d '[:space:]')"
  [ -n "$name" ] || continue
  [ -n "${SEG_TITLE[$name]:-}" ] || die "unknown segment: $name (see --list)"
  entry="$(run_segment "$name")"
  SCENARIO_JSON="$(
    SCENARIO_JSON="$SCENARIO_JSON" ENTRY_JSON="$entry" python3 - <<'PY'
import json, os
items = json.loads(os.environ["SCENARIO_JSON"])
items.append(json.loads(os.environ["ENTRY_JSON"]))
print(json.dumps(items))
PY
  )"
done

TOPICS_JSON="$(printf '%s\n' "${RECORD_TOPICS[@]}" | python3 -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))')"
SKIPPED_JSON="$(printf '%s\n' "${SKIPPED_TOPICS[@]:-}" | python3 -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))')"

META_JSON="$(
  SCENARIOS="$SCENARIO_JSON" TOPICS="$TOPICS_JSON" SKIPPED="$SKIPPED_JSON" \
  INTERACTIVE="$INTERACTIVE" SEGMENT_SECONDS="$SEGMENT_SECONDS" python3 - <<'PY'
import json, os, time
print(json.dumps({
    "generated_at_epoch": time.time(),
    "interactive": os.environ["INTERACTIVE"] == "1",
    "segment_seconds": int(os.environ["SEGMENT_SECONDS"]),
    "topics_recorded": json.loads(os.environ["TOPICS"]),
    "topics_skipped": json.loads(os.environ["SKIPPED"]),
    "scenarios": json.loads(os.environ["SCENARIOS"]),
}))
PY
)"
write_scenario_yaml "$META_JSON"

cat <<EOF

=== next steps ===
1. If the stack was started with the observation switches, keep:
     $OUT/filter_statistics.yaml      (written when the preprocessor exits cleanly)
2. Analyse the recording:
     python3 $REPO_ROOT/tools/local_costmap_forensics.py --root "$OUT" \\
       --statistics "$OUT/filter_statistics.yaml"
3. Interpretation rules are printed by the analyser and summarised in
     $OUT/analysis/summary.md
EOF
