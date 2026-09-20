#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=${1:-"$PWD/hardware_acceptance_$STAMP"}
mkdir -p -- "$OUTPUT_DIR"

run_check() {
  local label=$1
  local logfile=$2
  shift 2
  printf '\n[%s]\n' "$label"
  "$@" 2>&1 | tee "$logfile"
  local status=${PIPESTATUS[0]}
  if [[ "$status" -ne 0 ]]; then
    printf 'FAILED: %s (status=%s)\n' "$label" "$status" >&2
  fi
  return "$status"
}

failed=0
run_check 'runtime owner uniqueness' "$OUTPUT_DIR/runtime_check.txt" \
  "$SCRIPT_DIR/check_runtime.sh" || failed=1
run_check 'TF publisher authority' "$OUTPUT_DIR/tf_authority.log" \
  "$SCRIPT_DIR/check_tf_publishers.sh" "$OUTPUT_DIR/tf_authority.json" || failed=1
run_check 'critical topic rates' "$OUTPUT_DIR/topic_rates.log" \
  "$SCRIPT_DIR/check_topic_rates.sh" "$OUTPUT_DIR/topic_rates.json" || failed=1

if [[ "$failed" -ne 0 ]]; then
  "$SCRIPT_DIR/generate_acceptance_report.sh" "$OUTPUT_DIR" >/dev/null 2>&1 || true
  printf '\nPreflight failed; no motion test or navigation recording was started.\n' >&2
  printf 'Partial report: %s/acceptance_report.md\n' "$OUTPUT_DIR" >&2
  exit 1
fi

if [[ "${ACCEPTANCE_NON_INTERACTIVE:-0}" != "1" ]]; then
  printf '\nClear the test area, station an E-stop operator, and prepare an approved low-speed in-place rotation.\n'
  read -r -p 'Press Enter to open the rotation-center observation window... '
fi
run_check 'base_footprint rotation center' "$OUTPUT_DIR/base_footprint_center.log" \
  "$SCRIPT_DIR/check_base_footprint_center.sh" "$OUTPUT_DIR/base_footprint_center.json" || failed=1
if [[ "$failed" -ne 0 ]]; then
  "$SCRIPT_DIR/generate_acceptance_report.sh" "$OUTPUT_DIR" >/dev/null 2>&1 || true
  printf 'Motion-center check failed; navigation recording was not started.\n' >&2
  exit 1
fi

if [[ "${ACCEPTANCE_NON_INTERACTIVE:-0}" != "1" ]]; then
  printf '\nPrepare the approved Nav2 route. Recording starts after confirmation; send the goal from the operator station.\n'
  read -r -p 'Press Enter to start navigation recording... '
fi
run_check 'navigation recording' "$OUTPUT_DIR/record_navigation.log" \
  "$SCRIPT_DIR/record_navigation_run.sh" "$OUTPUT_DIR" || failed=1

"$SCRIPT_DIR/generate_acceptance_report.sh" "$OUTPUT_DIR"
report_status=$?
printf 'Hardware acceptance evidence: %s\n' "$OUTPUT_DIR"
if [[ "$failed" -ne 0 || "$report_status" -ne 0 ]]; then
  exit 1
fi
