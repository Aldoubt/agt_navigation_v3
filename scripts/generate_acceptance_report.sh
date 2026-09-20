#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
EVIDENCE_DIR=${1:?usage: generate_acceptance_report.sh EVIDENCE_DIR [OUTPUT]}
OUTPUT=${2:-"$EVIDENCE_DIR/acceptance_report.md"}

exec python3 "$REPO_ROOT/tools/hardware_acceptance/generate_acceptance_report.py" \
  "$EVIDENCE_DIR" --output "$OUTPUT"
