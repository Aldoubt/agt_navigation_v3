#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
PROFILE=${HARDWARE_ACCEPTANCE_PROFILE:-"$REPO_ROOT/tests/acceptance/hardware_acceptance_profile.yaml"}
OUTPUT=${1:-"$PWD/tf_authority.json"}
DURATION_SEC=${TF_AUDIT_DURATION_SEC:-0}

exec python3 "$REPO_ROOT/tools/hardware_acceptance/tf_authority_audit.py" \
  --profile "$PROFILE" \
  --output "$OUTPUT" \
  --duration-sec "$DURATION_SEC"
