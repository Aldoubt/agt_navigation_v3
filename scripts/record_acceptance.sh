#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# Compatibility entry point retained from v3.1. The acceptance workflow remains
# an external observer and does not alter launch composition or node ownership.
exec "$SCRIPT_DIR/run_hardware_acceptance.sh" "$@"
