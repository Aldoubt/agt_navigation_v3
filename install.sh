#!/usr/bin/env bash
set -eo pipefail
AGT_INSTALL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! "${AGT_PYTHON:-python3}" -c 'import yaml' >/dev/null 2>&1; then
  echo 'PyYAML missing. On Ubuntu: sudo apt-get install python3-yaml python3-venv xauth' >&2
  exit 2
fi
mkdir -p "${AGT_DATA_ROOT:-$HOME/agt}/logs"
"$AGT_INSTALL_ROOT/agt" install "$@" 2>&1 | tee "${AGT_DATA_ROOT:-$HOME/agt}/logs/install.log"
