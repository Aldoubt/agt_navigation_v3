#!/usr/bin/env bash
set -eo pipefail
AGT_DESKTOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
AGT_LAUNCH_LOG="${AGT_DATA_ROOT:-$HOME/agt}/logs/desktop-launch.log"
mkdir -p "$(dirname "$AGT_LAUNCH_LOG")"
if ! "$AGT_DESKTOP_ROOT/agt" up >"$AGT_LAUNCH_LOG" 2>&1; then
  message="AGT YHS Control failed. Read $AGT_LAUNCH_LOG or run ./agt doctor."
  if command -v zenity >/dev/null; then zenity --error --text="$message";
  elif command -v xmessage >/dev/null; then xmessage "$message";
  else cat "$AGT_LAUNCH_LOG" >&2; fi
  exit 1
fi
