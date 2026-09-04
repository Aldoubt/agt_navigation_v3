#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo: sudo bash $0" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${REPO_ROOT}/config/field_hardware.env"

missing_pkgs=()
command -v ip >/dev/null 2>&1 || missing_pkgs+=(iproute2)
command -v candump >/dev/null 2>&1 || missing_pkgs+=(can-utils)
if [[ "${#missing_pkgs[@]}" -gt 0 ]]; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing_pkgs[@]}"
fi

install -m 0755 "${REPO_ROOT}/scripts/agt_bunker_can" /usr/local/sbin/agt-bunker-can
install -m 0644 "${REPO_ROOT}/systemd/agt-bunker-can.service" /etc/systemd/system/agt-bunker-can.service

cat >/etc/default/agt-bunker-can <<EOF
AGT_CAN_IFACE=${AGT_CAN_IFACE}
AGT_CAN_BITRATE=${AGT_CAN_BITRATE}
AGT_CAN_RESTART_MS=${AGT_CAN_RESTART_MS}
EOF

systemctl daemon-reload
systemctl enable --now agt-bunker-can.service

echo "Installed and enabled agt-bunker-can.service"
echo "This only configures SocketCAN. It does NOT bypass Bunker remote/manual priority and does NOT send velocity commands."
systemctl --no-pager --full status agt-bunker-can.service || true
