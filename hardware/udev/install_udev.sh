#!/usr/bin/env bash
set -euo pipefail
RULES_SRC="$(dirname "$0")/99-aura-sensors.rules"
RULES_DST="/etc/udev/rules.d/99-aura-sensors.rules"
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root: sudo $0"
  exit 1
fi
cp "$RULES_SRC" "$RULES_DST"
udevadm control --reload-rules
udevadm trigger
echo "[ok] Installed $RULES_DST and reloaded udev rules."
echo "Update vendor/product IDs in the rules file to match your hardware (use lsusb)."
