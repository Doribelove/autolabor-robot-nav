#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
RULE_SOURCE="$ROBOT_WS/deploy/99-autolabor-dual-lidar.rules"
RULE_DEST="/etc/udev/rules.d/99-autolabor-dual-lidar.rules"
FRONT_PHYSICAL="/dev/serial/by-path/platform-3610000.xhci-usb-0:4.4:1.0-port0"
REAR_PHYSICAL="/dev/serial/by-path/platform-3610000.xhci-usb-0:4.3:1.0-port0"

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

[[ -s "$RULE_SOURCE" ]] || {
  echo "Missing udev rule: $RULE_SOURCE" >&2
  exit 2
}
[[ -e "$FRONT_PHYSICAL" && -e "$REAR_PHYSICAL" ]] || {
  echo "Both physically identified LD19 ports must be connected." >&2
  exit 3
}

install -m 0644 "$RULE_SOURCE" "$RULE_DEST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
udevadm settle

for role in front rear; do
  alias_path="/dev/autolabor/lidar_$role"
  if [[ "$role" == front ]]; then
    physical_path="$FRONT_PHYSICAL"
  else
    physical_path="$REAR_PHYSICAL"
  fi
  [[ -L "$alias_path" ]] || {
    echo "udev did not create $alias_path" >&2
    exit 4
  }
  [[ "$(readlink -f "$alias_path")" == "$(readlink -f "$physical_path")" ]] || {
    echo "$alias_path points to the wrong physical USB port." >&2
    exit 4
  }
  echo "$alias_path -> $(readlink -f "$alias_path")"
done

echo "Dual-LD19 physical-port aliases installed."
