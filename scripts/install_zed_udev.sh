#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
RULE_SOURCE="$ROBOT_WS/deploy/99-autolabor-zed.rules"
RULE_DEST="/etc/udev/rules.d/99-autolabor-zed.rules"
SERVICE_SOURCE="$ROBOT_WS/deploy/autolabor-zed-coldplug.service"
SERVICE_DEST="/etc/systemd/system/autolabor-zed-coldplug.service"

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

[[ -s "$RULE_SOURCE" ]] || {
  echo "Missing ZED udev rule: $RULE_SOURCE" >&2
  exit 2
}
[[ -s "$SERVICE_SOURCE" ]] || {
  echo "Missing ZED coldplug service: $SERVICE_SOURCE" >&2
  exit 2
}

install -m 0644 "$RULE_SOURCE" "$RULE_DEST"
install -m 0644 "$SERVICE_SOURCE" "$SERVICE_DEST"
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable autolabor-zed-coldplug.service
systemctl restart autolabor-zed-coldplug.service

echo "Installed the ZED udev rule and post-coldplug repair service."
echo "Current ZED device permissions:"
for device_path in /sys/bus/usb/devices/*; do
  [[ -r "$device_path/idVendor" && -r "$device_path/idProduct" ]] || continue
  [[ "$(<"$device_path/idVendor")" == 2b03 ]] || continue
  bus="$(<"$device_path/busnum")"
  dev="$(<"$device_path/devnum")"
  node="$(printf '/dev/bus/usb/%03d/%03d' "$((10#$bus))" "$((10#$dev))")"
  stat -c '%A %U:%G %n' "$node" 2>/dev/null || true
done
for node in /dev/hidraw*; do
  [[ -e "$node" ]] || continue
  stat -c '%A %U:%G %n' "$node"
done

echo "The rule repairs access after future boots; it cannot promote a USB2 link to USB3."
echo "Run '$ROBOT_WS/scripts/zed_camera_check.sh --wait 0' as the desktop user next."
