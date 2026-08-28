#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
RULE_SOURCE="$ROBOT_WS/deploy/99-autolabor-zed.rules"
RULE_DEST="/etc/udev/rules.d/99-autolabor-zed.rules"
SERVICE_SOURCE="$ROBOT_WS/deploy/autolabor-zed-coldplug.service"
SERVICE_DEST="/etc/systemd/system/autolabor-zed-coldplug.service"
HELPER_SOURCE="$ROBOT_WS/deploy/autolabor-usb-coldplug.sh"
HELPER_DEST="/usr/local/sbin/autolabor-usb-coldplug"
ENV_DEST="/etc/default/autolabor-usb-coldplug"
CONFIG_SOURCE="${DUAL_HOST_CONFIG:-$ROBOT_WS/config/dual_host.env}"

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
[[ -s "$HELPER_SOURCE" ]] || {
  echo "Missing USB coldplug helper: $HELPER_SOURCE" >&2
  exit 2
}
[[ -r "$CONFIG_SOURCE" ]] || {
  echo "Missing dual-host configuration: $CONFIG_SOURCE" >&2
  exit 2
}

config_value() {
  local key="$1"
  awk -F= -v key="$key" '$1 == key { print substr($0, index($0, "=") + 1); exit }' \
    "$CONFIG_SOURCE"
}

j6m_usb_id="$(config_value NVIDIA_J6M_USB_ID)"
j6m_usb_serial="$(config_value NVIDIA_J6M_USB_SERIAL)"
mid360_usb_id="$(config_value NVIDIA_LIVOX_USB_ID)"
mid360_usb_serial="$(config_value NVIDIA_LIVOX_USB_SERIAL)"
zed_serial="$(config_value NVIDIA_ZED_SERIAL)"
zed_serial="${zed_serial:-23748636}"

for usb_id in "$j6m_usb_id" "$mid360_usb_id"; do
  [[ "$usb_id" =~ ^[0-9A-Fa-f]{4}:[0-9A-Fa-f]{4}$ ]] || {
    echo "Invalid configured USB ID: $usb_id" >&2
    exit 2
  }
done
for serial in "$j6m_usb_serial" "$mid360_usb_serial" "$zed_serial"; do
  [[ "$serial" =~ ^[A-Za-z0-9._:-]+$ ]] || {
    echo "Invalid configured USB serial: $serial" >&2
    exit 2
  }
done

install -m 0644 "$RULE_SOURCE" "$RULE_DEST"
install -m 0644 "$SERVICE_SOURCE" "$SERVICE_DEST"
install -m 0755 "$HELPER_SOURCE" "$HELPER_DEST"
env_temp="$(mktemp /tmp/autolabor-usb-coldplug.XXXXXX)"
trap 'rm -f -- "$env_temp"' EXIT
{
  printf 'AUTOLABOR_J6M_USB_ID=%s\n' "${j6m_usb_id,,}"
  printf 'AUTOLABOR_J6M_USB_SERIAL=%s\n' "$j6m_usb_serial"
  printf 'AUTOLABOR_MID360_USB_ID=%s\n' "${mid360_usb_id,,}"
  printf 'AUTOLABOR_MID360_USB_SERIAL=%s\n' "$mid360_usb_serial"
  printf 'AUTOLABOR_ZED_SERIAL=%s\n' "$zed_serial"
} >"$env_temp"
install -m 0644 "$env_temp" "$ENV_DEST"
trap - EXIT
rm -f -- "$env_temp"
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable autolabor-zed-coldplug.service
systemctl restart autolabor-zed-coldplug.service

echo "Installed the ZED udev rule and Autolabor post-coldplug repair service."
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

echo "The service also restores USB network, CAN, LD19 and ZED driver binding after future boots."
echo "It cannot promote a USB2 link to USB3 or repair an unpowered peer/cable."
echo "Run '$ROBOT_WS/scripts/zed_camera_check.sh --wait 0' as the desktop user next."
