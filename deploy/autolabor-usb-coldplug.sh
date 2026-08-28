#!/usr/bin/env bash
set -euo pipefail

# Jetson can finish the first USB enumeration before udev has loaded all of
# the out-of-tree/optional modules.  Reapply only the devices owned by this
# robot; do not assign network roles here (load_config.sh remains authoritative
# for that).

USB_SYS_ROOT="${AUTOLABOR_USB_SYS_ROOT:-/sys/bus/usb/devices}"
USB_DEV_ROOT="${AUTOLABOR_USB_DEV_ROOT:-/dev/bus/usb}"
SYS_CLASS_ROOT="${AUTOLABOR_SYS_CLASS_ROOT:-/sys/class}"
UDEVADM="${AUTOLABOR_UDEVADM:-/usr/bin/udevadm}"
MODPROBE="${AUTOLABOR_MODPROBE:-/usr/sbin/modprobe}"
USBRESET="${AUTOLABOR_USBRESET:-/usr/bin/usbreset}"
SLEEP="${AUTOLABOR_SLEEP:-/usr/bin/sleep}"

J6M_USB_ID="${AUTOLABOR_J6M_USB_ID:-}"
J6M_USB_SERIAL="${AUTOLABOR_J6M_USB_SERIAL:-}"
MID360_USB_ID="${AUTOLABOR_MID360_USB_ID:-}"
MID360_USB_SERIAL="${AUTOLABOR_MID360_USB_SERIAL:-}"
ZED_SERIAL="${AUTOLABOR_ZED_SERIAL:-23748636}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "autolabor-usb-coldplug must run as root." >&2
  exit 2
fi

normalize_usb_id() {
  tr '[:upper:]' '[:lower:]' <<<"$1"
}

J6M_USB_ID="$(normalize_usb_id "$J6M_USB_ID")"
MID360_USB_ID="$(normalize_usb_id "$MID360_USB_ID")"

usb_identity() {
  local device_path="$1" vendor product
  [[ -r "$device_path/idVendor" && -r "$device_path/idProduct" ]] || return 1
  vendor="$(tr '[:upper:]' '[:lower:]' <"$device_path/idVendor" | tr -d '[:space:]')"
  product="$(tr '[:upper:]' '[:lower:]' <"$device_path/idProduct" | tr -d '[:space:]')"
  printf '%s:%s\n' "$vendor" "$product"
}

usb_serial() {
  local device_path="$1"
  [[ -r "$device_path/serial" ]] || return 0
  tr -d '[:space:]' <"$device_path/serial"
}

matches_configured_identity() {
  local device_path="$1" expected_id="$2" expected_serial="$3"
  local actual_id actual_serial
  [[ -n "$expected_id" && -n "$expected_serial" ]] || return 1
  actual_id="$(usb_identity "$device_path" 2>/dev/null || true)"
  actual_serial="$(usb_serial "$device_path" 2>/dev/null || true)"
  [[ "$actual_id" == "$expected_id" && "$actual_serial" == "$expected_serial" ]]
}

is_owned_usb_device() {
  local device_path="$1" identity serial
  matches_configured_identity "$device_path" "$J6M_USB_ID" "$J6M_USB_SERIAL" && return 0
  matches_configured_identity "$device_path" "$MID360_USB_ID" "$MID360_USB_SERIAL" && return 0

  identity="$(usb_identity "$device_path" 2>/dev/null || true)"
  serial="$(usb_serial "$device_path" 2>/dev/null || true)"
  case "$identity" in
    0403:6001|1a86:7523|2b03:f780)
      return 0
      ;;
    2b03:f781)
      [[ -z "$ZED_SERIAL" || "$serial" == "$ZED_SERIAL" ]]
      return
      ;;
  esac
  return 1
}

trigger_owned_usb_devices() {
  local device_path
  for device_path in "$USB_SYS_ROOT"/*; do
    [[ -e "$device_path" ]] || continue
    is_owned_usb_device "$device_path" || continue
    "$UDEVADM" trigger --action=add "$device_path"
  done
}

class_node_belongs_to_owned_usb() {
  local class_node="$1" cursor parent
  cursor="$(readlink -f "$class_node/device" 2>/dev/null || true)"
  while [[ -n "$cursor" && "$cursor" != / ]]; do
    if [[ -r "$cursor/idVendor" && -r "$cursor/idProduct" ]] &&
       is_owned_usb_device "$cursor"; then
      return 0
    fi
    parent="${cursor%/*}"
    [[ "$parent" != "$cursor" ]] || break
    cursor="$parent"
  done
  return 1
}

trigger_owned_class_nodes() {
  local class_root class_node
  for class_root in net tty video4linux hidraw; do
    [[ -d "$SYS_CLASS_ROOT/$class_root" ]] || continue
    for class_node in "$SYS_CLASS_ROOT/$class_root"/*; do
      [[ -e "$class_node" ]] || continue
      class_node_belongs_to_owned_usb "$class_node" || continue
      "$UDEVADM" trigger --action=add "$class_node"
    done
  done
}

usb_device_has_carrier() {
  local device_path="$1" carrier
  for carrier in "$device_path"/*/net/*/carrier; do
    [[ -r "$carrier" ]] || continue
    [[ "$(<"$carrier")" == 1 ]] && return 0
  done
  return 1
}

usb_device_node() {
  local device_path="$1" bus dev
  [[ -r "$device_path/busnum" && -r "$device_path/devnum" ]] || return 1
  bus="$(tr -d '[:space:]' <"$device_path/busnum")"
  dev="$(tr -d '[:space:]' <"$device_path/devnum")"
  [[ "$bus" =~ ^[0-9]+$ && "$dev" =~ ^[0-9]+$ ]] || return 1
  printf '%s/%03d/%03d\n' "$USB_DEV_ROOT" "$((10#$bus))" "$((10#$dev))"
}

reset_stuck_mid360_adapter() {
  local device_path node
  [[ -x "$USBRESET" ]] || return 0
  for device_path in "$USB_SYS_ROOT"/*; do
    [[ -e "$device_path" ]] || continue
    matches_configured_identity \
      "$device_path" "$MID360_USB_ID" "$MID360_USB_SERIAL" || continue
    if usb_device_has_carrier "$device_path"; then
      echo "Autolabor MID360 USB Ethernet already has carrier; reset skipped."
      return 0
    fi
    node="$(usb_device_node "$device_path" 2>/dev/null || true)"
    if [[ -n "$node" && -e "$node" ]]; then
      echo "Resetting the exact MID360 USB Ethernet device after carrier-less coldplug: $node"
      "$USBRESET" "$node"
      "$SLEEP" 2
    fi
    return 0
  done
}

for module in ax88179_178a cdc_ether ftdi_sio ch341 uvcvideo; do
  "$MODPROBE" "$module"
done

"$UDEVADM" control --reload-rules
trigger_owned_usb_devices
"$UDEVADM" settle --timeout=20
reset_stuck_mid360_adapter
"$UDEVADM" settle --timeout=20
trigger_owned_usb_devices
trigger_owned_class_nodes
"$UDEVADM" settle --timeout=20

echo "Autolabor USB coldplug repair completed."
