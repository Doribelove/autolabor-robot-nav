#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"

usage() {
  echo "Usage: $0 [--wait SECONDS]" >&2
}

wait_seconds=0
while (( $# > 0 )); do
  case "$1" in
    --wait)
      (( $# >= 2 )) || { usage; exit 2; }
      wait_seconds="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

[[ "$wait_seconds" =~ ^[0-9]+$ ]] || {
  echo "ZED USB wait time must be a non-negative integer: $wait_seconds" >&2
  exit 2
}

usb_sys_root="${DUAL_HOST_USB_SYS_ROOT:-/sys/bus/usb/devices}"
device_root="${DUAL_HOST_DEVICE_ROOT:-/dev}"
usb_dev_root="${DUAL_HOST_USB_DEV_ROOT:-$device_root/bus/usb}"
hidraw_sys_root="${DUAL_HOST_HIDRAW_SYS_ROOT:-/sys/class/hidraw}"
expected_serial="${NVIDIA_ZED_SERIAL:-23748636}"

video_found=false
video_superspeed=false
video_accessible=false
video_path=""
video_speed=""
video_node=""
hid_found=false
hid_accessible=false
hid_path=""
hid_node=""
hidraw_found=false
hidraw_accessible=false
hidraw_node=""

usb_device_node() {
  local device_path="$1" bus_value dev_value bus_number dev_number
  [[ -r "$device_path/busnum" && -r "$device_path/devnum" ]] || return 1
  bus_value="$(tr -d '[:space:]' <"$device_path/busnum")"
  dev_value="$(tr -d '[:space:]' <"$device_path/devnum")"
  [[ "$bus_value" =~ ^[0-9]+$ && "$dev_value" =~ ^[0-9]+$ ]] || return 1
  bus_number=$((10#$bus_value))
  dev_number=$((10#$dev_value))
  printf '%s/%03d/%03d\n' "$usb_dev_root" "$bus_number" "$dev_number"
}

probe_zed_usb() {
  local device_path vendor product serial node speed class_path ancestor parent
  video_found=false
  video_superspeed=false
  video_accessible=false
  video_path=""
  video_speed=""
  video_node=""
  hid_found=false
  hid_accessible=false
  hid_path=""
  hid_node=""
  hidraw_found=false
  hidraw_accessible=false
  hidraw_node=""

  for device_path in "$usb_sys_root"/*; do
    [[ -r "$device_path/idVendor" && -r "$device_path/idProduct" ]] || continue
    vendor="$(tr '[:upper:]' '[:lower:]' <"$device_path/idVendor" | tr -d '[:space:]')"
    [[ "$vendor" == 2b03 ]] || continue
    product="$(tr '[:upper:]' '[:lower:]' <"$device_path/idProduct" | tr -d '[:space:]')"
    node="$(usb_device_node "$device_path" 2>/dev/null || true)"
    case "$product" in
      f780)
        video_found=true
        video_path="${device_path##*/}"
        video_node="$node"
        speed="$(tr -d '[:space:]' <"$device_path/speed" 2>/dev/null || true)"
        video_speed="$speed"
        if [[ "$speed" =~ ^[0-9]+([.][0-9]+)?$ ]] &&
           awk -v speed="$speed" 'BEGIN { exit !(speed >= 5000) }'; then
          video_superspeed=true
        fi
        [[ -n "$node" && -r "$node" ]] && video_accessible=true
        ;;
      f781)
        serial=""
        [[ ! -r "$device_path/serial" ]] ||
          serial="$(tr -d '[:space:]' <"$device_path/serial")"
        [[ -z "$expected_serial" || "$serial" == "$expected_serial" ]] || continue
        hid_found=true
        hid_path="${device_path##*/}"
        hid_node="$node"
        [[ -n "$node" && -r "$node" && -w "$node" ]] && hid_accessible=true
        ;;
    esac
  done

  for class_path in "$hidraw_sys_root"/*; do
    [[ -e "$class_path/device" ]] || continue
    ancestor="$(readlink -f "$class_path/device" 2>/dev/null || true)"
    while [[ -n "$ancestor" && "$ancestor" != / ]]; do
      if [[ -r "$ancestor/idVendor" && -r "$ancestor/idProduct" ]] &&
         [[ "$(tr -d '[:space:]' <"$ancestor/idVendor")" == 2b03 ]] &&
         [[ "$(tr -d '[:space:]' <"$ancestor/idProduct")" == f781 ]]; then
        serial=""
        [[ ! -r "$ancestor/serial" ]] || serial="$(tr -d '[:space:]' <"$ancestor/serial")"
        [[ -z "$expected_serial" || "$serial" == "$expected_serial" ]] || break
        hidraw_found=true
        hidraw_node="$device_root/${class_path##*/}"
        [[ -r "$hidraw_node" && -w "$hidraw_node" ]] && hidraw_accessible=true
        break
      fi
      parent="${ancestor%/*}"
      [[ "$parent" != "$ancestor" ]] || break
      ancestor="$parent"
    done
  done

  [[ "$video_found" == true && "$video_superspeed" == true &&
     "$video_accessible" == true && "$hid_found" == true &&
     "$hid_accessible" == true ]]
}

deadline=$((SECONDS + wait_seconds))
while ! probe_zed_usb; do
  (( SECONDS < deadline )) || break
  sleep 1
done

if probe_zed_usb; then
  echo "OK  ZED $expected_serial is on SuperSpeed USB (${video_speed}M, $video_path) and accessible."
  if [[ "$hidraw_found" == true && "$hidraw_accessible" != true ]]; then
    echo "WARN optional ZED hidraw node is not accessible: ${hidraw_node:-unknown}; usbfs access remains available." >&2
  fi
  exit 0
fi

if [[ "$video_found" != true ]]; then
  echo "FAIL ZED video interface 2b03:f780 is not enumerated." >&2
elif [[ "$video_superspeed" != true ]]; then
  echo "FAIL ZED video interface $video_path negotiated only ${video_speed:-unknown}M; USB 3.x (at least 5000M) is required." >&2
fi
if [[ "$hid_found" != true ]]; then
  echo "FAIL ZED HID interface 2b03:f781 with serial $expected_serial is not enumerated." >&2
fi
if [[ "$video_found" == true && "$video_accessible" != true ]]; then
  echo "FAIL ZED video USB node is not readable by $(id -un): ${video_node:-unknown}." >&2
fi
if [[ "$hid_found" == true && "$hid_accessible" != true ]]; then
  echo "FAIL ZED HID USB node is not readable/writable by $(id -un): ${hid_node:-unknown}." >&2
fi
echo "Reconnect the ZED cable (try reversing its Type-C plug) or use a direct USB 3.x port." >&2
echo "Then verify that the ZED video device appears as 5000M or faster in 'lsusb -t'." >&2
echo "If USB speed is correct but access still fails, replug once to reapply the installed ZED udev rules." >&2
exit 1
