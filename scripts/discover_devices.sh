#!/usr/bin/env bash
set -euo pipefail

echo "NVIDIA network interfaces:"
ip -br link
ip -br -4 addr

echo
echo "Persistent serial aliases:"
ls -l /dev/serial/by-id /dev/serial/by-path 2>&1 || true

echo
echo "USB serial properties:"
found=false
for device in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$device" ]] || continue
  found=true
  echo "[$device]"
  udevadm info -q property -n "$device" |
    grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_USB_INTERFACE_NUM)=' |
    sort || true
  owners="$(fuser "$device" 2>/dev/null || true)"
  if [[ -n "$owners" ]]; then
    echo "IN_USE_BY_PID=$owners"
    read -r -a owner_pids <<<"$owners"
    for owner_pid in "${owner_pids[@]}"; do
      ps -o pid=,cmd= -p "$owner_pid" 2>/dev/null || true
    done
  fi
done
if [[ "$found" == false ]]; then
  echo "No ttyUSB/ttyACM device is currently present."
fi

echo
echo "Do not guess CAN/front/rear assignments. Update config/dual_host.env and"
echo "set the matching *_CONFIRMED flag only after one-device-at-a-time identification."
