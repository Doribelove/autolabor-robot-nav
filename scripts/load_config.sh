#!/usr/bin/env bash

DUAL_HOST_WS="${DUAL_HOST_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DUAL_HOST_CONFIG="${DUAL_HOST_CONFIG:-$DUAL_HOST_WS/config/dual_host.env}"

if [[ ! -r "$DUAL_HOST_CONFIG" ]]; then
  echo "Missing dual-host configuration: $DUAL_HOST_CONFIG" >&2
  return 2 2>/dev/null || exit 2
fi

set -a
source "$DUAL_HOST_CONFIG"
set +a

NVIDIA_J6M_MAC="${NVIDIA_J6M_MAC:-}"
NVIDIA_LIVOX_MAC="${NVIDIA_LIVOX_MAC:-}"
DUAL_HOST_DEVICE_WAIT_SEC="${DUAL_HOST_DEVICE_WAIT_SEC:-20}"
export NVIDIA_J6M_MAC NVIDIA_LIVOX_MAC DUAL_HOST_DEVICE_WAIT_SEC

dual_host_normalize_mac() {
  printf '%s\n' "${1,,}"
}

dual_host_find_interface_by_mac() {
  local wanted actual address_file
  wanted="$(dual_host_normalize_mac "${1:-}")"
  [[ "$wanted" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] || return 1
  for address_file in /sys/class/net/*/address; do
    [[ -r "$address_file" ]] || continue
    actual="$(<"$address_file")"
    if [[ "${actual,,}" == "$wanted" ]]; then
      printf '%s\n' "${address_file%/address}" | sed 's#.*/##'
      return 0
    fi
  done
  return 1
}

dual_host_resolve_interface_variable() {
  local variable_name="$1" mac="$2" resolved
  [[ "$variable_name" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || return 2
  resolved="$(dual_host_find_interface_by_mac "$mac" 2>/dev/null || true)"
  [[ -z "$resolved" ]] || printf -v "$variable_name" '%s' "$resolved"
  export "$variable_name"
}

dual_host_refresh_local_interfaces() {
  if [[ -n "${NVIDIA_J6M_MAC:-}" ]]; then
    dual_host_resolve_interface_variable NVIDIA_J6M_INTERFACE "$NVIDIA_J6M_MAC"
  fi
  if [[ -n "${NVIDIA_LIVOX_MAC:-}" ]]; then
    dual_host_resolve_interface_variable NVIDIA_LIVOX_INTERFACE "$NVIDIA_LIVOX_MAC"
  fi
}

dual_host_refresh_local_interfaces

export DUAL_HOST_WS DUAL_HOST_CONFIG
export ROS_MASTER_URI="http://${J6M_IP}:11311"
export ROS_IP="$NVIDIA_J6M_IP"
unset ROS_HOSTNAME

dual_host_is_bool() {
  [[ "$1" == true || "$1" == false ]]
}

dual_host_mode_enabled() {
  local mode="$1"
  shift
  case "$mode" in
    true) return 0 ;;
    false) return 1 ;;
    auto)
      local path
      for path in "$@"; do
        [[ -e "$path" ]] || return 1
      done
      return 0
      ;;
    *) echo "Invalid true/false/auto value: $mode" >&2; return 2 ;;
  esac
}

dual_host_select_ssh() {
  if timeout 4 ssh -o BatchMode=yes -o ConnectTimeout=3 "$J6M_SSH" true >/dev/null 2>&1; then
    printf '%s\n' "$J6M_SSH"
  elif timeout 4 ssh -o BatchMode=yes -o ConnectTimeout=3 "$J6M_SSH_CURRENT" true >/dev/null 2>&1; then
    printf '%s\n' "$J6M_SSH_CURRENT"
  else
    return 1
  fi
}
