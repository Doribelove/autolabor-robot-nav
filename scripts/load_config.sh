#!/usr/bin/env bash

DUAL_HOST_WS="${DUAL_HOST_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DUAL_HOST_CONFIG="${DUAL_HOST_CONFIG:-$DUAL_HOST_WS/config/dual_host.env}"

# Command-line launch selection is exported to child scripts and must take
# precedence over persistent hardware defaults in dual_host.env.
_map_enabled_was_set="${STATIC_MAP_ENABLED+x}"
_map_enabled_override="${STATIC_MAP_ENABLED:-}"
_map_set_was_set="${STATIC_MAP_SET+x}"
_map_set_override="${STATIC_MAP_SET:-}"
_map_source_was_set="${STATIC_MAP_SOURCE_MODE+x}"
_map_source_override="${STATIC_MAP_SOURCE_MODE:-}"
_map_file_was_set="${STATIC_MAP_FILE+x}"
_map_file_override="${STATIC_MAP_FILE:-}"
_lio_map_was_set="${FAST_LIO_MAP_FILE+x}"
_lio_map_override="${FAST_LIO_MAP_FILE:-}"
_lio_z_was_set="${FAST_LIO_INITIAL_BODY_Z+x}"
_lio_z_override="${FAST_LIO_INITIAL_BODY_Z:-}"

if [[ ! -r "$DUAL_HOST_CONFIG" ]]; then
  echo "Missing dual-host configuration: $DUAL_HOST_CONFIG" >&2
  return 2 2>/dev/null || exit 2
fi

set -a
source "$DUAL_HOST_CONFIG"
set +a

[[ -z "$_map_enabled_was_set" ]] || STATIC_MAP_ENABLED="$_map_enabled_override"
[[ -z "$_map_set_was_set" ]] || STATIC_MAP_SET="$_map_set_override"
[[ -z "$_map_source_was_set" ]] || STATIC_MAP_SOURCE_MODE="$_map_source_override"
[[ -z "$_map_file_was_set" ]] || STATIC_MAP_FILE="$_map_file_override"
[[ -z "$_lio_map_was_set" ]] || FAST_LIO_MAP_FILE="$_lio_map_override"
[[ -z "$_lio_z_was_set" ]] || FAST_LIO_INITIAL_BODY_Z="$_lio_z_override"

NVIDIA_J6M_MAC="${NVIDIA_J6M_MAC:-}"
NVIDIA_LIVOX_MAC="${NVIDIA_LIVOX_MAC:-}"
DUAL_HOST_DEVICE_WAIT_SEC="${DUAL_HOST_DEVICE_WAIT_SEC:-20}"
STATIC_MAP_ENABLED="${STATIC_MAP_ENABLED:-false}"
STATIC_MAP_SET="${STATIC_MAP_SET:-}"
STATIC_MAP_SOURCE_MODE="${STATIC_MAP_SOURCE_MODE:-fused}"
STATIC_MAP_FILE="${STATIC_MAP_FILE:-}"
FAST_LIO_MAP_FILE="${FAST_LIO_MAP_FILE:-}"
FAST_LIO_INITIAL_BODY_Z="${FAST_LIO_INITIAL_BODY_Z:-0.0}"
MAPPING_REQUIRE_DUAL_LIDAR="${MAPPING_REQUIRE_DUAL_LIDAR:-true}"
MAPPING_TOPIC_WAIT_SEC="${MAPPING_TOPIC_WAIT_SEC:-10}"
export NVIDIA_J6M_MAC NVIDIA_LIVOX_MAC DUAL_HOST_DEVICE_WAIT_SEC
export STATIC_MAP_ENABLED STATIC_MAP_SET STATIC_MAP_SOURCE_MODE STATIC_MAP_FILE
export FAST_LIO_MAP_FILE FAST_LIO_INITIAL_BODY_Z
export MAPPING_REQUIRE_DUAL_LIDAR MAPPING_TOPIC_WAIT_SEC

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
