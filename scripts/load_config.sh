#!/usr/bin/env bash

DUAL_HOST_WS="${DUAL_HOST_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DUAL_HOST_CONFIG="${DUAL_HOST_CONFIG:-$DUAL_HOST_WS/config/dual_host.env}"

# The managed one-run visual-motion authorization is deliberately separate
# from dual_host.env.  Only the supported start_dual_host.sh flag should set
# this exported override; every child that reloads the persistent config must
# retain the same effective gate for the lifetime of that supervisor.
_fod_motion_override="${DUAL_HOST_FOD_MOTION_OVERRIDE:-}"
case "$_fod_motion_override" in
  ""|true) ;;
  *)
    echo "DUAL_HOST_FOD_MOTION_OVERRIDE must be empty or literal true." >&2
    return 2 2>/dev/null || exit 2
    ;;
esac

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
[[ -z "$_fod_motion_override" ]] || FOD_MOTION_ENABLED=true

dual_host_resolve_workspace_path() {
  local value="$1"
  [[ -n "$value" ]] || return 1
  if [[ "$value" == /* ]]; then
    readlink -m -- "$value"
  else
    readlink -m -- "$DUAL_HOST_WS/$value"
  fi
}

NVIDIA_J6M_INTERFACE="${NVIDIA_J6M_INTERFACE:-}"
NVIDIA_LIVOX_INTERFACE="${NVIDIA_LIVOX_INTERFACE:-}"
NVIDIA_J6M_MAC="${NVIDIA_J6M_MAC:-}"
NVIDIA_LIVOX_MAC="${NVIDIA_LIVOX_MAC:-}"
NVIDIA_J6M_CONFIGURED_MAC="$NVIDIA_J6M_MAC"
NVIDIA_LIVOX_CONFIGURED_MAC="$NVIDIA_LIVOX_MAC"
NVIDIA_J6M_USB_ID="${NVIDIA_J6M_USB_ID:-}"
NVIDIA_J6M_USB_SERIAL="${NVIDIA_J6M_USB_SERIAL:-}"
NVIDIA_LIVOX_USB_ID="${NVIDIA_LIVOX_USB_ID:-}"
NVIDIA_LIVOX_USB_SERIAL="${NVIDIA_LIVOX_USB_SERIAL:-}"
DUAL_HOST_SYS_CLASS_NET_ROOT="${DUAL_HOST_SYS_CLASS_NET_ROOT:-/sys/class/net}"
DUAL_HOST_DEVICE_WAIT_SEC="${DUAL_HOST_DEVICE_WAIT_SEC:-20}"
DUAL_HOST_NETWORK_WAIT_SEC="${DUAL_HOST_NETWORK_WAIT_SEC:-90}"
DUAL_HOST_NETWORK_POLL_SEC="${DUAL_HOST_NETWORK_POLL_SEC:-1}"
NVIDIA_ZED_SERIAL="${NVIDIA_ZED_SERIAL:-23748636}"
ZED_USB_WAIT_SEC="${ZED_USB_WAIT_SEC:-20}"
ZED_IMAGE_WAIT_SEC="${ZED_IMAGE_WAIT_SEC:-60}"
NVIDIA_FOD_BACKEND="${NVIDIA_FOD_BACKEND:-yolo}"
NVIDIA_FOD_WEIGHTS="${NVIDIA_FOD_WEIGHTS:-src/application/autolabor_fod_vision/models/best6.pt}"
NVIDIA_FOD_ULTRALYTICS_ROOT="${NVIDIA_FOD_ULTRALYTICS_ROOT:-ultralytics_yolo11_custom}"
NVIDIA_LOCATEANYTHING_MODEL_ROOT="${NVIDIA_LOCATEANYTHING_MODEL_ROOT:-/home/slam/LocateAnything}"
NVIDIA_LOCATEANYTHING_MANIFEST="${NVIDIA_LOCATEANYTHING_MANIFEST:-$NVIDIA_LOCATEANYTHING_MODEL_ROOT/.runtime/deployment_manifest.json}"
NVIDIA_LOCATEANYTHING_WORKER_PYTHON="${NVIDIA_LOCATEANYTHING_WORKER_PYTHON:-${NVIDIA_DETECTOR_PYTHON:-/home/slam/robot_ws/.venv/fod_yolo/bin/python3}}"
NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT="${NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT:-/home/slam/yolo11/yolo11_GAM}"
NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS="${NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS:-/home/slam/yolo11/detect_classify/detect/trash_yolo11s_gam/best.pt}"
NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256="${NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256:-711b6bb4b4debebcf993f033f23e7e641a02dd279254779f8dafed11b6a79233}"
NVIDIA_DETECT_CLASSIFY_DETECTOR_CLASS_NAMES="${NVIDIA_DETECT_CLASSIFY_DETECTOR_CLASS_NAMES:-trash}"
NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS="${NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS:-/home/slam/yolo11/detect_classify/classify/material_yolo11s_cls/best.pt}"
NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256="${NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256:-d0cce9310e184e8acd7a6142face16d39aadc9a6e5405b18694346f2315899e9}"
NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES="${NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES:-metal,plastic,paper,glass,kitchen_waste}"
NVIDIA_FOD_WEIGHTS="$(dual_host_resolve_workspace_path "$NVIDIA_FOD_WEIGHTS")"
NVIDIA_FOD_ULTRALYTICS_ROOT="$(
  dual_host_resolve_workspace_path "$NVIDIA_FOD_ULTRALYTICS_ROOT"
)"
NVIDIA_LOCATEANYTHING_MODEL_ROOT="$(
  dual_host_resolve_workspace_path "$NVIDIA_LOCATEANYTHING_MODEL_ROOT"
)"
NVIDIA_LOCATEANYTHING_MANIFEST="$(
  dual_host_resolve_workspace_path "$NVIDIA_LOCATEANYTHING_MANIFEST"
)"
NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT="$(
  dual_host_resolve_workspace_path "$NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT"
)"
NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS="$(
  dual_host_resolve_workspace_path "$NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS"
)"
NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS="$(
  dual_host_resolve_workspace_path "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS"
)"
NVIDIA_YOLO_MODEL_SHA256="${NVIDIA_YOLO_MODEL_SHA256:-5efaafa1503db11c2ba261b4429389d96335b4eef4d0fc44d6ca41e7431f2d0f}"
NVIDIA_LOCATEANYTHING_MODEL_SHA256="${NVIDIA_LOCATEANYTHING_MODEL_SHA256:-a6a8903c529cd769270599fab141eb84f5d1d09d063fe2d1933ddf4ac8f11a15}"
NVIDIA_YOLO_REQUIRED_CLASS_NAMES="${NVIDIA_YOLO_REQUIRED_CLASS_NAMES:-metal,plastic,paper,glass,kitchen_waste}"
NVIDIA_LOCATEANYTHING_REQUIRED_CLASS_NAMES="${NVIDIA_LOCATEANYTHING_REQUIRED_CLASS_NAMES:-trash}"
case "$NVIDIA_FOD_BACKEND" in
  yolo)
    NVIDIA_FOD_MODEL_SHA256="$NVIDIA_YOLO_MODEL_SHA256"
    NVIDIA_FOD_REQUIRED_CLASS_NAMES="$NVIDIA_YOLO_REQUIRED_CLASS_NAMES"
    ;;
  locateanything)
    NVIDIA_FOD_MODEL_SHA256="$NVIDIA_LOCATEANYTHING_MODEL_SHA256"
    NVIDIA_FOD_REQUIRED_CLASS_NAMES="$NVIDIA_LOCATEANYTHING_REQUIRED_CLASS_NAMES"
    ;;
  detect_and_classify)
    NVIDIA_FOD_WEIGHTS="$NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS"
    NVIDIA_FOD_ULTRALYTICS_ROOT="$NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT"
    NVIDIA_FOD_MODEL_SHA256="$NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256"
    NVIDIA_FOD_REQUIRED_CLASS_NAMES="$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES"
    ;;
  *)
    NVIDIA_FOD_MODEL_SHA256="${NVIDIA_FOD_MODEL_SHA256:-}"
    NVIDIA_FOD_REQUIRED_CLASS_NAMES="${NVIDIA_FOD_REQUIRED_CLASS_NAMES:-}"
    ;;
esac
NAV_MAX_LINEAR_SPEED="${NAV_MAX_LINEAR_SPEED:-0.80}"
NAV_MAX_REVERSE_SPEED="${NAV_MAX_REVERSE_SPEED:-0.30}"
NAV_MAX_ANGULAR_SPEED="${NAV_MAX_ANGULAR_SPEED:-0.60}"
CMD_VEL_MAX_LINEAR_SPEED="${CMD_VEL_MAX_LINEAR_SPEED:-1.70}"
CMD_VEL_MAX_ANGULAR_SPEED="${CMD_VEL_MAX_ANGULAR_SPEED:-1.00}"
if ! awk -v nav="$NAV_MAX_ANGULAR_SPEED" \
         -v cap="$CMD_VEL_MAX_ANGULAR_SPEED" 'BEGIN {
  exit !(nav ~ /^[0-9]+([.][0-9]+)?$/ && cap ~ /^[0-9]+([.][0-9]+)?$/ &&
         nav > 0.0 && cap > 0.0 && nav <= cap)
}' </dev/null; then
  echo "NAV_MAX_ANGULAR_SPEED must be positive and no greater than CMD_VEL_MAX_ANGULAR_SPEED." >&2
  return 2 2>/dev/null || exit 2
fi
STATIC_MAP_ENABLED="${STATIC_MAP_ENABLED:-false}"
STATIC_MAP_SET="${STATIC_MAP_SET:-}"
STATIC_MAP_SOURCE_MODE="${STATIC_MAP_SOURCE_MODE:-fused}"
STATIC_MAP_FILE="${STATIC_MAP_FILE:-}"
FAST_LIO_MAP_FILE="${FAST_LIO_MAP_FILE:-}"
FAST_LIO_INITIAL_BODY_Z="${FAST_LIO_INITIAL_BODY_Z:-0.0}"
MOTION_ENABLED="${MOTION_ENABLED:-false}"
FOD_MOTION_ENABLED="${FOD_MOTION_ENABLED:-false}"
MAPPING_REQUIRE_DUAL_LIDAR="${MAPPING_REQUIRE_DUAL_LIDAR:-true}"
MAPPING_TOPIC_WAIT_SEC="${MAPPING_TOPIC_WAIT_SEC:-10}"
DUAL_LIDAR_CENTER_DISTANCE_M="${DUAL_LIDAR_CENTER_DISTANCE_M:-0.92}"
export NVIDIA_J6M_INTERFACE NVIDIA_LIVOX_INTERFACE
export NVIDIA_J6M_MAC NVIDIA_LIVOX_MAC
export NVIDIA_J6M_CONFIGURED_MAC NVIDIA_LIVOX_CONFIGURED_MAC
export NVIDIA_J6M_USB_ID NVIDIA_J6M_USB_SERIAL
export NVIDIA_LIVOX_USB_ID NVIDIA_LIVOX_USB_SERIAL
export DUAL_HOST_SYS_CLASS_NET_ROOT DUAL_HOST_DEVICE_WAIT_SEC
export DUAL_HOST_NETWORK_WAIT_SEC DUAL_HOST_NETWORK_POLL_SEC
export NVIDIA_ZED_SERIAL ZED_USB_WAIT_SEC ZED_IMAGE_WAIT_SEC
export NVIDIA_FOD_BACKEND NVIDIA_FOD_WEIGHTS NVIDIA_FOD_ULTRALYTICS_ROOT
export NVIDIA_LOCATEANYTHING_MODEL_ROOT NVIDIA_LOCATEANYTHING_MANIFEST
export NVIDIA_LOCATEANYTHING_WORKER_PYTHON
export NVIDIA_DETECT_CLASSIFY_ULTRALYTICS_ROOT
export NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS
export NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256
export NVIDIA_DETECT_CLASSIFY_DETECTOR_CLASS_NAMES
export NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS
export NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256
export NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES
export NVIDIA_YOLO_MODEL_SHA256 NVIDIA_LOCATEANYTHING_MODEL_SHA256
export NVIDIA_YOLO_REQUIRED_CLASS_NAMES
export NVIDIA_LOCATEANYTHING_REQUIRED_CLASS_NAMES
export NVIDIA_FOD_MODEL_SHA256 NVIDIA_FOD_REQUIRED_CLASS_NAMES
export NAV_MAX_LINEAR_SPEED NAV_MAX_REVERSE_SPEED NAV_MAX_ANGULAR_SPEED
export CMD_VEL_MAX_LINEAR_SPEED
export CMD_VEL_MAX_ANGULAR_SPEED
export STATIC_MAP_ENABLED STATIC_MAP_SET STATIC_MAP_SOURCE_MODE STATIC_MAP_FILE
export FAST_LIO_MAP_FILE FAST_LIO_INITIAL_BODY_Z
export MOTION_ENABLED FOD_MOTION_ENABLED
export MAPPING_REQUIRE_DUAL_LIDAR MAPPING_TOPIC_WAIT_SEC
export DUAL_LIDAR_CENTER_DISTANCE_M

dual_host_normalize_mac() {
  printf '%s\n' "${1,,}"
}

dual_host_interface_permanent_mac() {
  local interface="$1" value
  [[ -n "$interface" ]] || return 1
  if [[ "$DUAL_HOST_SYS_CLASS_NET_ROOT" == /sys/class/net ]] &&
     command -v ethtool >/dev/null 2>&1; then
    value="$(ethtool -P "$interface" 2>/dev/null |
      awk '/Permanent address:/ {print $3; exit}')"
    if [[ "${value,,}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] &&
       [[ "${value,,}" != 00:00:00:00:00:00 ]]; then
      printf '%s\n' "${value,,}"
      return 0
    fi
  fi
  [[ -r "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/address" ]] || return 1
  value="$(<"$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/address")"
  [[ "${value,,}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] || return 1
  printf '%s\n' "${value,,}"
}

dual_host_find_interface_by_mac() {
  local wanted actual interface interface_dir
  local -a matches=()
  wanted="$(dual_host_normalize_mac "${1:-}")"
  [[ "$wanted" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] || return 1
  for interface_dir in "$DUAL_HOST_SYS_CLASS_NET_ROOT"/*; do
    [[ -d "$interface_dir" || -L "$interface_dir" ]] || continue
    interface="${interface_dir##*/}"
    actual="$(dual_host_interface_permanent_mac "$interface" 2>/dev/null || true)"
    [[ "$actual" != "$wanted" ]] || matches+=("$interface")
  done
  (( ${#matches[@]} == 1 )) || return $(( ${#matches[@]} > 1 ? 2 : 1 ))
  printf '%s\n' "${matches[0]}"
}

dual_host_usb_device_path() {
  local interface="$1" wanted_usb_id="${2,,}" path parent vendor product
  [[ "$wanted_usb_id" =~ ^[0-9a-f]{4}:[0-9a-f]{4}$ ]] || return 1
  path="$(readlink -f "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/device" 2>/dev/null || true)"
  [[ -n "$path" ]] || return 1
  while [[ "$path" != / && -n "$path" ]]; do
    if [[ -r "$path/idVendor" && -r "$path/idProduct" ]]; then
      vendor="$(<"$path/idVendor")"
      product="$(<"$path/idProduct")"
      if [[ "${vendor,,}:${product,,}" == "$wanted_usb_id" ]]; then
        printf '%s\n' "$path"
        return 0
      fi
    fi
    parent="${path%/*}"
    [[ "$parent" != "$path" ]] || break
    path="$parent"
  done
  return 1
}

dual_host_interface_usb_serial() {
  local interface="$1" usb_id="$2" path
  path="$(dual_host_usb_device_path "$interface" "$usb_id" 2>/dev/null || true)"
  [[ -n "$path" && -r "$path/serial" ]] || return 1
  tr -d '\r\n' <"$path/serial"
}

dual_host_find_interface_by_usb_identity() {
  local wanted_usb_id="${1,,}" wanted_serial="$2"
  local interface interface_dir actual_serial
  local -a matches=()
  [[ "$wanted_usb_id" =~ ^[0-9a-f]{4}:[0-9a-f]{4}$ &&
     -n "$wanted_serial" ]] || return 1
  for interface_dir in "$DUAL_HOST_SYS_CLASS_NET_ROOT"/*; do
    [[ -d "$interface_dir" || -L "$interface_dir" ]] || continue
    interface="${interface_dir##*/}"
    actual_serial="$(dual_host_interface_usb_serial "$interface" "$wanted_usb_id" 2>/dev/null || true)"
    [[ "$actual_serial" != "$wanted_serial" ]] || matches+=("$interface")
  done
  (( ${#matches[@]} == 1 )) || return $(( ${#matches[@]} > 1 ? 2 : 1 ))
  printf '%s\n' "${matches[0]}"
}

dual_host_resolve_network_role() {
  local prefix="$1" configured_mac="$2" usb_id="$3" usb_serial="$4"
  local interface_variable="${prefix}_INTERFACE"
  local mac_variable="${prefix}_MAC"
  local source_variable="${prefix}_IDENTITY_SOURCE"
  local path_variable="${prefix}_USB_DEVICE_PATH"
  local configured_interface="${!interface_variable:-}"
  local resolved="" resolved_mac="" identity_source="" usb_path=""

  if [[ "${configured_mac,,}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
    resolved="$(dual_host_find_interface_by_mac "$configured_mac" 2>/dev/null || true)"
    [[ -z "$resolved" ]] || identity_source="configured-mac"
  fi
  if [[ -z "$resolved" && "${usb_id,,}" =~ ^[0-9a-f]{4}:[0-9a-f]{4}$ &&
        -n "$usb_serial" ]]; then
    resolved="$(dual_host_find_interface_by_usb_identity "$usb_id" "$usb_serial" 2>/dev/null || true)"
    [[ -z "$resolved" ]] || identity_source="usb-id+serial"
  fi
  if [[ -z "$resolved" && -z "$configured_mac" && -z "$usb_id" &&
        -n "$configured_interface" &&
        ( -d "$DUAL_HOST_SYS_CLASS_NET_ROOT/$configured_interface" ||
          -L "$DUAL_HOST_SYS_CLASS_NET_ROOT/$configured_interface" ) ]]; then
    resolved="$configured_interface"
    identity_source="explicit-interface"
  fi

  if [[ -n "$resolved" ]]; then
    resolved_mac="$(dual_host_interface_permanent_mac "$resolved" 2>/dev/null || true)"
    [[ -n "$resolved_mac" ]] || resolved=""
  fi
  if [[ -n "$resolved" ]]; then
    usb_path="$(dual_host_usb_device_path "$resolved" "$usb_id" 2>/dev/null || true)"
    printf -v "$interface_variable" '%s' "$resolved"
    printf -v "$mac_variable" '%s' "$resolved_mac"
    printf -v "$source_variable" '%s' "$identity_source"
    printf -v "$path_variable" '%s' "$usb_path"
  else
    # A configured hardware identity is authoritative.  Never retain a stale
    # ethN fallback when that identity is absent: ethN names change at reboot.
    printf -v "$interface_variable" '%s' ''
    printf -v "$mac_variable" '%s' "$configured_mac"
    printf -v "$source_variable" '%s' 'unresolved'
    printf -v "$path_variable" '%s' ''
  fi
  export "$interface_variable" "$mac_variable" "$source_variable" "$path_variable"
}

dual_host_network_roles_are_distinct() {
  [[ -n "${NVIDIA_J6M_INTERFACE:-}" && -n "${NVIDIA_LIVOX_INTERFACE:-}" ]] || return 1
  [[ "$NVIDIA_J6M_INTERFACE" != "$NVIDIA_LIVOX_INTERFACE" ]] || return 1
  if [[ -n "${NVIDIA_J6M_USB_DEVICE_PATH:-}" &&
        -n "${NVIDIA_LIVOX_USB_DEVICE_PATH:-}" ]]; then
    [[ "$NVIDIA_J6M_USB_DEVICE_PATH" != "$NVIDIA_LIVOX_USB_DEVICE_PATH" ]] || return 1
  fi
}

dual_host_refresh_local_interfaces() {
  dual_host_resolve_network_role NVIDIA_J6M "$NVIDIA_J6M_CONFIGURED_MAC" \
    "$NVIDIA_J6M_USB_ID" "$NVIDIA_J6M_USB_SERIAL"
  dual_host_resolve_network_role NVIDIA_LIVOX "$NVIDIA_LIVOX_CONFIGURED_MAC" \
    "$NVIDIA_LIVOX_USB_ID" "$NVIDIA_LIVOX_USB_SERIAL"
  return 0
}

dual_host_refresh_local_interfaces

export DUAL_HOST_WS DUAL_HOST_CONFIG
export ROS_MASTER_URI="http://${J6M_IP}:11311"
export ROS_IP="$NVIDIA_J6M_IP"
unset ROS_HOSTNAME

dual_host_is_bool() {
  [[ "$1" == true || "$1" == false ]]
}

dual_host_validate_fod_model_contract() {
  case "$NVIDIA_FOD_BACKEND" in
    yolo|locateanything|detect_and_classify) ;;
    *)
      echo "NVIDIA_FOD_BACKEND must be yolo, locateanything, or detect_and_classify." >&2
      return 1
      ;;
  esac
  if [[ ! "$NVIDIA_FOD_MODEL_SHA256" =~ ^[[:xdigit:]]{64}$ ]]; then
    echo "NVIDIA_FOD_MODEL_SHA256 must be exactly 64 hexadecimal characters." >&2
    return 1
  fi
  if ! awk -v value="$NVIDIA_FOD_REQUIRED_CLASS_NAMES" 'BEGIN {
    count = split(value, items, ",")
    if (count < 1) exit 1
    for (item_index = 1; item_index <= count; ++item_index) {
      name = items[item_index]
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", name)
      if (name == "" || seen[name]++) exit 1
    }
  }' </dev/null; then
    echo "NVIDIA_FOD_REQUIRED_CLASS_NAMES must be a non-empty, unique comma list." >&2
    return 1
  fi
  if [[ "$NVIDIA_FOD_BACKEND" == detect_and_classify ]]; then
    if [[ "$NVIDIA_DETECT_CLASSIFY_DETECTOR_CLASS_NAMES" != trash ]]; then
      echo "The two-stage detector class contract must be exactly trash." >&2
      return 1
    fi
    if [[ "$NVIDIA_FOD_REQUIRED_CLASS_NAMES" != \
          "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES" ]]; then
      echo "The active two-stage material class order does not match its classifier contract." >&2
      return 1
    fi
  fi
}

dual_host_validate_fod_weights() {
  local weights="${NVIDIA_FOD_WEIGHTS:-}" actual_sha256
  if [[ "$NVIDIA_FOD_BACKEND" == locateanything ]]; then
    if [[ ! -x "$NVIDIA_LOCATEANYTHING_WORKER_PYTHON" ]]; then
      echo "LocateAnything worker Python is not executable: $NVIDIA_LOCATEANYTHING_WORKER_PYTHON" >&2
      return 1
    fi
    LOCATEANYTHING_MODEL_ROOT="$NVIDIA_LOCATEANYTHING_MODEL_ROOT" \
      LOCATEANYTHING_MANIFEST="$NVIDIA_LOCATEANYTHING_MANIFEST" \
      LOCATEANYTHING_EXPECTED_SHA256="${NVIDIA_FOD_MODEL_SHA256,,}" \
      python3 - <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys

root = Path(os.environ["LOCATEANYTHING_MODEL_ROOT"]).resolve()
manifest = Path(os.environ["LOCATEANYTHING_MANIFEST"]).resolve()
expected_manifest_sha = os.environ["LOCATEANYTHING_EXPECTED_SHA256"]

def fail(message):
    print(message, file=sys.stderr)
    raise SystemExit(1)

if not root.is_dir():
    fail("LocateAnything model root is missing: {}".format(root))
if not manifest.is_file():
    fail("LocateAnything deployment manifest is missing: {}".format(manifest))
try:
    manifest.relative_to(root)
except ValueError:
    fail("LocateAnything deployment manifest must be inside the model root")

payload = manifest.read_bytes()
actual_manifest_sha = hashlib.sha256(payload).hexdigest()
if actual_manifest_sha != expected_manifest_sha:
    fail(
        "LocateAnything manifest SHA256 mismatch: expected {}, got {}".format(
            expected_manifest_sha, actual_manifest_sha
        )
    )
try:
    data = json.loads(payload.decode("utf-8"))
except (UnicodeDecodeError, json.JSONDecodeError) as exc:
    fail("LocateAnything deployment manifest is invalid: {}".format(exc))
if not isinstance(data, dict) or data.get("schema_version") != 1:
    fail("LocateAnything deployment manifest schema must be 1")
if data.get("repo_id") != "nvidia/LocateAnything-3B":
    fail("LocateAnything deployment manifest has the wrong repo_id")
revision = str(data.get("revision", ""))
if len(revision) != 40 or any(c not in "0123456789abcdef" for c in revision):
    fail("LocateAnything deployment manifest has an invalid revision")
entries = data.get("files")
if not isinstance(entries, list) or not entries:
    fail("LocateAnything deployment manifest has no files")

seen = set()
for entry in entries:
    if not isinstance(entry, dict):
        fail("LocateAnything deployment manifest has a non-object file entry")
    relative = str(entry.get("path", ""))
    expected_sha = str(entry.get("sha256", "")).lower()
    expected_size = entry.get("size")
    if not relative or relative in seen or Path(relative).is_absolute() or ".." in Path(relative).parts:
        fail("LocateAnything deployment manifest has an unsafe or duplicate path: {}".format(relative))
    if len(expected_sha) != 64 or any(c not in "0123456789abcdef" for c in expected_sha):
        fail("LocateAnything deployment manifest has an invalid SHA256 for {}".format(relative))
    if not isinstance(expected_size, int) or expected_size < 1:
        fail("LocateAnything deployment manifest has an invalid size for {}".format(relative))
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError:
        fail("LocateAnything model file escapes the model root: {}".format(relative))
    if not path.is_file() or path.stat().st_size != expected_size:
        fail("LocateAnything model file is missing or has the wrong size: {}".format(path))
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(4 * 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    if digest.hexdigest() != expected_sha:
        fail("LocateAnything model file SHA256 mismatch: {}".format(path))
    seen.add(relative)

required = {
    "config.json", "model.safetensors.index.json",
    "model-00001-of-00002.safetensors", "model-00002-of-00002.safetensors",
    "tokenizer_config.json", "preprocessor_config.json", "processor_config.json",
    "modeling_locateanything.py", "processing_locateanything.py",
}
missing = sorted(required - seen)
if missing:
    fail("LocateAnything deployment manifest omits: {}".format(", ".join(missing)))
PY
    return $?
  fi
  if [[ "$NVIDIA_FOD_BACKEND" == detect_and_classify ]]; then
    local detector_actual classifier_actual
    if [[ ! -r "$NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS" ||
          ! -r "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS" ]]; then
      echo "Two-stage detector or classifier weights are missing." >&2
      return 1
    fi
    detector_actual="$(sha256sum -- "$NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS" | awk '{print $1}')" || return 1
    classifier_actual="$(sha256sum -- "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS" | awk '{print $1}')" || return 1
    if [[ "${detector_actual,,}" != "${NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256,,}" ]]; then
      echo "Two-stage detector SHA256 mismatch: expected $NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256, got $detector_actual." >&2
      return 1
    fi
    if [[ "${classifier_actual,,}" != "${NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256,,}" ]]; then
      echo "Two-stage classifier SHA256 mismatch: expected $NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256, got $classifier_actual." >&2
      return 1
    fi
    return 0
  fi
  if [[ -z "$weights" || ! -r "$weights" ]]; then
    echo "NVIDIA_FOD_WEIGHTS is missing or unreadable: ${weights:-<empty>}" >&2
    return 1
  fi
  actual_sha256="$(sha256sum -- "$weights" 2>/dev/null | awk '{print $1}')" || {
    echo "Unable to calculate SHA256 for $weights." >&2
    return 1
  }
  if [[ "${actual_sha256,,}" != "${NVIDIA_FOD_MODEL_SHA256,,}" ]]; then
    echo "FOD weights SHA256 mismatch: expected $NVIDIA_FOD_MODEL_SHA256, got $actual_sha256." >&2
    return 1
  fi
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
