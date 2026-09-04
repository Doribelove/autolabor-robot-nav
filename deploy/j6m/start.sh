#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/process_control.sh"

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/autolabor_runtime}"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"
ENV_FILE="${DUAL_HOST_ENV_FILE:-$RUNTIME_BASE/dual_host/config/dual_host.env}"
PID_FILE="$RUNTIME_BASE/dual_host/run/j6m_stack.pid"
LAUNCHER_PID_FILE="$RUNTIME_BASE/dual_host/run/j6m_launcher.pid"
LAUNCHER_PATTERN="(^|[[:space:]])${RUNTIME_BASE}/dual_host/bin/start\\.sh([[:space:]]|$)"

[[ "$(id -u)" == 0 ]] || { echo "start.sh must run as root on J6M." >&2; exit 2; }
[[ -r "$ENV_FILE" ]] || { echo "Missing J6M dual-host config: $ENV_FILE" >&2; exit 2; }
requested_static_map_enabled="${STATIC_MAP_ENABLED:-}"
requested_static_map_file="${STATIC_MAP_FILE:-}"
requested_fast_lio_map_file="${FAST_LIO_MAP_FILE:-}"
requested_fast_lio_initial_body_z="${FAST_LIO_INITIAL_BODY_Z:-}"
requested_fod_motion_enabled="${FOD_MOTION_ENABLED:-}"
requested_fod_model_sha256="${NVIDIA_FOD_MODEL_SHA256:-}"
requested_fod_required_class_names="${NVIDIA_FOD_REQUIRED_CLASS_NAMES:-}"
set -a
source "$ENV_FILE"
set +a
MID360_SENSOR_X="${MID360_SENSOR_X:-0.20}"
MID360_SENSOR_Y="${MID360_SENSOR_Y:-0.0}"
MID360_SENSOR_Z="${MID360_SENSOR_Z:-0.9}"
MID360_CROP_ENABLED="${MID360_CROP_ENABLED:-true}"
MID360_CROP_MIN_X="${MID360_CROP_MIN_X:--0.75}"
MID360_CROP_MAX_X="${MID360_CROP_MAX_X:-0.75}"
MID360_CROP_MIN_Y="${MID360_CROP_MIN_Y:--0.50}"
MID360_CROP_MAX_Y="${MID360_CROP_MAX_Y:-0.50}"
STATIC_MAP_ENABLED="${requested_static_map_enabled:-false}"
STATIC_MAP_FILE="${requested_static_map_file:-}"
FAST_LIO_MAP_FILE="${requested_fast_lio_map_file:-}"
FAST_LIO_INITIAL_BODY_Z="${requested_fast_lio_initial_body_z:-0.0}"
FOD_MOTION_ENABLED="${requested_fod_motion_enabled:-${FOD_MOTION_ENABLED:-false}}"
NVIDIA_FOD_MODEL_SHA256="${requested_fod_model_sha256:-${NVIDIA_FOD_MODEL_SHA256-7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500}}"
NVIDIA_FOD_REQUIRED_CLASS_NAMES="${requested_fod_required_class_names:-${NVIDIA_FOD_REQUIRED_CLASS_NAMES-Metal,Soft,Plastic,Wire,Tool,w}}"
NAV_MAX_LINEAR_SPEED="${NAV_MAX_LINEAR_SPEED:-0.80}"
NAV_MAX_REVERSE_SPEED="${NAV_MAX_REVERSE_SPEED:-0.30}"
NAV_MAX_ANGULAR_SPEED="${NAV_MAX_ANGULAR_SPEED:-0.60}"
CMD_VEL_MAX_ANGULAR_SPEED="${CMD_VEL_MAX_ANGULAR_SPEED:-1.00}"

[[ -x "$ROOTFS/bin/bash" ]] || { echo "Invalid rootfs: $ROOTFS" >&2; exit 2; }
chroot "$ROOTFS" /usr/bin/test -r /opt/autolabor/dual_host/current/setup.bash || {
  echo "Dual-host overlay is not deployed in the J6M rootfs." >&2
  exit 2
}
if [[ "$STATIC_MAP_ENABLED" != true && "$STATIC_MAP_ENABLED" != false ]]; then
  echo "STATIC_MAP_ENABLED must be literal true or false." >&2
  exit 2
fi
if [[ "$FOD_MOTION_ENABLED" != true && "$FOD_MOTION_ENABLED" != false ]]; then
  echo "FOD_MOTION_ENABLED must be literal true or false." >&2
  exit 2
fi
if ! ip -o -4 address show dev "$J6M_INTERFACE" |
    awk '{print $4}' | grep -Fxq "$J6M_IP/24"; then
  echo "J6M $J6M_INTERFACE does not have $J6M_IP/24; complete the NVIDIA network cutover first." >&2
  exit 3
fi
if ! ping -I "$J6M_INTERFACE" -c 3 -W 1 "$NVIDIA_J6M_IP" >/dev/null; then
  echo "NVIDIA $NVIDIA_J6M_IP is unreachable over $J6M_INTERFACE." >&2
  exit 3
fi
if (( $(date +%s) < 1767225600 )); then
  echo "J6M clock is invalid: $(date -Is). Run sync_j6m_time.sh on NVIDIA." >&2
  exit 3
fi

mkdir -p "$RUNTIME_BASE/dual_host/run" "$RUNTIME_BASE/logs/dual_host"
if dual_host_pid_file_is_owned "$LAUNCHER_PID_FILE" "$LAUNCHER_PATTERN"; then
  old_pid="$(dual_host_pid_file_pid "$LAUNCHER_PID_FILE")"
  echo "J6M dual-host launcher is already running (PID $old_pid)." >&2
  exit 4
fi
if dual_host_pid_file_is_owned "$PID_FILE" '(^|/)j6m_stack\.sh([[:space:]]|$)'; then
  old_pid="$(dual_host_pid_file_pid "$PID_FILE")"
  echo "J6M dual-host stack is already running (PID $old_pid)." >&2
  exit 4
fi

"$RUNTIME_BASE/bin/mount_chroot.sh" >/dev/null
stamp="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUNTIME_BASE/logs/dual_host/$stamp"
child_pid=""
launcher_pid_line=""

cleanup() {
  trap - EXIT INT TERM
  dual_host_stop_pid_file "$PID_FILE" "J6M dual-host stack" \
    '(^|/)j6m_stack\.sh([[:space:]]|$)' || true
  if [[ -n "$child_pid" ]]; then wait "$child_pid" 2>/dev/null || true; fi
  for unmount_attempt in $(seq 1 25); do
    "$RUNTIME_BASE/bin/unmount_chroot.sh" >/dev/null 2>&1 || true
    ! mountpoint -q "$ROOTFS/proc" && break
    sleep 0.2
  done
  dual_host_remove_pid_file_if_unchanged "$LAUNCHER_PID_FILE" "$launcher_pid_line"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

dual_host_write_pid_file "$LAUNCHER_PID_FILE" "$$"
launcher_pid_line="$(sed -n '1p' "$LAUNCHER_PID_FILE")"

chroot "$ROOTFS" /usr/bin/env -i \
  HOME=/root \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH=/opt/autolabor/dual_host/current/bin:/opt/autolabor/ros/install/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ROS_MASTER_URI="http://${J6M_IP}:11311" \
  ROS_IP="$J6M_IP" \
  ROS_MASTER_PORT=11311 \
  ROS_HOME=/var/lib/autolabor/ros-home \
  ROS_LOG_DIR="/var/log/autolabor/dual_host/$stamp" \
  ROS_SETUP=/opt/ros/noetic/setup.bash \
  BASE_WORKSPACE_SETUP=/opt/autolabor/ros/install/setup.bash \
  DUAL_HOST_SETUP=/opt/autolabor/dual_host/current/setup.bash \
  WAIT_FOR_NVIDIA_SEC="$WAIT_FOR_NVIDIA_SEC" \
  REQUIRE_CAN="$REQUIRE_CAN" \
  USE_DUAL_LIDAR="$USE_DUAL_LIDAR" \
  FOD_MOTION_ENABLED="$FOD_MOTION_ENABLED" \
  NVIDIA_FOD_MODEL_SHA256="$NVIDIA_FOD_MODEL_SHA256" \
  NVIDIA_FOD_REQUIRED_CLASS_NAMES="$NVIDIA_FOD_REQUIRED_CLASS_NAMES" \
  NAV_MAX_LINEAR_SPEED="$NAV_MAX_LINEAR_SPEED" \
  NAV_MAX_REVERSE_SPEED="$NAV_MAX_REVERSE_SPEED" \
  NAV_MAX_ANGULAR_SPEED="$NAV_MAX_ANGULAR_SPEED" \
  CMD_VEL_MAX_ANGULAR_SPEED="$CMD_VEL_MAX_ANGULAR_SPEED" \
  MID360_SENSOR_X="$MID360_SENSOR_X" \
  MID360_SENSOR_Y="$MID360_SENSOR_Y" \
  MID360_SENSOR_Z="$MID360_SENSOR_Z" \
  MID360_CROP_ENABLED="$MID360_CROP_ENABLED" \
  MID360_CROP_MIN_X="$MID360_CROP_MIN_X" \
  MID360_CROP_MAX_X="$MID360_CROP_MAX_X" \
  MID360_CROP_MIN_Y="$MID360_CROP_MIN_Y" \
  MID360_CROP_MAX_Y="$MID360_CROP_MAX_Y" \
  STATIC_MAP_ENABLED="$STATIC_MAP_ENABLED" \
  STATIC_MAP_FILE="$STATIC_MAP_FILE" \
  FAST_LIO_MAP_FILE="$FAST_LIO_MAP_FILE" \
  FAST_LIO_INITIAL_BODY_Z="$FAST_LIO_INITIAL_BODY_Z" \
  /bin/bash -lc 'exec /opt/autolabor/dual_host/current/lib/autolabor_dual_host/j6m_stack.sh' \
  >"$RUNTIME_BASE/logs/dual_host/$stamp/console.log" 2>&1 &
child_pid=$!
dual_host_write_pid_file "$PID_FILE" "$child_pid"
echo "J6M master/waiter started (PID $child_pid)."
echo "Now start scripts/nvidia_gateway.sh on NVIDIA."
echo "Log: $RUNTIME_BASE/logs/dual_host/$stamp/console.log"
wait "$child_pid"
