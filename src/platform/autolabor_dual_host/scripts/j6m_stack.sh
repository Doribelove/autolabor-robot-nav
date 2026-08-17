#!/usr/bin/env bash
set -eo pipefail

: "${ROS_MASTER_URI:?ROS_MASTER_URI must point to the J6M master}"
: "${ROS_IP:?ROS_IP must be the J6M Ethernet address}"

if [[ -n "${ROS_HOSTNAME:-}" ]]; then
  echo "ROS_HOSTNAME must be unset in the dual-host runtime." >&2
  exit 2
fi

source "${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
source "${BASE_WORKSPACE_SETUP:-/opt/autolabor/ros/install/setup.bash}"
source "${DUAL_HOST_SETUP:-/opt/autolabor/dual_host/current/setup.bash}"
set -u

ROS_MASTER_PORT="${ROS_MASTER_PORT:-11311}"
WAIT_FOR_NVIDIA_SEC="${WAIT_FOR_NVIDIA_SEC:-300}"
REQUIRE_CAN="${REQUIRE_CAN:-true}"
USE_DUAL_LIDAR="${USE_DUAL_LIDAR:-true}"
FOD_MOTION_ENABLED="${FOD_MOTION_ENABLED:-false}"
NAV_MAX_LINEAR_SPEED="${NAV_MAX_LINEAR_SPEED:-0.30}"
NAV_MAX_REVERSE_SPEED="${NAV_MAX_REVERSE_SPEED:-0.30}"
MID360_SENSOR_X="${MID360_SENSOR_X:-0.20}"
MID360_SENSOR_Y="${MID360_SENSOR_Y:-0.0}"
MID360_SENSOR_Z="${MID360_SENSOR_Z:-0.9}"
MID360_CROP_ENABLED="${MID360_CROP_ENABLED:-true}"
MID360_CROP_MIN_X="${MID360_CROP_MIN_X:--0.75}"
MID360_CROP_MAX_X="${MID360_CROP_MAX_X:-0.75}"
MID360_CROP_MIN_Y="${MID360_CROP_MIN_Y:--0.50}"
MID360_CROP_MAX_Y="${MID360_CROP_MAX_Y:-0.50}"

case "$REQUIRE_CAN:$USE_DUAL_LIDAR:$FOD_MOTION_ENABLED" in
  true:true:true|true:true:false|true:false:true|true:false:false|false:true:true|false:true:false|false:false:true|false:false:false) ;;
  *) echo "Boolean environment values must be literal true or false." >&2; exit 2 ;;
esac

PIDS=()
STARTED_MASTER=false
cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if ! timeout 2 rosparam list >/dev/null 2>&1; then
  roscore -p "$ROS_MASTER_PORT" &
  PIDS+=("$!")
  STARTED_MASTER=true
fi

deadline=$((SECONDS + 20))
until timeout 2 rosparam list >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "ROS master did not become ready at $ROS_MASTER_URI." >&2
    exit 4
  fi
  sleep 0.2
done
echo "J6M ROS master is ready at $ROS_MASTER_URI (started_here=$STARTED_MASTER)."

node_exists() {
  rosnode list 2>/dev/null | grep -Fxq "$1"
}

topic_has_publisher() {
  local topic="$1"
  timeout 3 rostopic info "$topic" 2>/dev/null |
    awk '/^Publishers:/{getline; exit !($1 == "*")}'
}

gateway_ready() {
  node_exists /nvidia_cmd_vel_watchdog || return 1
  node_exists /livox_lidar_publisher2 || return 1
  topic_has_publisher /gateway/livox/lidar || return 1
  topic_has_publisher /gateway/livox/imu || return 1
  if [[ "$REQUIRE_CAN" == true ]]; then
    node_exists /m2_driver || return 1
    node_exists /canbus_driver || return 1
  fi
}

deadline=$((SECONDS + WAIT_FOR_NVIDIA_SEC))
echo "Waiting up to ${WAIT_FOR_NVIDIA_SEC}s for the NVIDIA sensor/actuator gateway..."
until gateway_ready; do
  if (( SECONDS >= deadline )); then
    echo "NVIDIA gateway readiness timed out; navigation was not started." >&2
    echo "Required: watchdog, Livox lidar+IMU, and REQUIRE_CAN=$REQUIRE_CAN." >&2
    exit 5
  fi
  sleep 1
done

echo "NVIDIA gateway is ready; starting J6M FAST-LIO navigation."
roslaunch autolabor_dual_host j6m_fastlio_navigation.launch \
  use_dual_lidar:="$USE_DUAL_LIDAR" \
  fod_motion_enabled:="$FOD_MOTION_ENABLED" \
  max_linear_speed:="$NAV_MAX_LINEAR_SPEED" \
  max_linear_speed_backwards:="$NAV_MAX_REVERSE_SPEED" \
  mid360_sensor_x:="$MID360_SENSOR_X" \
  mid360_sensor_y:="$MID360_SENSOR_Y" \
  mid360_sensor_z:="$MID360_SENSOR_Z" \
  mid360_crop_enabled:="$MID360_CROP_ENABLED" \
  mid360_crop_min_x:="$MID360_CROP_MIN_X" \
  mid360_crop_max_x:="$MID360_CROP_MAX_X" \
  mid360_crop_min_y:="$MID360_CROP_MIN_Y" \
  mid360_crop_max_y:="$MID360_CROP_MAX_Y" &
PIDS+=("$!")
wait "${PIDS[-1]}"
