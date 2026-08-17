#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${WS_SETUP:-$ROBOT_WS/devel/setup.bash}"
PRIVATE_SETUP="${PRIVATE_SETUP:-$ROBOT_WS/.deps/setup.bash}"

OPERATOR_NAV_MODE="${OPERATOR_NAV_MODE:-gps}"
NAV_MAX_SPEED="${1:-0.3}"
GPS_TEB_PROFILE="${2:-cruise}"
OPERATOR_START_VISION="${OPERATOR_START_VISION:-true}"
OPERATOR_START_CAMERA="${OPERATOR_START_CAMERA:-true}"
OPERATOR_IMAGE_QUALITY_CONTROL="${OPERATOR_IMAGE_QUALITY_CONTROL:-false}"
OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT="${OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT:-100}"
OPERATOR_DETECTOR_PYTHON="${OPERATOR_DETECTOR_PYTHON:-$ROBOT_WS/.venv/fod_yolo/bin/python3}"
GPS_GEOFENCE_FILE="${GPS_GEOFENCE_FILE:-$ROBOT_WS/src/scripts/robot_bringup/config/gps_geofences.yaml}"

BRINGUP_PID=""
BRINGUP_TAIL_PID=""
ROSCORE_PID=""
VISION_PID=""
GUI_PID=""
CLEANUP_STARTED=0
ROS_MASTER_PREEXISTED=false
NAV_DISPLAY_NAME=""
ODOM_TOPIC=""
LOG_STEM=""
BRINGUP_READY_TEXT=""

usage() {
  echo "Usage:"
  echo "  $SCRIPT_DIR/operator_all_in_one.sh [gps_max_speed_mps] [cruise|obstacle]"
  echo "  $SCRIPT_DIR/operator_fast_lio_all_in_one.sh [fast_lio_max_speed_mps]"
  echo
  echo "Defaults:"
  echo "  GPS:      0.3 m/s, cruise"
  echo "  FAST_LIO: 0.3 m/s"
  echo
  echo "Optional environment switches:"
  echo "  OPERATOR_START_VISION=true|false"
  echo "  OPERATOR_START_CAMERA=true|false"
  echo "  OPERATOR_IMAGE_QUALITY_CONTROL=true|false"
  echo "  OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT=100"
  echo "  OPERATOR_DETECTOR_PYTHON=$ROBOT_WS/.venv/fod_yolo/bin/python3"
  echo
  echo "Qt opens as soon as a shared ROS master is available. Navigation and"
  echo "perception continue starting in the background, and unavailable motion"
  echo "controls remain blocked by the GUI readiness gates."
  echo
  echo "Closing the Qt window or pressing Ctrl+C stops every process started here."
}

valid_boolean() {
  [[ "$1" == "true" || "$1" == "false" ]]
}

process_is_running() {
  local pid="$1"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" >/dev/null 2>&1 || return 1
  local state
  state="$(ps -o stat= -p "$pid" 2>/dev/null | awk '{print $1}')"
  [[ "$state" != Z* ]]
}

stop_process() {
  local pid="$1"
  local label="$2"
  local attempts="${3:-80}"
  [[ -n "$pid" ]] || return 0
  if ! process_is_running "$pid"; then
    wait "$pid" >/dev/null 2>&1 || true
    return 0
  fi
  echo "==> stopping $label"
  kill -TERM "$pid" >/dev/null 2>&1 || true
  local attempt
  for ((attempt = 0; attempt < attempts; ++attempt)); do
    if ! process_is_running "$pid"; then
      wait "$pid" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 0.1
  done
  echo "==> $label did not stop in time; forcing its launcher process to exit" >&2
  kill -KILL "$pid" >/dev/null 2>&1 || true
  wait "$pid" >/dev/null 2>&1 || true
}

ensure_ros_master() {
  # bringup.sh owns and stops a master that it created. Its process can remain
  # reachable for a few scheduler ticks after the wrapper itself exits, so do
  # not mistake that transient master for a stable external one.
  if [[ "$ROS_MASTER_PREEXISTED" == "false" ]]; then
    local shutdown_attempt
    for ((shutdown_attempt = 0; shutdown_attempt < 20; ++shutdown_attempt)); do
      if ! rosparam list >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
  fi

  if rosparam list >/dev/null 2>&1; then
    echo "==> an existing ROS master is available for the integrated console"
    return 0
  fi

  echo "==> starting a shared ROS master for navigation and the Qt status console"
  roscore >"$ROSCORE_LOG" 2>&1 &
  ROSCORE_PID=$!

  local attempt
  for ((attempt = 0; attempt < 40; ++attempt)); do
    if rosparam list >/dev/null 2>&1; then
      echo "==> fallback ROS master is ready"
      return 0
    fi
    if ! process_is_running "$ROSCORE_PID"; then
      wait "$ROSCORE_PID" >/dev/null 2>&1 || true
      ROSCORE_PID=""
      break
    fi
    sleep 0.25
  done

  echo "WARNING: no ROS master is reachable; Qt will still be attempted." >&2
  echo "Inspect $ROSCORE_LOG" >&2
  return 1
}

cleanup() {
  if (( CLEANUP_STARTED )); then
    return
  fi
  CLEANUP_STARTED=1
  trap - EXIT INT TERM

  # Stop navigation first so every velocity publisher disappears before the
  # CAN/M2 driver is closed. Sidecars cannot move the chassis after that route
  # has been torn down.
  stop_process "$BRINGUP_PID" "$NAV_DISPLAY_NAME navigation bringup" 120
  stop_process "$VISION_PID" "camera and YOLO11 launch" 80
  stop_process "$GUI_PID" "Qt operator console" 30
  stop_process "$BRINGUP_TAIL_PID" "bringup log follower" 10
  stop_process "$ROSCORE_PID" "fallback ROS master" 50
}

trap cleanup EXIT
trap 'exit 130' INT TERM

case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac

case "$OPERATOR_NAV_MODE" in
  gps)
    NAV_DISPLAY_NAME="GPS"
    ODOM_TOPIC="/gps/odom"
    LOG_STEM="operator_all_in_one"
    if (( $# > 2 )); then
      usage >&2
      exit 1
    fi
    case "$GPS_TEB_PROFILE" in
      cruise|obstacle) ;;
      *)
        echo "Invalid GPS profile: $GPS_TEB_PROFILE (use cruise or obstacle)" >&2
        exit 1
        ;;
    esac
    ;;
  fast_lio)
    NAV_DISPLAY_NAME="FAST_LIO"
    ODOM_TOPIC="/Odometry"
    LOG_STEM="operator_fast_lio_all_in_one"
    if (( $# > 1 )); then
      echo "FAST_LIO mode accepts only one optional speed argument." >&2
      usage >&2
      exit 1
    fi
    ;;
  *)
    echo "Invalid OPERATOR_NAV_MODE=$OPERATOR_NAV_MODE (use gps or fast_lio)." >&2
    exit 1
    ;;
esac

BRINGUP_READY_TEXT="Robot bringup is running in $OPERATOR_NAV_MODE mode."

if [[ ! "$NAV_MAX_SPEED" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v value="$NAV_MAX_SPEED" 'BEGIN { exit !(value > 0.0) }'; then
  echo "Invalid positive $NAV_DISPLAY_NAME speed: $NAV_MAX_SPEED" >&2
  exit 1
fi
for value_name in \
  OPERATOR_START_VISION \
  OPERATOR_START_CAMERA \
  OPERATOR_IMAGE_QUALITY_CONTROL; do
  if ! valid_boolean "${!value_name}"; then
    echo "Invalid $value_name=${!value_name}; use true or false." >&2
    exit 1
  fi
done
if [[ ! "$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v value="$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT" \
     'BEGIN { exit !(value >= 1.0 && value <= 100.0) }'; then
  echo "Invalid OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT=$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT (use 1..100)" >&2
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 2
fi
if [[ ! -f "$WS_SETUP" ]]; then
  echo "Missing workspace setup: $WS_SETUP" >&2
  echo "Build the workspace before using the all-in-one launcher." >&2
  exit 2
fi
if [[ "$OPERATOR_START_VISION" == "true" && ! -x "$OPERATOR_DETECTOR_PYTHON" ]]; then
  echo "WARNING: missing YOLO Python environment: $OPERATOR_DETECTOR_PYTHON" >&2
  echo "The visual sidecar will be skipped, but Qt will still open." >&2
  echo "Run $ROBOT_WS/scripts/setup_fod_yolo_env.sh before using visual mode." >&2
  OPERATOR_START_VISION=false
fi

source "$ROS_SETUP"
source "$WS_SETUP"
if [[ -f "$PRIVATE_SETUP" ]]; then
  # Load the user-space dependency prefixes last. Re-sourcing the workspace
  # after this would drop packages such as pointcloud_to_laserscan from
  # ROS_PACKAGE_PATH.
  source "$PRIVATE_SETUP"
fi

if rosparam list >/dev/null 2>&1; then
  ROS_MASTER_PREEXISTED=true
fi

LOG_DIR="${OPERATOR_LOG_DIR:-$ROBOT_WS/log/${LOG_STEM}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
BRINGUP_LOG="$LOG_DIR/bringup.log"
VISION_LOG="$LOG_DIR/vision.log"
GUI_LOG="$LOG_DIR/gui.log"
ROSCORE_LOG="$LOG_DIR/fallback_roscore.log"
touch "$BRINGUP_LOG"

echo "==> Autolabor 一体化操作台"
if [[ "$OPERATOR_NAV_MODE" == "gps" ]]; then
  echo "==> GPS: max_speed=$NAV_MAX_SPEED m/s, profile=$GPS_TEB_PROFILE"
else
  echo "==> FAST_LIO: max_speed=$NAV_MAX_SPEED m/s, localization=/Odometry"
fi
echo "==> Runtime logs: $LOG_DIR"
if ! ensure_ros_master; then
  echo "Unable to start the shared ROS master. Inspect $ROSCORE_LOG" >&2
  exit 3
fi

echo "==> starting $NAV_DISPLAY_NAME navigation without standalone RViz"
if [[ "$OPERATOR_NAV_MODE" == "gps" ]]; then
  TERMINAL_MODE=same NAV_START_RVIZ=false GPS_GEOFENCE_FILE="$GPS_GEOFENCE_FILE" \
    "$SCRIPT_DIR/bringup.sh" gps "$NAV_MAX_SPEED" "$GPS_TEB_PROFILE" \
    >"$BRINGUP_LOG" 2>&1 &
else
  TERMINAL_MODE=same NAV_START_RVIZ=false \
    FAST_LIO_NAV_MAX_VEL_X="$NAV_MAX_SPEED" \
    FAST_LIO_NAV_MAX_VEL_X_BACKWARDS="$NAV_MAX_SPEED" \
    "$SCRIPT_DIR/bringup.sh" fast_lio \
    >"$BRINGUP_LOG" 2>&1 &
fi
BRINGUP_PID=$!
tail -n +1 -F --pid="$BRINGUP_PID" "$BRINGUP_LOG" &
BRINGUP_TAIL_PID=$!

if [[ "$OPERATOR_START_VISION" == "true" ]]; then
  echo "==> starting ZED 2 camera and YOLO11 perception sidecar"
  roslaunch autolabor_fod_vision zed_fod_detection.launch \
    start_camera:="$OPERATOR_START_CAMERA" \
    enable_image_quality_controller:="$OPERATOR_IMAGE_QUALITY_CONTROL" \
    image_quality_exposure_max_percent:="$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT" \
    detector_python:="$OPERATOR_DETECTOR_PYTHON" \
    >"$VISION_LOG" 2>&1 &
  VISION_PID=$!
else
  echo "==> camera/YOLO11 autostart disabled; the Qt visual page will remain available"
fi

echo "==> opening the integrated Qt console while navigation initializes"
GUI_ARGS=(
  navigation_mode_label:="$NAV_DISPLAY_NAME"
  odom_topic:="$ODOM_TOPIC"
  cloud_topic:=/cloud_registered_body
  imu_topic:=/livox/imu
)
"$SCRIPT_DIR/operator_gui.sh" "${GUI_ARGS[@]}" >"$GUI_LOG" 2>&1 &
GUI_PID=$!

echo "==> waiting for the complete navigation readiness gate in the background"
while ! grep -Fq "$BRINGUP_READY_TEXT" "$BRINGUP_LOG"; do
  if ! process_is_running "$GUI_PID"; then
    gui_status=0
    wait "$GUI_PID" || gui_status=$?
    GUI_PID=""
    if (( gui_status != 0 )); then
      echo "Qt operator console exited with code $gui_status. Inspect $GUI_LOG" >&2
    else
      echo "Qt operator console closed before navigation became ready; stopping the stack."
    fi
    exit "$gui_status"
  fi

  if ! process_is_running "$BRINGUP_PID"; then
    bringup_status=0
    wait "$BRINGUP_PID" || bringup_status=$?
    BRINGUP_PID=""
    if (( bringup_status == 0 )); then
      bringup_status=3
    fi
    echo "WARNING: $NAV_DISPLAY_NAME bringup exited before it became ready (exit=$bringup_status)." >&2
    echo "Inspect $BRINGUP_LOG" >&2
    echo "==> continuing in degraded-console mode; Qt safety gates keep motion controls disabled"
    stop_process "$BRINGUP_TAIL_PID" "bringup log follower" 10
    BRINGUP_TAIL_PID=""
    if ! ensure_ros_master; then
      echo "==> continuing without a confirmed ROS master"
    fi
    break
  fi
  sleep 0.25
done
if grep -Fq "$BRINGUP_READY_TEXT" "$BRINGUP_LOG"; then
  echo "==> $NAV_DISPLAY_NAME navigation is ready; Qt motion controls may now pass their live-data gates"
else
  echo "==> $NAV_DISPLAY_NAME navigation is offline; use Qt for diagnostics until the fault is fixed"
fi

gui_status=0
wait "$GUI_PID" || gui_status=$?
GUI_PID=""
if (( gui_status != 0 )); then
  echo "Qt operator console exited with code $gui_status. Inspect $GUI_LOG" >&2
else
  echo "Qt operator console closed; stopping the stack started by this launcher."
fi
exit "$gui_status"
