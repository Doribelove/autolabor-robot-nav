#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${WS_SETUP:-$ROBOT_WS/devel/setup.bash}"

GPS_MAX_SPEED="${1:-1.6}"
GPS_TEB_PROFILE="${2:-cruise}"
# Collection runs approach small targets with a long front overhang. Start
# slowing before the front of the 1.6 m vehicle reaches the target. Reuse the
# existing terminal-goal limiter; explicit environment overrides still apply.
export GPS_GOAL_SPEED_CAP_ENABLED="${GPS_GOAL_SPEED_CAP_ENABLED:-true}"
export GPS_GOAL_APPROACH_TIME="${GPS_GOAL_APPROACH_TIME:-3.0}"
export GPS_GOAL_COMFORTABLE_DECEL="${GPS_GOAL_COMFORTABLE_DECEL:-0.25}"
# 0.20 m/s is the existing measured M2 start command; smaller terminal caps
# can leave a newly accepted nearby goal inside the chassis start deadband.
export GPS_GOAL_MIN_APPROACH_SPEED="${GPS_GOAL_MIN_APPROACH_SPEED:-0.20}"
OPERATOR_NAVIGATION_MODE="${OPERATOR_NAVIGATION_MODE:-gps}"
OPERATOR_START_VISION="${OPERATOR_START_VISION:-true}"
OPERATOR_START_CAMERA="${OPERATOR_START_CAMERA:-true}"
OPERATOR_CAMERA_DRIVER="${OPERATOR_CAMERA_DRIVER:-hikrobot}"
# Reuse the ZED ARM driver already built on this Jetson in its own process
# environment; navigation and FOD messages continue to use this workspace.
OPERATOR_ZED_LAUNCHER="${OPERATOR_ZED_LAUNCHER:-$SCRIPT_DIR/zed2_camera.sh}"
OPERATOR_ZED_SERIAL_NUMBER="${OPERATOR_ZED_SERIAL_NUMBER:-0}"
OPERATOR_ZED_GRAB_FPS="${OPERATOR_ZED_GRAB_FPS:-60}"
OPERATOR_ZED_PUBLISH_FPS="${OPERATOR_ZED_PUBLISH_FPS:-60.0}"
OPERATOR_ZED_DEPTH_MODE="${OPERATOR_ZED_DEPTH_MODE:-ULTRA}"
OPERATOR_ZED_TENSORRT_ENGINE="${OPERATOR_ZED_TENSORRT_ENGINE:-$HOME/.cache/autolabor_fod/fod_zed_roi_224x960_fp16.engine}"
OPERATOR_FOD_PROFILE="${OPERATOR_FOD_PROFILE:-airport_recall}"
OPERATOR_START_RABBITMQ="${OPERATOR_START_RABBITMQ:-true}"
OPERATOR_IMAGE_QUALITY_CONTROL="${OPERATOR_IMAGE_QUALITY_CONTROL:-true}"
OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US="${OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US:-12000}"
OPERATOR_DETECTOR_PYTHON="${OPERATOR_DETECTOR_PYTHON:-${FOD_YOLO_ENV:-$HOME/python_env/fod_yolo}/bin/python3}"

BRINGUP_PID=""
BRINGUP_TAIL_PID=""
ROSCORE_PID=""
VISION_PID=""
ZED_PID=""
RABBITMQ_PID=""
GUI_PID=""
CLEANUP_STARTED=0
ROS_MASTER_PREEXISTED=false

usage() {
  echo "Usage: $0 [gps_max_speed_mps] [cruise|obstacle]"
  echo
  echo "Default: $0 1.6 cruise"
  echo
  echo "Optional environment switches:"
  echo "  OPERATOR_NAVIGATION_MODE=gps|fast_lio_indoor"
  echo "  OPERATOR_START_VISION=true|false"
  echo "  OPERATOR_START_CAMERA=true|false"
  echo "  OPERATOR_CAMERA_DRIVER=hikrobot|zed"
  echo "  OPERATOR_ZED_LAUNCHER=$OPERATOR_ZED_LAUNCHER"
  echo "  OPERATOR_ZED_SERIAL_NUMBER=0 (auto-select the connected ZED)"
  echo "  OPERATOR_ZED_GRAB_FPS=60 OPERATOR_ZED_PUBLISH_FPS=60.0"
  echo "  OPERATOR_ZED_DEPTH_MODE=ULTRA (native depth on demand)"
  echo "  OPERATOR_ZED_TENSORRT_ENGINE=$OPERATOR_ZED_TENSORRT_ENGINE (off uses the original .pt model)"
  echo "  OPERATOR_FOD_PROFILE=airport_recall|original"
  echo "  OPERATOR_START_RABBITMQ=true|false"
  echo "  OPERATOR_IMAGE_QUALITY_CONTROL=true|false"
  echo "  OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US=12000"
  echo "  OPERATOR_DETECTOR_PYTHON=$OPERATOR_DETECTOR_PYTHON"
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

  # Stop the navigation bringup first. Its mode manager publishes a final zero
  # command and then the CAN/M2 processes are closed. Sidecars cannot move the
  # chassis after that command route disappears.
  stop_process "$BRINGUP_PID" "GPS navigation bringup" 120
  stop_process "$VISION_PID" "camera and YOLO11 launch" 80
  stop_process "$ZED_PID" "ZED camera launch" 80
  stop_process "$RABBITMQ_PID" "RabbitMQ GPS bridge" 50
  stop_process "$GUI_PID" "Qt operator console" 30
  stop_process "$BRINGUP_TAIL_PID" "bringup log follower" 10
  stop_process "$ROSCORE_PID" "fallback ROS master" 50
}

trap cleanup EXIT
trap 'exit 130' INT TERM

if (( $# > 2 )); then
  usage >&2
  exit 1
fi
case "${1:-}" in
  -h|--help|help)
    usage
    exit 0
    ;;
esac
case "$GPS_TEB_PROFILE" in
  cruise|obstacle) ;;
  *)
    echo "Invalid GPS profile: $GPS_TEB_PROFILE (use cruise or obstacle)" >&2
    exit 1
    ;;
esac
case "$OPERATOR_NAVIGATION_MODE" in
  gps|fast_lio_indoor) ;;
  *)
    echo "Invalid OPERATOR_NAVIGATION_MODE=$OPERATOR_NAVIGATION_MODE; use gps or fast_lio_indoor." >&2
    exit 1
    ;;
esac
case "$OPERATOR_CAMERA_DRIVER" in
  hikrobot|zed) ;;
  *)
    echo "Invalid OPERATOR_CAMERA_DRIVER=$OPERATOR_CAMERA_DRIVER; use hikrobot or zed." >&2
    exit 1
    ;;
esac
case "$OPERATOR_FOD_PROFILE" in
  airport_recall|original) ;;
  *)
    echo "Invalid OPERATOR_FOD_PROFILE=$OPERATOR_FOD_PROFILE; use airport_recall or original." >&2
    exit 1
    ;;
esac
if [[ ! "$GPS_MAX_SPEED" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v value="$GPS_MAX_SPEED" 'BEGIN { exit !(value > 0.0) }'; then
  echo "Invalid positive GPS speed: $GPS_MAX_SPEED" >&2
  exit 1
fi
for value_name in \
  OPERATOR_START_VISION \
  OPERATOR_START_CAMERA \
  OPERATOR_START_RABBITMQ \
  OPERATOR_IMAGE_QUALITY_CONTROL; do
  if ! valid_boolean "${!value_name}"; then
    echo "Invalid $value_name=${!value_name}; use true or false." >&2
    exit 1
  fi
done
if [[ ! "$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v value="$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US" \
     'BEGIN { exit !(value > 0.0) }'; then
  echo "Invalid positive OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US=$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US" >&2
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

if rosparam list >/dev/null 2>&1; then
  ROS_MASTER_PREEXISTED=true
fi

LOG_DIR="${OPERATOR_LOG_DIR:-$ROBOT_WS/log/operator_all_in_one_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
BRINGUP_LOG="$LOG_DIR/bringup.log"
VISION_LOG="$LOG_DIR/vision.log"
ZED_LOG="$LOG_DIR/zed.log"
RABBITMQ_LOG="$LOG_DIR/rabbitmq.log"
GUI_LOG="$LOG_DIR/gui.log"
ROSCORE_LOG="$LOG_DIR/fallback_roscore.log"
touch "$BRINGUP_LOG"

echo "==> Autolabor 一体化操作台"
echo "==> Navigation: mode=$OPERATOR_NAVIGATION_MODE, max_speed=$GPS_MAX_SPEED m/s, GPS profile=$GPS_TEB_PROFILE"
echo "==> Runtime logs: $LOG_DIR"
if ! ensure_ros_master; then
  echo "Unable to start the shared ROS master. Inspect $ROSCORE_LOG" >&2
  exit 3
fi

# Select one model identity before either navigation or perception starts.
# The servo must validate the artifact actually used by the detector, including
# the native TensorRT engine, rather than retaining the original .pt digest.
VISION_ARGS=()
if [[ "$OPERATOR_START_VISION" == "true" ]]; then
  export FOD_RECOVERY_EXPECTED_MODEL_SHA256="7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500"
  if [[ "$OPERATOR_CAMERA_DRIVER" == "zed" &&
        "$OPERATOR_FOD_PROFILE" == "airport_recall" &&
        -f "$OPERATOR_ZED_TENSORRT_ENGINE" &&
        -s "$OPERATOR_ZED_TENSORRT_ENGINE.sha256" ]]; then
    read -r FOD_RECOVERY_EXPECTED_MODEL_SHA256 < "$OPERATOR_ZED_TENSORRT_ENGINE.sha256"
    VISION_ARGS+=("weights:=$OPERATOR_ZED_TENSORRT_ENGINE")
    echo "==> ZED FOD inference: native TensorRT FP16 engine"
  elif [[ "$OPERATOR_CAMERA_DRIVER" == "zed" ]]; then
    echo "==> ZED FOD inference: original CUDA/PyTorch model"
  fi
  VISION_ARGS+=("expected_model_sha256:=$FOD_RECOVERY_EXPECTED_MODEL_SHA256")
fi

NAVIGATION_ARGS=("$OPERATOR_NAVIGATION_MODE" "$GPS_MAX_SPEED")
GUI_ARGS=("camera_driver:=$OPERATOR_CAMERA_DRIVER")
if [[ "$OPERATOR_NAVIGATION_MODE" == "gps" ]]; then
  NAVIGATION_ARGS+=("$GPS_TEB_PROFILE")
else
  GUI_ARGS+=("rviz_config:=$ROBOT_WS/src/application/autolabor_operator_gui/config/operator_indoor.rviz")
fi
NAVIGATION_READY="Robot bringup is running in $OPERATOR_NAVIGATION_MODE mode."
echo "==> starting $OPERATOR_NAVIGATION_MODE navigation without standalone RViz"
TERMINAL_MODE=same NAV_START_RVIZ=false \
  "$SCRIPT_DIR/bringup.sh" "${NAVIGATION_ARGS[@]}" \
  >"$BRINGUP_LOG" 2>&1 &
BRINGUP_PID=$!
tail -n +1 -F --pid="$BRINGUP_PID" "$BRINGUP_LOG" &
BRINGUP_TAIL_PID=$!

if [[ "$OPERATOR_START_VISION" == "true" ]]; then
  if [[ "$OPERATOR_CAMERA_DRIVER" == "zed" ]]; then
    if [[ "$OPERATOR_START_CAMERA" == "true" ]]; then
      echo "==> starting ZED camera using $OPERATOR_ZED_LAUNCHER"
      "$OPERATOR_ZED_LAUNCHER" serial_number:="$OPERATOR_ZED_SERIAL_NUMBER" \
        grab_frame_rate:="$OPERATOR_ZED_GRAB_FPS" pub_frame_rate:="$OPERATOR_ZED_PUBLISH_FPS" \
        depth_mode:="$OPERATOR_ZED_DEPTH_MODE" \
        publish_tf:=false publish_map_tf:=false publish_fod_aliases:=true \
        >"$ZED_LOG" 2>&1 &
      ZED_PID=$!
    fi
    # The detector consumes the same image aliases for either camera. The
    # migrated exposure controller uses Hikrobot services, so leave ZED's own
    # automatic exposure in control.
    OPERATOR_START_CAMERA=false
    OPERATOR_IMAGE_QUALITY_CONTROL=false
  fi
  echo "==> starting $OPERATOR_CAMERA_DRIVER camera and YOLO11 perception sidecar"
  roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
    start_camera:="$OPERATOR_START_CAMERA" \
    enable_image_quality_controller:="$OPERATOR_IMAGE_QUALITY_CONTROL" \
    image_quality_exposure_max_us:="$OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_US" \
    detector_python:="$OPERATOR_DETECTOR_PYTHON" \
    detector_profile:="$OPERATOR_FOD_PROFILE" \
    camera_driver:="$OPERATOR_CAMERA_DRIVER" \
    "${VISION_ARGS[@]}" \
    >"$VISION_LOG" 2>&1 &
  VISION_PID=$!
else
  echo "==> camera/YOLO11 autostart disabled; the Qt visual page will remain available"
fi

if [[ "$OPERATOR_START_RABBITMQ" == "true" ]]; then
  echo "==> starting RabbitMQ GPS target bridge (Qt buttons replace terminal input)"
  "$SCRIPT_DIR/rabbitmq_gps_goal_bridge.py" </dev/null >"$RABBITMQ_LOG" 2>&1 &
  RABBITMQ_PID=$!
else
  echo "==> RabbitMQ bridge autostart disabled"
fi

echo "==> opening the integrated Qt console while navigation initializes"
"$SCRIPT_DIR/operator_gui.sh" "${GUI_ARGS[@]}" >"$GUI_LOG" 2>&1 &
GUI_PID=$!

echo "==> waiting for the complete navigation readiness gate in the background"
while ! grep -Fq "$NAVIGATION_READY" "$BRINGUP_LOG"; do
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
    echo "WARNING: $OPERATOR_NAVIGATION_MODE bringup exited before it became ready (exit=$bringup_status)." >&2
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
if grep -Fq "$NAVIGATION_READY" "$BRINGUP_LOG"; then
  echo "==> $OPERATOR_NAVIGATION_MODE navigation is ready"
else
  echo "==> $OPERATOR_NAVIGATION_MODE navigation is offline; use Qt for diagnostics until the fault is fixed"
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
