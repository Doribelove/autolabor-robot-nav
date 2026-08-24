#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/process_control.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
PID_FILE="$RUN_DIR/nvidia_ui.pid"
CHILD_PID_DIR="$RUN_DIR/nvidia_ui.children"
UI_CHILD_PATTERN='roslaunch([[:space:]].*)?(autolabor_fod_vision[[:space:]]+zed_fod_detection\.launch|autolabor_operator_gui[[:space:]]+operator_gui\.launch)([[:space:]]|$)'
UI_ROS_NODES=(
  /zed2/zed_node /zed2/zed2_state_publisher /fod_detector
  /fod_image_quality_controller /fod_ground_projector /fod_tracker
  /autolabor_operator_gui /operator_map_display_anchor
)
mkdir -p "$RUN_DIR" "$DUAL_HOST_WS/log"

stop_child_records() {
  local status=0 pid_file
  local -a child_files=()
  if [[ -d "$CHILD_PID_DIR" ]]; then
    shopt -s nullglob
    child_files=("$CHILD_PID_DIR"/*.pid)
    shopt -u nullglob
  fi
  for pid_file in "${child_files[@]}"; do
    dual_host_stop_pid_file "$pid_file" "NVIDIA UI/vision child" \
      "$UI_CHILD_PATTERN" || status=$?
  done
  rmdir -- "$CHILD_PID_DIR" 2>/dev/null || true
  return "$status"
}

stop_existing() {
  local status=0
  dual_host_stop_pid_file "$PID_FILE" "NVIDIA UI/vision stack" '(^|/)nvidia_ui\.sh([[:space:]]|$)' || status=$?
  stop_child_records || status=$?
  if (( status == 0 )); then
    rm -f -- "$PID_FILE"
    echo "PID-recorded NVIDIA UI/vision stack is stopped."
  fi
  return "$status"
}

if [[ "${1:-}" == "--stop" ]]; then
  stop_existing
  exit 0
elif (( $# > 0 )); then
  echo "Usage: $0 [--stop]" >&2
  exit 2
fi

if ! timeout 5 rosparam list >/dev/null 2>&1; then
  echo "J6M ROS master is not reachable at $ROS_MASTER_URI." >&2
  exit 3
fi

cleanup_args=(--host "$NVIDIA_J6M_IP" --fail-if-live)
for node in "${UI_ROS_NODES[@]}"; do cleanup_args+=(--node "$node"); done
if ! timeout 30 "$SCRIPT_DIR/cleanup_stale_ros_nodes.py" "${cleanup_args[@]}"; then
  echo "A live managed NVIDIA UI/vision node is still registered; refusing a duplicate start." >&2
  exit 4
fi

if dual_host_pid_file_is_owned "$PID_FILE" '(^|/)nvidia_ui\.sh([[:space:]]|$)'; then
  old_pid="$(dual_host_pid_file_pid "$PID_FILE")"
  if dual_host_process_is_running "$old_pid"; then
    echo "NVIDIA UI/vision stack is already running (PID $old_pid)." >&2
    exit 4
  fi
fi

LOG_DIR="$DUAL_HOST_WS/log/nvidia_ui_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()
CHILD_PID_FILES=()
CLEANUP_STARTED=0
self_pid_line=""
last_started_pid=""

start_process() {
  local log_file="$1" pid child_pid_file
  shift
  "$@" >"$log_file" 2>&1 &
  pid=$!
  child_pid_file="$CHILD_PID_DIR/$pid.pid"
  PIDS+=("$pid")
  CHILD_PID_FILES+=("$child_pid_file")
  last_started_pid="$pid"
  if ! dual_host_write_pid_file "$child_pid_file" "$pid"; then
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    return 1
  fi
}

wait_for_zed_image() {
  local vision_pid="$1" vision_log="$2"
  local deadline=$((SECONDS + ZED_IMAGE_WAIT_SEC))
  echo "Waiting up to ${ZED_IMAGE_WAIT_SEC}s for the first ZED image on /fod_camera/image_raw..."
  while ! timeout 3 rostopic echo --noarr -n 1 /fod_camera/image_raw >/dev/null 2>&1; do
    if ! kill -0 "$vision_pid" 2>/dev/null; then
      wait "$vision_pid" 2>/dev/null || true
      echo "ZED/vision launch exited before publishing an image." >&2
      tail -n 80 "$vision_log" >&2 || true
      return 1
    fi
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for a live ZED image; a ROS node name alone is not camera readiness." >&2
      "$SCRIPT_DIR/zed_camera_check.sh" --wait 0 >&2 || true
      tail -n 80 "$vision_log" >&2 || true
      return 1
    fi
  done
  echo "ZED image stream is live on /fod_camera/image_raw."
}

cleanup() {
  (( CLEANUP_STARTED == 0 )) || return 0
  CLEANUP_STARTED=1
  trap - EXIT INT TERM
  local pid
  stop_child_records || true
  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
  dual_host_remove_pid_file_if_unchanged "$PID_FILE" "$self_pid_line"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
mkdir -p "$CHILD_PID_DIR"
dual_host_write_pid_file "$PID_FILE" "$$"
self_pid_line="$(sed -n '1p' "$PID_FILE")"

if [[ "$NVIDIA_START_VISION" == true ]]; then
  [[ -x "$NVIDIA_DETECTOR_PYTHON" ]] || {
    echo "YOLO Python is missing: $NVIDIA_DETECTOR_PYTHON" >&2
    exit 5
  }
  fod_weights="${NVIDIA_FOD_WEIGHTS:-$BASE_ROBOT_WS/src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt}"
  [[ -r "$fod_weights" ]] || {
    echo "YOLO weights are missing: $fod_weights" >&2
    exit 5
  }
  if [[ "$NVIDIA_START_CAMERA" == true ]]; then
    "$SCRIPT_DIR/zed_camera_check.sh" --wait "$ZED_USB_WAIT_SEC"
  fi
  vision_log="$LOG_DIR/vision.log"
  start_process "$LOG_DIR/vision.log" \
    roslaunch autolabor_fod_vision zed_fod_detection.launch \
      start_camera:="$NVIDIA_START_CAMERA" \
      serial_number:="$NVIDIA_ZED_SERIAL" \
      detector_python:="$NVIDIA_DETECTOR_PYTHON" \
      weights:="$fod_weights" \
      enable_image_quality_controller:=false
  vision_pid="$last_started_pid"
  if [[ "$NVIDIA_START_CAMERA" == true ]]; then
    wait_for_zed_image "$vision_pid" "$vision_log"
  fi
fi

if [[ "$NVIDIA_START_QT" == true ]]; then
  rviz_fixed_frame=camera_init
  [[ "$STATIC_MAP_ENABLED" == false ]] || rviz_fixed_frame=map
  start_process "$LOG_DIR/gui.log" \
    roslaunch autolabor_operator_gui operator_gui.launch \
      navigation_mode_label:=J6M_FAST_LIO \
      odom_topic:=/Odometry \
      cloud_topic:=/cloud_registered_body \
      imu_topic:=/livox/imu \
      static_map_mode:="$STATIC_MAP_ENABLED" \
      rviz_startup_fixed_frame:="$rviz_fixed_frame" \
      rviz_navigation_fixed_frame:="$rviz_fixed_frame"
fi

if (( ${#PIDS[@]} == 0 )); then
  echo "No NVIDIA UI/vision component is enabled." >&2
  exit 6
fi

echo "NVIDIA UI/vision sidecars are running; logs: $LOG_DIR"
echo "Stopping this script never stops J6M roscore/navigation."
wait -n "${PIDS[@]}"
