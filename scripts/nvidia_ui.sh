#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/process_control.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
PID_FILE="$RUN_DIR/nvidia_ui.pid"
CHILD_PID_DIR="$RUN_DIR/nvidia_ui.children"
UI_CHILD_PATTERN='roslaunch([[:space:]].*)?(autolabor_fod_vision[[:space:]]+zed_fod_detection\.launch|autolabor_operator_gui[[:space:]]+operator_gui\.launch|sweeper_mcp[[:space:]]+ai_control\.launch)([[:space:]]|$)'
UI_ROS_NODES=(
  /zed2/zed_node /zed2/zed2_state_publisher /fod_detector
  /fod_image_quality_controller /fod_ground_projector /fod_tracker
  /autolabor_operator_gui /operator_map_display_anchor /sweeper_ai
  /sweeper_mcp_backend
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

dual_host_validate_fod_model_contract || exit 2
[[ "$NVIDIA_START_VISION" != true ]] || dual_host_validate_fod_weights || exit 2

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

wait_for_fod_detector() {
  local vision_pid="$1" vision_log="$2" runtime_token="$3"
  local expected_root="$4" wait_sec="${FOD_DETECTOR_WAIT_SEC:-90}"
  local deadline ready_token actual_path
  [[ "$wait_sec" =~ ^[1-9][0-9]*$ ]] || {
    echo "FOD_DETECTOR_WAIT_SEC must be a positive integer." >&2
    return 1
  }
  deadline=$((SECONDS + wait_sec))
  echo "Waiting up to ${wait_sec}s for the GAM detector model to load..."
  while true; do
    ready_token="$(
      timeout 2 rosparam get /fod_detector/ready_token 2>/dev/null || true
    )"
    if [[ "$ready_token" == "$runtime_token" ]]; then
      actual_path="$(
        timeout 2 rosparam get /fod_detector/ultralytics_import_path \
          2>/dev/null || true
      )"
      case "$actual_path" in
        "$expected_root"/ultralytics/*)
          echo "FOD detector loaded the model with Ultralytics: $actual_path"
          return 0
          ;;
        *)
          echo "FOD detector reported an unexpected Ultralytics path: ${actual_path:-<empty>}" >&2
          tail -n 120 "$vision_log" >&2 || true
          return 1
          ;;
      esac
    fi
    if ! kill -0 "$vision_pid" 2>/dev/null; then
      wait "$vision_pid" 2>/dev/null || true
      echo "ZED/vision launch exited before the GAM model became ready." >&2
      tail -n 120 "$vision_log" >&2 || true
      return 1
    fi
    if grep -Fq "FOD detector failed to start:" "$vision_log" 2>/dev/null; then
      echo "GAM detector model loading failed; see the detailed error below." >&2
      tail -n 120 "$vision_log" >&2 || true
      return 1
    fi
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for the GAM detector model to load." >&2
      tail -n 120 "$vision_log" >&2 || true
      return 1
    fi
    sleep 0.25
  done
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
  fod_weights="${NVIDIA_FOD_WEIGHTS:-$DUAL_HOST_WS/src/application/autolabor_fod_vision/models/yolo11_gam_best.pt}"
  [[ -r "$fod_weights" ]] || {
    echo "YOLO weights are missing: $fod_weights" >&2
    exit 5
  }
  fod_model_sha256="${NVIDIA_FOD_MODEL_SHA256,,}"
  fod_required_class_names="$NVIDIA_FOD_REQUIRED_CLASS_NAMES"
  fod_ultralytics_config="${NVIDIA_FOD_ULTRALYTICS_ROOT:-$DUAL_HOST_WS/ultralytics_yolo11_custom}"
  fod_pythonpath="${PYTHONPATH:-}"
  if [[ -r "$fod_ultralytics_config/ultralytics/__init__.py" ]]; then
    fod_ultralytics_root="$(readlink -f -- "$fod_ultralytics_config")"
  elif [[ "${fod_ultralytics_config##*/}" == ultralytics &&
          -r "$fod_ultralytics_config/__init__.py" ]]; then
    fod_ultralytics_package="$(readlink -f -- "$fod_ultralytics_config")"
    fod_ultralytics_root="${fod_ultralytics_package%/ultralytics}"
  else
    echo "Project-local Ultralytics source is missing: $fod_ultralytics_config" >&2
    echo "Expected a root containing ultralytics/__init__.py or that package directory itself." >&2
    exit 5
  fi
  fod_pythonpath="$fod_ultralytics_root${fod_pythonpath:+:$fod_pythonpath}"
  if ! ultralytics_probe="$(
    env PYTHONDONTWRITEBYTECODE=1 \
      PYTHONPATH="$fod_pythonpath" \
      AUTOLABOR_FOD_ULTRALYTICS_ROOT="$fod_ultralytics_root" \
      "$NVIDIA_DETECTOR_PYTHON" - <<'PY'
import os
from pathlib import Path

import ultralytics
from ultralytics.nn.modules import GAM_Attention

expected = Path(os.environ["AUTOLABOR_FOD_ULTRALYTICS_ROOT"]).resolve() / "ultralytics"
actual = Path(ultralytics.__file__).resolve()
try:
    actual.relative_to(expected)
except ValueError as error:
    raise SystemExit(
        "Ultralytics import conflict: {} is outside {}".format(actual, expected)
    ) from error
print(
    "Ultralytics preflight: version={} path={} GAM={}".format(
        ultralytics.__version__, actual, GAM_Attention.__name__
    )
)
PY
  )"; then
    echo "Project-local Ultralytics preflight failed." >&2
    exit 5
  fi
  printf '%s\n' "$ultralytics_probe"
  fod_runtime_token="$(python3 -c 'import secrets; print(secrets.token_hex(16))')"
  [[ -n "$fod_runtime_token" ]] || {
    echo "Failed to create the FOD detector runtime token." >&2
    exit 5
  }
  if [[ "$NVIDIA_START_CAMERA" == true ]]; then
    "$SCRIPT_DIR/zed_camera_check.sh" --wait "$ZED_USB_WAIT_SEC"
  fi
  vision_log="$LOG_DIR/vision.log"
  start_process "$LOG_DIR/vision.log" \
    env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$fod_pythonpath" \
      AUTOLABOR_FOD_ULTRALYTICS_ROOT="$fod_ultralytics_root" \
    roslaunch autolabor_fod_vision zed_fod_detection.launch \
      start_camera:="$NVIDIA_START_CAMERA" \
      serial_number:="$NVIDIA_ZED_SERIAL" \
      detector_python:="$NVIDIA_DETECTOR_PYTHON" \
      ultralytics_root:="$fod_ultralytics_root" \
      require_gam:=true \
      runtime_token:="$fod_runtime_token" \
      weights:="$fod_weights" \
      expected_model_sha256:="$fod_model_sha256" \
      required_class_names:="$fod_required_class_names" \
      enable_image_quality_controller:=false
  vision_pid="$last_started_pid"
  if [[ "$NVIDIA_START_CAMERA" == true ]]; then
    wait_for_zed_image "$vision_pid" "$vision_log"
  fi
  wait_for_fod_detector \
    "$vision_pid" "$vision_log" "$fod_runtime_token" "$fod_ultralytics_root"
fi

if [[ "$NVIDIA_START_QT" == true ]]; then
  ai_config="$DUAL_HOST_WS/src/sweeper_mcp/config/sweeper_mcp.yaml"
  [[ -r "$ai_config" ]] || {
    echo "AI configuration is missing or unreadable: $ai_config" >&2
    exit 5
  }
  [[ "$(stat -c '%a:%u' -- "$ai_config")" == "600:$EUID" ]] || {
    echo "AI configuration must be owned by the current user with mode 0600: $ai_config" >&2
    exit 5
  }
  asr_config_state="$(AI_CONFIG_PATH="$ai_config" python3 - <<'PY'
import os
import yaml

with open(os.environ["AI_CONFIG_PATH"], "r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
asr = config.get("asr") or {}
print("{}\t{}".format(
    "true" if bool(asr.get("enabled", False)) else "false",
    str(asr.get("model", "medium")).strip().lower()))
PY
)"
  IFS=$'\t' read -r asr_enabled asr_default_model <<<"$asr_config_state"
  asr_python="${SWEEPER_ASR_PYTHON:-$DUAL_HOST_WS/runtime/asr/venv/bin/python3}"
  asr_model_dir="${SWEEPER_ASR_MODEL_DIR:-$DUAL_HOST_WS/runtime/asr/models}"
  if [[ "$asr_enabled" == true ]]; then
    case "$asr_default_model" in
      small|medium|large) ;;
      large-v3) asr_default_model=large ;;
      *)
        echo "Invalid default ASR model: $asr_default_model (expected small, medium, or large)" >&2
        exit 5
        ;;
    esac
    [[ -x "$asr_python" ]] || {
      echo "ASR Python is missing; run scripts/install_whisper_asr.sh: $asr_python" >&2
      exit 5
    }
    for asr_checkpoint in small.pt medium.pt large-v3.pt; do
      [[ -r "$asr_model_dir/$asr_checkpoint" ]] || {
        echo "Switchable Whisper checkpoint is missing: $asr_model_dir/$asr_checkpoint" >&2
        echo "Run scripts/install_whisper_asr.sh before starting Qt." >&2
        exit 5
      }
    done
  fi
  ai_session_token="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
  [[ -n "$ai_session_token" ]] || {
    echo "Failed to create the private NVIDIA UI session token." >&2
    exit 5
  }
  rviz_fixed_frame=camera_init
  [[ "$STATIC_MAP_ENABLED" == false ]] || rviz_fixed_frame=map
  start_process "$LOG_DIR/ai.log" \
    env SWEEPER_AI_SESSION_TOKEN="$ai_session_token" \
      SWEEPER_AI_CONFIG="$ai_config" \
      SWEEPER_ASR_PYTHON="$asr_python" \
      SWEEPER_ASR_MODEL_DIR="$asr_model_dir" \
      SWEEPER_MCP_BACKEND="${SWEEPER_MCP_BACKEND:-ros}" \
      SWEEPER_COVERAGE_REGION_ROOT="${STATIC_MAP_SET:-}" \
      SWEEPER_COVERAGE_REGION_LEGACY_ROOT="$DUAL_HOST_WS/global_maps/coverage_regions" \
      SWEEPER_STATIC_MAP_SOURCE_MODE="${STATIC_MAP_SOURCE_MODE:-fused}" \
    roslaunch sweeper_mcp ai_control.launch
  start_process "$LOG_DIR/gui.log" \
    env SWEEPER_AI_SESSION_TOKEN="$ai_session_token" \
    roslaunch autolabor_operator_gui operator_gui.launch \
      navigation_mode_label:=J6M_FAST_LIO \
      odom_topic:=/Odometry \
      cloud_topic:=/cloud_registered_body \
      imu_topic:=/livox/imu \
      static_map_mode:="$STATIC_MAP_ENABLED" \
      static_map_set:="${STATIC_MAP_SET:-}" \
      static_map_source_mode:="${STATIC_MAP_SOURCE_MODE:-fused}" \
      coverage_region_root:="${STATIC_MAP_SET:-}" \
      coverage_region_legacy_root:="$DUAL_HOST_WS/global_maps/coverage_regions" \
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
