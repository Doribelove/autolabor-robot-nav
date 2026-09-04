#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

mode=--static
if (( $# > 0 )); then
  mode="$1"
  shift
fi
case "$mode" in
  --static|--network|--runtime) ;;
  *) echo "Usage: $0 --static | --network | --runtime [--allow-missing-data]" >&2; exit 2 ;;
esac

allow_missing_runtime_data=false
while (( $# > 0 )); do
  case "$1" in
    --allow-missing-data) allow_missing_runtime_data=true ;;
    *) echo "Usage: $0 --static | --network | --runtime [--allow-missing-data]" >&2; exit 2 ;;
  esac
  shift
done
if [[ "$allow_missing_runtime_data" == true && "$mode" != --runtime ]]; then
  echo "--allow-missing-data is valid only with --runtime" >&2
  exit 2
fi

# The managed supervisor records the mode selected for the active run. A
# standalone runtime check must use that record instead of silently falling
# back to the map-free defaults from dual_host.env.
MAP_MODE_FILE="$DUAL_HOST_WS/runtime/run/map_mode.env"
if [[ "$mode" == --runtime && -r "$MAP_MODE_FILE" ]]; then
  source "$MAP_MODE_FILE"
fi
case "${STATIC_MAP_ENABLED:-false}" in
  true|false) ;;
  *) echo "Invalid STATIC_MAP_ENABLED in $MAP_MODE_FILE" >&2; exit 2 ;;
esac
case "${FOD_MOTION_ENABLED:-false}" in
  true|false) ;;
  *) echo "Invalid FOD_MOTION_ENABLED in $MAP_MODE_FILE" >&2; exit 2 ;;
esac

failures=0
data_warnings=0
test_results_output="$(mktemp /tmp/robot_j6m_test_results.XXXXXX)"
trap 'rm -f -- "$test_results_output"' EXIT
pass() { echo "OK   $*"; }
fail() { echo "FAIL $*" >&2; failures=$((failures + 1)); }
warn() { echo "WARN $*" >&2; }
warn_data() { echo "WARN $*" >&2; data_warnings=$((data_warnings + 1)); }

ASR_SMALL_SHA256="9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794"
ASR_MEDIUM_SHA256="345ae4da62f9b3d59415adc60127b97c714f32e89e936602e85993674d08dcb1"
ASR_LARGE_V3_SHA256="e5b1a55b89c1367dacf97e3e19bfd829a01529dbfdeefa8caeb59b3f1b81dadb"

check_nvidia_asr_static_contract() {
  local ai_config asr_config_state asr_enabled asr_input_state
  local asr_python asr_model_dir model_entry model_name model_file
  local expected_model_sha256 actual_model_sha256 model_path
  ai_config="${SWEEPER_AI_CONFIG:-$DUAL_HOST_WS/src/sweeper_mcp/config/sweeper_mcp.yaml}"

  if [[ ! -e "$ai_config" ]]; then
    pass "NVIDIA ASR static check skipped (private AI YAML is absent)"
    return 0
  fi
  if [[ ! -f "$ai_config" || -L "$ai_config" || ! -r "$ai_config" ]]; then
    fail "private AI YAML is not a readable regular file: $ai_config"
    return 0
  fi

  asr_config_state="$(
    AI_CONFIG_PATH="$ai_config" \
      ASR_SMALL_SHA256="$ASR_SMALL_SHA256" \
      ASR_MEDIUM_SHA256="$ASR_MEDIUM_SHA256" \
      ASR_LARGE_SHA256="$ASR_LARGE_V3_SHA256" \
      python3 - <<'PY'
import os
import sys

import yaml

path = os.environ["AI_CONFIG_PATH"]
expected_models = {
    "small": ("small.pt", os.environ["ASR_SMALL_SHA256"]),
    "medium": ("medium.pt", os.environ["ASR_MEDIUM_SHA256"]),
    "large": ("large-v3.pt", os.environ["ASR_LARGE_SHA256"]),
}
try:
    with open(path, "r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
except (OSError, yaml.YAMLError) as exc:
    print("invalid\tinvalid")
    print("ASR YAML parse failed: {}".format(exc), file=sys.stderr)
    raise SystemExit(2)

if not isinstance(root, dict):
    print("invalid\tinvalid")
    print("ASR YAML root must be an object", file=sys.stderr)
    raise SystemExit(2)
asr = root.get("asr") or {}
if not isinstance(asr, dict):
    print("invalid\tinvalid")
    print("ASR YAML section must be an object", file=sys.stderr)
    raise SystemExit(2)

enabled = asr.get("enabled", False)
if not isinstance(enabled, bool):
    print("invalid\tinvalid")
    print("asr.enabled must be a boolean", file=sys.stderr)
    raise SystemExit(2)
if not enabled:
    print("false\tdisabled")
    raise SystemExit(0)

errors = []
if asr.get("model") != "medium":
    errors.append("asr.model must default to medium")
if asr.get("device") != "cuda":
    errors.append("asr.device must be cuda")
models = asr.get("models")
if not isinstance(models, dict):
    errors.append("asr.models must define small, medium, and large")
    models = {}
for name, (filename, expected_sha256) in expected_models.items():
    entry = models.get(name)
    if not isinstance(entry, dict):
        errors.append("asr.models.{} must be an object".format(name))
        continue
    if entry.get("filename") != filename:
        errors.append("asr.models.{}.filename must be {}".format(
            name, filename))
    actual_sha256 = str(entry.get("checkpoint_sha256", "")).strip().lower()
    if actual_sha256 != expected_sha256:
        errors.append("asr.models.{} checkpoint SHA-256 mismatch".format(name))
input_device = asr.get("input_device", "")
if not isinstance(input_device, str):
    errors.append("asr.input_device must be a string")
    input_state = "invalid"
else:
    normalized_input = input_device.strip().lower()
    input_state = (
        "auto" if normalized_input in ("", "auto") else "configured"
    )

if errors:
    print("invalid\t{}".format(input_state))
    for error in errors:
        print(error, file=sys.stderr)
    raise SystemExit(2)
print("true\t{}".format(input_state))
PY
  )"
  if [[ $? -ne 0 ]]; then
    fail "private AI YAML has an invalid selectable ASR contract"
    return 0
  fi
  IFS=$'\t' read -r asr_enabled asr_input_state <<<"$asr_config_state"
  if [[ "$asr_enabled" == false ]]; then
    pass "NVIDIA ASR is disabled in the private AI YAML"
    return 0
  fi
  if [[ "$asr_enabled" != true ]]; then
    fail "private AI YAML returned an invalid ASR enabled state"
    return 0
  fi
  pass "private AI YAML defaults to medium and pins small/medium/large checkpoints"

  asr_python="${SWEEPER_ASR_PYTHON:-$DUAL_HOST_WS/runtime/asr/venv/bin/python3}"
  asr_model_dir="${SWEEPER_ASR_MODEL_DIR:-$DUAL_HOST_WS/runtime/asr/models}"

  if [[ -x "$asr_python" ]]; then
    pass "isolated NVIDIA ASR Python is executable"
    if "$asr_python" -c 'import whisper' >/dev/null 2>&1; then
      pass "OpenAI Whisper imports from the isolated ASR environment"
    else
      fail "OpenAI Whisper cannot import from the isolated ASR environment"
    fi
    if "$asr_python" - <<'PY' >/dev/null 2>&1
import torch

assert torch.version.cuda == "11.4"
assert torch.cuda.is_available()
PY
    then
      pass "isolated ASR PyTorch has available CUDA 11.4"
    else
      fail "isolated ASR PyTorch does not have available CUDA 11.4"
    fi
  else
    fail "isolated NVIDIA ASR Python is missing or not executable: $asr_python"
  fi

  for model_entry in \
    "small:small.pt:$ASR_SMALL_SHA256" \
    "medium:medium.pt:$ASR_MEDIUM_SHA256" \
    "large:large-v3.pt:$ASR_LARGE_V3_SHA256"; do
    IFS=: read -r model_name model_file expected_model_sha256 <<<"$model_entry"
    model_path="$asr_model_dir/$model_file"
    if [[ -f "$model_path" && ! -L "$model_path" && -r "$model_path" ]]; then
      actual_model_sha256="$(sha256sum -- "$model_path" | awk '{print $1}')"
      if [[ "${actual_model_sha256,,}" == "$expected_model_sha256" ]]; then
        pass "local Whisper $model_name checkpoint matches the fixed SHA256"
      else
        fail "local Whisper $model_name checkpoint SHA256 does not match"
      fi
    else
      fail "local Whisper $model_name checkpoint is missing or invalid: $model_path"
    fi
  done

  if [[ "$asr_input_state" == auto ]]; then
    capture_devices="$(
      /usr/bin/arecord -l 2>/dev/null |
        grep -E '^card[[:space:]]+[0-9]+:' |
        grep -Evi 'auto_null|monitor|null|NVIDIA Jetson AGX Orin APE|tegra-dlink|ADSP-FE' || true
    )"
    if [[ -n "$capture_devices" ]]; then
      pass "ASR automatic ALSA capture discovery found a microphone candidate (device was not opened)"
    else
      warn "ASR uses automatic capture discovery, but no physical microphone candidate is currently enumerated"
    fi
  elif [[ "$asr_input_state" == configured ]]; then
    pass "ASR input_device is configured (device was not opened)"
  else
    fail "private AI YAML returned an invalid ASR input_device state"
  fi
}

if dual_host_validate_fod_model_contract; then
  pass "FOD detector/controller model contract is valid"
else
  fail "FOD detector/controller model contract is invalid"
fi
if dual_host_validate_fod_weights; then
  pass "FOD $NVIDIA_FOD_BACKEND artifacts match the configured SHA256 contract"
else
  fail "FOD $NVIDIA_FOD_BACKEND artifacts do not match the configured SHA256 contract"
fi
fod_ultralytics_package_root=""
if [[ "$NVIDIA_FOD_BACKEND" == yolo ||
      "$NVIDIA_FOD_BACKEND" == detect_and_classify ]]; then
  if [[ -r "$NVIDIA_FOD_ULTRALYTICS_ROOT/ultralytics/__init__.py" ]]; then
    fod_ultralytics_package_root="$(
      readlink -f -- "$NVIDIA_FOD_ULTRALYTICS_ROOT/ultralytics"
    )"
  elif [[ "${NVIDIA_FOD_ULTRALYTICS_ROOT##*/}" == ultralytics &&
          -r "$NVIDIA_FOD_ULTRALYTICS_ROOT/__init__.py" ]]; then
    fod_ultralytics_package_root="$(
      readlink -f -- "$NVIDIA_FOD_ULTRALYTICS_ROOT"
    )"
  fi
  if [[ -n "$fod_ultralytics_package_root" &&
        -r "$fod_ultralytics_package_root/nn/modules/attention.py" ]] &&
     grep -Fq 'class GAM_Attention' \
       "$fod_ultralytics_package_root/nn/modules/attention.py"; then
    pass "configured Ultralytics contains the retained GAM runtime"
  else
    fail "configured Ultralytics GAM runtime is missing"
  fi
  if [[ "$NVIDIA_FOD_BACKEND" == detect_and_classify ]]; then
    if env PYTHONDONTWRITEBYTECODE=1 \
         PYTHONPATH="$NVIDIA_FOD_ULTRALYTICS_ROOT:${PYTHONPATH:-}" \
         "$NVIDIA_DETECTOR_PYTHON" - <<'PY' >/dev/null 2>&1
import lap
import torch
import ultralytics

assert torch.cuda.is_available()
assert ultralytics.__version__
assert lap.__version__
PY
    then
      pass "detect_and_classify imports custom Ultralytics, CUDA, and persistent BoT-SORT dependency"
    else
      fail "detect_and_classify CUDA/Ultralytics/lap runtime is unavailable"
    fi
  fi
elif [[ "$NVIDIA_FOD_BACKEND" == locateanything ]]; then
  locate_cache_root="$NVIDIA_LOCATEANYTHING_MODEL_ROOT/.cache"
  if env PYTHONDONTWRITEBYTECODE=1 \
       PYTHONPATH="${PYTHONPATH:-}" \
       HF_HOME="$locate_cache_root/huggingface" \
       HUGGINGFACE_HUB_CACHE="$locate_cache_root/huggingface/hub" \
       TRANSFORMERS_CACHE="$locate_cache_root/huggingface/transformers" \
       TORCH_HOME="$locate_cache_root/torch" \
       XDG_CACHE_HOME="$locate_cache_root/xdg" \
       CUDA_CACHE_PATH="$locate_cache_root/cuda" \
       "$NVIDIA_LOCATEANYTHING_WORKER_PYTHON" - <<'PY' >/dev/null 2>&1
import torch
import transformers

from autolabor_fod_vision.locateanything_runtime import LocateAnythingDetector

assert torch.cuda.is_available()
assert tuple(torch.cuda.get_device_capability(0)) >= (8, 0)
assert LocateAnythingDetector
assert transformers.__version__
PY
  then
    pass "LocateAnything isolated worker imports with compatible NVIDIA CUDA"
  else
    fail "LocateAnything isolated worker or NVIDIA CUDA runtime is unavailable"
  fi
fi

required_packages=(
  autolabor_coverage autolabor_dual_host autolabor_dual_lidar autolabor_fod_control
  autolabor_fod_msgs autolabor_operator_gui fast_lio fast_lio_localization livox_ros_driver2
  robot_bringup sweeper_mcp teb_local_planner zed_wrapper map_server
)
for package in "${required_packages[@]}"; do
  if rospack find "$package" >/dev/null 2>&1; then
    pass "package $package"
  else
    fail "package $package"
  fi
done

if roslaunch --files autolabor_dual_host nvidia_gateway.launch >/dev/null 2>&1 &&
   roslaunch --files autolabor_dual_host j6m_fastlio_navigation.launch >/dev/null 2>&1 &&
   roslaunch --files sweeper_mcp ai_control.launch >/dev/null 2>&1; then
  pass "dual-host launch files resolve"
else
  fail "dual-host launch files do not resolve"
fi

if [[ "$mode" == --static ]]; then
  check_nvidia_asr_static_contract
fi

if catkin_test_results "$DUAL_HOST_WS/build/test_results" >"$test_results_output" 2>&1; then
  pass "$(tail -n 1 "$test_results_output")"
else
  fail "$(tail -n 1 "$test_results_output")"
fi

if [[ "$MOTION_ENABLED" == false && "$FOD_MOTION_ENABLED" == false ]]; then
  pass "motion gates are fail-closed"
else
  if [[ -f "$DUAL_HOST_WS/runtime/motion_authorized.ok" ]]; then
    pass "motion is enabled with an authorization marker"
  else
    fail "motion is enabled without an authorization marker"
  fi
fi

if [[ "$mode" == --network || "$mode" == --runtime ]]; then
  if "$SCRIPT_DIR/network_check.sh"; then
    pass "dedicated networks"
  else
    fail "dedicated networks"
  fi
fi

topic_publishers() {
  local topic="$1"
  rostopic info "$topic" 2>/dev/null |
    awk '/^Publishers:/{inside=1; next} /^Subscribers:/{inside=0} inside && /^ \*/ {print $2}'
}

publisher_owner() {
  local topic="$1" expected="$2" publishers
  publishers="$(topic_publishers "$topic")"
  [[ "$publishers" == "$expected" ]]
}

topic_has_recent_message() {
  local topic="$1"
  timeout 8 rostopic echo --noarr -n 1 "$topic" >/dev/null 2>&1
}

transform_is_available() {
  local parent_frame="$1" child_frame="$2"
  timeout 5 rosrun tf tf_echo "$parent_frame" "$child_frame" 2>/dev/null |
    grep -m 1 -q '^[[:space:]-]*Translation:'
}

observed_string_topic_value=""
string_topic_starts_with_within() {
  local topic="$1" prefix="$2" attempt output value
  observed_string_topic_value=""
  timeout 2 rostopic info "$topic" >/dev/null 2>&1 || return 1
  for ((attempt = 0; attempt < 32; ++attempt)); do
    output="$(timeout 1 rostopic echo --noarr -n 1 "$topic" 2>/dev/null || true)"
    value="$(awk '/^data: / {sub(/^data: /, ""); print; exit}' <<<"$output")"
    value="${value#\"}"
    value="${value%\"}"
    if [[ -n "$value" ]]; then
      observed_string_topic_value="$value"
      [[ "$value" == "$prefix"* ]] && return 0
    fi
    sleep 0.25
  done
  return 1
}

topic_is_critical() {
  local topic="$1" critical_topic
  for critical_topic in "${critical_runtime_topics[@]:-}"; do
    [[ "$topic" != "$critical_topic" ]] || return 0
  done
  return 1
}

node_on_host() {
  local node="$1" expected_host="$2" uri
  uri="$(timeout 5 rosnode list -a 2>/dev/null |
    awk -v wanted="$node" '$2 == wanted {print $1; exit}')"
  [[ "$uri" == http://"$expected_host":* ]]
}

parameter_matches() {
  local parameter="$1" expected="$2" actual
  actual="$(timeout 5 rosparam get "$parameter" 2>/dev/null)" || return 1
  [[ "$actual" == "$expected" ]]
}

numeric_parameter_matches() {
  local parameter="$1" expected="$2" actual
  actual="$(timeout 5 rosparam get "$parameter" 2>/dev/null)" || return 1
  numeric_values_match "$actual" "$expected"
}

numeric_values_match() {
  local actual="$1" expected="$2"
  [[ -n "$actual" && -n "$expected" ]] || return 1
  awk -v actual="$actual" -v expected="$expected" 'BEGIN {
    difference = actual - expected
    if (difference < 0) difference = -difference
    exit !(difference <= 0.000001)
  }'
}

numeric_value_within_cap() {
  local actual="$1" cap="$2"
  [[ -n "$actual" && -n "$cap" ]] || return 1
  awk -v actual="$actual" -v cap="$cap" 'BEGIN {
    exit !(actual >= 0.0 && actual <= cap + 0.000001)
  }'
}

locateanything_expected_query_count() {
  local package_root config_path
  package_root="$(rospack find autolabor_fod_vision 2>/dev/null)" || return 1
  config_path="$package_root/config/locateanything.yaml"
  LOCATEANYTHING_CONFIG_PATH="$config_path" python3 - <<'PY'
import os

import yaml

from autolabor_fod_vision.locateanything_runtime import parse_categories

path = os.environ["LOCATEANYTHING_CONFIG_PATH"]
with open(path, "r", encoding="utf-8") as stream:
    root = yaml.safe_load(stream) or {}
if not isinstance(root, dict):
    raise SystemExit("LocateAnything YAML root must be an object")
categories = parse_categories(root.get("locateanything_categories", []))
query_count = sum(len(category.grounding_prompts) for category in categories)
if query_count < 1:
    raise SystemExit("LocateAnything YAML must define at least one query")
print(query_count)
PY
}

message_scalar_field() {
  local message="$1" field="$2"
  awk -v wanted="$field:" '$1 == wanted {print $2; exit}' <<<"$message"
}

if [[ "$mode" == --runtime ]]; then
  if ! timeout 5 rosparam list >/dev/null 2>&1; then
    fail "J6M ROS master is unreachable at $ROS_MASTER_URI"
  else
    nvidia_nodes=(/nvidia_cmd_vel_watchdog /livox_lidar_publisher2)
    j6m_nodes=(/laserMapping /avoidance_scan_fusion /move_base /fod_navigation_mode)
    runtime_nodes=("${nvidia_nodes[@]}" "${j6m_nodes[@]}")
    if [[ "$STATIC_MAP_ENABLED" != false ]]; then
      j6m_nodes+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate /coverage_manager /hybrid_teb_command_mux)
      runtime_nodes+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate /coverage_manager /hybrid_teb_command_mux)
    fi
    if [[ "$REQUIRE_CAN" != false ]]; then
      nvidia_nodes+=(/canbus_driver /m2_driver)
      runtime_nodes+=(/canbus_driver /m2_driver)
    fi
    [[ "$NVIDIA_START_VISION" != true ]] || nvidia_nodes+=(/fod_detector)
    [[ "$NVIDIA_START_CAMERA" != true ]] || nvidia_nodes+=(/zed2/zed_node)
    [[ "$NVIDIA_START_QT" != true ]] || nvidia_nodes+=(/autolabor_operator_gui /sweeper_ai)
    if [[ "$STATIC_MAP_ENABLED" == true && "$NVIDIA_START_QT" == true ]]; then
      nvidia_nodes+=(/operator_map_display_anchor)
    fi
    node_list="$(rosnode list 2>/dev/null || true)"
    for node in "${runtime_nodes[@]}"; do
      if grep -Fxq "$node" <<<"$node_list"; then pass "node $node"; else fail "node $node"; fi
    done

    for node in "${nvidia_nodes[@]}"; do
      if node_on_host "$node" "$NVIDIA_J6M_IP"; then
        pass "$node runs on NVIDIA $NVIDIA_J6M_IP"
      else
        fail "$node is not reachable on NVIDIA $NVIDIA_J6M_IP"
      fi
    done
    for node in "${j6m_nodes[@]}"; do
      if node_on_host "$node" "$J6M_IP"; then
        pass "$node runs on J6M $J6M_IP"
      else
        fail "$node is not reachable on J6M $J6M_IP"
      fi
    done

    if [[ "$STATIC_MAP_ENABLED" == true && "$NVIDIA_START_QT" == true ]]; then
      if transform_is_available map autolabor_map_display_anchor; then
        pass "pre-localization map display anchor is available without connecting robot TF"
      else
        fail "pre-localization map display anchor is unavailable"
      fi
    fi

    runtime_topics=(
      /gateway/livox/lidar /gateway/livox/imu /Odometry
      /cloud_registered_body /scan /cmd_vel_safe /cmd_vel
      /nvidia_cmd_vel_watchdog/status /fod_navigation_mode/status
    )
    critical_runtime_topics=()
    if [[ "$USE_DUAL_LIDAR" == true ]]; then
      runtime_topics+=(/dual_lidar/scan)
    fi
    [[ "$REQUIRE_CAN" == false ]] || runtime_topics+=(/odom)
    [[ "$NVIDIA_START_VISION" != true ]] || runtime_topics+=(/fod/detections)
    [[ "$NVIDIA_START_QT" != true ]] || runtime_topics+=(/sweeper_ai/status)
    if [[ "$NVIDIA_START_CAMERA" == true ]]; then
      critical_runtime_topics+=(/fod_camera/image_raw /fod_camera/depth_registered)
      runtime_topics+=("${critical_runtime_topics[@]}")
    fi
    if [[ "$STATIC_MAP_ENABLED" == true ]]; then
      runtime_topics+=(/map /fast_lio/localization_status)
    fi
    topic_check_pids=()
    camera_data_failed=false
    for topic in "${runtime_topics[@]}"; do
      topic_has_recent_message "$topic" &
      topic_check_pids+=("$!")
    done
    for topic_index in "${!runtime_topics[@]}"; do
      topic="${runtime_topics[$topic_index]}"
      if wait "${topic_check_pids[$topic_index]}"; then
        pass "message available on $topic"
      elif topic_is_critical "$topic"; then
        fail "no message received on required camera topic $topic within 8s"
        camera_data_failed=true
      elif [[ "$allow_missing_runtime_data" == true ]]; then
        warn_data "no message received on $topic within 8s (stack remains running)"
      else
        fail "no message received on $topic within 8s"
      fi
    done

    if [[ "$camera_data_failed" == true ]]; then
      echo "ZED USB diagnostic:" >&2
      "$SCRIPT_DIR/zed_camera_check.sh" --wait 0 >&2 || true
    fi

    if [[ "$NVIDIA_START_CAMERA" == true ]]; then
      if publisher_owner /fod_camera/image_raw /zed2/zed_node; then
        pass "/fod_camera/image_raw is owned by /zed2/zed_node"
      else
        fail "/fod_camera/image_raw owner is not exactly /zed2/zed_node"
      fi
      if publisher_owner /fod_camera/depth_registered /zed2/zed_node; then
        pass "/fod_camera/depth_registered is owned by /zed2/zed_node"
      else
        fail "/fod_camera/depth_registered owner is not exactly /zed2/zed_node"
      fi
    fi

    if [[ "$STATIC_MAP_ENABLED" == true && "$NVIDIA_START_QT" == true ]]; then
      if string_topic_starts_with_within \
          /autolabor_operator_gui/map_display_status 'READY;'; then
        pass "Qt embedded RViz confirmed the 2-D map texture is loaded"
      else
        fail "Qt embedded RViz map is not ready (status: ${observed_string_topic_value:-unavailable})"
      fi
      if publisher_owner /autolabor_operator_gui/map_display_status \
          /autolabor_operator_gui; then
        pass "Qt map display readiness has one GUI owner"
      else
        fail "Qt map display readiness owner is not exactly /autolabor_operator_gui"
      fi
    fi

    if parameter_matches /nvidia_cmd_vel_watchdog/motion_enabled "$MOTION_ENABLED"; then
      pass "watchdog motion_enabled matches configuration"
    else
      fail "watchdog motion_enabled does not match configuration"
    fi
    if parameter_matches /fod_visual_servo/allow_motion "$FOD_MOTION_ENABLED"; then
      pass "visual-servo motion gate matches configuration"
    else
      fail "visual-servo motion gate does not match configuration"
    fi
    if parameter_matches /fod_visual_servo/expected_model_sha256 \
         "${NVIDIA_FOD_MODEL_SHA256,,}" &&
       parameter_matches /fod_visual_servo/allowed_class_names \
         "$NVIDIA_FOD_REQUIRED_CLASS_NAMES"; then
      pass "visual-servo model contract matches configuration"
    else
      fail "visual-servo model contract does not match configuration"
    fi
    if [[ "$NVIDIA_START_VISION" == true ]]; then
      if parameter_matches /fod_detector/expected_model_sha256 \
           "${NVIDIA_FOD_MODEL_SHA256,,}" &&
         parameter_matches /fod_detector/required_class_names \
           "$NVIDIA_FOD_REQUIRED_CLASS_NAMES"; then
        pass "detector model contract matches configuration"
      else
        fail "detector model contract does not match configuration"
      fi
      if parameter_matches /fod_detector/backend "$NVIDIA_FOD_BACKEND"; then
        pass "detector backend matches configuration ($NVIDIA_FOD_BACKEND)"
      else
        fail "detector backend does not match configuration"
      fi
      if [[ "$NVIDIA_FOD_BACKEND" == yolo ]]; then
        fod_runtime_import_path="$(
          timeout 5 rosparam get /fod_detector/ultralytics_import_path \
            2>/dev/null || true
        )"
        fod_runtime_gam_layers="$(
          timeout 5 rosparam get /fod_detector/gam_layer_count \
            2>/dev/null || true
        )"
        if [[ -n "$fod_ultralytics_package_root" &&
              "$fod_runtime_import_path" == "$fod_ultralytics_package_root"/* ]]; then
          pass "detector imports retained project-local Ultralytics ($fod_runtime_import_path)"
        else
          fail "detector Ultralytics import is outside the configured project copy (${fod_runtime_import_path:-unavailable})"
        fi
        if [[ "$fod_runtime_gam_layers" =~ ^[1-9][0-9]*$ ]]; then
          pass "detector checkpoint contains GAM_Attention ($fod_runtime_gam_layers layer(s))"
        else
          fail "detector checkpoint did not report a GAM_Attention layer"
        fi
      elif [[ "$NVIDIA_FOD_BACKEND" == locateanything ]]; then
        locateanything_expected_queries="$(
          locateanything_expected_query_count 2>/dev/null || true
        )"
        locateanything_actual_queries="$(
          timeout 5 rosparam get /fod_detector/semantic_query_count \
            2>/dev/null || true
        )"
        if parameter_matches /fod_detector/runtime_path \
             "$NVIDIA_LOCATEANYTHING_MODEL_ROOT"; then
          pass "detector uses the external LocateAnything model directory"
        else
          fail "detector LocateAnything runtime path does not match configuration"
        fi
        if [[ ! "$locateanything_expected_queries" =~ ^[1-9][0-9]*$ ]]; then
          fail "cannot derive the LocateAnything query count from its YAML"
        elif parameter_matches /fod_detector/motion_eligible false &&
           parameter_matches /fod_detector/clip_filter_active true &&
           parameter_matches /fod_detector/source_pre_resize_enabled false &&
           [[ "$locateanything_actual_queries" == \
              "$locateanything_expected_queries" ]]; then
          pass "LocateAnything recognition-only gate, native input, ${locateanything_expected_queries} configured queries, and CLIP exclusion filter are active"
        else
          fail "LocateAnything recognition-only/native-input/query/CLIP runtime contract is not active (queries: runtime=${locateanything_actual_queries:-unavailable}, configured=${locateanything_expected_queries})"
        fi
        if parameter_matches /fod_vision_result_adapter/display_depth_enabled true &&
           parameter_matches /fod_vision_result_adapter/display_depth_motion_isolated true &&
           parameter_matches /fod_vision_result_adapter/depth_buffer_size 120 &&
           parameter_matches /fod_vision_result_adapter/depth_aggregation median &&
           parameter_matches /fod_vision_result_adapter/depth_cluster_method \
             organized_point_cloud_geometry; then
          pass "LocateAnything Qt-only synchronized point-cloud depth fusion is active"
        else
          fail "LocateAnything Qt-only synchronized point-cloud depth fusion contract is not active"
        fi
      else
        fod_runtime_import_path="$(
          timeout 5 rosparam get /fod_detector/ultralytics_import_path \
            2>/dev/null || true
        )"
        fod_runtime_gam_layers="$(
          timeout 5 rosparam get /fod_detector/gam_layer_count \
            2>/dev/null || true
        )"
        if [[ -n "$fod_ultralytics_package_root" &&
              "$fod_runtime_import_path" == "$fod_ultralytics_package_root"/* ]]; then
          pass "detect_and_classify imports the configured yolo11_GAM Ultralytics"
        else
          fail "detect_and_classify Ultralytics import path is unexpected (${fod_runtime_import_path:-unavailable})"
        fi
        if [[ "$fod_runtime_gam_layers" =~ ^[1-9][0-9]*$ ]] &&
           parameter_matches /fod_detector/detector_task detect &&
           parameter_matches /fod_detector/classifier_task classify &&
           parameter_matches /fod_detector/classifier_probability_dimensions 5 &&
           parameter_matches /fod_detector/model_load_count_detector 1 &&
           parameter_matches /fod_detector/model_load_count_classifier 1 &&
           parameter_matches /fod_detector/motion_eligible false; then
          pass "detect_and_classify model/task/load-once and recognition-only gates are active"
        else
          fail "detect_and_classify runtime contract is incomplete"
        fi
      fi
    fi
    if numeric_parameter_matches /nvidia_cmd_vel_watchdog/max_linear_speed "$CMD_VEL_MAX_LINEAR_SPEED"; then
      pass "watchdog linear cap matches configuration"
    else
      fail "watchdog linear cap does not match configuration"
    fi
    if numeric_parameter_matches /nvidia_cmd_vel_watchdog/max_angular_speed "$CMD_VEL_MAX_ANGULAR_SPEED"; then
      pass "watchdog angular cap matches configuration"
    else
      fail "watchdog angular cap does not match configuration"
    fi
    teb_linear_cap="$(
      timeout 5 rosparam get /move_base/TebLocalPlannerROS/max_vel_x \
        2>/dev/null || true
    )"
    teb_angular_cap="$(
      timeout 5 rosparam get /move_base/TebLocalPlannerROS/max_vel_theta \
        2>/dev/null || true
    )"
    if [[ "$STATIC_MAP_ENABLED" == true ]]; then
      # NAV_MAX_* is only the cold-start bootstrap profile.  In static-map
      # coverage mode the operator may atomically persist a different profile
      # from Qt; coverage_manager then applies that profile to TEB through
      # dynamic_reconfigure.  Treat the live coverage status as the runtime
      # authority, while still requiring every requested envelope to remain
      # inside the NVIDIA watchdog's final command caps.
      coverage_status_message="$(
        timeout 5 rostopic echo --noarr -n 1 /coverage/status \
          2>/dev/null || true
      )"
      coverage_active="$(
        message_scalar_field "$coverage_status_message" active
      )"
      coverage_linear_cap="$(
        message_scalar_field "$coverage_status_message" max_forward_speed_mps
      )"
      coverage_angular_cap="$(
        message_scalar_field "$coverage_status_message" max_angular_speed_rps
      )"
      transition_linear_cap="$(
        message_scalar_field \
          "$coverage_status_message" transition_max_forward_speed_mps
      )"
      transition_reverse_cap="$(
        message_scalar_field \
          "$coverage_status_message" transition_max_reverse_speed_mps
      )"
      transition_angular_cap="$(
        message_scalar_field \
          "$coverage_status_message" transition_max_angular_speed_rps
      )"

      if numeric_values_match "$teb_linear_cap" "$coverage_linear_cap"; then
        pass "TEB linear cap matches Qt coverage planning parameters"
      elif [[ "$coverage_active" == true ]] &&
           numeric_value_within_cap "$teb_linear_cap" "$coverage_linear_cap"; then
        pass "TEB linear cap is within the active Qt coverage planning limit"
      elif numeric_values_match "$teb_linear_cap" "$NAV_MAX_LINEAR_SPEED"; then
        pass "TEB linear cap matches the navigation bootstrap configuration"
      else
        fail "TEB linear cap matches neither Qt coverage nor bootstrap configuration"
      fi
      if numeric_values_match "$teb_angular_cap" "$coverage_angular_cap"; then
        pass "TEB angular cap matches Qt coverage planning parameters"
      elif [[ "$coverage_active" == true ]] &&
           numeric_value_within_cap "$teb_angular_cap" "$coverage_angular_cap"; then
        pass "TEB angular cap is within the active Qt coverage planning limit"
      elif numeric_values_match "$teb_angular_cap" "$NAV_MAX_ANGULAR_SPEED"; then
        pass "TEB angular cap matches the navigation bootstrap configuration"
      else
        fail "TEB angular cap matches neither Qt coverage nor bootstrap configuration"
      fi
      if numeric_value_within_cap "$coverage_linear_cap" "$CMD_VEL_MAX_LINEAR_SPEED" &&
         numeric_value_within_cap "$transition_linear_cap" "$CMD_VEL_MAX_LINEAR_SPEED" &&
         numeric_value_within_cap "$transition_reverse_cap" "$CMD_VEL_MAX_LINEAR_SPEED" &&
         numeric_value_within_cap "$coverage_angular_cap" "$CMD_VEL_MAX_ANGULAR_SPEED" &&
         numeric_value_within_cap "$transition_angular_cap" "$CMD_VEL_MAX_ANGULAR_SPEED"; then
        pass "Qt coverage and transition caps are within NVIDIA watchdog limits"
      else
        fail "Qt coverage or transition cap exceeds the NVIDIA watchdog envelope"
      fi
    else
      if numeric_values_match "$teb_linear_cap" "$NAV_MAX_LINEAR_SPEED"; then
        pass "TEB linear cap matches navigation configuration"
      else
        fail "TEB linear cap does not match navigation configuration"
      fi
      if numeric_values_match "$teb_angular_cap" "$NAV_MAX_ANGULAR_SPEED"; then
        pass "TEB angular cap matches navigation configuration"
      else
        fail "TEB angular cap does not match navigation configuration"
      fi
    fi
    if [[ "$STATIC_MAP_ENABLED" == true ]]; then
      if parameter_matches /fast_lio_map_localizer/good_matches_required 2; then
        pass "known-map localization requires consecutive ICP matches"
      else
        fail "known-map localization does not require two ICP matches"
      fi
      if parameter_matches /fast_lio_localization_cmd_vel_gate/goal_topic /move_base/goal &&
         parameter_matches /fast_lio_localization_cmd_vel_gate/cancel_topic /move_base/cancel; then
        pass "localization gate monitors and cancels move_base goals"
      else
        fail "localization gate move_base goal/cancel topics do not match"
      fi
      if publisher_owner /cmd_vel_teb /move_base; then
        pass "/cmd_vel_teb is owned by /move_base"
      else
        fail "/cmd_vel_teb owner is not exactly /move_base"
      fi
      if publisher_owner /cmd_vel_unlocalized /hybrid_teb_command_mux; then
        pass "/cmd_vel_unlocalized is owned by /hybrid_teb_command_mux"
      else
        fail "/cmd_vel_unlocalized owner is not exactly /hybrid_teb_command_mux"
      fi
      if parameter_matches /hybrid_teb_command_mux/use_tf_pose true &&
         parameter_matches /hybrid_teb_command_mux/global_frame map &&
         parameter_matches /hybrid_teb_command_mux/robot_base_frame base_link &&
         parameter_matches /hybrid_teb_command_mux/safety_topic /move_base/CoverageGlobalPlanner/hybrid_path_safe; then
        pass "Hybrid TEB mux uses the localized pose and planner safety permit"
      else
        fail "Hybrid TEB mux pose/safety contract does not match"
      fi
      hybrid_safety_topic=/move_base/CoverageGlobalPlanner/hybrid_path_safe
      if publisher_owner "$hybrid_safety_topic" /move_base; then
        pass "Hybrid path safety permit is owned by /move_base"
      else
        hybrid_safety_publishers="$(topic_publishers "$hybrid_safety_topic")"
        localization_status="$(
          timeout 2 rostopic echo --noarr -n 1 \
            /fast_lio/localization_status 2>/dev/null |
            awk '/^data: / {sub(/^data: /, ""); gsub(/^"|"$/, ""); print; exit}'
        )"
        if [[ -z "$hybrid_safety_publishers" &&
              "$localization_status" == state=WAITING_INITIAL_POSE\;* ]]; then
          pass "Hybrid path safety permit is fail-closed until initial localization initializes move_base"
        else
          fail "Hybrid path safety permit owner is not exactly /move_base"
        fi
      fi
    fi

    if publisher_owner /cmd_vel_safe /fod_navigation_mode; then
      pass "/cmd_vel_safe has one owner"
    else
      fail "/cmd_vel_safe owner is not exactly /fod_navigation_mode"
    fi
    if publisher_owner /cmd_vel /nvidia_cmd_vel_watchdog; then
      pass "/cmd_vel has one owner"
    else
      fail "/cmd_vel owner is not exactly /nvidia_cmd_vel_watchdog"
    fi
    if publisher_owner /scan /avoidance_scan_fusion; then
      pass "/scan has one owner"
    else
      fail "/scan owner is not exactly /avoidance_scan_fusion"
    fi
  fi
fi

if (( failures > 0 )); then
  echo "$failures health check(s) failed." >&2
  exit 1
fi
if (( data_warnings > 0 )); then
  echo "Health check passed ($mode) with $data_warnings missing-data warning(s)."
else
  echo "Health check passed ($mode)."
fi
