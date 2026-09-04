#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUAL_HOST_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<EOF
Usage: $0 --backend yolo|locateanything|detect_and_classify [--check-only | --restart-managed]

  --check-only       Validate and render the selected backend without changing
                     the live configuration.
  --restart-managed  Atomically select the backend, preserve the current map
                     mode, and schedule one complete managed cold restart.

The restart path deliberately does not preserve the one-run
--authorize-fod-motion override. Model switching is accepted only while visual
driving, coverage, move_base goals, and commanded/observed motion are inactive.
EOF
}

backend=""
mode=""
perform_stage=""
perform_stage_sha=""
perform_current_sha=""
perform_map_enabled=""
perform_map_set=""
perform_map_source=""

while (( $# > 0 )); do
  case "$1" in
    --backend)
      (( $# >= 2 )) || { echo "--backend requires yolo, locateanything, or detect_and_classify." >&2; exit 2; }
      backend="$2"
      shift 2
      ;;
    --check-only|--restart-managed)
      [[ -z "$mode" ]] || { echo "Select only one operation mode." >&2; exit 2; }
      mode="$1"
      shift
      ;;
    --perform-staged-restart)
      [[ -z "$mode" && $# -ge 7 ]] || { echo "Invalid staged restart invocation." >&2; exit 2; }
      mode="$1"
      perform_stage="$2"
      perform_stage_sha="$3"
      perform_current_sha="$4"
      perform_map_enabled="$5"
      perform_map_set="$6"
      perform_map_source="$7"
      shift 7
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

case "$backend" in
  yolo|locateanything|detect_and_classify) ;;
  *) echo "--backend must be yolo, locateanything, or detect_and_classify." >&2; exit 2 ;;
esac
[[ -n "$mode" ]] || { usage >&2; exit 2; }

source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
PREVIOUS_BACKEND="$NVIDIA_FOD_BACKEND"

CONFIG_DIR="$(cd "$(dirname "$DUAL_HOST_CONFIG")" && pwd)"
CONFIG_PATH="$CONFIG_DIR/$(basename "$DUAL_HOST_CONFIG")"
EXPECTED_YOLO_WEIGHTS="$DUAL_HOST_WS/src/application/autolabor_fod_vision/models/best6.pt"
EXPECTED_YOLO_WEIGHTS_CONFIG="src/application/autolabor_fod_vision/models/best6.pt"
EXPECTED_YOLO_SHA256="5efaafa1503db11c2ba261b4429389d96335b4eef4d0fc44d6ca41e7431f2d0f"
EXPECTED_YOLO_CLASSES="metal,plastic,paper,glass,kitchen_waste"
EXPECTED_LOCATEANYTHING_SHA256="a6a8903c529cd769270599fab141eb84f5d1d09d063fe2d1933ddf4ac8f11a15"
EXPECTED_LOCATEANYTHING_CLASSES="trash"
EXPECTED_TWO_STAGE_DETECTOR_SHA256="711b6bb4b4debebcf993f033f23e7e641a02dd279254779f8dafed11b6a79233"
EXPECTED_TWO_STAGE_CLASSIFIER_SHA256="d0cce9310e184e8acd7a6142face16d39aadc9a6e5405b18694346f2315899e9"
EXPECTED_TWO_STAGE_DETECTOR_CLASSES="trash"
EXPECTED_TWO_STAGE_CLASSES="metal,plastic,paper,glass,kitchen_waste"

target_contract() {
  case "$backend" in
    yolo)
      TARGET_SHA256="${NVIDIA_YOLO_MODEL_SHA256,,}"
      TARGET_CLASSES="$NVIDIA_YOLO_REQUIRED_CLASS_NAMES"
      TARGET_WEIGHTS="$EXPECTED_YOLO_WEIGHTS"
      ;;
    locateanything)
      TARGET_SHA256="${NVIDIA_LOCATEANYTHING_MODEL_SHA256,,}"
      TARGET_CLASSES="$NVIDIA_LOCATEANYTHING_REQUIRED_CLASS_NAMES"
      TARGET_WEIGHTS="$EXPECTED_YOLO_WEIGHTS"
      ;;
    detect_and_classify)
      TARGET_SHA256="${NVIDIA_DETECT_CLASSIFY_DETECTOR_SHA256,,}"
      TARGET_CLASSES="$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_CLASS_NAMES"
      TARGET_WEIGHTS="$NVIDIA_DETECT_CLASSIFY_DETECTOR_WEIGHTS"
      ;;
  esac
  [[ "$TARGET_SHA256" =~ ^[0-9a-f]{64}$ ]] || {
    echo "Selected backend has an invalid SHA256 contract: $backend" >&2
    return 1
  }
  [[ "$TARGET_CLASSES" =~ ^[A-Za-z0-9_]+(,[A-Za-z0-9_]+)*$ ]] || {
    echo "Selected backend has an invalid class contract: $backend" >&2
    return 1
  }
  case "$backend:$TARGET_SHA256:$TARGET_CLASSES" in
    "yolo:$EXPECTED_YOLO_SHA256:$EXPECTED_YOLO_CLASSES"|\
    "locateanything:$EXPECTED_LOCATEANYTHING_SHA256:$EXPECTED_LOCATEANYTHING_CLASSES"|\
    "detect_and_classify:$EXPECTED_TWO_STAGE_DETECTOR_SHA256:$EXPECTED_TWO_STAGE_CLASSES") ;;
    *)
      echo "Selected backend does not match the pinned model/class contract: $backend" >&2
      return 1
      ;;
  esac
  if [[ "$backend" == detect_and_classify ]]; then
    [[ "${NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256,,}" == \
       "$EXPECTED_TWO_STAGE_CLASSIFIER_SHA256" ]] || {
      echo "The pinned two-stage classifier SHA256 contract does not match." >&2
      return 1
    }
    [[ "$NVIDIA_DETECT_CLASSIFY_DETECTOR_CLASS_NAMES" == \
       "$EXPECTED_TWO_STAGE_DETECTOR_CLASSES" ]] || {
      echo "The two-stage detector class contract must be exactly trash." >&2
      return 1
    }
  fi
}

render_staged_config() {
  local output="$1"
  awk \
    -v backend="$backend" \
    -v weights="$EXPECTED_YOLO_WEIGHTS_CONFIG" \
    -v active_sha="$TARGET_SHA256" \
    -v active_classes="$TARGET_CLASSES" '
      BEGIN {
        replacement["NVIDIA_FOD_BACKEND"] = backend
        replacement["NVIDIA_FOD_WEIGHTS"] = weights
        replacement["NVIDIA_FOD_MODEL_SHA256"] = active_sha
        replacement["NVIDIA_FOD_REQUIRED_CLASS_NAMES"] = active_classes
      }
      /^[A-Za-z_][A-Za-z0-9_]*=/ {
        key = $0
        sub(/=.*/, "", key)
        if (key in replacement) {
          print key "=" replacement[key]
          seen[key]++
          next
        }
      }
      { print }
      END {
        for (key in replacement) {
          if (seen[key] != 1) {
            print "Expected exactly one " key " assignment" > "/dev/stderr"
            exit 42
          }
        }
      }
    ' "$CONFIG_PATH" >"$output"
  chmod 0600 "$output"
  bash -n "$output"
}

validate_staged_config() {
  local staged="$1"
  (
    export DUAL_HOST_CONFIG="$staged"
    source "$SCRIPT_DIR/load_config.sh"
    [[ "$NVIDIA_FOD_BACKEND" == "$backend" ]]
    [[ "${NVIDIA_FOD_MODEL_SHA256,,}" == "$TARGET_SHA256" ]]
    [[ "$NVIDIA_FOD_REQUIRED_CLASS_NAMES" == "$TARGET_CLASSES" ]]
    [[ "$NVIDIA_FOD_WEIGHTS" == "$TARGET_WEIGHTS" ]]
    dual_host_validate_fod_model_contract
    dual_host_validate_fod_weights
  )
}

assert_runtime_switch_safe() {
  local probe_status=0
  if ! timeout 3 rosparam list >/dev/null 2>&1; then
    echo "ROS master 当前不可达，不能确认停车状态或切换视觉模型。" >&2
    return 1
  fi
  if timeout 20 python3 - <<'PY'
import math
import sys

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def fail(message):
    print(message, file=sys.stderr)
    raise SystemExit(1)


def message(topic, message_type, timeout=2.0):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout)
    except Exception as error:
        fail("模型切换安全检查缺少 {}：{}".format(topic, error))


rospy.init_node("fod_model_switch_guard", anonymous=True, disable_signals=True)
mode = message("/fod_navigation_mode/state", String).data.strip()
visual = message("/fod_visual_servo/state", String).data.strip()
coverage_active = bool(message("/coverage/active", Bool).data)
odom = message("/odom", Odometry)
cmd_safe = message("/cmd_vel_safe", Twist)
cmd_final = message("/cmd_vel", Twist)
move_base = message("/move_base/status", GoalStatusArray)

if mode != "GPS_ACTIVE":
    fail("请先退出视觉行驶模式并恢复相对导航（当前：{}）".format(mode or "未知"))
if visual not in {"DISABLED", "COMPLETE", "ABORT"}:
    fail("视觉控制器尚未停车（当前：{}）".format(visual or "未知"))
if coverage_active:
    fail("覆盖清扫任务仍在活动，不能切换视觉模型")

active_goal_states = {
    GoalStatus.PENDING,
    GoalStatus.ACTIVE,
    GoalStatus.PREEMPTING,
    GoalStatus.RECALLING,
}
if any(status.status in active_goal_states for status in move_base.status_list):
    fail("move_base 仍有活动目标，请先取消并等待停车")


def twist_speed(twist):
    linear = math.sqrt(
        twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2
    )
    angular = math.sqrt(
        twist.angular.x ** 2 + twist.angular.y ** 2 + twist.angular.z ** 2
    )
    return linear, angular


checks = {
    "/odom": twist_speed(odom.twist.twist),
    "/cmd_vel_safe": twist_speed(cmd_safe),
    "/cmd_vel": twist_speed(cmd_final),
}
for topic, (linear, angular) in checks.items():
    if not math.isfinite(linear) or not math.isfinite(angular):
        fail("{} 包含非法速度".format(topic))
    if linear > 0.02 or angular > 0.05:
        fail(
            "车辆尚未停车：{} linear={:.3f} m/s angular={:.3f} rad/s".format(
                topic, linear, angular
            )
        )

print("视觉模型切换停车/空闲检查通过")
PY
  then
    return 0
  else
    probe_status=$?
    if (( probe_status == 124 )); then
      echo "视觉模型切换停车/空闲检查超时，拒绝修改配置。" >&2
    fi
    return "$probe_status"
  fi
}

make_stage() {
  local staged
  staged="$(mktemp "$CONFIG_DIR/.dual_host.env.model-switch.XXXXXX")"
  render_staged_config "$staged"
  validate_staged_config "$staged"
  printf '%s\n' "$staged"
}

schedule_restart() {
  local staged="$1" staged_sha current_sha map_enabled map_set map_source
  local stamp unit variable value
  local -a restart_command systemd_command

  assert_runtime_switch_safe
  current_sha="$(sha256sum -- "$CONFIG_PATH" | awk '{print $1}')"
  staged_sha="$(sha256sum -- "$staged" | awk '{print $1}')"

  map_enabled=false
  map_set=""
  map_source=fused
  if [[ -r "$DUAL_HOST_WS/runtime/run/map_mode.env" ]]; then
    # Generated by start_dual_host.sh with shell-escaped values and owned by
    # the current user. It is read before the managed service is stopped.
    source "$DUAL_HOST_WS/runtime/run/map_mode.env"
    map_enabled="${STATIC_MAP_ENABLED:-false}"
    map_set="${STATIC_MAP_SET:-}"
    map_source="${STATIC_MAP_SOURCE_MODE:-fused}"
  fi
  case "$map_enabled:$map_source" in
    true:fused|true:lidar2d|false:fused|false:lidar2d) ;;
    *) echo "Current map-mode record is invalid." >&2; return 1 ;;
  esac
  if [[ "$map_enabled" == true ]]; then
    [[ -d "$map_set" ]] || {
      echo "Current map set is unavailable: ${map_set:-<empty>}" >&2
      return 1
    }
  else
    map_set=""
  fi

  stamp="$(date +%Y%m%d_%H%M%S)"
  unit="autolabor-fod-model-switch-${stamp}-$$"
  restart_command=(
    "$SCRIPT_DIR/switch_fod_backend.sh"
    --backend "$backend"
    --perform-staged-restart "$staged" "$staged_sha" "$current_sha"
    "$map_enabled" "$map_set" "$map_source"
  )
  systemd_command=(
    systemd-run --user
    --unit="$unit"
    --collect
    --service-type=exec
    --working-directory="$DUAL_HOST_WS"
    --description="Autolabor FOD model switch to $backend"
    --property=KillMode=control-group
    --property=KillSignal=SIGINT
    --property=TimeoutStopSec=180s
    --setenv="DUAL_HOST_CONFIG=$CONFIG_PATH"
  )
  for variable in DISPLAY XAUTHORITY DBUS_SESSION_BUS_ADDRESS XDG_RUNTIME_DIR LANG LC_ALL; do
    value="${!variable:-}"
    [[ -z "$value" ]] || systemd_command+=(--setenv="$variable=$value")
  done
  systemd_command+=("${restart_command[@]}")
  "${systemd_command[@]}"
  printf 'SCHEDULED|%s|%s\n' "$backend" "$unit"
}

perform_staged_restart() {
  local staged_real expected_prefix actual_stage_sha actual_current_sha rollback
  local restart_status=0 live_backend=""
  local -a restart_args=(--restart)

  expected_prefix="$CONFIG_DIR/.dual_host.env.model-switch."
  staged_real="$(readlink -m -- "$perform_stage")"
  [[ "$staged_real" == "$expected_prefix"* && -f "$staged_real" ]] || {
    echo "Refusing an unexpected staged configuration path." >&2
    return 1
  }
  [[ "$(stat -c %u -- "$staged_real")" == "$(id -u)" ]] || {
    echo "Staged configuration has the wrong owner." >&2
    return 1
  }
  actual_stage_sha="$(sha256sum -- "$staged_real" | awk '{print $1}')"
  [[ "$actual_stage_sha" == "$perform_stage_sha" ]] || {
    echo "Staged configuration changed after validation." >&2
    return 1
  }
  actual_current_sha="$(sha256sum -- "$CONFIG_PATH" | awk '{print $1}')"
  [[ "$actual_current_sha" == "$perform_current_sha" ]] || {
    echo "The active configuration changed while the model switch was pending." >&2
    return 1
  }
  validate_staged_config "$staged_real"
  assert_runtime_switch_safe

  case "$perform_map_enabled:$perform_map_source" in
    true:fused|true:lidar2d)
      [[ -d "$perform_map_set" ]] || {
        echo "Preserved map set is unavailable: ${perform_map_set:-<empty>}" >&2
        return 1
      }
      restart_args+=(--map-set "$perform_map_set" --static-map-source "$perform_map_source")
      ;;
    false:fused|false:lidar2d) ;;
    *) echo "Invalid preserved map mode." >&2; return 1 ;;
  esac

  rollback="$(mktemp "$CONFIG_DIR/.dual_host.env.model-switch-rollback.XXXXXX")"
  cp --preserve=mode,ownership,timestamps -- "$CONFIG_PATH" "$rollback"
  chmod 0600 "$rollback"
  mv -f -- "$staged_real" "$CONFIG_PATH"
  echo "Visual backend configuration changed to $backend; starting a complete cold restart."
  echo "The one-run FOD motion authorization is intentionally not carried into the new run."

  if "$SCRIPT_DIR/start_dual_host.sh" "${restart_args[@]}"; then
    unlink -- "$rollback"
    echo "Visual backend switch completed: $backend"
    return 0
  else
    restart_status=$?
    live_backend="$(timeout 3 rosparam get /fod_detector/backend 2>/dev/null || true)"
    if systemctl --user is-active autolabor-dual-host.service >/dev/null 2>&1 &&
       [[ "$live_backend" == "$PREVIOUS_BACKEND" ]]; then
      mv -f -- "$rollback" "$CONFIG_PATH"
      echo "Cold restart failed before the old managed stack stopped; configuration was restored." >&2
    else
      unlink -- "$rollback"
      echo "Cold restart failed after the old detector exited; selected configuration is retained for diagnosis." >&2
    fi
    return "$restart_status"
  fi
}

target_contract

if [[ "$mode" == --perform-staged-restart ]]; then
  trap '[[ -z "${perform_stage:-}" || ! -f "${perform_stage:-}" ]] || unlink -- "$perform_stage"' EXIT
  perform_staged_restart
  exit $?
fi

stage=""
trap '[[ -z "${stage:-}" || ! -f "${stage:-}" ]] || unlink -- "$stage"' EXIT
stage="$(make_stage)"

if [[ "$mode" == --check-only ]]; then
  printf 'VALID|backend=%s|sha256=%s|classes=%s|weights=%s' \
    "$backend" "$TARGET_SHA256" "$TARGET_CLASSES" "$TARGET_WEIGHTS"
  if [[ "$backend" == detect_and_classify ]]; then
    printf '|classifier_sha256=%s|classifier_weights=%s' \
      "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_SHA256" \
      "$NVIDIA_DETECT_CLASSIFY_CLASSIFIER_WEIGHTS"
  fi
  printf '\n'
  exit 0
fi

if [[ "$NVIDIA_FOD_BACKEND" == "$backend" ]]; then
  printf 'UNCHANGED|%s is already the configured backend\n' "$backend"
  exit 0
fi

schedule_restart "$stage"
# Ownership of the staged file has passed to the independent transient unit.
stage=""
