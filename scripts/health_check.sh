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
warn_data() { echo "WARN $*" >&2; data_warnings=$((data_warnings + 1)); }

if dual_host_validate_fod_model_contract; then
  pass "FOD detector/controller model contract is valid"
else
  fail "FOD detector/controller model contract is invalid"
fi
if dual_host_validate_fod_weights; then
  pass "FOD weights match the configured SHA256"
else
  fail "FOD weights do not match the configured SHA256"
fi

required_packages=(
  autolabor_coverage autolabor_dual_host autolabor_dual_lidar autolabor_fod_control
  autolabor_fod_msgs autolabor_operator_gui fast_lio fast_lio_localization livox_ros_driver2
  robot_bringup teb_local_planner zed_wrapper map_server
)
for package in "${required_packages[@]}"; do
  if rospack find "$package" >/dev/null 2>&1; then
    pass "package $package"
  else
    fail "package $package"
  fi
done

if roslaunch --files autolabor_dual_host nvidia_gateway.launch >/dev/null 2>&1 &&
   roslaunch --files autolabor_dual_host j6m_fastlio_navigation.launch >/dev/null 2>&1; then
  pass "dual-host launch files resolve"
else
  fail "dual-host launch files do not resolve"
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

publisher_owner() {
  local topic="$1" expected="$2" publishers
  publishers="$(rostopic info "$topic" 2>/dev/null |
    awk '/^Publishers:/{inside=1; next} /^Subscribers:/{inside=0} inside && /^ \*/ {print $2}')"
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
  awk -v actual="$actual" -v expected="$expected" 'BEGIN {
    difference = actual - expected
    if (difference < 0) difference = -difference
    exit !(difference <= 0.000001)
  }'
}

if [[ "$mode" == --runtime ]]; then
  if ! timeout 5 rosparam list >/dev/null 2>&1; then
    fail "J6M ROS master is unreachable at $ROS_MASTER_URI"
  else
    nvidia_nodes=(/nvidia_cmd_vel_watchdog /livox_lidar_publisher2)
    j6m_nodes=(/laserMapping /avoidance_scan_fusion /move_base /fod_navigation_mode)
    runtime_nodes=("${nvidia_nodes[@]}" "${j6m_nodes[@]}")
    if [[ "$STATIC_MAP_ENABLED" != false ]]; then
      j6m_nodes+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate /coverage_manager)
      runtime_nodes+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate /coverage_manager)
    fi
    if [[ "$REQUIRE_CAN" != false ]]; then
      nvidia_nodes+=(/canbus_driver /m2_driver)
      runtime_nodes+=(/canbus_driver /m2_driver)
    fi
    [[ "$NVIDIA_START_VISION" != true ]] || nvidia_nodes+=(/fod_detector)
    [[ "$NVIDIA_START_CAMERA" != true ]] || nvidia_nodes+=(/zed2/zed_node)
    [[ "$NVIDIA_START_QT" != true ]] || nvidia_nodes+=(/autolabor_operator_gui)
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
    if numeric_parameter_matches /move_base/TebLocalPlannerROS/max_vel_x "$NAV_MAX_LINEAR_SPEED"; then
      pass "TEB linear cap matches navigation configuration"
    else
      fail "TEB linear cap does not match navigation configuration"
    fi
    if numeric_parameter_matches /move_base/TebLocalPlannerROS/max_vel_theta "$CMD_VEL_MAX_ANGULAR_SPEED"; then
      pass "TEB angular cap matches NVIDIA watchdog"
    else
      fail "TEB angular cap does not match NVIDIA watchdog"
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
