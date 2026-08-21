#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

mode="${1:---static}"
case "$mode" in
  --static|--network|--runtime) ;;
  *) echo "Usage: $0 --static | --network | --runtime" >&2; exit 2 ;;
esac

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

failures=0
test_results_output="$(mktemp /tmp/robot_j6m_test_results.XXXXXX)"
trap 'rm -f -- "$test_results_output"' EXIT
pass() { echo "OK   $*"; }
fail() { echo "FAIL $*" >&2; failures=$((failures + 1)); }

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

if [[ "$mode" == --runtime ]]; then
  if ! timeout 5 rosparam list >/dev/null 2>&1; then
    fail "J6M ROS master is unreachable at $ROS_MASTER_URI"
  else
    runtime_nodes=(/nvidia_cmd_vel_watchdog /livox_lidar_publisher2 /laserMapping /avoidance_scan_fusion /move_base /fod_navigation_mode)
    [[ "$STATIC_MAP_ENABLED" == false ]] || runtime_nodes+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate /coverage_manager)
    [[ "$REQUIRE_CAN" == false ]] || runtime_nodes+=(/canbus_driver /m2_driver)
    node_list="$(rosnode list 2>/dev/null || true)"
    for node in "${runtime_nodes[@]}"; do
      if grep -Fxq "$node" <<<"$node_list"; then pass "node $node"; else fail "node $node"; fi
    done

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
echo "Health check passed ($mode)."
