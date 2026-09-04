#!/usr/bin/env bash
set -euo pipefail

tree_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="${tree_root}/ws"
label="${1:-baseline_current_architecture}"
mode="${2:-baseline}"
case "${mode}" in
  baseline|simplified|simplified_cross|recommended|fault_test|direct_event|direct_1hz) ;;
  *)
    printf 'unsupported experiment mode: %s\n' "${mode}" >&2
    exit 2
    ;;
esac
shift $(( $# >= 2 ? 2 : $# ))
run_stamp="$(date +%Y%m%d_%H%M%S)"
result_dir="${tree_root}/results/${label}_${run_stamp}"
map_yaml="/home/slam/robot_j6m_ws/global_maps/map_sets/latest/map_fused_2d/map.yaml"
resolved_map_yaml="$(readlink -f "${map_yaml}")"
mkdir -p "${result_dir}"
ln -sfn "${result_dir}" "${tree_root}/runtime/latest_result"

source /opt/ros/noetic/setup.bash
source /home/slam/robot_j6m_ws/devel/setup.bash
source "${workspace_root}/devel/setup.bash"

# The project build already links against this pinned Noetic dependency sysroot.
# Expose its package manifests as well so roslaunch can resolve the upstream
# move_base and map_server executables used only by this isolated simulator.
navigation_sysroot="/home/slam/robot_ws/.deps/sysroot/opt/ros/noetic"
export ROS_PACKAGE_PATH="${navigation_sysroot}/share:${ROS_PACKAGE_PATH:-}"

export ROS_MASTER_URI="http://127.0.0.1:11321"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME || true
export GAZEBO_MASTER_URI="http://127.0.0.1:11351"
# Gazebo Classic otherwise advertises transport discovery on every host
# interface.  The navigation simulator is intentionally self-contained, so
# keep its transport traffic on loopback as well.  This avoids a tight stream
# of multicast send errors when a robot Ethernet link is down and prevents
# that unrelated logging load from distorting transition timing.
export GAZEBO_IP="127.0.0.1"

printf '%s\n' "${result_dir}" > "${tree_root}/runtime/latest_result_path"
printf '%s\n' "${resolved_map_yaml}" > "${result_dir}/map_yaml_resolved.txt"
set +e
roslaunch coverage_gz_sim "${mode}.launch" \
  run_label:="${label}" \
  result_dir:="${result_dir}" \
  gui:=false \
  "$@"
launch_status=$?
set -e

audit_status=0
if [[ -s "${result_dir}/navigation.bag" ]]; then
  python3 "${workspace_root}/src/coverage_gz_sim/scripts/audit_navigation_bag.py" \
    --bag "${result_dir}/navigation.bag" \
    --map-yaml "${resolved_map_yaml}" \
    --minimum-turning-radius 1.35 \
    --output "${result_dir}/audit_summary.json" || audit_status=$?
  python3 "${workspace_root}/src/coverage_gz_sim/scripts/analyze_navigation_details.py" \
    --bag "${result_dir}/navigation.bag" \
    --output "${result_dir}/navigation_details.json" || audit_status=$?
  python3 "${workspace_root}/src/coverage_gz_sim/scripts/analyze_teb_plan_deviation.py" \
    --bag "${result_dir}/navigation.bag" \
    --output "${result_dir}/teb_plan_deviation.json" || audit_status=$?
  if [[ "${mode}" == direct_event || "${mode}" == direct_1hz ]]; then
    python3 "${workspace_root}/src/coverage_gz_sim/scripts/check_transition_budget.py" \
      --details "${result_dir}/navigation_details.json" \
      --output "${result_dir}/transition_budget.json" \
      --limit-sec 10.0 \
      --timing-advisory || audit_status=$?
  fi
fi

if (( launch_status != 0 )); then
  exit "${launch_status}"
fi
exit "${audit_status}"
