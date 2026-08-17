#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stop_mode="${1:-}"
(( $# <= 1 )) || { echo "Usage: $0 [--list-orphans]" >&2; exit 2; }
set --
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/process_control.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
RUN_TOKEN_FILE="$RUN_DIR/nvidia_run.token"

MANAGED_NVIDIA_ROS_NODES=(
  /nvidia_cmd_vel_watchdog /livox_lidar_publisher2 /canbus_driver /m2_driver
  /ld19_front /ld19_rear /dual_laser_fusion /front_lidar_tf /rear_lidar_tf
  /zed2/zed_node /zed2/zed2_state_publisher /fod_detector
  /fod_image_quality_controller /fod_ground_projector /fod_tracker
  /autolabor_operator_gui
)

status=0

managed_nvidia_node_name() {
  local command="$1" node
  local -a names=(
    nvidia_cmd_vel_watchdog livox_lidar_publisher2 canbus_driver m2_driver
    ld19_front ld19_rear dual_laser_fusion front_lidar_tf rear_lidar_tf
    zed_node zed2_state_publisher fod_detector fod_image_quality_controller
    fod_ground_projector fod_tracker autolabor_operator_gui
  )
  for node in "${names[@]}"; do
    [[ "$command" == *"__name:=$node"* ]] && return 0
  done
  return 1
}

managed_nvidia_legacy_command() {
  local _pid="$1" command="$2"
  if [[ "$command" == "$DUAL_HOST_WS/devel/lib/"* ]] &&
     managed_nvidia_node_name "$command"; then
    return 0
  fi
  if [[ "$command" == *" $DUAL_HOST_WS/devel/lib/"* ]] &&
     managed_nvidia_node_name "$command"; then
    return 0
  fi
  if [[ "$command" == /opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher\ * ]] &&
     [[ "$command" == *"__name:=zed2_state_publisher"* ]]; then
    return 0
  fi
  if [[ "$command" == *"/opt/ros/noetic/bin/roslaunch autolabor_dual_host nvidia_gateway.launch"* ||
        "$command" == *"/opt/ros/noetic/bin/roslaunch autolabor_fod_vision zed_fod_detection.launch"* ||
        "$command" == *"/opt/ros/noetic/bin/roslaunch autolabor_operator_gui operator_gui.launch"* ]]; then
    return 0
  fi
  if [[ "$command" == *"$DUAL_HOST_WS/scripts/start_nvidia.sh"* ||
        "$command" == *"$DUAL_HOST_WS/scripts/nvidia_gateway.sh"* ||
        "$command" == *"$DUAL_HOST_WS/scripts/nvidia_ui.sh"* ]]; then
    return 0
  fi
  if [[ "$command" == ssh\ *"$J6M_RUNTIME_BASE/dual_host/bin/start.sh"* ]]; then
    return 0
  fi
  return 1
}

collect_managed_nvidia_orphans() {
  local record pid
  local -A seen=()
  while IFS= read -r record; do
    [[ -n "$record" ]] || continue
    pid="${record%%:*}"
    [[ -z "${seen[$pid]:-}" ]] || continue
    seen["$pid"]=1
    printf '%s\n' "$record"
  done < <(
    dual_host_collect_tagged_process_records "$RUN_TOKEN_FILE" "$DUAL_HOST_WS"
    dual_host_collect_workspace_process_records \
      "$DUAL_HOST_WS" "$ROS_MASTER_URI" managed_nvidia_legacy_command
  )
}

recover_managed_nvidia_orphans() {
  local record pid
  local -a records=()
  mapfile -t records < <(collect_managed_nvidia_orphans)
  if (( ${#records[@]} == 0 )); then
    [[ ! -f "$RUN_TOKEN_FILE" ]] || unlink "$RUN_TOKEN_FILE"
    echo "No provenance-verified NVIDIA orphan processes remain."
    return 0
  fi

  echo "Found NVIDIA processes whose launcher records were lost; ownership was verified:"
  for record in "${records[@]}"; do
    pid="${record%%:*}"
    printf '  PID %s: %s\n' "$pid" \
      "$(dual_host_pid_command "$pid" 2>/dev/null || echo unavailable)"
  done
  dual_host_stop_records "NVIDIA dual-host orphan" "${records[@]}" || return 1

  mapfile -t records < <(collect_managed_nvidia_orphans)
  if (( ${#records[@]} > 0 )); then
    echo "Provenance-verified NVIDIA orphan processes remain after bounded shutdown." >&2
    return 1
  fi
  [[ ! -f "$RUN_TOKEN_FILE" ]] || unlink "$RUN_TOKEN_FILE"
  echo "Provenance-verified NVIDIA orphan recovery is complete."
}

if [[ "$stop_mode" == --list-orphans ]]; then
  mapfile -t listed_orphans < <(collect_managed_nvidia_orphans)
  if (( ${#listed_orphans[@]} == 0 )); then
    echo "No provenance-verified NVIDIA orphan processes found."
    exit 0
  fi
  echo "Provenance-verified NVIDIA orphan processes:"
  for listed_record in "${listed_orphans[@]}"; do
    listed_pid="${listed_record%%:*}"
    printf '  PID %s: %s\n' "$listed_pid" \
      "$(dual_host_pid_command "$listed_pid" 2>/dev/null || echo unavailable)"
  done
  exit 0
elif [[ -n "$stop_mode" ]]; then
  echo "Usage: $0 [--list-orphans]" >&2
  exit 2
fi

echo "[1/6] Cancelling the active navigation goal and pausing navigation..."
if timeout 4 rosparam list >/dev/null 2>&1; then
  if ! timeout 6 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1; then
    echo "No move_base cancel subscriber responded; continuing with bounded process shutdown." >&2
  fi
  if ! timeout 6 rosservice call /navigation_pause/set_paused 'data: true' >/dev/null 2>&1; then
    echo "Navigation pause service did not respond; continuing with bounded process shutdown." >&2
  fi
else
  echo "J6M ROS master is unavailable; there is no reachable goal to cancel."
fi

echo "[2/6] Stopping only PID-recorded NVIDIA dual-host processes..."
"$SCRIPT_DIR/start_nvidia.sh" --stop || status=$?

echo "[3/6] Recovering only provenance-verified NVIDIA orphan processes..."
recover_managed_nvidia_orphans || status=$?

echo "[4/6] Removing only unreachable registrations from the managed NVIDIA node whitelist..."
if timeout 4 rosparam list >/dev/null 2>&1; then
  cleanup_args=(--host "$NVIDIA_J6M_IP" --fail-if-live)
  for node in "${MANAGED_NVIDIA_ROS_NODES[@]}"; do
    cleanup_args+=(--node "$node")
  done
  timeout 45 "$SCRIPT_DIR/cleanup_stale_ros_nodes.py" "${cleanup_args[@]}" || status=$?
fi

echo "[5/6] Verifying that the managed CAN device is no longer open..."
if [[ -e "$CAN_PORT" ]]; then
  can_owners="$(fuser "$CAN_PORT" 2>/dev/null || true)"
  if [[ -n "$can_owners" ]]; then
    echo "CAN port remains busy after bounded NVIDIA shutdown: $CAN_PORT (PID(s):$can_owners)" >&2
    echo "Those PIDs were not killed because neither PID records nor strict provenance checks identify them as this project." >&2
    status=1
  fi
fi

echo "[6/6] Stopping and verifying the PID-recorded J6M ROS/navigation stack..."
target="$(dual_host_select_ssh 2>/dev/null || true)"
if [[ -n "$target" ]]; then
  ssh "$target" "'$J6M_RUNTIME_BASE/dual_host/bin/stop.sh'" || status=$?
else
  echo "J6M is unreachable; local processes were stopped, but remote stop was not confirmed." >&2
  status=1
fi

if (( status == 0 )); then
  echo "Dual-host shutdown complete: managed NVIDIA PID records, ROS registrations, CAN ownership and the J6M stack are clear."
  if [[ -f "$DUAL_HOST_WS/runtime/motion_authorized.ok" ]]; then
    echo "NOTE: the temporary motion authorization marker is still present; shutdown does not silently change operator authorization."
  fi
else
  echo "Dual-host shutdown found a residual condition. No unrelated host process was killed; review the messages above before restarting." >&2
fi
exit "$status"
