#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/process_control.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
PID_FILE="$RUN_DIR/nvidia_gateway.pid"
CHILD_PID_FILE="$RUN_DIR/nvidia_gateway.child.pid"
MARKER="$DUAL_HOST_WS/runtime/network_cutover.ok"
GATEWAY_CHILD_PATTERN='roslaunch([[:space:]].*)?autolabor_dual_host[[:space:]]+nvidia_gateway\.launch([[:space:]]|$)'
GATEWAY_ROS_NODES=(
  /nvidia_cmd_vel_watchdog /livox_lidar_publisher2 /canbus_driver /m2_driver
  /ld19_front /ld19_rear /dual_laser_fusion /front_lidar_tf /rear_lidar_tf
)
mkdir -p "$RUN_DIR" "$DUAL_HOST_WS/log"

stop_existing() {
  local status=0
  dual_host_stop_pid_file "$PID_FILE" "NVIDIA gateway" '(^|/)nvidia_gateway\.sh([[:space:]]|$)' || status=$?
  # If the wrapper died unexpectedly, this file still identifies its exact
  # roslaunch child by PID and process start time. Never sweep by process name.
  dual_host_stop_pid_file "$CHILD_PID_FILE" "NVIDIA gateway roslaunch" \
    "$GATEWAY_CHILD_PATTERN" || status=$?
  if (( status == 0 )); then
    rm -f -- "$PID_FILE" "$CHILD_PID_FILE"
    echo "PID-recorded NVIDIA gateway is stopped."
  fi
  return "$status"
}

case "${1:-}" in
  --stop) stop_existing; exit 0 ;;
  --check|"") ;;
  *) echo "Usage: $0 [--check|--stop]" >&2; exit 2 ;;
esac

if [[ ! -f "$MARKER" && "${ALLOW_UNCUT_NETWORK:-false}" != true ]]; then
  echo "The dedicated 192.168.10.0/24 J6M network has not been activated." >&2
  echo "Run: sudo $SCRIPT_DIR/configure_network.sh --apply" >&2
  exit 3
fi

start_livox=false
if dual_host_mode_enabled "$NVIDIA_START_LIVOX"; then start_livox=true; fi
start_can=false
if [[ "$CAN_PORT_CONFIRMED" == true ]] && dual_host_mode_enabled "$NVIDIA_START_CAN" "$CAN_PORT"; then
  start_can=true
elif [[ "$NVIDIA_START_CAN" != false ]]; then
  echo "CAN remains disabled until CAN_PORT exists and CAN_PORT_CONFIRMED=true."
fi
start_dual_lidar=false
if [[ "$DUAL_LIDAR_PORTS_CONFIRMED" == true ]] &&
   dual_host_mode_enabled "$NVIDIA_START_DUAL_LIDAR" "$FRONT_LIDAR_PORT" "$REAR_LIDAR_PORT"; then
  start_dual_lidar=true
elif [[ "$NVIDIA_START_DUAL_LIDAR" != false ]]; then
  echo "Dual LD19 remains optional/disabled until both ports are present and confirmed."
fi

if [[ "$start_livox" == true ]]; then
  [[ -r "$LIVOX_CONFIG_FILE" ]] || { echo "Unreadable Livox config: $LIVOX_CONFIG_FILE" >&2; exit 4; }
  ip -o -4 address show dev "$NVIDIA_LIVOX_INTERFACE" |
    awk '{print $4}' | grep -Fxq "$NVIDIA_LIVOX_IP/24" || {
      echo "$NVIDIA_LIVOX_INTERFACE lacks $NVIDIA_LIVOX_IP/24." >&2
      exit 4
    }
  route_device="$(ip route get "$MID360_IP" | awk '{for (i=1; i<=NF; ++i) if ($i == "dev") {print $(i+1); exit}}')"
  [[ "$route_device" == "$NVIDIA_LIVOX_INTERFACE" ]] || {
    echo "MID360 route uses $route_device, expected $NVIDIA_LIVOX_INTERFACE." >&2
    exit 4
  }
fi
if [[ "$start_can" == true && ! -w "$CAN_PORT" ]]; then
  echo "CAN port is not writable by $(id -un): $CAN_PORT" >&2
  exit 4
fi
if [[ "$start_dual_lidar" == true && ( ! -r "$FRONT_LIDAR_PORT" || ! -r "$REAR_LIDAR_PORT" ) ]]; then
  echo "A confirmed LD19 port is unreadable." >&2
  exit 4
fi

refuse_process_conflict() {
  local pattern="$1" label="$2" matches
  matches="$(pgrep -af "$pattern" 2>/dev/null || true)"
  if [[ -n "$matches" ]]; then
    echo "A pre-existing $label process would conflict with the dual-host gateway:" >&2
    printf '%s\n' "$matches" >&2
    echo "Stop the old stack explicitly, then retry; this script will not kill it." >&2
    exit 4
  fi
}

refuse_busy_port() {
  local port="$1" label="$2" owners
  owners="$(fuser "$port" 2>/dev/null || true)"
  if [[ -n "$owners" ]]; then
    echo "$label port is already open by PID(s):$owners ($port)" >&2
    exit 4
  fi
}

if [[ "$start_livox" == true ]]; then
  refuse_process_conflict '/livox_ros_driver2_node([[:space:]]|$)' "Livox"
fi
if [[ "$start_can" == true ]]; then
  refuse_process_conflict '/(m2_driver|canbus_driver)([[:space:]]|$)' "CAN/M2"
  refuse_busy_port "$CAN_PORT" "CAN"
fi
if [[ "$start_dual_lidar" == true ]]; then
  refuse_process_conflict '/ldlidar_stl_ros_node([[:space:]]|$)' "LD19"
  refuse_busy_port "$FRONT_LIDAR_PORT" "front LD19"
  refuse_busy_port "$REAR_LIDAR_PORT" "rear LD19"
fi

echo "Resolved gateway: Livox=$start_livox CAN=$start_can dual_LD19=$start_dual_lidar motion=$MOTION_ENABLED"
if [[ "${1:-}" == "--check" ]]; then
  exit 0
fi

if ! timeout 5 rosparam list >/dev/null 2>&1; then
  echo "J6M ROS master is not reachable at $ROS_MASTER_URI." >&2
  echo "Start /map/autolabor_runtime/dual_host/bin/start.sh on J6M first." >&2
  exit 5
fi

cleanup_args=(--host "$NVIDIA_J6M_IP" --fail-if-live)
for node in "${GATEWAY_ROS_NODES[@]}"; do cleanup_args+=(--node "$node"); done
if ! timeout 20 "$SCRIPT_DIR/cleanup_stale_ros_nodes.py" "${cleanup_args[@]}"; then
  echo "A live managed NVIDIA gateway node is still registered; refusing a duplicate start." >&2
  exit 7
fi

if [[ "$MOTION_ENABLED" == true ]]; then
  [[ "$start_can" == true ]] || { echo "Motion cannot be enabled without confirmed CAN." >&2; exit 6; }
  [[ -f "$DUAL_HOST_WS/runtime/motion_authorized.ok" ]] || {
    echo "Motion authorization marker is absent; keeping the vehicle inhibited." >&2
    exit 6
  }
elif [[ "$MOTION_ENABLED" != false ]]; then
  echo "MOTION_ENABLED must be literal true or false." >&2
  exit 6
fi

for node in "${GATEWAY_ROS_NODES[@]}"; do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    echo "Refusing duplicate node already registered on J6M master: $node" >&2
    exit 7
  fi
done

if dual_host_pid_file_is_owned "$PID_FILE" '(^|/)nvidia_gateway\.sh([[:space:]]|$)'; then
  old_pid="$(dual_host_pid_file_pid "$PID_FILE")"
  if dual_host_process_is_running "$old_pid"; then
    echo "NVIDIA gateway is already running (PID $old_pid)." >&2
    exit 7
  fi
fi
if dual_host_pid_file_is_owned "$CHILD_PID_FILE" "$GATEWAY_CHILD_PATTERN"; then
  old_pid="$(dual_host_pid_file_pid "$CHILD_PID_FILE")"
  echo "NVIDIA gateway roslaunch is already running (PID $old_pid)." >&2
  exit 7
fi

LOG_DIR="$DUAL_HOST_WS/log/nvidia_gateway_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
child_pid=""
self_pid_line=""
cleanup() {
  trap - EXIT INT TERM
  dual_host_stop_pid_file "$CHILD_PID_FILE" "NVIDIA gateway roslaunch" \
    "$GATEWAY_CHILD_PATTERN" || true
  if [[ -n "$child_pid" ]]; then wait "$child_pid" 2>/dev/null || true; fi
  dual_host_remove_pid_file_if_unchanged "$PID_FILE" "$self_pid_line"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
dual_host_write_pid_file "$PID_FILE" "$$"
self_pid_line="$(sed -n '1p' "$PID_FILE")"

roslaunch autolabor_dual_host nvidia_gateway.launch \
  start_livox:="$start_livox" \
  livox_config:="$LIVOX_CONFIG_FILE" \
  start_can:="$start_can" \
  can_port:="$CAN_PORT" \
  start_dual_lidar:="$start_dual_lidar" \
  front_lidar_port:="$FRONT_LIDAR_PORT" \
  rear_lidar_port:="$REAR_LIDAR_PORT" \
  lidar_center_distance_m:="$DUAL_LIDAR_CENTER_DISTANCE_M" \
  motion_enabled:="$MOTION_ENABLED" \
  max_linear_speed:="$CMD_VEL_MAX_LINEAR_SPEED" \
  max_angular_speed:="$CMD_VEL_MAX_ANGULAR_SPEED" \
  command_timeout_sec:="$CMD_VEL_TIMEOUT_SEC" \
  >"$LOG_DIR/gateway.log" 2>&1 &
child_pid=$!
if ! dual_host_write_pid_file "$CHILD_PID_FILE" "$child_pid"; then
  kill -INT "$child_pid" 2>/dev/null || true
  wait "$child_pid" 2>/dev/null || true
  exit 8
fi
echo "NVIDIA gateway started with PID $child_pid; log: $LOG_DIR/gateway.log"
wait "$child_pid"
