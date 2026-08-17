#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/process_control.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
PID_FILE="$RUN_DIR/nvidia_stack.pid"
mkdir -p "$RUN_DIR"

stop_stack() {
  local status=0
  # Stop children synchronously before touching the wrapper. This gives each
  # roslaunch supervisor its full grace period and avoids overlapping cleanup.
  "$SCRIPT_DIR/nvidia_ui.sh" --stop || status=$?
  "$SCRIPT_DIR/nvidia_gateway.sh" --stop || status=$?
  dual_host_stop_pid_file "$PID_FILE" "NVIDIA stack wrapper" \
    '(^|/)start_nvidia\.sh([[:space:]]|$)' || status=$?
  if (( status == 0 )); then
    rm -f "$PID_FILE"
    echo "PID-recorded NVIDIA stack is stopped. J6M was not stopped."
  fi
  return "$status"
}

case "${1:-}" in
  --stop) stop_stack; exit 0 ;;
  "") ;;
  *) echo "Usage: $0 [--stop]" >&2; exit 2 ;;
esac

if dual_host_pid_file_is_owned "$PID_FILE" '(^|/)start_nvidia\.sh([[:space:]]|$)'; then
  old_pid="$(dual_host_pid_file_pid "$PID_FILE")"
  if dual_host_process_is_running "$old_pid"; then
    echo "NVIDIA stack is already running (PID $old_pid)." >&2
    exit 3
  fi
fi

gateway_pid=""
ui_pid=""
cleanup_started=false
self_pid_line=""
cleanup() {
  [[ "$cleanup_started" == false ]] || return 0
  cleanup_started=true
  trap - EXIT INT TERM
  "$SCRIPT_DIR/nvidia_ui.sh" --stop >/dev/null 2>&1 || true
  "$SCRIPT_DIR/nvidia_gateway.sh" --stop >/dev/null 2>&1 || true
  dual_host_remove_pid_file_if_unchanged "$PID_FILE" "$self_pid_line"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
dual_host_write_pid_file "$PID_FILE" "$$"
self_pid_line="$(sed -n '1p' "$PID_FILE")"

"$SCRIPT_DIR/nvidia_gateway.sh" &
gateway_pid=$!

deadline=$((SECONDS + WAIT_FOR_NVIDIA_SEC + 60))
required_nodes=(/nvidia_cmd_vel_watchdog /livox_lidar_publisher2 /laserMapping /avoidance_scan_fusion /move_base /fod_navigation_mode)
echo "Waiting for the J6M FAST-LIO navigation graph..."
while true; do
  if ! kill -0 "$gateway_pid" 2>/dev/null; then
    wait "$gateway_pid" || status=$?
    echo "NVIDIA gateway exited before J6M navigation became ready (status ${status:-0})." >&2
    exit "${status:-1}"
  fi

  ready=true
  for node in "${required_nodes[@]}"; do
    if ! timeout 3 rosnode ping -c 1 "$node" >/dev/null 2>&1; then
      ready=false
      break
    fi
  done
  [[ "$ready" == false ]] || break
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for the J6M navigation graph." >&2
    exit 4
  fi
  sleep 1
done

if [[ "$NVIDIA_START_VISION" == true || "$NVIDIA_START_QT" == true ]]; then
  "$SCRIPT_DIR/nvidia_ui.sh" &
  ui_pid=$!
fi

echo "NVIDIA gateway is ready; J6M FAST-LIO/move_base is online."
echo "Motion gate: MOTION_ENABLED=$MOTION_ENABLED, FOD_MOTION_ENABLED=$FOD_MOTION_ENABLED"
echo "Press Ctrl+C to stop NVIDIA components only."

set +e
if [[ -n "$ui_pid" ]]; then
  wait -n "$gateway_pid" "$ui_pid"
else
  wait "$gateway_pid"
fi
status=$?
set -e
exit "$status"
