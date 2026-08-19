#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage: $0 [--start | --restart | --status | --stop | --foreground]

  --start       Start the complete stack as a managed user service (default).
  --restart     Cold-stop both hosts, then start the managed service.
  --status      Show service state and run the dual-host health check.
  --stop        Stop both hosts synchronously and verify all residuals.
  --foreground  Diagnostic mode: keep the supervisor attached to this terminal.

The default start waits until the complete graph is ready, then returns to the
shell. The stack remains owned by autolabor-dual-host.service; closing this
terminal or restarting the graphical desktop cannot orphan its ROS children.
EOF
}

mode="${1:---start}"
(( $# <= 1 )) || { usage >&2; exit 2; }
case "$mode" in
  --start|--restart|--status|--stop|--foreground|--supervise) ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

# Catkin setup files inspect the caller's positional parameters when sourced.
set --
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

RUN_DIR="$DUAL_HOST_WS/runtime/run"
READY_FILE="$RUN_DIR/dual_host.ready"
RUN_TOKEN_FILE="$RUN_DIR/nvidia_run.token"
SERVICE_TOKEN_FILE="$RUN_DIR/service_run.token"
SERVICE_UNIT="autolabor-dual-host.service"
mkdir -p "$RUN_DIR" "$DUAL_HOST_WS/log"

REQUIRED_NODES=(
  /laserMapping
  /relay_livox_imu
  /relay_livox_lidar
  /livox_custom_to_pointcloud
  /mid360_pointcloud_to_laserscan
  /avoidance_scan_fusion
  /move_base
  /navigation_pause
  /optional_cloud_enhancer
  /fod_navigation_mode
  /fod_visual_servo
  /nvidia_cmd_vel_watchdog
)

if [[ "$STATIC_MAP_ENABLED" == true ]]; then
  REQUIRED_NODES+=(/map_server /amcl)
elif [[ "$STATIC_MAP_ENABLED" != false ]]; then
  echo "STATIC_MAP_ENABLED must be literal true or false." >&2
  exit 3
fi

if dual_host_mode_enabled "$NVIDIA_START_LIVOX"; then
  REQUIRED_NODES+=(/livox_lidar_publisher2)
fi
if [[ "$CAN_PORT_CONFIRMED" == true ]] &&
   dual_host_mode_enabled "$NVIDIA_START_CAN" "$CAN_PORT"; then
  REQUIRED_NODES+=(/canbus_driver /m2_driver)
fi
if [[ "$NVIDIA_START_VISION" == true ]]; then
  REQUIRED_NODES+=(/fod_detector)
fi
if [[ "$NVIDIA_START_CAMERA" == true ]]; then
  REQUIRED_NODES+=(/zed2/zed_node)
fi
if [[ "$NVIDIA_START_QT" == true ]]; then
  REQUIRED_NODES+=(/autolabor_operator_gui)
fi

ros_master_reachable() {
  timeout 5 rosparam list >/dev/null 2>&1
}

missing_runtime_nodes() {
  local node_list node
  if ! node_list="$(timeout 8 rosnode list 2>/dev/null)"; then
    echo "ROS_MASTER"
    return 0
  fi
  for node in "${REQUIRED_NODES[@]}"; do
    if ! grep -Fxq "$node" <<<"$node_list"; then
      echo "$node"
    fi
  done
}

runtime_ready() {
  [[ -z "$(missing_runtime_nodes)" ]]
}

show_runtime_status() {
  local missing
  missing="$(missing_runtime_nodes)"
  if [[ -n "$missing" ]]; then
    echo "Dual-host stack is not fully ready. Missing:" >&2
    sed 's/^/  - /' <<<"$missing" >&2
    return 1
  fi
  "$SCRIPT_DIR/health_check.sh" --runtime
  echo "Qt, ZED, YOLO, FAST-LIO, localization, navigation, MID360 and CAN are ready."
}

service_state() {
  systemctl --user is-active "$SERVICE_UNIT" 2>/dev/null || true
}

service_is_running() {
  local state
  state="$(service_state)"
  [[ "$state" == active || "$state" == activating || "$state" == deactivating ]]
}

remove_ready_file() {
  [[ ! -f "$READY_FILE" ]] || unlink "$READY_FILE"
}

write_single_line_file() {
  local destination="$1" value="$2" temporary
  temporary="${destination}.tmp.$$"
  printf '%s\n' "$value" >"$temporary"
  mv -f -- "$temporary" "$destination"
}

ready_file_matches() {
  local expected="$1" actual
  [[ -r "$READY_FILE" ]] || return 1
  IFS= read -r actual <"$READY_FILE" || return 1
  [[ "$actual" == "$expected" ]]
}

remove_service_token_if_matches() {
  local expected="$1" actual
  [[ -r "$SERVICE_TOKEN_FILE" ]] || return 0
  IFS= read -r actual <"$SERVICE_TOKEN_FILE" || return 0
  [[ "$actual" != "$expected" ]] || unlink "$SERVICE_TOKEN_FILE"
}

manager_environment_value() {
  local key="$1" line
  while IFS= read -r line; do
    [[ "${line%%=*}" == "$key" ]] || continue
    printf '%s\n' "${line#*=}"
    return 0
  done < <(systemctl --user show-environment 2>/dev/null)
  return 1
}

show_service_log() {
  journalctl _SYSTEMD_USER_UNIT="$SERVICE_UNIT" -n 120 --no-pager 2>/dev/null ||
    journalctl --user -u "$SERVICE_UNIT" -n 120 --no-pager 2>/dev/null || true
}

wait_for_managed_service() {
  local token="$1" deadline state last_report=$SECONDS
  deadline=$((SECONDS + WAIT_FOR_NVIDIA_SEC + 240))
  while true; do
    state="$(service_state)"
    if ready_file_matches "$token" && runtime_ready; then
      show_runtime_status
      echo
      echo "Dual-host project is ready and managed by $SERVICE_UNIT."
      echo "This terminal may now be closed. Use '$0 --stop' for a clean stop."
      return 0
    fi
    if [[ "$state" != active && "$state" != activating ]]; then
      echo "Managed dual-host service exited before readiness (state: ${state:-not-found})." >&2
      show_service_log >&2
      remove_service_token_if_matches "$token"
      return 1
    fi
    if (( SECONDS >= deadline )); then
      echo "Timed out waiting for the managed dual-host service." >&2
      printf 'Still missing:\n%s\n' "$(missing_runtime_nodes)" >&2
      show_service_log >&2
      return 1
    fi
    if (( SECONDS - last_report >= 10 )); then
      echo "Managed startup is still running; hardware and ROS readiness checks are in progress..."
      last_report=$SECONDS
    fi
    sleep 1
  done
}

start_managed_service() {
  local token display xauthority dbus_address runtime_dir variable value state deadline
  local -a command=(
    systemd-run --user
    --unit="$SERVICE_UNIT"
    --collect
    --service-type=exec
    --working-directory="$DUAL_HOST_WS"
    --description="Autolabor J6M/NVIDIA dual-host supervisor"
    --property=KillMode=control-group
    --property=KillSignal=SIGINT
    --property=TimeoutStopSec=180s
  )

  state="$(service_state)"
  if [[ "$state" == deactivating ]]; then
    echo "$SERVICE_UNIT is finishing a previous bounded shutdown; waiting..."
    deadline=$((SECONDS + 185))
    while [[ "$(service_state)" == deactivating && SECONDS -lt deadline ]]; do
      sleep 1
    done
    state="$(service_state)"
  fi
  if [[ "$state" == active || "$state" == activating ]]; then
    token="$(sed -n '1p' "$SERVICE_TOKEN_FILE" 2>/dev/null || true)"
    if [[ ! "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
      token="$(sed -n '1p' "$RUN_TOKEN_FILE" 2>/dev/null || true)"
    fi
    if [[ ! "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
      echo "$SERVICE_UNIT is already active but has no valid run token." >&2
      echo "Run '$0 --restart' to rebuild its ownership records." >&2
      return 1
    fi
    echo "$SERVICE_UNIT is already active; waiting for full readiness..."
    wait_for_managed_service "$token"
    return
  fi

  systemctl --user reset-failed "$SERVICE_UNIT" >/dev/null 2>&1 || true
  remove_ready_file
  token="$(< /proc/sys/kernel/random/uuid)"
  [[ "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]] || {
    echo "Cannot create a valid dual-host run token." >&2
    return 1
  }

  display="${DISPLAY:-$(manager_environment_value DISPLAY 2>/dev/null || true)}"
  xauthority="${XAUTHORITY:-$(manager_environment_value XAUTHORITY 2>/dev/null || true)}"
  dbus_address="${DBUS_SESSION_BUS_ADDRESS:-$(manager_environment_value DBUS_SESSION_BUS_ADDRESS 2>/dev/null || true)}"
  runtime_dir="${XDG_RUNTIME_DIR:-$(manager_environment_value XDG_RUNTIME_DIR 2>/dev/null || true)}"
  if [[ "$NVIDIA_START_QT" == true && ( -z "$display" || -z "$xauthority" ) ]]; then
    echo "Qt is enabled, but DISPLAY/XAUTHORITY cannot be determined." >&2
    echo "Start once from the logged-in desktop terminal, or use --foreground for diagnosis." >&2
    return 1
  fi

  command+=(--setenv="DUAL_HOST_RUN_TOKEN=$token")
  command+=(--setenv="DUAL_HOST_CONFIG=$DUAL_HOST_CONFIG")
  for variable in DISPLAY XAUTHORITY DBUS_SESSION_BUS_ADDRESS XDG_RUNTIME_DIR LANG LC_ALL; do
    case "$variable" in
      DISPLAY) value="$display" ;;
      XAUTHORITY) value="$xauthority" ;;
      DBUS_SESSION_BUS_ADDRESS) value="$dbus_address" ;;
      XDG_RUNTIME_DIR) value="$runtime_dir" ;;
      *) value="${!variable:-}" ;;
    esac
    [[ -z "$value" ]] || command+=(--setenv="$variable=$value")
  done
  command+=("$SCRIPT_DIR/start_dual_host.sh" --supervise)

  echo "Starting $SERVICE_UNIT; this command will return after the full health check passes..."
  write_single_line_file "$SERVICE_TOKEN_FILE" "$token"
  if ! "${command[@]}"; then
    [[ ! -f "$SERVICE_TOKEN_FILE" ]] || unlink "$SERVICE_TOKEN_FILE"
    return 1
  fi
  wait_for_managed_service "$token"
}

stop_managed_service() {
  local service_status=0
  if service_is_running; then
    echo "Stopping $SERVICE_UNIT and its complete cgroup..."
    systemctl --user stop "$SERVICE_UNIT" || service_status=$?
  fi
  # A second bounded pass handles a supervisor that died before systemd could
  # run its EXIT trap, and also supports stacks started by older revisions.
  "$SCRIPT_DIR/stop_dual_host.sh" || service_status=$?
  remove_ready_file
  [[ ! -f "$SERVICE_TOKEN_FILE" ]] || unlink "$SERVICE_TOKEN_FILE"
  systemctl --user reset-failed "$SERVICE_UNIT" >/dev/null 2>&1 || true
  return "$service_status"
}

if [[ "$mode" == --stop ]]; then
  stop_managed_service
  exit $?
fi

if [[ "$mode" == --status ]]; then
  echo "Supervisor service: $(service_state) ($SERVICE_UNIT)"
  show_runtime_status
  exit $?
fi

if [[ "$NVIDIA_START_QT" != true || "$NVIDIA_START_VISION" != true ||
      "$NVIDIA_START_CAMERA" != true ]]; then
  echo "Full bringup requires NVIDIA_START_QT/VISION/CAMERA=true." >&2
  echo "Update $DUAL_HOST_CONFIG before starting." >&2
  exit 3
fi

if [[ "$MOTION_ENABLED" == true ]]; then
  if [[ ! -f "$DUAL_HOST_WS/runtime/motion_authorized.ok" ]]; then
    echo "MOTION_ENABLED=true but the motion authorization marker is absent." >&2
    exit 3
  fi
  echo "WARNING: this cold start has an active motion authorization marker." >&2
elif [[ "$MOTION_ENABLED" != false ]]; then
  echo "MOTION_ENABLED must be literal true or false." >&2
  exit 3
fi

if [[ "$mode" == --restart ]]; then
  stop_managed_service
  start_managed_service
  exit $?
elif [[ "$mode" == --start ]]; then
  start_managed_service
  exit $?
fi

# --supervise is the systemd-owned implementation. --foreground uses the same
# lifecycle for diagnostics but remains attached to the invoking terminal.
if [[ ! "${DUAL_HOST_RUN_TOKEN:-}" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
  DUAL_HOST_RUN_TOKEN="$(< /proc/sys/kernel/random/uuid)"
fi
export DUAL_HOST_RUN_TOKEN

echo "[1/6] Preparing a clean dual-host cold start..."
"$SCRIPT_DIR/stop_dual_host.sh"

write_single_line_file "$RUN_TOKEN_FILE" "$DUAL_HOST_RUN_TOKEN"
export DUAL_HOST_RUN_TOKEN

remote_ssh_pid=""
nvidia_pid=""
cleanup_started=false

cleanup() {
  [[ "$cleanup_started" == false ]] || return 0
  cleanup_started=true
  trap - EXIT INT TERM HUP
  remove_ready_file
  echo
  echo "Synchronizing shutdown on NVIDIA and J6M..."
  "$SCRIPT_DIR/stop_dual_host.sh" || true
  if [[ -n "$nvidia_pid" ]]; then wait "$nvidia_pid" 2>/dev/null || true; fi
  if [[ -n "$remote_ssh_pid" ]]; then wait "$remote_ssh_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

wait_for_interface() {
  local variable_name="$1" mac="$2" label="$3"
  local wait_seconds="${DUAL_HOST_DEVICE_WAIT_SEC:-20}" deadline interface
  dual_host_refresh_local_interfaces
  interface="${!variable_name}"
  [[ -e "/sys/class/net/$interface" ]] && return 0
  if [[ ! "$mac" =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]]; then
    echo "$label interface $interface is absent and no valid fallback MAC is configured." >&2
    return 1
  fi
  echo "$label interface is absent; waiting up to ${wait_seconds}s for USB enumeration (MAC ${mac^^})..."
  deadline=$((SECONDS + wait_seconds))
  while (( SECONDS < deadline )); do
    sleep 1
    dual_host_refresh_local_interfaces
    interface="${!variable_name}"
    [[ -e "/sys/class/net/$interface" ]] && return 0
  done
  echo "$label USB Ethernet adapter is not connected (expected MAC ${mac^^})." >&2
  return 1
}

activate_network_profile() {
  local connection="$1" interface="$2" mac="$3" label="$4" address="$5"
  local profile_interface profile_mac
  profile_interface="$(nmcli -g connection.interface-name connection show "$connection" 2>/dev/null || true)"
  profile_mac="$(nmcli -g 802-3-ethernet.mac-address connection show "$connection" 2>/dev/null || true)"
  if [[ -n "$mac" &&
        ( "$profile_interface" != "$interface" || "${profile_mac,,}" != "${mac,,}" ) ]]; then
    echo "Updating $label profile binding to $interface (${mac^^})..."
    nmcli connection modify "$connection" \
      connection.interface-name "$interface" \
      802-3-ethernet.mac-address "$mac"
  fi
  if ! ip -o -4 address show dev "$interface" 2>/dev/null |
       awk '{print $4}' | grep -Fxq "$address/24"; then
    echo "Activating $label profile $connection on $interface..."
    nmcli connection up "$connection" >/dev/null
  fi
}

echo "[2/6] Resolving, activating and checking the two dedicated Ethernet profiles..."
wait_for_interface NVIDIA_J6M_INTERFACE "$NVIDIA_J6M_MAC" "J6M"
wait_for_interface NVIDIA_LIVOX_INTERFACE "$NVIDIA_LIVOX_MAC" "MID360"
activate_network_profile "$NVIDIA_J6M_CONNECTION" "$NVIDIA_J6M_INTERFACE" \
  "$NVIDIA_J6M_MAC" "J6M" "$NVIDIA_J6M_IP"
activate_network_profile "$NVIDIA_LIVOX_CONNECTION" "$NVIDIA_LIVOX_INTERFACE" \
  "$NVIDIA_LIVOX_MAC" "MID360" "$NVIDIA_LIVOX_IP"
"$SCRIPT_DIR/network_check.sh"

if [[ ! -r /dev/nvhost-vic || ! -w /dev/nvhost-vic ]]; then
  echo "Jetson video engine is inaccessible: /dev/nvhost-vic" >&2
  echo "Expected root:video mode 0660; repair the udev permissions before starting ZED." >&2
  exit 4
fi
if [[ "$CAN_PORT_CONFIRMED" == true && ! -w "$CAN_PORT" ]]; then
  echo "Confirmed CAN device is absent or not writable: $CAN_PORT" >&2
  exit 4
fi

echo "[3/6] Synchronizing the J6M clock..."
"$SCRIPT_DIR/sync_j6m_time.sh"

target="$(dual_host_select_ssh)" || {
  echo "J6M SSH is unavailable at both configured addresses." >&2
  exit 5
}

if [[ "$STATIC_MAP_ENABLED" == true ]]; then
  echo "[3/6] Synchronizing the selected static map to J6M..."
  "$SCRIPT_DIR/sync_static_map.sh" "$target"
fi

stamp="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$DUAL_HOST_WS/log/dual_host_launcher_$stamp"
mkdir -p "$LOG_DIR"

echo "[4/6] Starting the J6M ROS master/waiter..."
ssh -o BatchMode=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  "$target" "$J6M_RUNTIME_BASE/dual_host/bin/start.sh" \
  >"$LOG_DIR/j6m_ssh.log" 2>&1 &
remote_ssh_pid=$!

master_deadline=$((SECONDS + 45))
while ! ros_master_reachable; do
  if ! kill -0 "$remote_ssh_pid" >/dev/null 2>&1; then
    wait "$remote_ssh_pid" || remote_status=$?
    echo "J6M launcher exited early (status ${remote_status:-0})." >&2
    tail -n 80 "$LOG_DIR/j6m_ssh.log" >&2 || true
    exit 6
  fi
  if (( SECONDS >= master_deadline )); then
    echo "Timed out waiting for the J6M ROS master." >&2
    tail -n 80 "$LOG_DIR/j6m_ssh.log" >&2 || true
    exit 6
  fi
  sleep 0.5
done

echo "[5/6] Starting NVIDIA MID360/CAN, then ZED/YOLO/Qt..."
"$SCRIPT_DIR/start_nvidia.sh" >"$LOG_DIR/nvidia.log" 2>&1 &
nvidia_pid=$!

ready_deadline=$((SECONDS + WAIT_FOR_NVIDIA_SEC + 120))
last_report=$SECONDS
while ! runtime_ready; do
  if ! kill -0 "$nvidia_pid" >/dev/null 2>&1; then
    wait "$nvidia_pid" || nvidia_status=$?
    echo "NVIDIA launcher exited early (status ${nvidia_status:-0})." >&2
    tail -n 120 "$LOG_DIR/nvidia.log" >&2 || true
    exit 7
  fi
  if ! kill -0 "$remote_ssh_pid" >/dev/null 2>&1; then
    wait "$remote_ssh_pid" || remote_status=$?
    echo "J6M launcher exited early (status ${remote_status:-0})." >&2
    tail -n 120 "$LOG_DIR/j6m_ssh.log" >&2 || true
    exit 7
  fi
  if (( SECONDS >= ready_deadline )); then
    echo "Timed out waiting for the complete Qt/navigation graph." >&2
    printf 'Still missing:\n%s\n' "$(missing_runtime_nodes)" >&2
    exit 7
  fi
  if (( SECONDS - last_report >= 10 )); then
    printf 'Waiting for: %s\n' "$(missing_runtime_nodes | tr '\n' ' ')"
    last_report=$SECONDS
  fi
  sleep 1
done

sleep 3
echo "[6/6] Running the final runtime health check..."
"$SCRIPT_DIR/health_check.sh" --runtime

ready_temporary="$READY_FILE.tmp.$$"
printf '%s\n' "$DUAL_HOST_RUN_TOKEN" >"$ready_temporary"
mv -f -- "$ready_temporary" "$READY_FILE"

echo
echo "Dual-host project is ready; Qt is open with the full module console."
echo "Motion gates: MOTION_ENABLED=$MOTION_ENABLED, FOD_MOTION_ENABLED=$FOD_MOTION_ENABLED"
echo "Supervisor logs: $LOG_DIR"
if [[ "$mode" == --supervise ]]; then
  echo "Supervisor: $SERVICE_UNIT (independent of the launching terminal/GDM session)."
else
  echo "Foreground diagnostic supervisor is attached to this terminal."
fi

set +e
wait -n "$remote_ssh_pid" "$nvidia_pid"
child_status=$?
set -e
echo "A supervised component exited (status $child_status); stopping both hosts." >&2
exit "$child_status"
