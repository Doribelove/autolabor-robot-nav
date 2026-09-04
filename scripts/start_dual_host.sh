#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage: $0 [--start | --restart | --status | --stop | --foreground]
          [--map-set DIR] [--static-map-source fused|lidar2d]
          [--authorize-fod-motion]

  --start       Start the complete stack as a managed user service (default).
  --restart     Cold-stop both hosts, then start the managed service.
  --status      Show service state and run the dual-host health check.
  --stop        Stop both hosts synchronously and verify all residuals.
  --foreground  Diagnostic mode: keep the supervisor attached to this terminal.
  --map-set DIR EXPERIMENTAL opt-in: enable known-map ICP localization.
  --static-map-source
                Select fused (default) or lidar2d as move_base's static map.
  --authorize-fod-motion
                For this managed run only, set FOD_MOTION_ENABLED=true. This
                still requires the main motion gate, authorization marker,
                explicit runtime mode request, localization and live safety
                prechecks; it never enters visual driving automatically.

The default start waits until the complete graph is ready, then returns to the
shell. Optional sensor messages may be reported as degraded; an enabled ZED
camera must publish live image and depth data. The stack remains owned by
autolabor-dual-host.service;
closing this terminal or restarting the graphical desktop cannot orphan its ROS
children.
EOF
}

mode=""
requested_map_set=""
requested_static_source="fused"
authorize_fod_motion=false
while (( $# > 0 )); do
  case "$1" in
    --start|--restart|--status|--stop|--foreground|--supervise)
      [[ -z "$mode" ]] || { echo "Only one lifecycle mode may be selected." >&2; exit 2; }
      mode="$1"
      shift
      ;;
    --map-set)
      (( $# >= 2 )) || { echo "--map-set requires a directory." >&2; exit 2; }
      requested_map_set="$2"
      shift 2
      ;;
    --static-map-source)
      (( $# >= 2 )) || { echo "--static-map-source requires fused or lidar2d." >&2; exit 2; }
      requested_static_source="$2"
      shift 2
      ;;
    --authorize-fod-motion)
      authorize_fod_motion=true
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
mode="${mode:---start}"
case "$requested_static_source" in fused|lidar2d) ;; *)
  echo "--static-map-source must be fused or lidar2d." >&2; exit 2 ;;
esac
if [[ "$mode" == --supervise &&
      "${DUAL_HOST_FOD_MOTION_OVERRIDE:-}" == true ]]; then
  authorize_fod_motion=true
fi
if [[ "$authorize_fod_motion" == true ]]; then
  case "$mode" in
    --start|--restart|--foreground|--supervise) ;;
    *)
      echo "--authorize-fod-motion is valid only with --start, --restart or --foreground." >&2
      exit 2
      ;;
  esac
  export DUAL_HOST_FOD_MOTION_OVERRIDE=true
elif [[ "$mode" != --supervise ]]; then
  # Do not accept a hidden inherited authorization on a user-facing command.
  unset DUAL_HOST_FOD_MOTION_OVERRIDE
fi

# Catkin setup files inspect the caller's positional parameters when sourced.
inherited_static_map_enabled="${STATIC_MAP_ENABLED:-}"
inherited_static_map_set="${STATIC_MAP_SET:-}"
inherited_static_map_source_mode="${STATIC_MAP_SOURCE_MODE:-}"
inherited_static_map_file="${STATIC_MAP_FILE:-}"
inherited_fast_lio_map_file="${FAST_LIO_MAP_FILE:-}"
inherited_fast_lio_initial_body_z="${FAST_LIO_INITIAL_BODY_Z:-}"
set --
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
source "$SCRIPT_DIR/network_prepare.sh"

if [[ "$mode" == --supervise && -n "$inherited_static_map_enabled" ]]; then
  STATIC_MAP_ENABLED="$inherited_static_map_enabled"
  STATIC_MAP_SET="$inherited_static_map_set"
  STATIC_MAP_SOURCE_MODE="${inherited_static_map_source_mode:-fused}"
  STATIC_MAP_FILE="$inherited_static_map_file"
  FAST_LIO_MAP_FILE="$inherited_fast_lio_map_file"
  FAST_LIO_INITIAL_BODY_Z="${inherited_fast_lio_initial_body_z:-0.0}"
  export STATIC_MAP_ENABLED STATIC_MAP_SET STATIC_MAP_SOURCE_MODE STATIC_MAP_FILE
  export FAST_LIO_MAP_FILE FAST_LIO_INITIAL_BODY_Z
elif [[ "$mode" != --supervise ]]; then
  if [[ -n "$requested_map_set" ]]; then
    map_sets_root="$(readlink -f -- "$DUAL_HOST_WS/global_maps/map_sets" 2>/dev/null || true)"
    selected_map_set="$(readlink -f -- "$requested_map_set" 2>/dev/null || true)"
    [[ -n "$map_sets_root" && -n "$selected_map_set" && -d "$selected_map_set" &&
       "$selected_map_set" == "$map_sets_root/"* ]] || {
      echo "--map-set must select a directory below $DUAL_HOST_WS/global_maps/map_sets" >&2
      exit 3
    }
    grep -Eq '^status:[[:space:]]*"?complete"?[[:space:]]*$' \
      "$selected_map_set/manifest.yaml" 2>/dev/null || {
      echo "Map set is missing a complete manifest: $selected_map_set" >&2
      exit 3
    }
    for required in map_3d/map.pcd map_2d/map.yaml map_2d/map.pgm \
                    map_fused_2d/map.yaml map_fused_2d/map.pgm; do
      [[ -s "$selected_map_set/$required" ]] || {
        echo "Map set is incomplete: $selected_map_set/$required" >&2
        exit 3
      }
    done
    STATIC_MAP_ENABLED=true
    STATIC_MAP_SET="$selected_map_set"
    STATIC_MAP_SOURCE_MODE="$requested_static_source"
    FAST_LIO_MAP_FILE=/var/lib/autolabor/maps/current/map_3d/map.pcd
    if [[ "$requested_static_source" == fused ]]; then
      STATIC_MAP_FILE=/var/lib/autolabor/maps/current/map_fused_2d/map.yaml
    else
      STATIC_MAP_FILE=/var/lib/autolabor/maps/current/map_2d/map.yaml
    fi
    FAST_LIO_INITIAL_BODY_Z="$(awk '/^initial_body_z_m:/{print $2; exit}' "$selected_map_set/manifest.yaml")"
    FAST_LIO_INITIAL_BODY_Z="${FAST_LIO_INITIAL_BODY_Z:-0.0}"
  else
    STATIC_MAP_ENABLED=false
    STATIC_MAP_SET=""
    STATIC_MAP_SOURCE_MODE=fused
    STATIC_MAP_FILE=""
    FAST_LIO_MAP_FILE=""
    FAST_LIO_INITIAL_BODY_Z=0.0
  fi
  export STATIC_MAP_ENABLED STATIC_MAP_SET STATIC_MAP_SOURCE_MODE STATIC_MAP_FILE
  export FAST_LIO_MAP_FILE FAST_LIO_INITIAL_BODY_Z
fi

case "$mode:$STATIC_MAP_ENABLED" in
  --start:true|--restart:true|--foreground:true|--supervise:true)
    echo "WARNING: experimental 3-D known-map localization + 2-D navigation mode is enabled." >&2
    echo "Navigation remains stopped until a fresh /initialpose produces consecutive accepted ICP matches." >&2
    ;;
esac

RUN_DIR="$DUAL_HOST_WS/runtime/run"
READY_FILE="$RUN_DIR/dual_host.ready"
RUN_TOKEN_FILE="$RUN_DIR/nvidia_run.token"
SERVICE_TOKEN_FILE="$RUN_DIR/service_run.token"
MAP_MODE_FILE="$RUN_DIR/map_mode.env"
SERVICE_UNIT="autolabor-dual-host.service"
mkdir -p "$RUN_DIR" "$DUAL_HOST_WS/log"

if [[ "$mode" == --status && -r "$MAP_MODE_FILE" ]]; then
  # This file is generated by the managed supervisor with shell-escaped values.
  source "$MAP_MODE_FILE"
  export STATIC_MAP_ENABLED STATIC_MAP_SET STATIC_MAP_SOURCE_MODE STATIC_MAP_FILE
  export FAST_LIO_MAP_FILE FAST_LIO_INITIAL_BODY_Z
  export FOD_MOTION_ENABLED
fi

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
  REQUIRED_NODES+=(/map_server /fast_lio_map_localizer /fast_lio_localization_cmd_vel_gate)
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
  [[ "$STATIC_MAP_ENABLED" != true ]] || REQUIRED_NODES+=(/operator_map_display_anchor)
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
  local missing health_policy="${1:-strict}"
  missing="$(missing_runtime_nodes)"
  if [[ -n "$missing" ]]; then
    echo "Dual-host stack is not fully ready. Missing:" >&2
    sed 's/^/  - /' <<<"$missing" >&2
    return 1
  fi
  if [[ "$health_policy" == allow-missing-data ]]; then
    "$SCRIPT_DIR/health_check.sh" --runtime --allow-missing-data
  else
    "$SCRIPT_DIR/health_check.sh" --runtime
  fi
  if [[ "$STATIC_MAP_ENABLED" == true ]]; then
    echo "Selected map set: ${STATIC_MAP_SET:-unknown} (${STATIC_MAP_SOURCE_MODE:-fused})"
    timeout 3 rostopic echo -n 1 /fast_lio/localization_status 2>/dev/null |
      sed -n 's/^data: /FAST-LIO map localization /p' || true
  else
    echo "Selected map set: none (incremental FAST-LIO mode)"
  fi
  echo "The dual-host ROS graph is ready; live data availability is reported above."
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
      if ! show_runtime_status allow-missing-data; then
        echo "The service has a ready marker, but a mandatory runtime check failed." >&2
        return 1
      fi
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
  local running_fod_motion
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
    if [[ "$authorize_fod_motion" == true ]]; then
      running_fod_motion="$(
        (
          unset FOD_MOTION_ENABLED
          source "$MAP_MODE_FILE" 2>/dev/null || exit 1
          printf '%s\n' "${FOD_MOTION_ENABLED:-}"
        ) || true
      )"
      if [[ "$running_fod_motion" != true ]]; then
        echo "$SERVICE_UNIT is already running without one-run FOD motion authorization." >&2
        echo "Use '$0 --restart --authorize-fod-motion ...' for a deliberate cold restart." >&2
        return 1
      fi
    fi
    token="$(sed -n '1p' "$SERVICE_TOKEN_FILE" 2>/dev/null || true)"
    if [[ ! "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
      token="$(sed -n '1p' "$RUN_TOKEN_FILE" 2>/dev/null || true)"
    fi
    if [[ ! "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
      echo "$SERVICE_UNIT is already active but has no valid run token." >&2
      echo "Run '$0 --restart' to rebuild its ownership records." >&2
      return 1
    fi
    echo "$SERVICE_UNIT is already active; checking full readiness..."
    if wait_for_managed_service "$token"; then
      return 0
    fi
    echo "The active stack is degraded; performing one complete cold restart..." >&2
    stop_managed_service
  fi

  systemctl --user reset-failed "$SERVICE_UNIT" >/dev/null 2>&1 || true
  remove_ready_file
  [[ ! -f "$MAP_MODE_FILE" ]] || unlink "$MAP_MODE_FILE"
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
  command+=(--setenv="STATIC_MAP_ENABLED=$STATIC_MAP_ENABLED")
  command+=(--setenv="STATIC_MAP_SET=${STATIC_MAP_SET:-}")
  command+=(--setenv="STATIC_MAP_SOURCE_MODE=${STATIC_MAP_SOURCE_MODE:-fused}")
  command+=(--setenv="STATIC_MAP_FILE=${STATIC_MAP_FILE:-}")
  command+=(--setenv="FAST_LIO_MAP_FILE=${FAST_LIO_MAP_FILE:-}")
  command+=(--setenv="FAST_LIO_INITIAL_BODY_Z=${FAST_LIO_INITIAL_BODY_Z:-0.0}")
  if [[ "$authorize_fod_motion" == true ]]; then
    command+=(--setenv="DUAL_HOST_FOD_MOTION_OVERRIDE=true")
  fi
  if [[ "${DUAL_HOST_NETWORK_PREFLIGHT_DONE:-}" == 1 ]]; then
    command+=(--setenv="DUAL_HOST_NETWORK_PREFLIGHT_DONE=1")
  fi
  if [[ "${DUAL_HOST_ZED_PREFLIGHT_DONE:-}" == 1 ]]; then
    command+=(--setenv="DUAL_HOST_ZED_PREFLIGHT_DONE=1")
  fi
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

  echo "Starting $SERVICE_UNIT; this command will return after structural runtime checks pass..."
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
  [[ ! -f "$MAP_MODE_FILE" ]] || unlink "$MAP_MODE_FILE"
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

dual_host_validate_fod_model_contract || exit 3
dual_host_validate_fod_weights || exit 3

if [[ "$authorize_fod_motion" == true && "$MOTION_ENABLED" != true ]]; then
  echo "--authorize-fod-motion requires MOTION_ENABLED=true in $DUAL_HOST_CONFIG." >&2
  exit 3
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
if [[ "$authorize_fod_motion" == true ]]; then
  echo "WARNING: FOD visual motion is authorized for this managed run only." >&2
  echo "The controller remains disabled until an operator explicitly enters visual driving mode." >&2
fi

if [[ "$mode" == --restart ]]; then
  echo "Self-checking and repairing the two dedicated Ethernet links before restart..."
  dual_host_prepare_network
  export DUAL_HOST_NETWORK_PREFLIGHT_DONE=1
  echo "Checking ZED USB 3.x transport and access before restart..."
  "$SCRIPT_DIR/zed_camera_check.sh" --wait "$ZED_USB_WAIT_SEC"
  export DUAL_HOST_ZED_PREFLIGHT_DONE=1
  stop_managed_service
  start_managed_service
  exit $?
elif [[ "$mode" == --start ]]; then
  echo "Self-checking and repairing the two dedicated Ethernet links before start..."
  dual_host_prepare_network
  export DUAL_HOST_NETWORK_PREFLIGHT_DONE=1
  echo "Checking ZED USB 3.x transport and access before start..."
  "$SCRIPT_DIR/zed_camera_check.sh" --wait "$ZED_USB_WAIT_SEC"
  export DUAL_HOST_ZED_PREFLIGHT_DONE=1
  start_managed_service
  exit $?
fi

# --supervise is the systemd-owned implementation. --foreground uses the same
# lifecycle for diagnostics but remains attached to the invoking terminal.
if [[ ! "${DUAL_HOST_RUN_TOKEN:-}" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]]; then
  DUAL_HOST_RUN_TOKEN="$(< /proc/sys/kernel/random/uuid)"
fi
export DUAL_HOST_RUN_TOKEN

echo "[1/6] Self-checking and repairing the two dedicated Ethernet links..."
if [[ "${DUAL_HOST_NETWORK_PREFLIGHT_DONE:-}" == 1 ]]; then
  echo "The invoking start command completed the initial network preflight."
else
  dual_host_prepare_network
fi

echo "Checking ZED USB 3.x transport and access..."
if [[ "${DUAL_HOST_ZED_PREFLIGHT_DONE:-}" == 1 ]]; then
  echo "The invoking start command completed the initial ZED USB preflight."
else
  "$SCRIPT_DIR/zed_camera_check.sh" --wait "$ZED_USB_WAIT_SEC"
fi

echo "[2/6] Preparing a clean dual-host cold start..."
"$SCRIPT_DIR/stop_dual_host.sh"

echo "Rechecking both Ethernet links after the synchronized stop..."
dual_host_prepare_network

write_single_line_file "$RUN_TOKEN_FILE" "$DUAL_HOST_RUN_TOKEN"
export DUAL_HOST_RUN_TOKEN
{
  printf 'STATIC_MAP_ENABLED=%q\n' "$STATIC_MAP_ENABLED"
  printf 'STATIC_MAP_SET=%q\n' "${STATIC_MAP_SET:-}"
  printf 'STATIC_MAP_SOURCE_MODE=%q\n' "${STATIC_MAP_SOURCE_MODE:-fused}"
  printf 'STATIC_MAP_FILE=%q\n' "${STATIC_MAP_FILE:-}"
  printf 'FAST_LIO_MAP_FILE=%q\n' "${FAST_LIO_MAP_FILE:-}"
  printf 'FAST_LIO_INITIAL_BODY_Z=%q\n' "${FAST_LIO_INITIAL_BODY_Z:-0.0}"
  printf 'FOD_MOTION_ENABLED=%q\n' "$FOD_MOTION_ENABLED"
} >"$MAP_MODE_FILE.tmp.$$"
mv -f -- "$MAP_MODE_FILE.tmp.$$" "$MAP_MODE_FILE"

remote_ssh_pid=""
nvidia_pid=""
cleanup_started=false

cleanup() {
  [[ "$cleanup_started" == false ]] || return 0
  cleanup_started=true
  trap - EXIT INT TERM HUP
  remove_ready_file
  [[ ! -f "$MAP_MODE_FILE" ]] || unlink "$MAP_MODE_FILE"
  echo
  echo "Synchronizing shutdown on NVIDIA and J6M..."
  "$SCRIPT_DIR/stop_dual_host.sh" || true
  if [[ -n "$nvidia_pid" ]]; then wait "$nvidia_pid" 2>/dev/null || true; fi
  if [[ -n "$remote_ssh_pid" ]]; then wait "$remote_ssh_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

verify_j6m_static_localization_release() {
  local remote_host="$1"
  local current_link="$J6M_RUNTIME_BASE/rootfs/opt/autolabor/dual_host/current"
  ssh -o BatchMode=yes "$remote_host" "set -eu
    release=\$(readlink '$current_link')
    case \"\$release\" in
      /opt/autolabor/dual_host/releases/*/install) ;;
      *) echo \"Invalid J6M current release: \$release\" >&2; exit 1 ;;
    esac
    install='$J6M_RUNTIME_BASE/rootfs'\"\$release\"
    test -x \"\$install/lib/fast_lio_localization/fast_lio_map_localizer\"
    test -r \"\$install/share/fast_lio_localization/launch/known_map_localization.launch\"
    launch=\"\$install/share/autolabor_dual_host/launch/j6m_fastlio_navigation.launch\"
    grep -Fq 'fast_lio_localization' \"\$launch\"
    ! grep -Fq 'localization_enabled' \"\$launch\"
  "
}

verify_j6m_visual_model_contract_release() {
  local remote_host="$1"
  local current_link="$J6M_RUNTIME_BASE/rootfs/opt/autolabor/dual_host/current"
  ssh -o BatchMode=yes "$remote_host" "set -eu
    release=\$(readlink '$current_link')
    case \"\$release\" in
      /opt/autolabor/dual_host/releases/*/install) ;;
      *) echo \"Invalid J6M current release: \$release\" >&2; exit 1 ;;
    esac
    install='$J6M_RUNTIME_BASE/rootfs'\"\$release\"
    controller_launch=\"\$install/share/autolabor_fod_control/launch/visual_recovery.launch\"
    dual_host_launch=\"\$install/share/autolabor_dual_host/launch/j6m_fastlio_navigation.launch\"
    stack=\"\$install/lib/autolabor_dual_host/j6m_stack.sh\"
    test -x \"\$install/lib/autolabor_fod_control/fod_visual_servo_node.py\"
    grep -Fq 'arg name=\"expected_model_sha256\"' \"\$controller_launch\"
    grep -Fq 'arg name=\"fod_model_sha256\"' \"\$dual_host_launch\"
    grep -Fq 'NVIDIA_FOD_MODEL_SHA256' \"\$stack\"
    grep -Fq 'NVIDIA_FOD_MODEL_SHA256' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
    grep -Fq 'requested_fod_motion_enabled' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
    grep -Fq 'requested_fod_model_sha256' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
    grep -Fq 'requested_fod_required_class_names' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
  "
}

sync_j6m_runtime_config() {
  local remote_host="$1"
  local remote_dir="$J6M_RUNTIME_BASE/dual_host/config"
  local remote_temp="$remote_dir/dual_host.env.next.$$"
  ssh -o BatchMode=yes "$remote_host" "mkdir -p '$remote_dir'"
  rsync -a --chmod=F600 -- "$DUAL_HOST_CONFIG" "$remote_host:$remote_temp"
  ssh -o BatchMode=yes "$remote_host" "set -eu
    trap 'rm -f -- \"$remote_temp\"' EXIT
    bash -n '$remote_temp'
    chmod 0600 '$remote_temp'
    mv -f -- '$remote_temp' '$remote_dir/dual_host.env'
    trap - EXIT
  "
}

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

if ! verify_j6m_visual_model_contract_release "$target"; then
  echo "J6M current release cannot consume the configured visual model contract." >&2
  echo "Run ./scripts/deploy_j6m.sh successfully before the next cold start." >&2
  exit 5
fi
echo "[3/6] Synchronizing the authoritative runtime/model configuration to J6M..."
if ! sync_j6m_runtime_config "$target"; then
  echo "Failed to synchronize $DUAL_HOST_CONFIG to J6M." >&2
  exit 5
fi

if [[ "$STATIC_MAP_ENABLED" == true ]]; then
  if ! verify_j6m_static_localization_release "$target"; then
    echo "J6M current release does not contain the separate known-map localizer." >&2
    echo "Run ./scripts/deploy_j6m.sh successfully before static-map startup." >&2
    exit 5
  fi
  echo "[3/6] Synchronizing the selected static map to J6M..."
  "$SCRIPT_DIR/sync_static_map.sh" "$target"
fi

stamp="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$DUAL_HOST_WS/log/dual_host_launcher_$stamp"
mkdir -p "$LOG_DIR"

echo "[4/6] Starting the J6M ROS master/waiter..."
ssh -o BatchMode=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  "$target" env \
  STATIC_MAP_ENABLED="$STATIC_MAP_ENABLED" \
  STATIC_MAP_FILE="${STATIC_MAP_FILE:-}" \
  FAST_LIO_MAP_FILE="${FAST_LIO_MAP_FILE:-}" \
  FAST_LIO_INITIAL_BODY_Z="${FAST_LIO_INITIAL_BODY_Z:-0.0}" \
  FOD_MOTION_ENABLED="$FOD_MOTION_ENABLED" \
  NVIDIA_FOD_MODEL_SHA256="$NVIDIA_FOD_MODEL_SHA256" \
  NVIDIA_FOD_REQUIRED_CLASS_NAMES="$NVIDIA_FOD_REQUIRED_CLASS_NAMES" \
  "$J6M_RUNTIME_BASE/dual_host/bin/start.sh" \
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
"$SCRIPT_DIR/health_check.sh" --runtime --allow-missing-data

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
