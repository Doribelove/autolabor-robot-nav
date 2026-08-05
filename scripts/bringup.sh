#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
PRIVATE_SETUP="${PRIVATE_SETUP:-$ROBOT_WS/.deps/setup.bash}"
MODE="${1:-fast_lio}"
GPS_NAV_MAX_SPEED_ARG="${2:-}"
GPS_TEB_PROFILE_ARG="${3:-}"
TERMINAL_MODE="${TERMINAL_MODE:-auto}"
CLEAN_START="${CLEAN_START:-true}"
NAV_START_RVIZ="${NAV_START_RVIZ:-true}"
CAN_PORT="${CAN_PORT:-/dev/ttyUSB0}"
GPS_PORT="${GPS_PORT:-/dev/ttyUSB1}"
GPS_BAUD_RATE="${GPS_BAUD_RATE:-115200}"
FAST_LIO_GPS_YAW_OFFSET_DEG="${FAST_LIO_GPS_YAW_OFFSET_DEG:-0.0}"
case "$(uname -m)" in
  aarch64|arm64)
    DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR="/lib/aarch64-linux-gnu"
    ;;
  x86_64|amd64)
    DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR="/lib/x86_64-linux-gnu"
    ;;
  *)
    DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR="/lib"
    ;;
esac
FAST_LIO_SYSTEM_LIBRARY_DIR="${FAST_LIO_SYSTEM_LIBRARY_DIR:-$DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR}"
FAST_LIO_NAV_MAX_VEL_X="${FAST_LIO_NAV_MAX_VEL_X:-1.2}"
FAST_LIO_NAV_MAX_VEL_X_BACKWARDS="${FAST_LIO_NAV_MAX_VEL_X_BACKWARDS:-1.2}"
GPS_NAV_MAX_VEL_X="${GPS_NAV_MAX_VEL_X:-1.5}"
GPS_NAV_MAX_VEL_X_BACKWARDS="${GPS_NAV_MAX_VEL_X_BACKWARDS:-1.0}"
GPS_USE_WHEEL_ODOM="${GPS_USE_WHEEL_ODOM:-false}"
GPS_USE_WHEEL_TWIST="${GPS_USE_WHEEL_TWIST:-true}"
GPS_WHEEL_TWIST_TIMEOUT="${GPS_WHEEL_TWIST_TIMEOUT:-0.5}"
GPS_RMC_SPEED_TIMEOUT="${GPS_RMC_SPEED_TIMEOUT:-1.0}"
GPS_HEADING_SOURCE="${GPS_HEADING_SOURCE:-dual_antenna}"
GPS_HEADING_TIMEOUT="${GPS_HEADING_TIMEOUT:-1.0}"
GPS_ODOM_STARTUP_TIMEOUT="${GPS_ODOM_STARTUP_TIMEOUT:-120.0}"
GPS_HEADING_REQUIRED_SOLUTION_STATUS="${GPS_HEADING_REQUIRED_SOLUTION_STATUS:-SOL_COMPUTED}"
GPS_HEADING_REQUIRED_POSITION_TYPES="${GPS_HEADING_REQUIRED_POSITION_TYPES:-NARROW_INT}"
GPS_HEADING_JUMP_GUARD_ENABLED="${GPS_HEADING_JUMP_GUARD_ENABLED:-}"
GPS_HEADING_JUMP_THRESHOLD_DEG="${GPS_HEADING_JUMP_THRESHOLD_DEG:-1.5}"
GPS_HEADING_RECOVERY_TOLERANCE_DEG="${GPS_HEADING_RECOVERY_TOLERANCE_DEG:-0.8}"
GPS_HEADING_RECOVERY_SAMPLES="${GPS_HEADING_RECOVERY_SAMPLES:-3}"
GPS_POSITION_FILTER_ALPHA="${GPS_POSITION_FILTER_ALPHA:-}"
GPS_ANTENNA_OFFSET_X="${GPS_ANTENNA_OFFSET_X:--0.3}"
GPS_ANTENNA_OFFSET_Y="${GPS_ANTENNA_OFFSET_Y:--0.05}"
GPS_HEADING_MIN_SPEED="${GPS_HEADING_MIN_SPEED:-0.05}"
GPS_MIN_COURSE_DISTANCE="${GPS_MIN_COURSE_DISTANCE:-0.2}"
GPS_INITIAL_YAW="${GPS_INITIAL_YAW:-}"
GPS_COMPASS_HEADING="${GPS_COMPASS_HEADING:-}"
GPS_COMPASS_HEADING_DEG="${GPS_COMPASS_HEADING_DEG:-}"
GPS_TEB_PROFILE="${GPS_TEB_PROFILE:-cruise}"
GPS_TEB_PROFILE_FILE=""
GPS_TEB_PENALTY_EPSILON="${GPS_TEB_PENALTY_EPSILON:-}"
GPS_TEB_FORWARD_DRIVE_WEIGHT="${GPS_TEB_FORWARD_DRIVE_WEIGHT:-}"
GPS_GLOBAL_PLANNER_FREQUENCY="${GPS_GLOBAL_PLANNER_FREQUENCY:-}"
GPS_XY_GOAL_TOLERANCE="${GPS_XY_GOAL_TOLERANCE:-0.3}"
GPS_GOAL_SLOWDOWN_ENABLED="${GPS_GOAL_SLOWDOWN_ENABLED:-true}"
GPS_GOAL_SPEED_CAP_ENABLED="${GPS_GOAL_SPEED_CAP_ENABLED:-false}"
GPS_GOAL_COMFORTABLE_DECEL="${GPS_GOAL_COMFORTABLE_DECEL:-0.4}"
GPS_GOAL_MIN_APPROACH_SPEED="${GPS_GOAL_MIN_APPROACH_SPEED:-0.15}"
GPS_GOAL_HARD_STOP_DISTANCE="${GPS_GOAL_HARD_STOP_DISTANCE:-0.2}"
GPS_GOAL_CMD_TIMEOUT="${GPS_GOAL_CMD_TIMEOUT:-0.5}"
GPS_GOAL_ODOM_TIMEOUT="${GPS_GOAL_ODOM_TIMEOUT:-1.0}"
GPS_GOAL_NEAR_COMMIT_DISTANCE="${GPS_GOAL_NEAR_COMMIT_DISTANCE:-1.0}"
GPS_GOAL_NEAR_TIMEOUT="${GPS_GOAL_NEAR_TIMEOUT:-15.0}"
GPS_GOAL_NEAR_MAX_REGRESSION="${GPS_GOAL_NEAR_MAX_REGRESSION:-0.5}"
GPS_LONG_RANGE_GOAL_ENABLED="${GPS_LONG_RANGE_GOAL_ENABLED:-true}"
GPS_LONG_RANGE_LOOKAHEAD_DISTANCE="${GPS_LONG_RANGE_LOOKAHEAD_DISTANCE:-15.0}"
GPS_LONG_RANGE_ADVANCE_DISTANCE="${GPS_LONG_RANGE_ADVANCE_DISTANCE:-5.0}"
GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE="${GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE:-18.0}"
GPS_LONG_RANGE_MAX_FINAL_DISTANCE="${GPS_LONG_RANGE_MAX_FINAL_DISTANCE:-1000.0}"
GPS_LONG_RANGE_ODOM_TIMEOUT="${GPS_LONG_RANGE_ODOM_TIMEOUT:-1.0}"
GPS_LONG_RANGE_MOVE_BASE_STATUS_TIMEOUT="${GPS_LONG_RANGE_MOVE_BASE_STATUS_TIMEOUT:-2.0}"
GPS_LONG_RANGE_UPDATE_RATE="${GPS_LONG_RANGE_UPDATE_RATE:-10.0}"
FOD_RECOVERY_STANDBY_ENABLED="${FOD_RECOVERY_STANDBY_ENABLED:-true}"
FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE="${FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE:-false}"
FOD_RECOVERY_BLIND_DISTANCE_M="${FOD_RECOVERY_BLIND_DISTANCE_M:-0.50}"
FOD_RECOVERY_TRANSITION_TIMEOUT="${FOD_RECOVERY_TRANSITION_TIMEOUT:-12.0}"
FILTER_REMOVE_ABOVE_Z="${FILTER_REMOVE_ABOVE_Z:-0.1}"
FILTER_NEAR_RADIUS="${FILTER_NEAR_RADIUS:-0.4}"
FILTER_NEAR_MIN_Z="${FILTER_NEAR_MIN_Z:--0.1}"
FILTER_NEAR_MAX_Z="${FILTER_NEAR_MAX_Z:-0.1}"

PIDS=()
SPLIT_TERMINALS=0
TERMINAL_KIND=""
TERMINAL_SCRIPT_DIR=""

usage() {
  echo "Usage:"
  echo "  $0 fast_lio"
  echo "  $0 fast_lio_gps"
  echo "  $0 gps [max_speed_mps] [cruise|obstacle]"
  echo "  $0 --print-gps-yaw"
  echo
  echo "Examples:"
  echo "  $0 gps                       # 1.5 m/s, high-speed cruise TEB profile"
  echo "  $0 gps 2.0 cruise            # open-road high-speed cruise profile"
  echo "  $0 gps 1.0 obstacle          # dense static-obstacle avoidance profile"
  echo
  echo "Environment:"
  echo "  TERMINAL_MODE=auto|split|same   # auto opens split terminals when possible"
  echo "  CLEAN_START=true|false          # kill old nodes from this bringup before starting"
  echo "  NAV_START_RVIZ=true|false       # false when using the embedded operator GUI RViz"
  echo "  CAN_PORT=/dev/ttyUSB0"
  echo "  GPS_PORT=/dev/ttyUSB1"
  echo "  GPS_BAUD_RATE=115200"
  echo "  FAST_LIO_GPS_YAW_OFFSET_DEG=0.0"
  echo "  FAST_LIO_SYSTEM_LIBRARY_DIR=$DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR # isolate FAST_LIO from the MVS SDK libusb"
  echo "  FAST_LIO_NAV_MAX_VEL_X=1.2       # FAST_LIO forward planner limit"
  echo "  FAST_LIO_NAV_MAX_VEL_X_BACKWARDS=1.2 # FAST_LIO reverse planner limit"
  echo "  GPS_NAV_MAX_VEL_X=1.5       # overridden by the optional gps speed argument"
  echo "  GPS_NAV_MAX_VEL_X_BACKWARDS=1.0 # never raised by the gps speed argument"
  echo "  GPS_TEB_PROFILE=cruise|obstacle # used when the third argument is omitted"
  echo "  GPS_USE_WHEEL_ODOM=false        # gps mode uses GPS position directly by default"
  echo "  GPS_USE_WHEEL_TWIST=true        # publish fresh signed chassis twist in /gps/odom"
  echo "  GPS_WHEEL_TWIST_TIMEOUT=0.5     # s; then fall back to GNSS motion estimates"
  echo "  GPS_RMC_SPEED_TIMEOUT=1.0       # s; discard cached RMC speed/course after this"
  echo "  GPS_HEADING_SOURCE=dual_antenna"
  echo "  GPS_HEADING_TIMEOUT=1.0"
  echo "  GPS_ODOM_STARTUP_TIMEOUT=120.0  # s; wait for strict dual-antenna heading to become fixed"
  echo "  GPS_HEADING_REQUIRED_SOLUTION_STATUS=SOL_COMPUTED"
  echo "  GPS_HEADING_REQUIRED_POSITION_TYPES=NARROW_INT"
  echo "  GPS_HEADING_JUMP_GUARD_ENABLED=true|false # defaults false; optional diagnostic override"
  echo "  GPS_HEADING_JUMP_THRESHOLD_DEG=1.5        # mismatch against chassis-predicted yaw"
  echo "  GPS_HEADING_RECOVERY_TOLERANCE_DEG=0.8    # live heading must return within this"
  echo "  GPS_HEADING_RECOVERY_SAMPLES=3            # consecutive stable samples before recovery"
  echo "  GPS_POSITION_FILTER_ALPHA=0.70             # cruise default; moving-position responsiveness"
  echo "  GPS_ANTENNA_OFFSET_X=-0.3        # main antenna x offset in base_link"
  echo "  GPS_ANTENNA_OFFSET_Y=-0.05       # main antenna is 0.05 m right of base_link"
  echo "  GPS_HEADING_MIN_SPEED=0.05"
  echo "  GPS_MIN_COURSE_DISTANCE=0.2"
  echo "  GPS_INITIAL_YAW=0.0             # radians, used until GPS course is available"
  echo "  GPS_COMPASS_HEADING=东北45度     # phone compass heading, 0=N, 90=E"
  echo "  GPS_COMPASS_HEADING_DEG=45      # numeric compass heading, 0=N, 90=E"
  echo "  GPS_TEB_PENALTY_EPSILON=0.03     # optional profile-default override"
  echo "  GPS_TEB_FORWARD_DRIVE_WEIGHT=... # optional profile-default override"
  echo "  GPS_GLOBAL_PLANNER_FREQUENCY=0.0 # cruise default; stable route until a new goal/failure"
  echo "  GPS_XY_GOAL_TOLERANCE=0.3        # m; TEB declares the GPS goal reached"
  echo "  GPS_GOAL_SLOWDOWN_ENABLED=true   # keep the GPS command safety relay enabled"
  echo "  GPS_GOAL_SPEED_CAP_ENABLED=false # do not add a forward-speed cap near the final goal"
  echo "  GPS_GOAL_COMFORTABLE_DECEL=0.4   # m/s^2; used only when the speed cap is enabled"
  echo "  GPS_GOAL_MIN_APPROACH_SPEED=0.15 # m/s outside the limiter hard-stop radius"
  echo "  GPS_GOAL_HARD_STOP_DISTANCE=0.2  # m; must be below TEB goal tolerance"
  echo "  GPS_GOAL_NEAR_COMMIT_DISTANCE=1.0 # m; arm bounded final approach"
  echo "  GPS_GOAL_NEAR_TIMEOUT=15.0        # s; lock stopped if final approach never completes"
  echo "  GPS_GOAL_NEAR_MAX_REGRESSION=0.5 # m; lock stopped if vehicle moves away after commit"
  echo "  GPS_LONG_RANGE_GOAL_ENABLED=true  # roll distant GPS targets through bounded move_base goals"
  echo "  GPS_LONG_RANGE_LOOKAHEAD_DISTANCE=15.0 # m; rolling subgoal horizon"
  echo "  GPS_LONG_RANGE_ADVANCE_DISTANCE=5.0    # m; replace an intermediate goal this early"
  echo "  GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE=18.0 # m; guard for the 40 x 40 m rolling map"
  echo "  GPS_LONG_RANGE_MAX_FINAL_DISTANCE=1000.0 # m; reject likely coordinate mistakes beyond this"
  echo "  FOD_RECOVERY_STANDBY_ENABLED=true # start safe GPS/FOD arbiter and disabled visual controller"
  echo "  FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE=false # keep CAN/VCU safety gates active"
  echo "  FOD_RECOVERY_BLIND_DISTANCE_M=0.50 # m; post-loss straight crossing distance"
  echo "  FOD_RECOVERY_TRANSITION_TIMEOUT=12.0 # s; maximum wait for confirmed chassis stop"
  echo "  FILTER_REMOVE_ABOVE_Z=0.1"
  echo "  FILTER_NEAR_RADIUS=0.4"
  echo "  FILTER_NEAR_MIN_Z=-0.1"
  echo "  FILTER_NEAR_MAX_Z=0.1"
}

is_positive_number() {
  local value="$1"
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 1
  awk -v value="$value" 'BEGIN { exit !(value > 0.0) }'
}

is_nonnegative_number() {
  local value="$1"
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 1
  awk -v value="$value" 'BEGIN { exit !(value >= 0.0) }'
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

min_number() {
  awk -v first="$1" -v second="$2" 'BEGIN {
    if (first < second) print first;
    else print second;
  }'
}

clamp_navigation_speed_to_chassis_limit() {
  local service_name="/m2_driver/chassis_parameter"
  local deadline=$((SECONDS + 15))
  local response=""
  local chassis_max_speed=""
  local planner_label=""
  local requested_forward=""
  local requested_backward=""

  if [[ "$MODE" == "gps" ]]; then
    planner_label="GPS"
    requested_forward="$GPS_NAV_MAX_VEL_X"
    requested_backward="$GPS_NAV_MAX_VEL_X_BACKWARDS"
  else
    planner_label="FAST_LIO"
    requested_forward="$FAST_LIO_NAV_MAX_VEL_X"
    requested_backward="$FAST_LIO_NAV_MAX_VEL_X_BACKWARDS"
  fi

  while (( SECONDS < deadline )); do
    if rosservice info "$service_name" >/dev/null 2>&1; then
      response="$(timeout 2 rosservice call "$service_name" 2>/dev/null || true)"
      if grep -Eq '^success:[[:space:]]+True$' <<<"$response"; then
        chassis_max_speed="$(awk '/^[[:space:]]*max_speed:/ {print $2; exit}' <<<"$response")"
        if is_positive_number "$chassis_max_speed"; then
          break
        fi
      fi
    fi
    sleep 0.25
  done

  if ! is_positive_number "$chassis_max_speed"; then
    echo "Unable to read a valid max_speed from $service_name after 15 seconds." >&2
    echo "Refusing to start $planner_label navigation with an unchecked planner/chassis speed mismatch." >&2
    return 1
  fi

  local capped_forward
  local capped_backward
  capped_forward="$(min_number "$requested_forward" "$chassis_max_speed")"
  capped_backward="$(min_number "$requested_backward" "$chassis_max_speed")"
  if [[ "$MODE" == "gps" ]]; then
    GPS_NAV_MAX_VEL_X="$capped_forward"
    GPS_NAV_MAX_VEL_X_BACKWARDS="$capped_backward"
  else
    FAST_LIO_NAV_MAX_VEL_X="$capped_forward"
    FAST_LIO_NAV_MAX_VEL_X_BACKWARDS="$capped_backward"
  fi

  echo "==> M2 chassis-reported max speed: $chassis_max_speed m/s"
  if [[ "$capped_forward" != "$requested_forward" ||
        "$capped_backward" != "$requested_backward" ]]; then
    echo "==> $planner_label planner speed capped to chassis: forward=$capped_forward m/s, backward=$capped_backward m/s"
  else
    echo "==> $planner_label planner speed is within chassis limit: forward=$capped_forward m/s, backward=$capped_backward m/s"
  fi
}

normalize_compass_heading_deg() {
  awk -v heading="$1" 'BEGIN {
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    printf "%.10g\n", heading;
  }'
}

parse_compass_heading_deg() {
  local raw="$1"
  local compact="${raw//[[:space:]]/}"
  compact="${compact//度/}"
  compact="${compact//°/}"

  local numeric=""
  numeric="$(grep -Eo '[+-]?[0-9]+([.][0-9]+)?' <<<"$compact" | head -n 1 || true)"
  if [[ -n "$numeric" ]]; then
    normalize_compass_heading_deg "$numeric"
    return 0
  fi

  case "$compact" in
    北|正北) normalize_compass_heading_deg 0 ;;
    东北|北东) normalize_compass_heading_deg 45 ;;
    东|正东) normalize_compass_heading_deg 90 ;;
    东南|南东) normalize_compass_heading_deg 135 ;;
    南|正南) normalize_compass_heading_deg 180 ;;
    西南|南西) normalize_compass_heading_deg 225 ;;
    西|正西) normalize_compass_heading_deg 270 ;;
    西北|北西) normalize_compass_heading_deg 315 ;;
    *)
      echo "Invalid GPS_COMPASS_HEADING: $raw" >&2
      echo "Use a compass degree like GPS_COMPASS_HEADING_DEG=45, or a direction like GPS_COMPASS_HEADING=东北." >&2
      return 1
      ;;
  esac
}

compass_heading_deg_to_ros_yaw_rad() {
  awk -v heading="$1" 'BEGIN {
    pi = atan2(0, -1);
    yaw = (90.0 - heading) * pi / 180.0;
    while (yaw > pi) yaw -= 2.0 * pi;
    while (yaw <= -pi) yaw += 2.0 * pi;
    printf "%.10g\n", yaw;
  }'
}

resolve_gps_initial_yaw() {
  if [[ -n "$GPS_INITIAL_YAW" ]]; then
    echo "$GPS_INITIAL_YAW"
    return 0
  fi

  local compass_heading=""
  if [[ -n "$GPS_COMPASS_HEADING_DEG" ]]; then
    compass_heading="$(parse_compass_heading_deg "$GPS_COMPASS_HEADING_DEG")"
  elif [[ -n "$GPS_COMPASS_HEADING" ]]; then
    compass_heading="$(parse_compass_heading_deg "$GPS_COMPASS_HEADING")"
  else
    echo "0.0"
    return 0
  fi

  compass_heading_deg_to_ros_yaw_rad "$compass_heading"
}

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  if [[ -n "$TERMINAL_SCRIPT_DIR" && -d "$TERMINAL_SCRIPT_DIR" ]]; then
    rm -rf "$TERMINAL_SCRIPT_DIR"
  fi
}

require_file() {
  [[ -f "$1" ]] || {
    echo "Missing file: $1" >&2
    exit 2
  }
}

make_writable() {
  local dev="$1"
  [[ -e "$dev" ]] || {
    echo "Device does not exist: $dev" >&2
    exit 3
  }
  if [[ ! -w "$dev" ]]; then
    echo "==> $dev is not writable; trying sudo chmod 666 $dev"
    if sudo -n chmod 666 "$dev" 2>/dev/null; then
      return 0
    fi
    echo "Device is not writable and non-interactive sudo is unavailable: $dev" >&2
    echo "Run this in a local terminal, then start bringup again:" >&2
    echo "  sudo chmod 666 $dev" >&2
    echo "For a persistent fix, add the robot user to the device group and relogin:" >&2
    echo "  sudo usermod -aG dialout slam" >&2
    exit 3
  fi
}

cleanup_existing_nodes() {
  [[ "$CLEAN_START" == "true" ]] || return 0

  if ! rostopic list >/dev/null 2>&1; then
    return 0
  fi

  local existing
  existing="$(rosnode list 2>/dev/null || true)"
  [[ -n "$existing" ]] || return 0

  local nodes=(
    /all_in_one_planner
    /base_link_to_livox_frame
    /body_to_base_link
    /canbus_driver
    /fod_navigation_mode
    /fod_visual_servo
    /gps_goal
    /gps_long_range_goal_manager
    /gps_goal_speed_limiter
    /gps_localization
    /laserMapping
    /livox_lidar_publisher2
    /m2_driver
    /move_base
    /pointcloud_self_filter
    /pointcloud_to_laserscan
    /robot_state_publisher
    /rviz
    /spacial_horizon_node
    /teb_visualization
    /viz_path
  )

  local node
  local killed=0
  for node in "${nodes[@]}"; do
    if grep -qx "$node" <<<"$existing"; then
      echo "==> stopping old node: $node"
      rosnode kill "$node" >/dev/null 2>&1 || true
      killed=1
    fi
  done

  if (( killed )); then
    sleep 2
    printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true
  fi
}

setup_terminal_mode() {
  case "$TERMINAL_MODE" in
    auto|split|same) ;;
    *)
      echo "Invalid TERMINAL_MODE: $TERMINAL_MODE" >&2
      usage >&2
      exit 1
      ;;
  esac

  if [[ "$TERMINAL_MODE" == "same" ]]; then
    return 0
  fi

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    if [[ "$TERMINAL_MODE" == "split" ]]; then
      echo "TERMINAL_MODE=split requires a graphical session (DISPLAY or WAYLAND_DISPLAY)." >&2
      exit 4
    fi
    echo "==> no graphical session found; using same-terminal mode"
    return 0
  fi

  local candidate
  for candidate in gnome-terminal mate-terminal xfce4-terminal konsole terminator xterm; do
    if command -v "$candidate" >/dev/null 2>&1; then
      TERMINAL_KIND="$candidate"
      SPLIT_TERMINALS=1
      TERMINAL_SCRIPT_DIR="$(mktemp -d /tmp/robot_ws_bringup.XXXXXX)"
      echo "==> using split terminals with $TERMINAL_KIND"
      return 0
    fi
  done

  if [[ "$TERMINAL_MODE" == "split" ]]; then
    echo "No supported terminal emulator found for TERMINAL_MODE=split." >&2
    exit 4
  fi

  echo "==> no supported terminal emulator found; using same-terminal mode"
}

write_terminal_script() {
  local label="$1"
  local script_file="$2"
  local pid_file="$3"
  shift 3

  {
    printf '#!/usr/bin/env bash\n'
    printf 'set -uo pipefail\n'
    printf 'cd %q\n' "$ROBOT_WS"
    printf 'echo $$ > %q\n' "$pid_file"
    printf 'echo "==> %s"\n' "$label"
    printf 'echo "==> sourcing ROS environment"\n'
    printf 'source %q\n' "$ROS_SETUP"
    printf 'source %q\n' "$ROBOT_WS/devel/setup.bash"
    if [[ -f "$PRIVATE_SETUP" ]]; then
      printf 'source %q\n' "$PRIVATE_SETUP"
    fi
    printf 'child_pid=""\n'
    printf 'cleanup_child() {\n'
    printf '  if [[ -n "${child_pid:-}" ]] && kill -0 "$child_pid" >/dev/null 2>&1; then\n'
    printf '    kill "$child_pid" >/dev/null 2>&1 || true\n'
    printf '    wait "$child_pid" >/dev/null 2>&1 || true\n'
    printf '  fi\n'
    printf '}\n'
    printf 'trap '\''cleanup_child; exit 130'\'' INT TERM\n'
    printf 'echo "==> command:"\n'
    printf 'printf '\''  %%q'\'''
    local arg
    for arg in "$@"; do
      printf ' %q' "$arg"
    done
    printf '\n'
    printf 'echo\n'
    for arg in "$@"; do
      printf '%q ' "$arg"
    done
    printf '&\n'
    printf 'child_pid=$!\n'
    printf 'wait "$child_pid"\n'
    printf 'rc=$?\n'
    printf 'trap - INT TERM\n'
    printf 'echo\n'
    printf 'echo "[%s] exited with code ${rc}"\n' "$label"
    printf 'echo "Close this terminal, or press Ctrl+D to exit the shell."\n'
    printf 'exec bash\n'
  } >"$script_file"
  chmod +x "$script_file"
}

open_terminal() {
  local label="$1"
  local script_file="$2"

  case "$TERMINAL_KIND" in
    gnome-terminal)
      gnome-terminal --tab --title="$label" -- "$script_file" >/dev/null 2>&1 &
      ;;
    mate-terminal)
      mate-terminal --tab --title="$label" -- "$script_file" >/dev/null 2>&1 &
      ;;
    xfce4-terminal)
      xfce4-terminal --tab --title="$label" --command "$script_file" >/dev/null 2>&1 &
      ;;
    konsole)
      konsole --new-tab --workdir "$ROBOT_WS" -p "tabtitle=$label" -e "$script_file" >/dev/null 2>&1 &
      ;;
    terminator)
      terminator -T "$label" -x "$script_file" >/dev/null 2>&1 &
      ;;
    xterm)
      xterm -T "$label" -e "$script_file" >/dev/null 2>&1 &
      ;;
    *)
      return 1
      ;;
  esac
}

start_terminal_command() {
  local label="$1"
  shift
  local safe_label="${label//[^A-Za-z0-9_.-]/_}"
  local script_file="$TERMINAL_SCRIPT_DIR/${safe_label}.sh"
  local pid_file="$TERMINAL_SCRIPT_DIR/${safe_label}.pid"

  echo "==> opening terminal: $label"
  write_terminal_script "$label" "$script_file" "$pid_file" "$@"
  open_terminal "$label" "$script_file"

  local deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    if [[ -s "$pid_file" ]]; then
      PIDS+=("$(<"$pid_file")")
      return 0
    fi
    sleep 0.1
  done

  echo "Timed out opening terminal for $label" >&2
  return 1
}

start_launch_command() {
  local label="$1"
  shift
  if (( SPLIT_TERMINALS )); then
    start_terminal_command "$label" "$@"
  else
    echo "==> starting $label"
    "$@" &
    PIDS+=("$!")
  fi
  wait_ros_master 15
}

start_launch() {
  local label="$1"
  shift
  start_launch_command "$label" roslaunch "$@"
}

start_fast_lio_launch() {
  local label="$1"
  shift
  local fast_lio_library_path="$FAST_LIO_SYSTEM_LIBRARY_DIR"
  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    fast_lio_library_path+=":$LD_LIBRARY_PATH"
  fi

  echo "==> FAST_LIO libusb: preferring $FAST_LIO_SYSTEM_LIBRARY_DIR"
  start_launch_command "$label" \
    env "LD_LIBRARY_PATH=$fast_lio_library_path" \
    roslaunch "$@"
}

wait_ros_master() {
  local timeout="${1:-15}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done
  echo "Timed out waiting for ROS master" >&2
  return 1
}

start_ros_master() {
  if rostopic list >/dev/null 2>&1; then
    echo "==> ROS master already running"
    return 0
  fi

  echo "==> starting ROS master"
  if (( SPLIT_TERMINALS )); then
    start_terminal_command "ROS master" roscore
  else
    roscore >/tmp/robot_ws_roscore.log 2>&1 &
    PIDS+=("$!")
  fi
  wait_ros_master 15
}

wait_topics() {
  local topics="$1"
  local timeout="${2:-45.0}"
  rosrun robot_diagnostics wait_for_topics.py _topics:="$topics" _timeout:="$timeout"
}

check_odom() {
  local topic="$1"
  local frame="$2"
  local child_frame="${3:-}"
  local timeout="${4:-15.0}"
  rosrun robot_diagnostics check_odom.py \
    _topic:="$topic" \
    _timeout:="$timeout" \
    _required_frame:="$frame" \
    _required_child_frame:="$child_frame"
}

check_tf() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout="${3:-20.0}"
  rosrun robot_diagnostics check_tf.py \
    _target_frame:="$target_frame" \
    _source_frame:="$source_frame" \
    _timeout:="$timeout"
}

check_topic_route() {
  local topic="$1"
  local expected_publisher="${2:-}"
  local expected_subscriber="${3:-}"
  local timeout="${4:-10}"
  local deadline=$((SECONDS + timeout))
  local info=""
  local publishers=""
  local subscribers=""
  local publisher_ok=0
  local subscriber_ok=0

  while (( SECONDS < deadline )); do
    info="$(rostopic info "$topic" 2>/dev/null || true)"
    publishers="$(awk '
      /^Publishers:/ { section = "publishers"; next }
      /^Subscribers:/ { section = "subscribers"; next }
      section == "publishers" && /^[[:space:]]*\*/ { print $2 }
    ' <<<"$info")"
    subscribers="$(awk '
      /^Publishers:/ { section = "publishers"; next }
      /^Subscribers:/ { section = "subscribers"; next }
      section == "subscribers" && /^[[:space:]]*\*/ { print $2 }
    ' <<<"$info")"

    publisher_ok=0
    subscriber_ok=0
    if [[ -z "$expected_publisher" ]] || grep -Fxq "$expected_publisher" <<<"$publishers"; then
      publisher_ok=1
    fi
    if [[ -z "$expected_subscriber" ]] || grep -Fxq "$expected_subscriber" <<<"$subscribers"; then
      subscriber_ok=1
    fi
    if (( publisher_ok && subscriber_ok )); then
      return 0
    fi
    sleep 0.2
  done

  echo "ROS topic route is not connected on $topic." >&2
  echo "Expected publisher: ${expected_publisher:-any}; subscriber: ${expected_subscriber:-any}." >&2
  echo "Observed publishers: ${publishers:-none}" >&2
  echo "Observed subscribers: ${subscribers:-none}" >&2
  return 1
}

check_single_topic_subscriber() {
  local topic="$1"
  local expected_subscriber="$2"
  local timeout="${3:-10}"
  local deadline=$((SECONDS + timeout))
  local info=""
  local subscribers=""
  local subscriber_count=0

  while (( SECONDS < deadline )); do
    info="$(rostopic info "$topic" 2>/dev/null || true)"
    subscribers="$(awk '
      /^Subscribers:/ { section = "subscribers"; next }
      section == "subscribers" && /^[[:space:]]*\*/ { print $2 }
    ' <<<"$info")"
    subscriber_count="$(grep -c '^/' <<<"$subscribers" || true)"
    if (( subscriber_count == 1 )) &&
       grep -Fxq "$expected_subscriber" <<<"$subscribers"; then
      return 0
    fi
    sleep 0.2
  done

  echo "GPS goal input route is not exclusive on $topic." >&2
  echo "Expected exactly one subscriber: $expected_subscriber." >&2
  echo "Observed subscribers: ${subscribers:-none}" >&2
  return 1
}

check_cmd_vel_route() {
  local topic="${1:-/cmd_vel}"
  local expected_publisher="${2:-/move_base}"
  local expected_subscriber="${3:-/m2_driver}"
  local timeout="${4:-10}"
  local deadline=$((SECONDS + timeout))
  local info=""
  local publishers=""
  local subscribers=""
  local publisher_count=0

  while (( SECONDS < deadline )); do
    info="$(rostopic info "$topic" 2>/dev/null || true)"
    publishers="$(awk '
      /^Publishers:/ { section = "publishers"; next }
      /^Subscribers:/ { section = "subscribers"; next }
      section == "publishers" && /^[[:space:]]*\*/ { print $2 }
    ' <<<"$info")"
    subscribers="$(awk '
      /^Publishers:/ { section = "publishers"; next }
      /^Subscribers:/ { section = "subscribers"; next }
      section == "subscribers" && /^[[:space:]]*\*/ { print $2 }
    ' <<<"$info")"
    publisher_count="$(grep -c '^/' <<<"$publishers" || true)"
    if (( publisher_count == 1 )) &&
       grep -Fxq "$expected_publisher" <<<"$publishers" &&
       grep -Fxq "$expected_subscriber" <<<"$subscribers"; then
      return 0
    fi
    sleep 0.5
  done

  echo "Command velocity route is not connected on $topic." >&2
  echo "Expected exactly one publisher ($expected_publisher) and subscriber $expected_subscriber." >&2
  echo "Observed publishers: ${publishers:-none}" >&2
  echo "Observed subscribers: ${subscribers:-none}" >&2
  if [[ -n "$info" ]]; then
    echo "$info" >&2
  fi
  return 1
}

check_service_provider() {
  local service="$1"
  local expected_provider="$2"
  local timeout="${3:-10}"
  local deadline=$((SECONDS + timeout))
  local info=""
  local provider=""

  while (( SECONDS < deadline )); do
    info="$(rosservice info "$service" 2>/dev/null || true)"
    provider="$(awk '/^Node:/ { print $2; exit }' <<<"$info")"
    if [[ "$provider" == "$expected_provider" ]]; then
      return 0
    fi
    sleep 0.2
  done

  echo "ROS service is not available from the expected provider: $service" >&2
  echo "Expected provider: $expected_provider; observed: ${provider:-none}" >&2
  return 1
}

trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

if (( $# > 3 )); then
  echo "Too many arguments." >&2
  usage >&2
  exit 1
fi

case "$MODE" in
  fast_lio|fast_lio_gps|gps) ;;
  print-gps-yaw|--print-gps-yaw)
    GPS_INITIAL_YAW="$(resolve_gps_initial_yaw)"
    echo "$GPS_INITIAL_YAW"
    exit 0
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac

case "$NAV_START_RVIZ" in
  true|false) ;;
  *)
    echo "Invalid NAV_START_RVIZ: $NAV_START_RVIZ (use true or false)" >&2
    exit 1
    ;;
esac

if [[ -n "$GPS_TEB_PROFILE_ARG" ]]; then
  if [[ "$MODE" != "gps" ]]; then
    echo "The optional TEB profile argument is only supported in gps mode." >&2
    usage >&2
    exit 1
  fi
  GPS_TEB_PROFILE="$GPS_TEB_PROFILE_ARG"
fi

if [[ "$MODE" == "gps" ]]; then
  case "$GPS_TEB_PROFILE" in
    cruise)
      GPS_TEB_PROFILE_FILE="$ROBOT_WS/config/teb_profiles/gps_cruise.yaml"
      GPS_TEB_PENALTY_EPSILON="${GPS_TEB_PENALTY_EPSILON:-0.03}"
      GPS_TEB_FORWARD_DRIVE_WEIGHT="${GPS_TEB_FORWARD_DRIVE_WEIGHT:-100.0}"
      GPS_HEADING_JUMP_GUARD_ENABLED="${GPS_HEADING_JUMP_GUARD_ENABLED:-false}"
      GPS_POSITION_FILTER_ALPHA="${GPS_POSITION_FILTER_ALPHA:-0.70}"
      GPS_GLOBAL_PLANNER_FREQUENCY="${GPS_GLOBAL_PLANNER_FREQUENCY:-0.0}"
      ;;
    obstacle)
      GPS_TEB_PROFILE_FILE="$ROBOT_WS/config/teb_profiles/gps_obstacle.yaml"
      GPS_TEB_PENALTY_EPSILON="${GPS_TEB_PENALTY_EPSILON:-0.03}"
      GPS_TEB_FORWARD_DRIVE_WEIGHT="${GPS_TEB_FORWARD_DRIVE_WEIGHT:-60.0}"
      GPS_HEADING_JUMP_GUARD_ENABLED="${GPS_HEADING_JUMP_GUARD_ENABLED:-false}"
      GPS_POSITION_FILTER_ALPHA="${GPS_POSITION_FILTER_ALPHA:-0.25}"
      GPS_GLOBAL_PLANNER_FREQUENCY="${GPS_GLOBAL_PLANNER_FREQUENCY:-1.0}"
      ;;
    *)
      echo "Invalid GPS TEB profile: $GPS_TEB_PROFILE" >&2
      echo "Use cruise or obstacle, for example: $0 gps 2.0 cruise" >&2
      exit 1
      ;;
  esac
fi

if [[ -n "$GPS_NAV_MAX_SPEED_ARG" ]]; then
  if [[ "$MODE" != "gps" ]]; then
    echo "The optional max_speed_mps argument is only supported in gps mode." >&2
    usage >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_NAV_MAX_SPEED_ARG"; then
    echo "Invalid GPS max speed: $GPS_NAV_MAX_SPEED_ARG" >&2
    echo "Use a positive number in m/s, for example: $0 gps 2.0" >&2
    exit 1
  fi
  GPS_NAV_MAX_VEL_X="$GPS_NAV_MAX_SPEED_ARG"
  GPS_NAV_MAX_VEL_X_BACKWARDS="$(min_number "$GPS_NAV_MAX_VEL_X_BACKWARDS" "$GPS_NAV_MAX_SPEED_ARG")"
fi

if [[ "$MODE" == "fast_lio" || "$MODE" == "fast_lio_gps" ]]; then
  if ! is_positive_number "$FAST_LIO_NAV_MAX_VEL_X"; then
    echo "Invalid FAST_LIO_NAV_MAX_VEL_X: $FAST_LIO_NAV_MAX_VEL_X" >&2
    exit 1
  fi
  if ! is_positive_number "$FAST_LIO_NAV_MAX_VEL_X_BACKWARDS"; then
    echo "Invalid FAST_LIO_NAV_MAX_VEL_X_BACKWARDS: $FAST_LIO_NAV_MAX_VEL_X_BACKWARDS" >&2
    exit 1
  fi
  echo "==> FAST_LIO requested navigation speed limits: forward=$FAST_LIO_NAV_MAX_VEL_X m/s, backward=$FAST_LIO_NAV_MAX_VEL_X_BACKWARDS m/s"
fi

if [[ "$MODE" == "gps" ]]; then
  case "$GPS_USE_WHEEL_TWIST" in
    true|false) ;;
    *)
      echo "Invalid GPS_USE_WHEEL_TWIST: $GPS_USE_WHEEL_TWIST (use true or false)" >&2
      exit 1
      ;;
  esac
  if ! is_nonnegative_number "$GPS_WHEEL_TWIST_TIMEOUT"; then
    echo "Invalid GPS_WHEEL_TWIST_TIMEOUT: $GPS_WHEEL_TWIST_TIMEOUT" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_RMC_SPEED_TIMEOUT"; then
    echo "Invalid GPS_RMC_SPEED_TIMEOUT: $GPS_RMC_SPEED_TIMEOUT" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_ODOM_STARTUP_TIMEOUT"; then
    echo "Invalid GPS_ODOM_STARTUP_TIMEOUT: $GPS_ODOM_STARTUP_TIMEOUT" >&2
    exit 1
  fi
  case "$GPS_HEADING_JUMP_GUARD_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid GPS_HEADING_JUMP_GUARD_ENABLED: $GPS_HEADING_JUMP_GUARD_ENABLED (use true or false)" >&2
      exit 1
      ;;
  esac
  if ! is_positive_number "$GPS_HEADING_JUMP_THRESHOLD_DEG"; then
    echo "Invalid GPS_HEADING_JUMP_THRESHOLD_DEG: $GPS_HEADING_JUMP_THRESHOLD_DEG" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_HEADING_RECOVERY_TOLERANCE_DEG"; then
    echo "Invalid GPS_HEADING_RECOVERY_TOLERANCE_DEG: $GPS_HEADING_RECOVERY_TOLERANCE_DEG" >&2
    exit 1
  fi
  if ! awk -v recovery="$GPS_HEADING_RECOVERY_TOLERANCE_DEG" -v jump="$GPS_HEADING_JUMP_THRESHOLD_DEG" \
    'BEGIN { exit !(recovery <= jump) }'; then
    echo "GPS_HEADING_RECOVERY_TOLERANCE_DEG ($GPS_HEADING_RECOVERY_TOLERANCE_DEG) must be <= GPS_HEADING_JUMP_THRESHOLD_DEG ($GPS_HEADING_JUMP_THRESHOLD_DEG)" >&2
    exit 1
  fi
  if ! is_positive_integer "$GPS_HEADING_RECOVERY_SAMPLES"; then
    echo "Invalid GPS_HEADING_RECOVERY_SAMPLES: $GPS_HEADING_RECOVERY_SAMPLES (use a positive integer)" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_POSITION_FILTER_ALPHA" ||
     ! awk -v alpha="$GPS_POSITION_FILTER_ALPHA" 'BEGIN { exit !(alpha <= 1.0) }'; then
    echo "Invalid GPS_POSITION_FILTER_ALPHA: $GPS_POSITION_FILTER_ALPHA (use 0 < alpha <= 1)" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_NAV_MAX_VEL_X"; then
    echo "Invalid GPS_NAV_MAX_VEL_X: $GPS_NAV_MAX_VEL_X" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_NAV_MAX_VEL_X_BACKWARDS"; then
    echo "Invalid GPS_NAV_MAX_VEL_X_BACKWARDS: $GPS_NAV_MAX_VEL_X_BACKWARDS" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_TEB_PENALTY_EPSILON"; then
    echo "Invalid GPS_TEB_PENALTY_EPSILON: $GPS_TEB_PENALTY_EPSILON" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_TEB_FORWARD_DRIVE_WEIGHT"; then
    echo "Invalid GPS_TEB_FORWARD_DRIVE_WEIGHT: $GPS_TEB_FORWARD_DRIVE_WEIGHT" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_GLOBAL_PLANNER_FREQUENCY"; then
    echo "Invalid GPS_GLOBAL_PLANNER_FREQUENCY: $GPS_GLOBAL_PLANNER_FREQUENCY" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_XY_GOAL_TOLERANCE"; then
    echo "Invalid GPS_XY_GOAL_TOLERANCE: $GPS_XY_GOAL_TOLERANCE" >&2
    exit 1
  fi
  if ! is_nonnegative_number "$GPS_GOAL_HARD_STOP_DISTANCE"; then
    echo "Invalid GPS_GOAL_HARD_STOP_DISTANCE: $GPS_GOAL_HARD_STOP_DISTANCE" >&2
    exit 1
  fi
  if ! awk -v hard_stop="$GPS_GOAL_HARD_STOP_DISTANCE" -v tolerance="$GPS_XY_GOAL_TOLERANCE" \
    'BEGIN { exit !(hard_stop < tolerance) }'; then
    echo "GPS_GOAL_HARD_STOP_DISTANCE ($GPS_GOAL_HARD_STOP_DISTANCE) must be smaller than GPS_XY_GOAL_TOLERANCE ($GPS_XY_GOAL_TOLERANCE)" >&2
    exit 1
  fi
  case "$GPS_GOAL_SLOWDOWN_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid GPS_GOAL_SLOWDOWN_ENABLED: $GPS_GOAL_SLOWDOWN_ENABLED (use true or false)" >&2
      exit 1
      ;;
  esac
  case "$GPS_GOAL_SPEED_CAP_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid GPS_GOAL_SPEED_CAP_ENABLED: $GPS_GOAL_SPEED_CAP_ENABLED (use true or false)" >&2
      exit 1
      ;;
  esac
  if [[ "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
    if ! is_positive_number "$GPS_GOAL_COMFORTABLE_DECEL"; then
      echo "Invalid GPS_GOAL_COMFORTABLE_DECEL: $GPS_GOAL_COMFORTABLE_DECEL" >&2
      exit 1
    fi
    if ! is_nonnegative_number "$GPS_GOAL_MIN_APPROACH_SPEED"; then
      echo "Invalid GPS_GOAL_MIN_APPROACH_SPEED: $GPS_GOAL_MIN_APPROACH_SPEED" >&2
      exit 1
    fi
    if ! is_positive_number "$GPS_GOAL_CMD_TIMEOUT"; then
      echo "Invalid GPS_GOAL_CMD_TIMEOUT: $GPS_GOAL_CMD_TIMEOUT" >&2
      exit 1
    fi
    if ! is_positive_number "$GPS_GOAL_ODOM_TIMEOUT"; then
      echo "Invalid GPS_GOAL_ODOM_TIMEOUT: $GPS_GOAL_ODOM_TIMEOUT" >&2
      exit 1
    fi
    if ! is_positive_number "$GPS_GOAL_NEAR_COMMIT_DISTANCE"; then
      echo "Invalid GPS_GOAL_NEAR_COMMIT_DISTANCE: $GPS_GOAL_NEAR_COMMIT_DISTANCE" >&2
      exit 1
    fi
    if ! awk -v commit="$GPS_GOAL_NEAR_COMMIT_DISTANCE" -v hard_stop="$GPS_GOAL_HARD_STOP_DISTANCE" \
      'BEGIN { exit !(commit > hard_stop) }'; then
      echo "GPS_GOAL_NEAR_COMMIT_DISTANCE ($GPS_GOAL_NEAR_COMMIT_DISTANCE) must be greater than GPS_GOAL_HARD_STOP_DISTANCE ($GPS_GOAL_HARD_STOP_DISTANCE)" >&2
      exit 1
    fi
    if ! is_positive_number "$GPS_GOAL_NEAR_TIMEOUT"; then
      echo "Invalid GPS_GOAL_NEAR_TIMEOUT: $GPS_GOAL_NEAR_TIMEOUT" >&2
      exit 1
    fi
    if ! is_positive_number "$GPS_GOAL_NEAR_MAX_REGRESSION"; then
      echo "Invalid GPS_GOAL_NEAR_MAX_REGRESSION: $GPS_GOAL_NEAR_MAX_REGRESSION" >&2
      exit 1
    fi
  fi
  case "$GPS_LONG_RANGE_GOAL_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid GPS_LONG_RANGE_GOAL_ENABLED: $GPS_LONG_RANGE_GOAL_ENABLED (use true or false)" >&2
      exit 2
      ;;
  esac
  case "$FOD_RECOVERY_STANDBY_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid FOD_RECOVERY_STANDBY_ENABLED: $FOD_RECOVERY_STANDBY_ENABLED (use true or false)" >&2
      exit 2
      ;;
  esac
  case "$FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE" in
    true|false) ;;
    *)
      echo "Invalid FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE: $FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE (use true or false)" >&2
      exit 2
      ;;
  esac
  if [[ "$FOD_RECOVERY_STANDBY_ENABLED" == "true" ]]; then
    if [[ "$GPS_LONG_RANGE_GOAL_ENABLED" != "true" ]]; then
      echo "FOD standby requires GPS_LONG_RANGE_GOAL_ENABLED=true so the final GPS route can be paused and resumed safely." >&2
      echo "Set FOD_RECOVERY_STANDBY_ENABLED=false only for legacy direct-goal diagnostics." >&2
      exit 2
    fi
    if ! is_positive_number "$FOD_RECOVERY_BLIND_DISTANCE_M" ||
       ! awk -v distance="$FOD_RECOVERY_BLIND_DISTANCE_M" \
         'BEGIN { exit !(distance <= 0.50) }'; then
      echo "Invalid FOD_RECOVERY_BLIND_DISTANCE_M: $FOD_RECOVERY_BLIND_DISTANCE_M (use 0 < distance <= 0.50)" >&2
      exit 2
    fi
    if ! is_positive_number "$FOD_RECOVERY_TRANSITION_TIMEOUT"; then
      echo "Invalid FOD_RECOVERY_TRANSITION_TIMEOUT: $FOD_RECOVERY_TRANSITION_TIMEOUT" >&2
      exit 2
    fi
  fi
  if [[ "$GPS_LONG_RANGE_GOAL_ENABLED" == "true" ]]; then
    for value_name in \
      GPS_LONG_RANGE_LOOKAHEAD_DISTANCE \
      GPS_LONG_RANGE_ADVANCE_DISTANCE \
      GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE \
      GPS_LONG_RANGE_MAX_FINAL_DISTANCE \
      GPS_LONG_RANGE_ODOM_TIMEOUT \
      GPS_LONG_RANGE_MOVE_BASE_STATUS_TIMEOUT \
      GPS_LONG_RANGE_UPDATE_RATE; do
      if ! is_positive_number "${!value_name}"; then
        echo "Invalid $value_name: ${!value_name}" >&2
        exit 2
      fi
    done
    if ! awk \
      -v advance="$GPS_LONG_RANGE_ADVANCE_DISTANCE" \
      -v lookahead="$GPS_LONG_RANGE_LOOKAHEAD_DISTANCE" \
      'BEGIN { exit !(advance < lookahead) }'; then
      echo "GPS_LONG_RANGE_ADVANCE_DISTANCE ($GPS_LONG_RANGE_ADVANCE_DISTANCE) must be smaller than GPS_LONG_RANGE_LOOKAHEAD_DISTANCE ($GPS_LONG_RANGE_LOOKAHEAD_DISTANCE)" >&2
      exit 2
    fi
    if ! awk \
      -v lookahead="$GPS_LONG_RANGE_LOOKAHEAD_DISTANCE" \
      -v maximum="$GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE" \
      'BEGIN { exit !(lookahead <= maximum) }'; then
      echo "GPS_LONG_RANGE_LOOKAHEAD_DISTANCE ($GPS_LONG_RANGE_LOOKAHEAD_DISTANCE) must not exceed GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE ($GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE)" >&2
      exit 2
    fi
  fi
  require_file "$GPS_TEB_PROFILE_FILE"
  echo "==> GPS TEB profile: $GPS_TEB_PROFILE ($GPS_TEB_PROFILE_FILE)"
  echo "==> GPS requested navigation speed limits: forward=$GPS_NAV_MAX_VEL_X m/s, backward=$GPS_NAV_MAX_VEL_X_BACKWARDS m/s"
  echo "==> GPS odom twist: wheel=$GPS_USE_WHEEL_TWIST, wheel timeout=$GPS_WHEEL_TWIST_TIMEOUT s, RMC timeout=$GPS_RMC_SPEED_TIMEOUT s"
  echo "==> GPS moving-position filter alpha: $GPS_POSITION_FILTER_ALPHA"
  echo "==> GPS global planner frequency: $GPS_GLOBAL_PLANNER_FREQUENCY Hz (0 keeps the route stable while controlling)"
  if [[ "$GPS_HEADING_JUMP_GUARD_ENABLED" == "true" ]]; then
    echo "==> GPS cruise heading jump guard: enabled, reject >$GPS_HEADING_JUMP_THRESHOLD_DEG deg, recover within $GPS_HEADING_RECOVERY_TOLERANCE_DEG deg for $GPS_HEADING_RECOVERY_SAMPLES samples; no stop is generated"
  else
    echo "==> GPS heading jump guard: disabled; navigation uses live dual-antenna yaw"
  fi
  echo "==> GPS goal distances: TEB tolerance=$GPS_XY_GOAL_TOLERANCE m, relay hard stop=$GPS_GOAL_HARD_STOP_DISTANCE m"
  echo "==> GPS TEB forward-drive weight: $GPS_TEB_FORWARD_DRIVE_WEIGHT"
  if [[ "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
    echo "==> GPS goal safety relay: enabled; near-goal fence=$GPS_GOAL_NEAR_COMMIT_DISTANCE m/$GPS_GOAL_NEAR_TIMEOUT s/$GPS_GOAL_NEAR_MAX_REGRESSION m regression"
    if [[ "$GPS_GOAL_SPEED_CAP_ENABLED" == "true" ]]; then
      echo "==> GPS terminal speed cap: enabled; decel=$GPS_GOAL_COMFORTABLE_DECEL m/s^2, minimum approach=$GPS_GOAL_MIN_APPROACH_SPEED m/s"
    else
      echo "==> GPS terminal speed cap: disabled; final-approach TEB commands pass through unchanged"
    fi
  else
    echo "==> GPS goal safety relay: disabled"
  fi
  if [[ "$GPS_LONG_RANGE_GOAL_ENABLED" == "true" ]]; then
    echo "==> GPS rolling goals: lookahead=$GPS_LONG_RANGE_LOOKAHEAD_DISTANCE m, advance=$GPS_LONG_RANGE_ADVANCE_DISTANCE m, max final distance=$GPS_LONG_RANGE_MAX_FINAL_DISTANCE m"
  else
    echo "==> GPS rolling goals: disabled; direct local-map goal conversion is active"
  fi
  if [[ "$FOD_RECOVERY_STANDBY_ENABLED" == "true" ]]; then
    echo "==> FOD recovery standby: enabled; GPS/FOD command arbiter owns /cmd_vel"
    echo "==> FOD CAN/VCU safety override: $FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE"
  else
    echo "==> FOD recovery standby: disabled"
  fi
fi

if [[ "$MODE" == "gps" ]]; then
  if [[ -z "$GPS_INITIAL_YAW" && ( -n "$GPS_COMPASS_HEADING" || -n "$GPS_COMPASS_HEADING_DEG" ) ]]; then
    GPS_INITIAL_YAW="$(resolve_gps_initial_yaw)"
    echo "==> GPS initial_yaw from compass: $GPS_INITIAL_YAW rad"
  else
    GPS_INITIAL_YAW="$(resolve_gps_initial_yaw)"
  fi
fi

require_file "$ROS_SETUP"
require_file "$ROBOT_WS/devel/setup.bash"
require_file "$FAST_LIO_SYSTEM_LIBRARY_DIR/libusb-1.0.so.0"

source "$ROS_SETUP"
source "$ROBOT_WS/devel/setup.bash"
if [[ -f "$PRIVATE_SETUP" ]]; then
  # Keep user-space ROS packages (for example pointcloud_to_laserscan under
  # .deps/sysroot) ahead of the system/workspace prefixes.
  source "$PRIVATE_SETUP"
fi

cleanup_existing_nodes
setup_terminal_mode
start_ros_master

make_writable "$CAN_PORT"
echo "==> checking CAN device: $CAN_PORT"
rosrun robot_diagnostics check_can.py _port:="$CAN_PORT" _require_write:=true
start_launch "CAN chassis driver" robot_bringup can.launch port_name:="$CAN_PORT" publish_tf:=false
wait_topics "/canbus_msg" 30.0
clamp_navigation_speed_to_chassis_limit

if [[ "$MODE" == "fast_lio" || "$MODE" == "fast_lio_gps" ]]; then
  start_launch "Livox Mid-360 driver" robot_bringup livox_mid360.launch
  wait_topics "/livox/lidar,/livox/imu" 45.0

  start_fast_lio_launch "FAST_LIO localization" robot_bringup fast_lio.launch
  wait_topics "/Odometry,/cloud_registered_body" 60.0
  check_odom "/Odometry" "camera_init" "body"

  start_launch "FAST_LIO filtered scan projection" robot_bringup scan_fast_lio.launch \
    remove_above_z:="$FILTER_REMOVE_ABOVE_Z" \
    near_radius:="$FILTER_NEAR_RADIUS" \
    near_min_z:="$FILTER_NEAR_MIN_Z" \
    near_max_z:="$FILTER_NEAR_MAX_Z"
  check_tf "camera_init" "base_link" 30.0
  wait_topics "/scan" 45.0

  make_writable "$GPS_PORT"
  start_launch "GPS fix reader for FAST_LIO goals" robot_bringup gps_localization.launch \
    port:="$GPS_PORT" \
    baud_rate:="$GPS_BAUD_RATE" \
    use_wheel_twist:="$GPS_USE_WHEEL_TWIST" \
    wheel_twist_timeout:="$GPS_WHEEL_TWIST_TIMEOUT" \
    rmc_speed_timeout:="$GPS_RMC_SPEED_TIMEOUT" \
    broadcast_tf:=false
  wait_topics "/gps/fix" 60.0

  start_launch "GPS goal converter for FAST_LIO" gps_module gps_goal.launch \
    frame_id:=camera_init \
    odom_topic:=/Odometry \
    goal_yaw_mode:=bearing \
    yaw_offset_deg:="$FAST_LIO_GPS_YAW_OFFSET_DEG"

  check_tf "camera_init" "base_link" 10.0
  start_launch "Arena navigation" robot_bringup navigation_arena.launch \
    localization_source:=fast_lio \
    start_rviz:="$NAV_START_RVIZ" \
    max_vel_x:="$FAST_LIO_NAV_MAX_VEL_X" \
    max_vel_x_backwards:="$FAST_LIO_NAV_MAX_VEL_X_BACKWARDS"
else
  make_writable "$GPS_PORT"
  start_launch "Livox Mid-360 driver" robot_bringup livox_mid360.launch
  wait_topics "/livox/lidar,/livox/imu" 45.0

  start_fast_lio_launch "FAST_LIO point cloud registration" robot_bringup fast_lio.launch odom_pub_en:=false tf_pub_en:=false
  wait_topics "/cloud_registered_body" 60.0

  start_launch "FAST_LIO filtered scan projection" robot_bringup scan_fast_lio.launch \
    remove_above_z:="$FILTER_REMOVE_ABOVE_Z" \
    near_radius:="$FILTER_NEAR_RADIUS" \
    near_min_z:="$FILTER_NEAR_MIN_Z" \
    near_max_z:="$FILTER_NEAR_MAX_Z" \
    body_tf_parent:=base_link \
    body_tf_child:=body \
    body_tf_z:=0.6
  wait_topics "/scan" 45.0

  wait_topics "/odom" 30.0
  start_launch "GPS localization" robot_bringup gps_localization.launch \
    port:="$GPS_PORT" \
    baud_rate:="$GPS_BAUD_RATE" \
    use_wheel_odom:="$GPS_USE_WHEEL_ODOM" \
    use_wheel_twist:="$GPS_USE_WHEEL_TWIST" \
    wheel_odom_topic:=/odom \
    wheel_twist_timeout:="$GPS_WHEEL_TWIST_TIMEOUT" \
    rmc_speed_timeout:="$GPS_RMC_SPEED_TIMEOUT" \
    heading_source:="$GPS_HEADING_SOURCE" \
    heading_timeout:="$GPS_HEADING_TIMEOUT" \
    heading_required_solution_status:="$GPS_HEADING_REQUIRED_SOLUTION_STATUS" \
    heading_required_position_types:="$GPS_HEADING_REQUIRED_POSITION_TYPES" \
    heading_jump_guard_enabled:="$GPS_HEADING_JUMP_GUARD_ENABLED" \
    heading_jump_threshold_deg:="$GPS_HEADING_JUMP_THRESHOLD_DEG" \
    heading_recovery_tolerance_deg:="$GPS_HEADING_RECOVERY_TOLERANCE_DEG" \
    heading_recovery_samples:="$GPS_HEADING_RECOVERY_SAMPLES" \
    position_filter_alpha:="$GPS_POSITION_FILTER_ALPHA" \
    gps_antenna_offset_x:="$GPS_ANTENNA_OFFSET_X" \
    gps_antenna_offset_y:="$GPS_ANTENNA_OFFSET_Y" \
    heading_min_speed:="$GPS_HEADING_MIN_SPEED" \
    min_course_distance:="$GPS_MIN_COURSE_DISTANCE" \
    initial_yaw:="$GPS_INITIAL_YAW"
  wait_topics "/gps/fix,/gps/pose,/gps/odom" 60.0
  echo "==> waiting up to $GPS_ODOM_STARTUP_TIMEOUT s for GPS odom; required heading quality: $GPS_HEADING_REQUIRED_SOLUTION_STATUS + $GPS_HEADING_REQUIRED_POSITION_TYPES"
  if ! check_odom "/gps/odom" "camera_init" "base_link" "$GPS_ODOM_STARTUP_TIMEOUT"; then
    echo "GPS localization did not become navigation-ready." >&2
    echo "Check the GPS localization terminal for the latest UNIHEADINGA status." >&2
    echo "NARROW_FLOAT is intentionally rejected by the default NARROW_INT safety gate." >&2
    exit 1
  fi
  check_tf "camera_init" "base_link" 30.0

  if [[ "$GPS_LONG_RANGE_GOAL_ENABLED" == "true" ]]; then
    start_launch "GPS long-range goal manager" gps_module gps_long_range_goal.launch \
      frame_id:=camera_init \
      odom_topic:=/gps/odom \
      lookahead_distance:="$GPS_LONG_RANGE_LOOKAHEAD_DISTANCE" \
      advance_distance:="$GPS_LONG_RANGE_ADVANCE_DISTANCE" \
      max_lookahead_distance:="$GPS_LONG_RANGE_MAX_LOOKAHEAD_DISTANCE" \
      max_final_goal_distance:="$GPS_LONG_RANGE_MAX_FINAL_DISTANCE" \
      odom_timeout:="$GPS_LONG_RANGE_ODOM_TIMEOUT" \
      move_base_status_timeout:="$GPS_LONG_RANGE_MOVE_BASE_STATUS_TIMEOUT" \
      update_rate:="$GPS_LONG_RANGE_UPDATE_RATE"
  else
    start_launch "GPS goal converter" gps_module gps_goal.launch \
      frame_id:=camera_init \
      odom_topic:=/gps/odom \
      goal_yaw_mode:=bearing
  fi

  GPS_OUTPUT_CMD_VEL_TOPIC="/cmd_vel"
  if [[ "$FOD_RECOVERY_STANDBY_ENABLED" == "true" ]]; then
    GPS_OUTPUT_CMD_VEL_TOPIC="/cmd_vel_gps"
    start_launch "GPS/FOD recovery mode arbiter" autolabor_fod_control gps_visual_recovery_standby.launch \
      allow_motion:=true \
      external_estop_override:="$FOD_RECOVERY_EXTERNAL_ESTOP_OVERRIDE" \
      blind_distance_m:="$FOD_RECOVERY_BLIND_DISTANCE_M" \
      transition_timeout_sec:="$FOD_RECOVERY_TRANSITION_TIMEOUT"
  fi

  check_tf "camera_init" "base_link" 10.0
  start_launch "Arena navigation" robot_bringup navigation_arena.launch \
    localization_source:=gps \
    start_rviz:="$NAV_START_RVIZ" \
    planner_frequency:="$GPS_GLOBAL_PLANNER_FREQUENCY" \
    teb_profile_file:="$GPS_TEB_PROFILE_FILE" \
    goal_slowdown_enabled:="$GPS_GOAL_SLOWDOWN_ENABLED" \
    goal_speed_cap_enabled:="$GPS_GOAL_SPEED_CAP_ENABLED" \
    goal_slowdown_decel:="$GPS_GOAL_COMFORTABLE_DECEL" \
    goal_slowdown_min_speed:="$GPS_GOAL_MIN_APPROACH_SPEED" \
    goal_slowdown_hard_stop_distance:="$GPS_GOAL_HARD_STOP_DISTANCE" \
    goal_slowdown_cmd_timeout:="$GPS_GOAL_CMD_TIMEOUT" \
    goal_slowdown_odom_timeout:="$GPS_GOAL_ODOM_TIMEOUT" \
    goal_near_commit_distance:="$GPS_GOAL_NEAR_COMMIT_DISTANCE" \
    goal_near_timeout:="$GPS_GOAL_NEAR_TIMEOUT" \
    goal_near_max_regression:="$GPS_GOAL_NEAR_MAX_REGRESSION" \
    output_cmd_vel_topic:="$GPS_OUTPUT_CMD_VEL_TOPIC" \
    xy_goal_tolerance:="$GPS_XY_GOAL_TOLERANCE" \
    max_vel_x:="$GPS_NAV_MAX_VEL_X" \
    max_vel_x_backwards:="$GPS_NAV_MAX_VEL_X_BACKWARDS" \
    penalty_epsilon:="$GPS_TEB_PENALTY_EPSILON" \
    weight_kinematics_forward_drive:="$GPS_TEB_FORWARD_DRIVE_WEIGHT"
fi

wait_topics "/move_base/status" 45.0
wait_topics "/move_base/local_costmap/costmap,/move_base/global_costmap/costmap" 45.0
if [[ "$MODE" == "gps" && "$GPS_LONG_RANGE_GOAL_ENABLED" == "true" ]]; then
  check_single_topic_subscriber "/gps/goal_fix" "/gps_long_range_goal_manager" 10
  check_topic_route "/move_base/goal" "/gps_long_range_goal_manager" "/move_base" 10
else
  check_topic_route "/gps/goal_fix" "" "/gps_goal" 10
  check_topic_route "/move_base_simple/goal" "/gps_goal" "/move_base" 10
fi
if [[ "$MODE" == "gps" && "$FOD_RECOVERY_STANDBY_ENABLED" == "true" ]]; then
  check_service_provider "/gps/long_range/set_paused" "/gps_long_range_goal_manager" 10
  check_service_provider "/fod_navigation_mode/set_fod_enabled" "/fod_navigation_mode" 10
  if [[ "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
    check_cmd_vel_route "/cmd_vel_navigation" "/move_base" "/gps_goal_speed_limiter" 10
    check_cmd_vel_route "/cmd_vel_gps" "/gps_goal_speed_limiter" "/fod_navigation_mode" 10
  else
    check_cmd_vel_route "/cmd_vel_gps" "/move_base" "/fod_navigation_mode" 10
  fi
  check_cmd_vel_route "/cmd_vel_fod" "/fod_visual_servo" "/fod_navigation_mode" 10
  check_cmd_vel_route "/cmd_vel" "/fod_navigation_mode" "/m2_driver" 10
elif [[ "$MODE" == "gps" && "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
  check_cmd_vel_route "/cmd_vel_navigation" "/move_base" "/gps_goal_speed_limiter" 10
  check_cmd_vel_route "/cmd_vel" "/gps_goal_speed_limiter" "/m2_driver" 10
else
  check_cmd_vel_route "/cmd_vel" "/move_base" "/m2_driver" 10
fi
echo "Robot bringup is running in $MODE mode."
if [[ "$MODE" == "gps" && "$FOD_RECOVERY_STANDBY_ENABLED" == "true" ]]; then
  echo "FOD mode command: $ROBOT_WS/scripts/fod_mode.sh start"
  echo "FOD/GPS status:  $ROBOT_WS/scripts/fod_mode.sh status"
fi
if (( SPLIT_TERMINALS )); then
  echo "Split terminals are open. Keep this terminal running; Ctrl+C here stops the launched processes."
  while true; do
    sleep 3600
  done
else
  wait
fi
