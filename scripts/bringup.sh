#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
MODE="${1:-fast_lio}"
GPS_NAV_MAX_SPEED_ARG="${2:-}"
GPS_TEB_PROFILE_ARG="${3:-}"
TERMINAL_MODE="${TERMINAL_MODE:-auto}"
CLEAN_START="${CLEAN_START:-true}"
CAN_PORT="${CAN_PORT:-/dev/ttyUSB0}"
GPS_PORT="${GPS_PORT:-/dev/ttyUSB1}"
GPS_BAUD_RATE="${GPS_BAUD_RATE:-115200}"
FAST_LIO_GPS_YAW_OFFSET_DEG="${FAST_LIO_GPS_YAW_OFFSET_DEG:-0.0}"
GPS_NAV_MAX_VEL_X="${GPS_NAV_MAX_VEL_X:-1.5}"
GPS_NAV_MAX_VEL_X_BACKWARDS="${GPS_NAV_MAX_VEL_X_BACKWARDS:-1.4}"
GPS_USE_WHEEL_ODOM="${GPS_USE_WHEEL_ODOM:-false}"
GPS_USE_WHEEL_TWIST="${GPS_USE_WHEEL_TWIST:-true}"
GPS_WHEEL_TWIST_TIMEOUT="${GPS_WHEEL_TWIST_TIMEOUT:-0.5}"
GPS_RMC_SPEED_TIMEOUT="${GPS_RMC_SPEED_TIMEOUT:-1.0}"
GPS_HEADING_SOURCE="${GPS_HEADING_SOURCE:-dual_antenna}"
GPS_HEADING_TIMEOUT="${GPS_HEADING_TIMEOUT:-1.0}"
GPS_HEADING_REQUIRED_SOLUTION_STATUS="${GPS_HEADING_REQUIRED_SOLUTION_STATUS:-SOL_COMPUTED}"
GPS_HEADING_REQUIRED_POSITION_TYPES="${GPS_HEADING_REQUIRED_POSITION_TYPES:-NARROW_INT}"
GPS_ANTENNA_OFFSET_X="${GPS_ANTENNA_OFFSET_X:--0.3}"
GPS_ANTENNA_OFFSET_Y="${GPS_ANTENNA_OFFSET_Y:-0.0}"
GPS_HEADING_MIN_SPEED="${GPS_HEADING_MIN_SPEED:-0.05}"
GPS_MIN_COURSE_DISTANCE="${GPS_MIN_COURSE_DISTANCE:-0.2}"
GPS_INITIAL_YAW="${GPS_INITIAL_YAW:-}"
GPS_COMPASS_HEADING="${GPS_COMPASS_HEADING:-}"
GPS_COMPASS_HEADING_DEG="${GPS_COMPASS_HEADING_DEG:-}"
GPS_TEB_PROFILE="${GPS_TEB_PROFILE:-cruise}"
GPS_TEB_PROFILE_FILE=""
GPS_TEB_PENALTY_EPSILON="${GPS_TEB_PENALTY_EPSILON:-}"
GPS_TEB_FORWARD_DRIVE_WEIGHT="${GPS_TEB_FORWARD_DRIVE_WEIGHT:-}"
GPS_XY_GOAL_TOLERANCE="${GPS_XY_GOAL_TOLERANCE:-0.3}"
GPS_GLOBAL_COSTMAP_SIZE="${GPS_GLOBAL_COSTMAP_SIZE:-200.0}"
GPS_GLOBAL_COSTMAP_RESOLUTION="${GPS_GLOBAL_COSTMAP_RESOLUTION:-0.25}"
GPS_GOAL_SLOWDOWN_ENABLED="${GPS_GOAL_SLOWDOWN_ENABLED:-true}"
GPS_GOAL_COMFORTABLE_DECEL="${GPS_GOAL_COMFORTABLE_DECEL:-0.4}"
GPS_GOAL_MIN_APPROACH_SPEED="${GPS_GOAL_MIN_APPROACH_SPEED:-0.15}"
GPS_GOAL_HARD_STOP_DISTANCE="${GPS_GOAL_HARD_STOP_DISTANCE:-0.2}"
GPS_GOAL_CMD_TIMEOUT="${GPS_GOAL_CMD_TIMEOUT:-0.5}"
GPS_GOAL_ODOM_TIMEOUT="${GPS_GOAL_ODOM_TIMEOUT:-1.0}"
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
  echo "  CAN_PORT=/dev/ttyUSB0"
  echo "  GPS_PORT=/dev/ttyUSB1"
  echo "  GPS_BAUD_RATE=115200"
  echo "  FAST_LIO_GPS_YAW_OFFSET_DEG=0.0"
  echo "  GPS_NAV_MAX_VEL_X=1.5       # overridden by the optional gps speed argument"
  echo "  GPS_NAV_MAX_VEL_X_BACKWARDS=1.4 # capped by the optional gps speed argument"
  echo "  GPS_TEB_PROFILE=cruise|obstacle # used when the third argument is omitted"
  echo "  GPS_USE_WHEEL_ODOM=false        # gps mode uses GPS position directly by default"
  echo "  GPS_USE_WHEEL_TWIST=true        # publish fresh signed chassis twist in /gps/odom"
  echo "  GPS_WHEEL_TWIST_TIMEOUT=0.5     # s; then fall back to GNSS motion estimates"
  echo "  GPS_RMC_SPEED_TIMEOUT=1.0       # s; discard cached RMC speed/course after this"
  echo "  GPS_HEADING_SOURCE=dual_antenna"
  echo "  GPS_HEADING_TIMEOUT=1.0"
  echo "  GPS_HEADING_REQUIRED_SOLUTION_STATUS=SOL_COMPUTED"
  echo "  GPS_HEADING_REQUIRED_POSITION_TYPES=NARROW_INT"
  echo "  GPS_ANTENNA_OFFSET_X=-0.3        # main antenna x offset in base_link"
  echo "  GPS_ANTENNA_OFFSET_Y=0.0         # main antenna y offset in base_link"
  echo "  GPS_HEADING_MIN_SPEED=0.05"
  echo "  GPS_MIN_COURSE_DISTANCE=0.2"
  echo "  GPS_INITIAL_YAW=0.0             # radians, used until GPS course is available"
  echo "  GPS_COMPASS_HEADING=东北45度     # phone compass heading, 0=N, 90=E"
  echo "  GPS_COMPASS_HEADING_DEG=45      # numeric compass heading, 0=N, 90=E"
  echo "  GPS_TEB_PENALTY_EPSILON=0.03     # optional profile-default override"
  echo "  GPS_TEB_FORWARD_DRIVE_WEIGHT=... # optional profile-default override"
  echo "  GPS_XY_GOAL_TOLERANCE=0.3        # m; TEB declares the GPS goal reached"
  echo "  GPS_GLOBAL_COSTMAP_SIZE=200.0     # m; rolling square, about +/-100 m around robot"
  echo "  GPS_GLOBAL_COSTMAP_RESOLUTION=0.25 # m/cell; size/resolution is capped at 1M cells"
  echo "  GPS_GOAL_SLOWDOWN_ENABLED=true   # only caps forward speed near the goal"
  echo "  GPS_GOAL_COMFORTABLE_DECEL=0.4   # m/s^2; smaller starts gentler braking earlier"
  echo "  GPS_GOAL_MIN_APPROACH_SPEED=0.15 # m/s outside the limiter hard-stop radius"
  echo "  GPS_GOAL_HARD_STOP_DISTANCE=0.2  # m; safety stop, independent of TEB tolerance"
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

min_number() {
  awk -v first="$1" -v second="$2" 'BEGIN {
    if (first < second) print first;
    else print second;
  }'
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
    echo "  sudo usermod -aG dialout robot" >&2
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
    /gps_goal
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

start_launch() {
  local label="$1"
  shift
  if (( SPLIT_TERMINALS )); then
    start_terminal_command "$label" roslaunch "$@"
  else
    echo "==> starting $label"
    roslaunch "$@" &
    PIDS+=("$!")
  fi
  wait_ros_master 15
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
  rosrun robot_diagnostics check_odom.py \
    _topic:="$topic" \
    _timeout:=15.0 \
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

check_cmd_vel_route() {
  local topic="${1:-/cmd_vel}"
  local expected_publisher="${2:-/move_base}"
  local expected_subscriber="${3:-/m2_driver}"
  local timeout="${4:-10}"
  local deadline=$((SECONDS + timeout))
  local info=""

  while (( SECONDS < deadline )); do
    info="$(rostopic info "$topic" 2>/dev/null || true)"
    if grep -Fq "$expected_publisher" <<<"$info" && grep -Fq "$expected_subscriber" <<<"$info"; then
      return 0
    fi
    sleep 0.5
  done

  echo "Command velocity route is not connected on $topic." >&2
  echo "Expected $expected_publisher publisher and $expected_subscriber subscriber." >&2
  if [[ -n "$info" ]]; then
    echo "$info" >&2
  fi
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
      ;;
    obstacle)
      GPS_TEB_PROFILE_FILE="$ROBOT_WS/config/teb_profiles/gps_obstacle.yaml"
      GPS_TEB_PENALTY_EPSILON="${GPS_TEB_PENALTY_EPSILON:-0.03}"
      GPS_TEB_FORWARD_DRIVE_WEIGHT="${GPS_TEB_FORWARD_DRIVE_WEIGHT:-60.0}"
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
  if ! is_positive_number "$GPS_XY_GOAL_TOLERANCE"; then
    echo "Invalid GPS_XY_GOAL_TOLERANCE: $GPS_XY_GOAL_TOLERANCE" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_GLOBAL_COSTMAP_SIZE"; then
    echo "Invalid GPS_GLOBAL_COSTMAP_SIZE: $GPS_GLOBAL_COSTMAP_SIZE" >&2
    exit 1
  fi
  if ! is_positive_number "$GPS_GLOBAL_COSTMAP_RESOLUTION"; then
    echo "Invalid GPS_GLOBAL_COSTMAP_RESOLUTION: $GPS_GLOBAL_COSTMAP_RESOLUTION" >&2
    exit 1
  fi
  GPS_GLOBAL_COSTMAP_CELLS="$(awk \
    -v size="$GPS_GLOBAL_COSTMAP_SIZE" \
    -v resolution="$GPS_GLOBAL_COSTMAP_RESOLUTION" \
    'BEGIN {
      cells_per_side = int(size / resolution);
      if (cells_per_side * resolution < size) cells_per_side++;
      printf "%.0f\n", cells_per_side * cells_per_side;
    }')"
  if ! awk -v cells="$GPS_GLOBAL_COSTMAP_CELLS" \
    'BEGIN { exit !(cells <= 1000000) }'; then
    echo "GPS global costmap would contain $GPS_GLOBAL_COSTMAP_CELLS cells; limit is 1000000." >&2
    echo "Increase GPS_GLOBAL_COSTMAP_RESOLUTION or reduce GPS_GLOBAL_COSTMAP_SIZE." >&2
    exit 1
  fi
  case "$GPS_GOAL_SLOWDOWN_ENABLED" in
    true|false) ;;
    *)
      echo "Invalid GPS_GOAL_SLOWDOWN_ENABLED: $GPS_GOAL_SLOWDOWN_ENABLED (use true or false)" >&2
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
    if ! is_nonnegative_number "$GPS_GOAL_HARD_STOP_DISTANCE"; then
      echo "Invalid GPS_GOAL_HARD_STOP_DISTANCE: $GPS_GOAL_HARD_STOP_DISTANCE" >&2
      exit 1
    fi
    if ! awk -v hard_stop="$GPS_GOAL_HARD_STOP_DISTANCE" -v tolerance="$GPS_XY_GOAL_TOLERANCE" \
      'BEGIN { exit !(hard_stop < tolerance) }'; then
      echo "GPS_GOAL_HARD_STOP_DISTANCE ($GPS_GOAL_HARD_STOP_DISTANCE) must be smaller than GPS_XY_GOAL_TOLERANCE ($GPS_XY_GOAL_TOLERANCE)" >&2
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
  fi
  require_file "$GPS_TEB_PROFILE_FILE"
  echo "==> GPS TEB profile: $GPS_TEB_PROFILE ($GPS_TEB_PROFILE_FILE)"
  echo "==> GPS navigation speed limits: forward=$GPS_NAV_MAX_VEL_X m/s, backward=$GPS_NAV_MAX_VEL_X_BACKWARDS m/s"
  echo "==> GPS odom twist: wheel=$GPS_USE_WHEEL_TWIST, wheel timeout=$GPS_WHEEL_TWIST_TIMEOUT s, RMC timeout=$GPS_RMC_SPEED_TIMEOUT s"
  echo "==> GPS goal distances: TEB tolerance=$GPS_XY_GOAL_TOLERANCE m, limiter hard stop=$GPS_GOAL_HARD_STOP_DISTANCE m"
  echo "==> GPS global costmap: ${GPS_GLOBAL_COSTMAP_SIZE} x ${GPS_GLOBAL_COSTMAP_SIZE} m, resolution=$GPS_GLOBAL_COSTMAP_RESOLUTION m, cells=$GPS_GLOBAL_COSTMAP_CELLS"
  echo "==> GPS TEB forward-drive weight: $GPS_TEB_FORWARD_DRIVE_WEIGHT"
  if [[ "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
    echo "==> GPS goal slowdown: decel=$GPS_GOAL_COMFORTABLE_DECEL m/s^2, minimum approach=$GPS_GOAL_MIN_APPROACH_SPEED m/s, hard stop=$GPS_GOAL_HARD_STOP_DISTANCE m"
  else
    echo "==> GPS goal slowdown: disabled"
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

source "$ROS_SETUP"
source "$ROBOT_WS/devel/setup.bash"

cleanup_existing_nodes
setup_terminal_mode
start_ros_master

make_writable "$CAN_PORT"
roslaunch robot_diagnostics check_can.launch port:="$CAN_PORT"
start_launch "CAN chassis driver" robot_bringup can.launch port_name:="$CAN_PORT" publish_tf:=false
wait_topics "/canbus_msg" 30.0

if [[ "$MODE" == "fast_lio" || "$MODE" == "fast_lio_gps" ]]; then
  start_launch "Livox MID360 driver" robot_bringup livox_mid360.launch
  wait_topics "/livox/lidar,/livox/imu" 45.0

  start_launch "FAST_LIO localization" robot_bringup fast_lio.launch
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
  start_launch "Arena navigation" robot_bringup navigation_arena.launch localization_source:=fast_lio
else
  make_writable "$GPS_PORT"
  start_launch "Livox MID360 driver" robot_bringup livox_mid360.launch
  wait_topics "/livox/lidar,/livox/imu" 45.0

  start_launch "FAST_LIO point cloud registration" robot_bringup fast_lio.launch odom_pub_en:=false tf_pub_en:=false
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
    gps_antenna_offset_x:="$GPS_ANTENNA_OFFSET_X" \
    gps_antenna_offset_y:="$GPS_ANTENNA_OFFSET_Y" \
    heading_min_speed:="$GPS_HEADING_MIN_SPEED" \
    min_course_distance:="$GPS_MIN_COURSE_DISTANCE" \
    initial_yaw:="$GPS_INITIAL_YAW"
  wait_topics "/gps/fix,/gps/pose,/gps/odom" 60.0
  check_odom "/gps/odom" "camera_init" "base_link"
  check_tf "camera_init" "base_link" 30.0

  start_launch "GPS goal converter" gps_module gps_goal.launch \
    frame_id:=camera_init \
    odom_topic:=/gps/odom \
    goal_yaw_mode:=bearing

  check_tf "camera_init" "base_link" 10.0
  start_launch "Arena navigation" robot_bringup navigation_arena.launch \
    localization_source:=gps \
    teb_profile_file:="$GPS_TEB_PROFILE_FILE" \
    goal_slowdown_enabled:="$GPS_GOAL_SLOWDOWN_ENABLED" \
    goal_slowdown_decel:="$GPS_GOAL_COMFORTABLE_DECEL" \
    goal_slowdown_min_speed:="$GPS_GOAL_MIN_APPROACH_SPEED" \
    goal_slowdown_hard_stop_distance:="$GPS_GOAL_HARD_STOP_DISTANCE" \
    goal_slowdown_cmd_timeout:="$GPS_GOAL_CMD_TIMEOUT" \
    goal_slowdown_odom_timeout:="$GPS_GOAL_ODOM_TIMEOUT" \
    xy_goal_tolerance:="$GPS_XY_GOAL_TOLERANCE" \
    global_costmap_size:="$GPS_GLOBAL_COSTMAP_SIZE" \
    global_costmap_resolution:="$GPS_GLOBAL_COSTMAP_RESOLUTION" \
    max_vel_x:="$GPS_NAV_MAX_VEL_X" \
    max_vel_x_backwards:="$GPS_NAV_MAX_VEL_X_BACKWARDS" \
    penalty_epsilon:="$GPS_TEB_PENALTY_EPSILON" \
    weight_kinematics_forward_drive:="$GPS_TEB_FORWARD_DRIVE_WEIGHT"
fi

wait_topics "/move_base/status" 45.0
wait_topics "/move_base/local_costmap/costmap,/move_base/global_costmap/costmap" 45.0
if [[ "$MODE" == "gps" && "$GPS_GOAL_SLOWDOWN_ENABLED" == "true" ]]; then
  check_cmd_vel_route "/cmd_vel_navigation" "/move_base" "/gps_goal_speed_limiter" 10
  check_cmd_vel_route "/cmd_vel" "/gps_goal_speed_limiter" "/m2_driver" 10
else
  check_cmd_vel_route "/cmd_vel" "/move_base" "/m2_driver" 10
fi
echo "Robot bringup is running in $MODE mode."
if (( SPLIT_TERMINALS )); then
  echo "Split terminals are open. Keep this terminal running; Ctrl+C here stops the launched processes."
  while true; do
    sleep 3600
  done
else
  wait
fi
