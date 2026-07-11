#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${WS_SETUP:-$ROBOT_WS/devel/setup.bash}"

MODE="${1:-mode1}"
OUT_DIR="${BAG_DIR:-$PWD}"
BAG_PREFIX="${BAG_PREFIX:-mode1_nav}"
SPLIT_SIZE_MB="${SPLIT_SIZE_MB:-4096}"
COMPRESSION="${COMPRESSION:-none}"
WAIT_ROS_MASTER_SEC="${WAIT_ROS_MASTER_SEC:-10}"

MODE1_TOPICS=(
  /tf
  /tf_static
  /rosout
  /rosout_agg
  /canbus_msg
  /odom
  /cmd_vel
  /livox/lidar
  /livox/imu
  /cloud_registered_body
  /cloud_filtered_for_scan
  /scan
  /gps/fix
  /gps/heading
  /gps/pose
  /gps/odom
  /gps/static_error/current
  /gps/static_error/rms
  /gps/static_error/max
  /gps/static_error/std_x
  /gps/static_error/std_y
  /gps/static_error/summary
  /gps/goal_fix
  /move_base_simple/goal
  /move_base/status
  /move_base/feedback
  /move_base/result
  /move_base/current_goal
  /move_base/global_costmap/costmap
  /move_base/global_costmap/costmap_updates
  /move_base/global_costmap/footprint
  /move_base/local_costmap/costmap
  /move_base/local_costmap/costmap_updates
  /move_base/local_costmap/footprint
  /move_base/TebLocalPlannerROS/global_plan
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/TebLocalPlannerROS/teb_poses
  /move_base/TebLocalPlannerROS/teb_markers
)

PIDS=()

usage() {
  cat <<'EOF'
Usage:
  scripts/record_rosbag.sh mode1    # record mode1 GPS navigation replay topics
  scripts/record_rosbag.sh all      # record every ROS topic with rosbag record -a
  scripts/record_rosbag.sh both     # record mode1 topics and all topics into two bags
  scripts/record_rosbag.sh topics   # print the mode1 topic list

Output:
  Bags are written to the current directory by default.

Environment:
  BAG_DIR=/path/to/dir          # output directory, default: current directory
  BAG_PREFIX=mode1_nav          # output filename prefix
  SPLIT_SIZE_MB=4096            # split bag files at this size, 0 disables split
  COMPRESSION=none|lz4|bz2      # default: none
  WAIT_ROS_MASTER_SEC=10        # wait for roscore before recording

Examples:
  cd /home/robot/robot_ws
  ./scripts/record_rosbag.sh mode1
  ./scripts/record_rosbag.sh all
  BAG_PREFIX=field_test_01 ./scripts/record_rosbag.sh both
EOF
}

print_topics() {
  printf '%s\n' "${MODE1_TOPICS[@]}"
}

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
}

require_file() {
  [[ -f "$1" ]] || {
    echo "Missing file: $1" >&2
    exit 2
  }
}

wait_ros_master() {
  local timeout="$1"
  local deadline=$((SECONDS + timeout))

  while (( SECONDS < deadline )); do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  echo "ROS master is not reachable." >&2
  echo "Start mode1 first, for example:" >&2
  echo "  TERMINAL_MODE=split ./scripts/bringup.sh gps" >&2
  exit 3
}

build_common_args() {
  local -n out_args="$1"

  if [[ "$SPLIT_SIZE_MB" != "0" ]]; then
    out_args+=(--split --size="$SPLIT_SIZE_MB")
  fi

  case "$COMPRESSION" in
    none) ;;
    lz4) out_args+=(--lz4) ;;
    bz2) out_args+=(--bz2) ;;
    *)
      echo "Invalid COMPRESSION: $COMPRESSION" >&2
      echo "Use COMPRESSION=none, lz4, or bz2." >&2
      exit 1
      ;;
  esac
}

record_mode1() {
  local stamp="$1"
  local output_base="$OUT_DIR/${BAG_PREFIX}_mode1_${stamp}"
  local -a args=(record)

  build_common_args args
  args+=(-O "$output_base")
  args+=("${MODE1_TOPICS[@]}")

  echo "==> recording mode1 topics"
  echo "==> output: ${output_base}.bag"
  echo "==> stop with Ctrl+C"
  exec rosbag "${args[@]}"
}

record_all() {
  local stamp="$1"
  local output_base="$OUT_DIR/${BAG_PREFIX}_all_${stamp}"
  local -a args=(record)

  build_common_args args
  args+=(-O "$output_base" -a)

  echo "==> recording all ROS topics"
  echo "==> output: ${output_base}.bag"
  echo "==> stop with Ctrl+C"
  exec rosbag "${args[@]}"
}

case "$MODE" in
  -h|--help|help)
    usage
    exit 0
    ;;
  topics|--topics|list-topics)
    print_topics
    exit 0
    ;;
  mode1|all|both) ;;
  *)
    usage >&2
    exit 1
    ;;
esac

require_file "$ROS_SETUP"
require_file "$WS_SETUP"
source "$ROS_SETUP"
source "$WS_SETUP"

command -v rosbag >/dev/null 2>&1 || {
  echo "rosbag command not found after sourcing ROS environment." >&2
  exit 2
}

mkdir -p "$OUT_DIR"
wait_ros_master "$WAIT_ROS_MASTER_SEC"

STAMP="$(date +%Y%m%d_%H%M%S)"

case "$MODE" in
  mode1)
    record_mode1 "$STAMP"
    ;;
  all)
    record_all "$STAMP"
    ;;
  both)
    echo "==> recording two bags into: $OUT_DIR"
    echo "==> all-topic recording can be large because it includes raw Livox data"
    trap 'cleanup; exit 130' INT TERM
    record_mode1 "$STAMP" &
    PIDS+=("$!")
    sleep 1
    record_all "$STAMP" &
    PIDS+=("$!")
    wait
    ;;
esac
