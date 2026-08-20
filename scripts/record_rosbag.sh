#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="${1:-mode1}"
OUT_DIR="${BAG_DIR:-$ROBOT_WS/rosbags}"
BAG_PREFIX="${BAG_PREFIX:-mode1_nav}"
SPLIT_SIZE_MB="${SPLIT_SIZE_MB:-4096}"
COMPRESSION="${COMPRESSION:-none}"
WAIT_ROS_MASTER_SEC="${WAIT_ROS_MASTER_SEC:-10}"

# Keep this list explicit so a recording is reproducible and does not silently
# grow when unrelated high-bandwidth topics are added to the ROS graph.
MODE1_TOPICS=(
  /tf
  /tf_static
  /rosout
  /rosout_agg
  /canbus_msg
  /m2_driver/chassis_info
  /m2_driver/chassis_monitor
  /m2_driver/control_timeout
  /m2_driver/left_wheel_vel
  /m2_driver/right_wheel_vel
  /m2_driver/wheel_angle
  /m2_driver/emergency_stop
  /m2_driver/brake_set
  /odom
  /Odometry
  /cmd_vel_navigation
  /cmd_vel_gps
  /cmd_vel_fod
  /cmd_vel
  /fod_navigation_mode/state
  /fod_navigation_mode/status
  /fod_visual_servo/state
  /fod_visual_servo/status
  /fod_visual_servo/completed
  /livox/lidar
  /livox/imu
  /cloud_registered
  /cloud_registered_body
  /cloud_filtered_for_scan
  /mid360/scan
  /dual_lidar/scan
  /scan
  /avoidance/dual_lidar_active
  /avoidance/source_mode
  /map
  /map_metadata
  /fast_lio/localization_status
  /localization
  /fast_lio_localization/aligned_scan
  /initialpose
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
  /gps/long_range/final_goal
  /gps/long_range/subgoal
  /gps/long_range/status
  /gps/long_range/active
  /move_base_simple/goal
  /move_base/goal
  /move_base/cancel
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
  scripts/record_rosbag.sh mode1    # record mode1 mapping/navigation topics
  scripts/record_rosbag.sh all      # record every ROS topic with rosbag record -a
  scripts/record_rosbag.sh both     # record mode1 topics and all topics into two bags
  scripts/record_rosbag.sh topics   # print the mode1 topic list

Output:
  Bags are written to <workspace>/rosbags by default.

Environment:
  BAG_DIR=/path/to/dir          # output directory, default: <workspace>/rosbags
  BAG_PREFIX=mode1_nav          # output filename prefix
  SPLIT_SIZE_MB=4096            # split bag files at this size, 0 disables split
  COMPRESSION=none|lz4|bz2      # default: none
  WAIT_ROS_MASTER_SEC=10        # wait for roscore before recording

Examples:
  cd /home/slam/robot_j6m_ws
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

wait_ros_master() {
  local timeout="$1"
  local deadline=$((SECONDS + timeout))

  while (( SECONDS < deadline )); do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  echo "ROS master is not reachable at ${ROS_MASTER_URI:-<unset>}." >&2
  echo "Start the dual-host stack before recording." >&2
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

  echo "==> recording mode1 mapping/navigation topics"
  echo "==> output prefix: $output_base"
  echo "==> stop from the Qt console to finalize the bag"
  exec rosbag "${args[@]}"
}

record_all() {
  local stamp="$1"
  local output_base="$OUT_DIR/${BAG_PREFIX}_all_${stamp}"
  local -a args=(record)

  build_common_args args
  args+=(-O "$output_base" -a)

  echo "==> recording all ROS topics"
  echo "==> output prefix: $output_base"
  echo "==> stop from the Qt console to finalize the bag"
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

# Resolve the dual-host ROS master and the current workspace dependencies even
# when the script is launched outside the already-configured Qt environment.
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

command -v rosbag >/dev/null 2>&1 || {
  echo "rosbag command not found after sourcing the ROS environment." >&2
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
