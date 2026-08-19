#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
BAG_ROOT="$ROBOT_WS/rosbags"
MAP_ROOT="$ROBOT_WS/global_maps"
FAST_LIO_CONFIG="$ROBOT_WS/src/localization_fastlio/FAST_LIO/config/mid360.yaml"
FAST_LIO_RUNTIME="$ROBOT_WS/runtime/fast_lio"
FAST_LIO_PCD_DIR="$FAST_LIO_RUNTIME/PCD"

ROS_PORT="${OFFLINE_ROS_PORT:-11312}"
PLAY_RATE="${PLAY_RATE:-1.0}"
PLAY_START_SEC="${PLAY_START_SEC:-0}"
PLAY_DURATION_SEC="${PLAY_DURATION_SEC:-}"
VOXEL_LEAF_SIZE="${VOXEL_LEAF_SIZE:-0.10}"

roscore_pid=""
fast_lio_pid=""
bag_player_pid=""

usage() {
  cat <<'EOF'
Usage:
  ./scripts/build_global_map.sh [bag-file-or-name] [map-name]

Defaults:
  bag-file  newest complete .bag under <workspace>/rosbags
  map-name  bag filename without .bag

Output:
  <workspace>/global_maps/<map-name>/global_map_raw.pcd
  <workspace>/global_maps/<map-name>/global_map.pcd

Environment:
  OFFLINE_ROS_PORT=11312   isolated ROS master port
  PLAY_RATE=1.0            bag playback speed
  PLAY_START_SEC=0         start offset for diagnostics
  PLAY_DURATION_SEC=       optional duration for diagnostics
  VOXEL_LEAF_SIZE=0.10     displayed map voxel size in metres
EOF
}

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "$bag_player_pid" "$fast_lio_pid" "$roscore_pid"; do
    [[ -n "$pid" ]] || continue
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
      for _ in $(seq 1 20); do
        kill -0 "$pid" >/dev/null 2>&1 || break
        sleep 0.1
      done
      if kill -0 "$pid" >/dev/null 2>&1; then
        kill -TERM "$pid" >/dev/null 2>&1 || true
      fi
    fi
    wait "$pid" 2>/dev/null || true
  done
}

handle_signal() {
  echo "Interrupted; stopping isolated offline mapping processes." >&2
  exit 130
}

latest_bag() {
  find "$BAG_ROOT" -maxdepth 1 -type f -name '*.bag' \
    -printf '%T@ %p\n' | sort -nr | sed -n '1s/^[^ ]* //p'
}

resolve_bag() {
  local requested="${1:-}" candidate
  if [[ -z "$requested" ]]; then
    candidate="$(latest_bag)"
  elif [[ -f "$requested" ]]; then
    candidate="$requested"
  else
    candidate="$BAG_ROOT/$requested"
  fi
  [[ -n "$candidate" && -f "$candidate" ]] || return 1
  realpath -e "$candidate"
}

wait_for_master() {
  for _ in $(seq 1 100); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    kill -0 "$roscore_pid" >/dev/null 2>&1 || return 1
    sleep 0.1
  done
  return 1
}

wait_for_fast_lio() {
  for _ in $(seq 1 200); do
    if rosnode ping -c 1 /laserMapping >/dev/null 2>&1 &&
       rostopic info /livox/lidar 2>/dev/null | grep -q '/laserMapping' &&
       rostopic info /livox/imu 2>/dev/null | grep -q '/laserMapping'; then
      return 0
    fi
    kill -0 "$fast_lio_pid" >/dev/null 2>&1 || return 1
    sleep 0.1
  done
  return 1
}

stop_fast_lio_and_wait_for_map() {
  local fast_status=0
  echo "==> stopping FAST-LIO so it flushes the accumulated PCD"
  kill -INT "$fast_lio_pid"
  for _ in $(seq 1 1200); do
    kill -0 "$fast_lio_pid" >/dev/null 2>&1 || break
    sleep 0.25
  done
  if kill -0 "$fast_lio_pid" >/dev/null 2>&1; then
    fail "FAST-LIO did not finish writing its PCD within 300 seconds"
  fi
  wait "$fast_lio_pid" || fast_status=$?
  fast_lio_pid=""
  case "$fast_status" in
    0|130|143) ;;
    *) fail "FAST-LIO exited with status $fast_status; inspect fast_lio.log" ;;
  esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi
(( $# <= 2 )) || {
  usage >&2
  exit 2
}

[[ "$ROS_PORT" =~ ^[0-9]+$ ]] && (( ROS_PORT >= 1024 && ROS_PORT <= 65535 )) ||
  fail "OFFLINE_ROS_PORT must be an integer from 1024 to 65535"
[[ "$PLAY_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "PLAY_RATE must be positive"
[[ "$PLAY_START_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "PLAY_START_SEC must be non-negative"
[[ -z "$PLAY_DURATION_SEC" || "$PLAY_DURATION_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
  fail "PLAY_DURATION_SEC must be empty or positive"
[[ "$VOXEL_LEAF_SIZE" =~ ^0*[.]?[0-9]+$ ]] || fail "VOXEL_LEAF_SIZE must be positive"

mkdir -p "$BAG_ROOT" "$MAP_ROOT" "$FAST_LIO_PCD_DIR"
bag_path="$(resolve_bag "${1:-}" || true)"
[[ -n "$bag_path" ]] || fail "no complete bag found; place one under $BAG_ROOT"
[[ "$bag_path" == *.bag ]] || fail "input must end in .bag (not .bag.active)"

bag_filename="$(basename "$bag_path")"
bag_stem="${bag_filename%.bag}"
map_name="${2:-$bag_stem}"
[[ "$map_name" =~ ^[A-Za-z0-9._-]+$ ]] ||
  fail "map-name may contain only letters, digits, dot, underscore and hyphen"
output_dir="$MAP_ROOT/$map_name"
[[ ! -e "$output_dir" ]] || fail "output already exists: $output_dir"

command -v roscore >/dev/null 2>&1 || fail "roscore command not found"
command -v rosbag >/dev/null 2>&1 || fail "rosbag command not found"
command -v pcl_voxel_grid >/dev/null 2>&1 || fail "pcl_voxel_grid command not found"
command -v flock >/dev/null 2>&1 || fail "flock command not found"
[[ -r "$FAST_LIO_CONFIG" ]] || fail "missing FAST-LIO config: $FAST_LIO_CONFIG"
[[ -x "$ROBOT_WS/devel/lib/fast_lio/fastlio_mapping" ]] ||
  fail "FAST-LIO is not built in this workspace"

exec 9>"$FAST_LIO_RUNTIME/offline_mapping.lock"
flock -n 9 || fail "another offline mapping job is already running"
if compgen -G "$FAST_LIO_PCD_DIR/scans*.pcd" >/dev/null; then
  fail "unclaimed FAST-LIO PCD files exist in $FAST_LIO_PCD_DIR; move them before retrying"
fi

if ss -ltnH | awk '{print $4}' | grep -Eq "[:.]${ROS_PORT}$"; then
  fail "local port $ROS_PORT is already in use; set OFFLINE_ROS_PORT to another port"
fi

mkdir "$output_dir"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME

rosbag info "$bag_path" >"$output_dir/bag_info.txt"
grep -Eq '^[[:space:]]+/livox/lidar[[:space:]]' "$output_dir/bag_info.txt" ||
  fail "bag does not contain /livox/lidar"
grep -Eq '^[[:space:]]+/livox/imu[[:space:]]' "$output_dir/bag_info.txt" ||
  fail "bag does not contain /livox/imu"

trap cleanup EXIT
trap handle_signal INT TERM

echo "==> bag: $bag_path"
echo "==> map output: $output_dir"
echo "==> isolated ROS master: $ROS_MASTER_URI"
echo "==> playback rate: ${PLAY_RATE}x"

roscore -p "$ROS_PORT" >"$output_dir/roscore.log" 2>&1 &
roscore_pid=$!
wait_for_master || fail "isolated roscore did not start; inspect roscore.log"

rosparam load "$FAST_LIO_CONFIG"
rosparam set /feature_extract_enable false
rosparam set /point_filter_num 5
rosparam set /max_iteration 3
rosparam set /filter_size_surf 0.5
rosparam set /filter_size_map 0.5
rosparam set /cube_side_length 1000
rosparam set /runtime_pos_log_enable false
rosparam set /publish/odom_pub_en true
rosparam set /publish/tf_pub_en true
rosparam set /pcd_save/pcd_save_en true
rosparam set /pcd_save/interval -1

rosrun fast_lio fastlio_mapping >"$output_dir/fast_lio.log" 2>&1 &
fast_lio_pid=$!
wait_for_fast_lio || fail "FAST-LIO did not subscribe to Livox topics; inspect fast_lio.log"

# FAST-LIO calculates from the sensor header timestamps. Keep its ROS loop on
# the wall clock so it can still wake up and flush the PCD after playback ends.
play_args=(play --delay=1 --queue=1000 --rate="$PLAY_RATE" --start="$PLAY_START_SEC")
if [[ -n "$PLAY_DURATION_SEC" ]]; then
  play_args+=(--duration="$PLAY_DURATION_SEC")
fi
play_args+=("$bag_path" --topics /livox/lidar /livox/imu)

echo "==> replaying Livox LiDAR and IMU into offline FAST-LIO"
rosbag "${play_args[@]}" >"$output_dir/rosbag_play.log" 2>&1 &
bag_player_pid=$!
bag_status=0
wait "$bag_player_pid" || bag_status=$?
bag_player_pid=""
(( bag_status == 0 )) || fail "rosbag playback failed with status $bag_status; inspect rosbag_play.log"

# Give the callback queue a short drain window after the final bag message.
sleep 3
stop_fast_lio_and_wait_for_map

raw_runtime_map="$FAST_LIO_PCD_DIR/scans.pcd"
[[ -s "$raw_runtime_map" ]] || fail "FAST-LIO completed but did not create scans.pcd"
raw_map="$output_dir/global_map_raw.pcd"
map_file="$output_dir/global_map.pcd"
mv "$raw_runtime_map" "$raw_map"

echo "==> downsampling the map to ${VOXEL_LEAF_SIZE} m voxels"
pcl_voxel_grid "$raw_map" "$map_file" \
  -leaf "$VOXEL_LEAF_SIZE,$VOXEL_LEAF_SIZE,$VOXEL_LEAF_SIZE" \
  >"$output_dir/voxel_grid.log" 2>&1
[[ -s "$map_file" ]] || fail "voxel filtering did not produce global_map.pcd"

generated_at="$(date --iso-8601=seconds)"
raw_points="$(grep -a -m1 '^POINTS ' "$raw_map" | awk '{print $2}')"
map_points="$(grep -a -m1 '^POINTS ' "$map_file" | awk '{print $2}')"
{
  echo "status=complete"
  echo "generated_at=$generated_at"
  echo "source_bag=$bag_path"
  echo "fast_lio_config=$FAST_LIO_CONFIG"
  echo "play_rate=$PLAY_RATE"
  echo "play_start_sec=$PLAY_START_SEC"
  echo "play_duration_sec=${PLAY_DURATION_SEC:-full}"
  echo "voxel_leaf_size_m=$VOXEL_LEAF_SIZE"
  echo "raw_points=${raw_points:-unknown}"
  echo "map_points=${map_points:-unknown}"
} >"$output_dir/generation_info.txt"

ln -sfn "$map_name" "$MAP_ROOT/latest"
echo "==> global map complete"
echo "    raw:  $raw_map"
echo "    view: $map_file"
echo "    run:  ./scripts/view_global_map.sh $map_name"
