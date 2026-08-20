#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_PORT="${OFFLINE_ROS_PORT:-11313}"
PLAY_RATE="${PLAY_RATE:-1.0}"
VOXEL_SIZE="${MAPPING_3D_VOXEL_SIZE:-0.10}"
GRID_RESOLUTION="${MAPPING_2D_RESOLUTION:-0.10}"
SLICE_CENTER_Z="${MAPPING_SLICE_CENTER_Z:--0.42}"
SLICE_HALF_WIDTH="${MAPPING_SLICE_HALF_WIDTH:-0.10}"

source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

usage() {
  cat <<'EOF'
Usage: scripts/build_static_map_from_bag.sh BAG [MAP_NAME]

Offline-build the same three-map set produced by the Qt static-map workflow.
The bag must contain /cloud_registered, /dual_lidar/scan and /Odometry. A bag
whose only LaserScan is the historical MID360/LD19 fused /scan is rejected.
EOF
}

(( $# >= 1 && $# <= 2 )) || { usage >&2; exit 2; }
BAG_PATH="$(readlink -f -- "$1" 2>/dev/null || true)"
[[ -r "$BAG_PATH" && "$BAG_PATH" == *.bag ]] || {
  echo "Bag is not a readable complete .bag: $1" >&2
  exit 2
}
map_name="${2:-bag_$(basename "${BAG_PATH%.bag}")_$(date +%Y%m%d_%H%M%S)}"
[[ "$map_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "MAP_NAME contains unsafe characters: $map_name" >&2
  exit 2
}

bag_topics="$(rosbag info --yaml "$BAG_PATH" | awk '/^[[:space:]]+- topic:/{print $3}')"
for topic in /cloud_registered /dual_lidar/scan /Odometry; do
  grep -Fxq "$topic" <<<"$bag_topics" || {
    echo "Bag lacks required pure three-map topic: $topic" >&2
    exit 3
  }
done

export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

MAP_ROOT="$ROBOT_WS/global_maps/map_sets"
OUTPUT_DIR="$MAP_ROOT/$map_name"
MAP_3D_DIR="$OUTPUT_DIR/map_3d"
MAP_2D_DIR="$OUTPUT_DIR/map_2d"
MAP_FUSED_DIR="$OUTPUT_DIR/map_fused_2d"
[[ ! -e "$OUTPUT_DIR" ]] || {
  echo "Map output already exists: $OUTPUT_DIR" >&2
  exit 3
}
mkdir -p "$MAP_3D_DIR" "$MAP_2D_DIR" "$MAP_FUSED_DIR"

roscore_pid=""; pointcloud_pid=""; grid_pid=""; player_pid=""
cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "$player_pid" "$pointcloud_pid" "$grid_pid" "$roscore_pid"; do
    [[ -z "$pid" ]] || kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "$player_pid" "$pointcloud_pid" "$grid_pid" "$roscore_pid"; do
    [[ -z "$pid" ]] || wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

roscore -p "$ROS_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 & roscore_pid=$!
for _ in $(seq 1 100); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
rosparam list >/dev/null 2>&1 || { echo "Offline ROS master failed to start." >&2; exit 4; }
rosparam set /use_sim_time true

rosrun robot_bringup voxel_cloud_mapper \
  _input_topic:=/cloud_registered _output_file:="$MAP_3D_DIR/map.pcd" \
  _voxel_size:="$VOXEL_SIZE" >"$OUTPUT_DIR/voxel.log" 2>&1 &
pointcloud_pid=$!
rosrun robot_bringup fused_scan_mapper.py --ros \
  --output-dir "$MAP_2D_DIR" --scan-topic /dual_lidar/scan \
  --odom-topic /Odometry --resolution "$GRID_RESOLUTION" \
  >"$OUTPUT_DIR/grid.log" 2>&1 &
grid_pid=$!

rosbag play --clock --rate="$PLAY_RATE" "$BAG_PATH" --topics \
  /cloud_registered /dual_lidar/scan /Odometry >"$OUTPUT_DIR/play.log" 2>&1 &
player_pid=$!
wait "$player_pid"; player_pid=""
sleep 2
kill -INT "$pointcloud_pid" "$grid_pid"
wait "$pointcloud_pid"; pointcloud_pid=""
wait "$grid_pid"; grid_pid=""

for required in "$MAP_3D_DIR/map.pcd" "$MAP_2D_DIR/map.pgm" "$MAP_2D_DIR/map.yaml"; do
  [[ -s "$required" ]] || { echo "Offline mapper did not produce $required" >&2; exit 5; }
done
rosrun robot_bringup map_set_fuser.py \
  --map-2d "$MAP_2D_DIR/map.yaml" --map-3d "$MAP_3D_DIR/map.pcd" \
  --output-dir "$MAP_FUSED_DIR" --slice-center-z "$SLICE_CENTER_Z" \
  --slice-half-width "$SLICE_HALF_WIDTH" --resolution "$GRID_RESOLUTION"
cp -- "$MAP_2D_DIR/mapping_info.yaml" "$MAP_2D_DIR/config.yaml"
printf 'status: complete\ninput_topic: /cloud_registered\nvoxel_size_m: %s\n' \
  "$VOXEL_SIZE" >"$MAP_3D_DIR/config.yaml"
cat >"$OUTPUT_DIR/manifest.yaml" <<EOF
schema_version: 1
map_id: "$map_name"
status: "complete"
frame_id: map
source_bag: "$BAG_PATH"
map_3d:
  pcd: map_3d/map.pcd
map_2d:
  yaml: map_2d/map.yaml
  occupancy_source: dual_ld19_only
map_fused_2d:
  yaml: map_fused_2d/map.yaml
default_static_map_source: fused
initial_body_z_m: 0.0
EOF
latest_temporary="$MAP_ROOT/.latest.$$"
ln -s "$map_name" "$latest_temporary"
mv -Tf -- "$latest_temporary" "$MAP_ROOT/latest"
echo "STATIC_MAPPING_COMPLETE=$OUTPUT_DIR"

trap - EXIT INT TERM
cleanup
