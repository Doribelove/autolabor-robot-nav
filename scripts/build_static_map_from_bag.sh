#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

ROS_PORT="${OFFLINE_ROS_PORT:-11313}"
PLAY_RATE="${PLAY_RATE:-1.0}"
PUBLISH_LATEST="${PUBLISH_LATEST:-true}"
VOXEL_SIZE="${MAPPING_3D_VOXEL_SIZE:-0.10}"
VOXEL_MIN_FRAMES="${MAPPING_3D_MIN_FRAME_OBSERVATIONS:-3}"
POINT_MAX_RANGE="${MAPPING_3D_MAX_POINT_RANGE:-20.0}"
GRID_RESOLUTION="${MAPPING_2D_RESOLUTION:-0.10}"
LIDAR_MIN_FRAMES="${MAPPING_2D_MIN_OCCUPIED_OBSERVATIONS:-5}"
SLICE_CENTER_Z="${MAPPING_SLICE_CENTER_Z:--0.756}"
SLICE_HALF_WIDTH="${MAPPING_SLICE_HALF_WIDTH:-0.10}"
SLICE_MIN_FRAMES="${MAPPING_SLICE_MIN_FRAME_OBSERVATIONS:-20}"
BASE_OFFSET_X="${MAPPING_BASE_OFFSET_X:--0.211}"
BASE_OFFSET_Y="${MAPPING_BASE_OFFSET_Y:--0.02329}"

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
SLICE_OBSERVATIONS="$MAP_3D_DIR/slice_observations.yaml"
[[ "$PUBLISH_LATEST" == true || "$PUBLISH_LATEST" == false ]] || {
  echo "PUBLISH_LATEST must be true or false." >&2
  exit 2
}
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
  _input_topic:=/cloud_registered _odom_topic:=/Odometry \
  _output_file:="$MAP_3D_DIR/map.pcd" \
  _slice_observations_file:="$SLICE_OBSERVATIONS" \
  _voxel_size:="$VOXEL_SIZE" _min_frame_observations:="$VOXEL_MIN_FRAMES" \
  _max_point_range:="$POINT_MAX_RANGE" \
  _slice_center_z:="$SLICE_CENTER_Z" _slice_half_width:="$SLICE_HALF_WIDTH" \
  _slice_resolution:="$GRID_RESOLUTION" \
  _slice_min_frame_observations:="$SLICE_MIN_FRAMES" \
  >"$OUTPUT_DIR/voxel.log" 2>&1 &
pointcloud_pid=$!
rosrun robot_bringup fused_scan_mapper.py --ros \
  --output-dir "$MAP_2D_DIR" --scan-topic /dual_lidar/scan \
  --odom-topic /Odometry --resolution "$GRID_RESOLUTION" \
  --base-offset-x "$BASE_OFFSET_X" --base-offset-y "$BASE_OFFSET_Y" \
  --min-occupied-observations "$LIDAR_MIN_FRAMES" \
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

for required in "$MAP_3D_DIR/map.pcd" "$SLICE_OBSERVATIONS" \
                "$MAP_2D_DIR/map.pgm" "$MAP_2D_DIR/map.yaml"; do
  [[ -s "$required" ]] || { echo "Offline mapper did not produce $required" >&2; exit 5; }
done
rosrun robot_bringup map_set_fuser.py \
  --map-2d "$MAP_2D_DIR/map.yaml" --map-3d "$MAP_3D_DIR/map.pcd" \
  --slice-observations "$SLICE_OBSERVATIONS" \
  --output-dir "$MAP_FUSED_DIR" --slice-center-z "$SLICE_CENTER_Z" \
  --slice-half-width "$SLICE_HALF_WIDTH" --resolution "$GRID_RESOLUTION"
cp -- "$MAP_2D_DIR/mapping_info.yaml" "$MAP_2D_DIR/config.yaml"
printf 'status: complete\ninput_topic: /cloud_registered\nvoxel_size_m: %s\nmin_frame_observations: %s\nmax_point_range_m: %s\n' \
  "$VOXEL_SIZE" "$VOXEL_MIN_FRAMES" "$POINT_MAX_RANGE" >"$MAP_3D_DIR/config.yaml"
cat >"$OUTPUT_DIR/manifest.yaml" <<EOF
schema_version: 1
map_id: "$map_name"
status: "complete"
frame_id: map
source_bag: "$BAG_PATH"
map_3d:
  pcd: map_3d/map.pcd
  min_frame_observations: $VOXEL_MIN_FRAMES
  max_point_range_m: $POINT_MAX_RANGE
map_2d:
  yaml: map_2d/map.yaml
  occupancy_source: dual_ld19_only
  min_occupied_observations: $LIDAR_MIN_FRAMES
map_fused_2d:
  yaml: map_fused_2d/map.yaml
  fusion_policy: persistent_occupied_union
  slice_center_z_m: $SLICE_CENTER_Z
  slice_half_width_m: $SLICE_HALF_WIDTH
  min_frame_observations: $SLICE_MIN_FRAMES
default_static_map_source: fused
initial_body_z_m: 0.0
EOF
if [[ "$PUBLISH_LATEST" == true ]]; then
  latest_temporary="$MAP_ROOT/.latest.$$"
  ln -s "$map_name" "$latest_temporary"
  mv -Tf -- "$latest_temporary" "$MAP_ROOT/latest"
fi
echo "STATIC_MAPPING_COMPLETE=$OUTPUT_DIR"

trap - EXIT INT TERM
cleanup
