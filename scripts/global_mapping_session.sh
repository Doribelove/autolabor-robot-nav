#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

MAP_ROOT="${STATIC_MAP_OUTPUT_ROOT:-$ROBOT_WS/global_maps/map_sets}"
WAIT_SECONDS="${MAPPING_TOPIC_WAIT_SEC:-15}"
VOXEL_SIZE="${MAPPING_3D_VOXEL_SIZE:-0.10}"
VOXEL_MIN_FRAMES="${MAPPING_3D_MIN_FRAME_OBSERVATIONS:-3}"
POINT_MAX_RANGE="${MAPPING_3D_MAX_POINT_RANGE:-20.0}"
GRID_RESOLUTION="${MAPPING_2D_RESOLUTION:-0.10}"
LIDAR_MIN_FRAMES="${MAPPING_2D_MIN_OCCUPIED_OBSERVATIONS:-5}"
SLICE_CENTER_Z="${MAPPING_SLICE_CENTER_Z:--0.4}"
SLICE_HALF_WIDTH="${MAPPING_SLICE_HALF_WIDTH:-0.20}"
SLICE_MIN_FRAMES="${MAPPING_SLICE_MIN_FRAME_OBSERVATIONS:-20}"
BASE_OFFSET_X="${MAPPING_BASE_OFFSET_X:--0.211}"
BASE_OFFSET_Y="${MAPPING_BASE_OFFSET_Y:--0.02329}"
BASE_OFFSET_Z="${MAPPING_BASE_OFFSET_Z:--0.95588}"
SLICE_SELF_CROP_ENABLED="${MAPPING_SLICE_SELF_CROP_ENABLED:-true}"
SLICE_SELF_CROP_MIN_X="${MAPPING_SLICE_SELF_CROP_MIN_X:--0.75}"
SLICE_SELF_CROP_MAX_X="${MAPPING_SLICE_SELF_CROP_MAX_X:-0.75}"
SLICE_SELF_CROP_MIN_Y="${MAPPING_SLICE_SELF_CROP_MIN_Y:--0.50}"
SLICE_SELF_CROP_MAX_Y="${MAPPING_SLICE_SELF_CROP_MAX_Y:-0.50}"
SLICE_SWEEP_FRONT="${MAPPING_SLICE_SWEEP_FRONT:-0.62}"
SLICE_SWEEP_REAR="${MAPPING_SLICE_SWEEP_REAR:-0.62}"
SLICE_SWEEP_HALF_WIDTH="${MAPPING_SLICE_SWEEP_HALF_WIDTH:-0.45}"
SLICE_SWEEP_LINEAR_STEP="${MAPPING_SLICE_SWEEP_LINEAR_STEP:-0.05}"
SLICE_SWEEP_ANGULAR_STEP="${MAPPING_SLICE_SWEEP_ANGULAR_STEP:-0.03490658503988659}"

[[ "$SLICE_SELF_CROP_ENABLED" == true ]] || {
  echo "MAPPING_SLICE_SELF_CROP_ENABLED must remain true for fused-map generation." >&2
  exit 2
}
[[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]] || {
  echo "MAPPING_TOPIC_WAIT_SEC must be a non-negative integer." >&2
  exit 2
}

wait_for_message() {
  timeout "$WAIT_SECONDS" rostopic echo -n 1 "$1" >/dev/null 2>&1
}

for topic in /Odometry /cloud_registered /dual_lidar/scan; do
  wait_for_message "$topic" || {
    echo "Required static-mapping topic is unavailable: $topic" >&2
    exit 3
  }
done
dual_state="$(timeout "$WAIT_SECONDS" rostopic echo -n 1 /avoidance/dual_lidar_active 2>/dev/null || true)"
grep -Eq '^data: (True|true)$' <<<"$dual_state" || {
  echo "Both fixed-port LD19 sensors must be active before static mapping." >&2
  exit 3
}

stamp="$(date +%Y%m%d_%H%M%S)"
map_id="map_${stamp}"
output_dir="$MAP_ROOT/$map_id"
map_3d_dir="$output_dir/map_3d"
map_2d_dir="$output_dir/map_2d"
map_fused_dir="$output_dir/map_fused_2d"
slice_observations="$map_3d_dir/slice_observations.yaml"
mkdir -p "$map_3d_dir" "$map_2d_dir" "$map_fused_dir"

pointcloud_pid=""
grid_pid=""
finalizing=false

write_manifest() {
  local status="$1"
  local temporary="$output_dir/manifest.yaml.tmp"
  {
    printf 'schema_version: 1\n'
    printf 'map_id: "%s"\n' "$map_id"
    printf 'status: "%s"\n' "$status"
    printf 'created_at: "%s"\n' "$(date --iso-8601=seconds)"
    printf 'frame_id: map\n'
    printf 'pose_source: fast_lio_odometry\n'
    printf 'map_3d:\n  pcd: map_3d/map.pcd\n  voxel_size_m: %s\n' "$VOXEL_SIZE"
    printf '  min_frame_observations: %s\n  max_point_range_m: %s\n' "$VOXEL_MIN_FRAMES" "$POINT_MAX_RANGE"
    printf 'map_2d:\n  yaml: map_2d/map.yaml\n  occupancy_source: dual_ld19_only\n  resolution_m: %s\n' "$GRID_RESOLUTION"
    printf '  min_occupied_observations: %s\n' "$LIDAR_MIN_FRAMES"
    printf 'map_fused_2d:\n  yaml: map_fused_2d/map.yaml\n  fusion_policy: persistent_occupied_union\n'
    printf '  slice_center_z_m: %s\n  slice_half_width_m: %s\n' "$SLICE_CENTER_Z" "$SLICE_HALF_WIDTH"
    printf '  min_frame_observations: %s\n' "$SLICE_MIN_FRAMES"
    printf '  moving_self_crop:\n    enabled: true\n'
    printf '    point_bounds_xy_m: [%s, %s, %s, %s]\n' "$SLICE_SELF_CROP_MIN_X" "$SLICE_SELF_CROP_MAX_X" "$SLICE_SELF_CROP_MIN_Y" "$SLICE_SELF_CROP_MAX_Y"
    printf '    sweep_bounds_xy_m: [-%s, %s, -%s, %s]\n' "$SLICE_SWEEP_REAR" "$SLICE_SWEEP_FRONT" "$SLICE_SWEEP_HALF_WIDTH" "$SLICE_SWEEP_HALF_WIDTH"
    printf '    body_to_base_xyz_m: [%s, %s, %s]\n' "$BASE_OFFSET_X" "$BASE_OFFSET_Y" "$BASE_OFFSET_Z"
    printf 'default_static_map_source: fused\n'
    printf 'initial_body_z_m: 0.0\n'
  } >"$temporary"
  mv -f -- "$temporary" "$output_dir/manifest.yaml"
}

write_manifest recording

stop_mapper() {
  local pid="$1"
  [[ -z "$pid" ]] || kill -INT "$pid" 2>/dev/null || true
}

finish_session() {
  local reason_status="$1"
  local pointcloud_status=0 grid_status=0 required
  [[ "$finalizing" == false ]] || return 0
  finalizing=true
  trap - EXIT INT TERM HUP
  echo "Finalizing three-map static mapping session..."
  set +e
  stop_mapper "$pointcloud_pid"
  stop_mapper "$grid_pid"
  [[ -z "$pointcloud_pid" ]] || wait "$pointcloud_pid"
  pointcloud_status=$?
  [[ -z "$grid_pid" ]] || wait "$grid_pid"
  grid_status=$?
  set -e

  if [[ "$reason_status" -ne 0 || "$pointcloud_status" -ne 0 || "$grid_status" -ne 0 ]]; then
    write_manifest failed
    echo "Static mapping processes failed; partial data retained at $output_dir" >&2
    exit 5
  fi
  for required in "$map_3d_dir/map.pcd" "$slice_observations" "$map_2d_dir/map.pgm" \
                  "$map_2d_dir/map.yaml" "$map_2d_dir/mapping_info.yaml"; do
    if [[ ! -s "$required" ]]; then
      write_manifest failed
      echo "Static map save failed: missing $required" >&2
      exit 5
    fi
  done

  if ! rosrun robot_bringup map_set_fuser.py \
      --map-2d "$map_2d_dir/map.yaml" \
      --map-3d "$map_3d_dir/map.pcd" \
      --slice-observations "$slice_observations" \
      --output-dir "$map_fused_dir" \
      --slice-center-z "$SLICE_CENTER_Z" \
      --slice-half-width "$SLICE_HALF_WIDTH" \
      --resolution "$GRID_RESOLUTION"; then
    write_manifest failed
    echo "3-D slice fusion failed; partial data retained at $output_dir" >&2
    exit 6
  fi

  {
    printf 'status: complete\n'
    printf 'input_topic: /cloud_registered\n'
    printf 'frame_id: map\n'
    printf 'voxel_size_m: %s\n' "$VOXEL_SIZE"
    printf 'min_frame_observations: %s\n' "$VOXEL_MIN_FRAMES"
    printf 'max_point_range_m: %s\n' "$POINT_MAX_RANGE"
    printf 'moving_slice_self_crop: true\n'
  } >"$map_3d_dir/config.yaml"
  cp -- "$map_2d_dir/mapping_info.yaml" "$map_2d_dir/config.yaml"
  write_manifest complete
  latest_temporary="$MAP_ROOT/.latest.$$"
  ln -s "$map_id" "$latest_temporary"
  mv -Tf -- "$latest_temporary" "$MAP_ROOT/latest"
  echo "STATIC_MAPPING_COMPLETE=$output_dir"
  echo "STATIC_MAPPING_LATEST=$MAP_ROOT/latest"
  exit 0
}

trap 'finish_session 0' INT TERM
trap 'finish_session 1' HUP EXIT

echo "Starting MID360 3-D voxel mapping and dual-LD19-only 2-D grid mapping."
echo "STATIC_MAPPING_DIRECTORY=$output_dir"
rosrun robot_bringup voxel_cloud_mapper \
  _input_topic:=/cloud_registered \
  _odom_topic:=/Odometry \
  _output_file:="$map_3d_dir/map.pcd" \
  _slice_observations_file:="$slice_observations" \
  _voxel_size:="$VOXEL_SIZE" \
  _min_frame_observations:="$VOXEL_MIN_FRAMES" \
  _max_point_range:="$POINT_MAX_RANGE" \
  _slice_center_z:="$SLICE_CENTER_Z" \
  _slice_half_width:="$SLICE_HALF_WIDTH" \
  _slice_resolution:="$GRID_RESOLUTION" \
  _slice_min_frame_observations:="$SLICE_MIN_FRAMES" \
  _slice_self_crop_enabled:="$SLICE_SELF_CROP_ENABLED" \
  _slice_self_crop_min_x:="$SLICE_SELF_CROP_MIN_X" \
  _slice_self_crop_max_x:="$SLICE_SELF_CROP_MAX_X" \
  _slice_self_crop_min_y:="$SLICE_SELF_CROP_MIN_Y" \
  _slice_self_crop_max_y:="$SLICE_SELF_CROP_MAX_Y" \
  _slice_sweep_front:="$SLICE_SWEEP_FRONT" \
  _slice_sweep_rear:="$SLICE_SWEEP_REAR" \
  _slice_sweep_half_width:="$SLICE_SWEEP_HALF_WIDTH" \
  _body_to_base_x:="$BASE_OFFSET_X" _body_to_base_y:="$BASE_OFFSET_Y" \
  _body_to_base_z:="$BASE_OFFSET_Z" \
  _slice_sweep_linear_step:="$SLICE_SWEEP_LINEAR_STEP" \
  _slice_sweep_angular_step:="$SLICE_SWEEP_ANGULAR_STEP" &
pointcloud_pid=$!

rosrun robot_bringup fused_scan_mapper.py \
  --ros \
  --output-dir "$map_2d_dir" \
  --scan-topic /dual_lidar/scan \
  --odom-topic /Odometry \
  --resolution "$GRID_RESOLUTION" \
  --base-offset-x "$BASE_OFFSET_X" \
  --base-offset-y "$BASE_OFFSET_Y" \
  --min-occupied-observations "$LIDAR_MIN_FRAMES" &
grid_pid=$!

while kill -0 "$pointcloud_pid" 2>/dev/null && kill -0 "$grid_pid" 2>/dev/null; do
  sleep 0.5
done
echo "A static mapping process exited unexpectedly." >&2
finish_session 1
