#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

MAP_ROOT="${STATIC_MAP_OUTPUT_ROOT:-$ROBOT_WS/global_maps/map_sets}"
WAIT_SECONDS="${MAPPING_TOPIC_WAIT_SEC:-15}"
VOXEL_SIZE="${MAPPING_3D_VOXEL_SIZE:-0.10}"
GRID_RESOLUTION="${MAPPING_2D_RESOLUTION:-0.10}"
SLICE_CENTER_Z="${MAPPING_SLICE_CENTER_Z:--0.42}"
SLICE_HALF_WIDTH="${MAPPING_SLICE_HALF_WIDTH:-0.10}"
SLICE_MIN_POINTS="${MAPPING_SLICE_MIN_POINTS_PER_CELL:-2}"
BASE_OFFSET_X="${MAPPING_BASE_OFFSET_X:--0.20}"
BASE_OFFSET_Y="${MAPPING_BASE_OFFSET_Y:-0.0}"

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
    printf 'map_2d:\n  yaml: map_2d/map.yaml\n  occupancy_source: dual_ld19_only\n  resolution_m: %s\n' "$GRID_RESOLUTION"
    printf 'map_fused_2d:\n  yaml: map_fused_2d/map.yaml\n  fusion_policy: occupied_union\n'
    printf '  slice_center_z_m: %s\n  slice_half_width_m: %s\n' "$SLICE_CENTER_Z" "$SLICE_HALF_WIDTH"
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
  for required in "$map_3d_dir/map.pcd" "$map_2d_dir/map.pgm" \
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
      --output-dir "$map_fused_dir" \
      --slice-center-z "$SLICE_CENTER_Z" \
      --slice-half-width "$SLICE_HALF_WIDTH" \
      --min-points-per-cell "$SLICE_MIN_POINTS" \
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
  _output_file:="$map_3d_dir/map.pcd" \
  _voxel_size:="$VOXEL_SIZE" &
pointcloud_pid=$!

rosrun robot_bringup fused_scan_mapper.py \
  --ros \
  --output-dir "$map_2d_dir" \
  --scan-topic /dual_lidar/scan \
  --odom-topic /Odometry \
  --resolution "$GRID_RESOLUTION" \
  --base-offset-x "$BASE_OFFSET_X" \
  --base-offset-y "$BASE_OFFSET_Y" &
grid_pid=$!

while kill -0 "$pointcloud_pid" 2>/dev/null && kill -0 "$grid_pid" 2>/dev/null; do
  sleep 0.5
done
echo "A static mapping process exited unexpectedly." >&2
finish_session 1
