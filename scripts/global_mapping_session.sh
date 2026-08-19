#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

MAP_ROOT="${STATIC_MAP_OUTPUT_ROOT:-$ROBOT_WS/global_maps/static_maps}"
SESSION_BAG_DIR="${BAG_DIR:-$ROBOT_WS/rosbags}"
REQUIRE_DUAL="${MAPPING_REQUIRE_DUAL_LIDAR:-true}"
WAIT_SECONDS="${MAPPING_TOPIC_WAIT_SEC:-10}"

case "$REQUIRE_DUAL" in true|false) ;; *)
  echo "MAPPING_REQUIRE_DUAL_LIDAR must be true or false." >&2
  exit 2
esac
[[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]] || {
  echo "MAPPING_TOPIC_WAIT_SEC must be a non-negative integer." >&2
  exit 2
}

wait_for_message() {
  local topic="$1"
  timeout "$WAIT_SECONDS" rostopic echo -n 1 "$topic" >/dev/null 2>&1
}

wait_for_message /Odometry || {
  echo "FAST-LIO /Odometry is unavailable; global mapping was not started." >&2
  exit 3
}
wait_for_message /scan || {
  echo "Fused /scan is unavailable; global mapping was not started." >&2
  exit 3
}
wait_for_message /mid360/scan || {
  echo "MID360 /mid360/scan is unavailable; global mapping was not started." >&2
  exit 3
}
if [[ "$REQUIRE_DUAL" == true ]]; then
  wait_for_message /dual_lidar/scan || {
    echo "Dual LD19 /dual_lidar/scan is unavailable; refusing a MID360-only map." >&2
    exit 3
  }
  dual_state="$(timeout "$WAIT_SECONDS" rostopic echo -n 1 /avoidance/dual_lidar_active 2>/dev/null || true)"
  grep -Eq '^data: (True|true)$' <<<"$dual_state" || {
    echo "Scan fusion reports dual_lidar_active=false; refusing a MID360-only map." >&2
    exit 3
  }
fi

stamp="$(date +%Y%m%d_%H%M%S)"
map_name="fused_${stamp}"
output_dir="$MAP_ROOT/$map_name"
mkdir -p "$output_dir"

recorder_pid=""
mapper_pid=""
shutdown_started=false

finish_session() {
  local requested_status="$1" recorder_status=0 mapper_status=0 required
  [[ "$shutdown_started" == false ]] || return 0
  shutdown_started=true
  trap - EXIT INT TERM HUP
  set +e
  [[ -z "$recorder_pid" ]] || kill -INT "$recorder_pid" 2>/dev/null
  [[ -z "$mapper_pid" ]] || kill -INT "$mapper_pid" 2>/dev/null
  [[ -z "$recorder_pid" ]] || wait "$recorder_pid"
  recorder_status=$?
  [[ -z "$mapper_pid" ]] || wait "$mapper_pid"
  mapper_status=$?
  set -e

  for required in map.pgm map.yaml mapping_info.yaml; do
    if [[ ! -s "$output_dir/$required" ]]; then
      echo "Global map save failed: missing $output_dir/$required" >&2
      echo "Partial session retained at $output_dir" >&2
      exit 5
    fi
  done

  printf '%s\n' \
    'status: complete' \
    'mode: live_fused_scan' \
    "requires_dual_lidar: $REQUIRE_DUAL" \
    'component_scan_topics:' \
    '  - /mid360/scan' \
    '  - /dual_lidar/scan' \
    'fused_scan_topic: /scan' \
    'fast_lio_odometry_topic: /Odometry' \
    "rosbag_directory: $SESSION_BAG_DIR" \
    "recorder_exit_status: $recorder_status" \
    "mapper_exit_status: $mapper_status" \
    >"$output_dir/session_info.yaml.tmp"
  mv -f -- "$output_dir/session_info.yaml.tmp" "$output_dir/session_info.yaml"

  latest_temporary="$MAP_ROOT/.latest.$$"
  ln -s "$map_name" "$latest_temporary"
  mv -Tf -- "$latest_temporary" "$MAP_ROOT/latest"
  echo "GLOBAL_MAPPING_COMPLETE=$output_dir/map.yaml"
  echo "GLOBAL_MAPPING_LATEST=$MAP_ROOT/latest/map.yaml"
  exit "$requested_status"
}

trap 'finish_session 0' INT TERM
trap 'finish_session 1' HUP
trap 'finish_session 1' EXIT

echo "Starting synchronized rosbag recording and fused 2-D global mapping."
echo "Map output: $output_dir"
BAG_PREFIX="fused_mapping_${stamp}" "$SCRIPT_DIR/record_rosbag.sh" mode1 &
recorder_pid=$!
rosrun robot_bringup fused_scan_mapper.py \
  --ros \
  --output-dir "$output_dir" &
mapper_pid=$!

while kill -0 "$recorder_pid" 2>/dev/null && kill -0 "$mapper_pid" 2>/dev/null; do
  sleep 0.5
done
echo "Mapping or recording process exited unexpectedly; finalizing available data." >&2
finish_session 1
