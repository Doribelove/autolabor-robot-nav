#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: scripts/build_static_map_from_bag.sh BAG [MAP_NAME]

Build a standard map_server PGM/YAML map from the bag's fused /scan and
FAST-LIO /Odometry streams. The result is written below:
  <workspace>/global_maps/static_maps/<MAP_NAME>/

On success, global_maps/static_maps/latest is atomically updated to this map.
EOF
}

if (( $# < 1 || $# > 2 )); then
  usage >&2
  exit 2
fi

BAG_PATH="$(readlink -f -- "$1")"
[[ -r "$BAG_PATH" ]] || { echo "Bag is not readable: $1" >&2; exit 2; }

if (( $# == 2 )); then
  MAP_NAME="$2"
else
  bag_stem="$(basename "$BAG_PATH")"
  MAP_NAME="bag_${bag_stem%.bag}_$(date +%Y%m%d_%H%M%S)"
fi
[[ "$MAP_NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "MAP_NAME may contain only letters, digits, dot, underscore and dash." >&2
  exit 2
}

source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

MAP_ROOT="$ROBOT_WS/global_maps/static_maps"
OUTPUT_DIR="$MAP_ROOT/$MAP_NAME"
[[ ! -e "$OUTPUT_DIR" ]] || {
  echo "Map output already exists; choose another MAP_NAME: $OUTPUT_DIR" >&2
  exit 3
}
mkdir -p "$OUTPUT_DIR"

set +e
rosrun robot_bringup fused_scan_mapper.py \
  --bag "$BAG_PATH" \
  --output-dir "$OUTPUT_DIR"
mapper_status=$?
set -e
if (( mapper_status != 0 )); then
  echo "Static map build failed; partial output retained at $OUTPUT_DIR" >&2
  exit "$mapper_status"
fi

for required in map.pgm map.yaml mapping_info.yaml; do
  [[ -s "$OUTPUT_DIR/$required" ]] || {
    echo "Mapper did not produce $required; latest was not changed." >&2
    exit 4
  }
done

latest_temporary="$MAP_ROOT/.latest.$$"
ln -s "$MAP_NAME" "$latest_temporary"
mv -Tf -- "$latest_temporary" "$MAP_ROOT/latest"
echo "STATIC_MAP_SAVED=$OUTPUT_DIR/map.yaml"
echo "STATIC_MAP_LATEST=$MAP_ROOT/latest/map.yaml"
