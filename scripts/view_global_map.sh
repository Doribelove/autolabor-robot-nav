#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
MAP_ROOT="$ROBOT_WS/global_maps"
requested="${1:-latest}"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/view_global_map.sh [map-name|map-directory|pcd-file]

With no argument, the most recently completed map is opened.
EOF
}

if [[ "$requested" == "-h" || "$requested" == "--help" || "$requested" == "help" ]]; then
  usage
  exit 0
fi
(( $# <= 1 )) || {
  usage >&2
  exit 2
}

if [[ -f "$requested" ]]; then
  map_file="$requested"
elif [[ -d "$requested" ]]; then
  map_file="$requested/global_map.pcd"
elif [[ -f "$MAP_ROOT/$requested/global_map.pcd" ]]; then
  map_file="$MAP_ROOT/$requested/global_map.pcd"
else
  echo "Map not found: $requested" >&2
  echo "Available maps:" >&2
  find "$MAP_ROOT" -mindepth 2 -maxdepth 2 -type f -name global_map.pcd \
    -printf '  %h\n' | sort >&2
  exit 1
fi

map_file="$(realpath -e "$map_file")"
command -v pcl_viewer >/dev/null 2>&1 || {
  echo "pcl_viewer is not installed." >&2
  exit 2
}
[[ -n "${DISPLAY:-}" ]] || {
  echo "DISPLAY is not set; run this command in the robot desktop terminal." >&2
  exit 3
}

echo "Opening: $map_file"
exec pcl_viewer -bc 0.05,0.05,0.05 -ps 1 -ax 1 "$map_file"

