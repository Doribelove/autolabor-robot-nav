#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"

source_link="${STATIC_MAP_SOURCE:-$ROBOT_WS/global_maps/static_maps/latest}"
source_dir="$(readlink -f -- "$source_link" 2>/dev/null || true)"
map_root="$(readlink -f -- "$ROBOT_WS/global_maps/static_maps")"
[[ -n "$source_dir" && -d "$source_dir" && "$source_dir" == "$map_root/"* ]] || {
  echo "No valid project static map is selected at: $source_link" >&2
  echo "Build one from Qt or scripts/build_static_map_from_bag.sh first." >&2
  exit 2
}
for required in map.pgm map.yaml mapping_info.yaml; do
  [[ -s "$source_dir/$required" ]] || {
    echo "Static map is incomplete: $source_dir/$required" >&2
    exit 2
  }
done
grep -Eq '^image:[[:space:]]+map\.pgm[[:space:]]*$' "$source_dir/map.yaml" || {
  echo "Only a self-contained map.yaml using image: map.pgm can be synchronized." >&2
  exit 2
}

source_name="$(basename "$source_dir")"
[[ "$source_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "Unsafe static map directory name: $source_name" >&2
  exit 2
}
map_hash="$(sha256sum "$source_dir/map.pgm" "$source_dir/map.yaml" | sha256sum | cut -c1-12)"
release_id="${source_name}_${map_hash}"
target="${1:-}"
if [[ -z "$target" ]]; then
  target="$(dual_host_select_ssh)" || {
    echo "J6M SSH is unavailable at both configured addresses." >&2
    exit 3
  }
fi

remote_root="$J6M_RUNTIME_BASE/maps"
remote_release="$remote_root/releases/$release_id"
ssh "$target" "set -eu; mkdir -p '$remote_release'"
files=("$source_dir/map.pgm" "$source_dir/map.yaml" "$source_dir/mapping_info.yaml")
[[ ! -s "$source_dir/session_info.yaml" ]] || files+=("$source_dir/session_info.yaml")
rsync -a --chmod=F0644 "${files[@]}" "$target:$remote_release/"
ssh "$target" "set -eu
  test -s '$remote_release/map.pgm'
  test -s '$remote_release/map.yaml'
  temporary='$remote_root/.current.$release_id'
  ln -sfn 'releases/$release_id' \"\$temporary\"
  mv -Tf -- \"\$temporary\" '$remote_root/current'"

echo "STATIC_MAP_SYNCED=$target:$remote_release/map.yaml"
echo "STATIC_MAP_RUNTIME=/var/lib/autolabor/maps/current/map.yaml"
