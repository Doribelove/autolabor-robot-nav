#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"

source_link="${STATIC_MAP_SET:-${STATIC_MAP_SOURCE:-$ROBOT_WS/global_maps/map_sets/latest}}"
source_dir="$(readlink -f -- "$source_link" 2>/dev/null || true)"
map_root="$(readlink -f -- "$ROBOT_WS/global_maps/map_sets" 2>/dev/null || true)"
[[ -n "$source_dir" && -d "$source_dir" && -n "$map_root" &&
   "$source_dir" == "$map_root/"* ]] || {
  echo "No valid project map set is selected at: $source_link" >&2
  exit 2
}
grep -Eq '^status:[[:space:]]*"?complete"?[[:space:]]*$' \
  "$source_dir/manifest.yaml" 2>/dev/null || {
  echo "Map set manifest is not complete: $source_dir/manifest.yaml" >&2
  exit 2
}
required_files=(
  manifest.yaml
  map_3d/map.pcd
  map_3d/config.yaml
  map_2d/map.pgm
  map_2d/map.yaml
  map_2d/config.yaml
  map_fused_2d/map.pgm
  map_fused_2d/map.yaml
  map_fused_2d/config.yaml
)
for required in "${required_files[@]}"; do
  [[ -s "$source_dir/$required" ]] || {
    echo "Map set is incomplete: $source_dir/$required" >&2
    exit 2
  }
done
for map_yaml in "$source_dir/map_2d/map.yaml" "$source_dir/map_fused_2d/map.yaml"; do
  grep -Eq '^image:[[:space:]]+map\.pgm[[:space:]]*$' "$map_yaml" || {
    echo "Map YAML must reference its self-contained map.pgm: $map_yaml" >&2
    exit 2
  }
done

source_name="$(basename "$source_dir")"
[[ "$source_name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || {
  echo "Unsafe map set directory name: $source_name" >&2
  exit 2
}
map_hash="$(sha256sum "${required_files[@]/#/$source_dir/}" | sha256sum | cut -c1-12)"
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
rsync -a --delete --chmod=D0755,F0644 "$source_dir/" "$target:$remote_release/"
ssh "$target" "set -eu
  test -s '$remote_release/map_3d/map.pcd'
  test -s '$remote_release/map_fused_2d/map.yaml'
  temporary='$remote_root/.current.$release_id'
  ln -sfn 'releases/$release_id' \"\$temporary\"
  mv -Tf -- \"\$temporary\" '$remote_root/current'"

echo "STATIC_MAP_SET_SYNCED=$target:$remote_release"
echo "STATIC_MAP_3D_RUNTIME=/var/lib/autolabor/maps/current/map_3d/map.pcd"
echo "STATIC_MAP_FUSED_RUNTIME=/var/lib/autolabor/maps/current/map_fused_2d/map.yaml"
