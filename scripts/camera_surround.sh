#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
export CAMERA_BEV_CONFIG="${CAMERA_BEV_CONFIG:-$project_root/config/four_camera_bev.json}"
export CAMERA_BEV_ASSETS="${CAMERA_BEV_ASSETS:-$project_root/runtime/surround_recalibrated_20261101}"
export CAMERA_BEV_BOARD_ROOT="${CAMERA_BEV_BOARD_ROOT:-/map/conbine_camera/surround_recalibrated_20261101}"
# Same camera/Qt lifecycle as the existing BEV entry; no navigation is started.
if [[ "${1:-start}" == check ]]; then
  exec bash "$project_root/scripts/camera_bev.sh" check
fi
exec bash "$project_root/scripts/camera_bev_qt.sh" "${1:-start}"
