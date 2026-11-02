#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
preview_root="${CAMERA_VIEW_ASSETS:-$project_root/runtime/four_camera_view_20261101}"
board_root="${CAMERA_BEV_BOARD_ROOT:-/map/conbine_camera/surround_recalibrated_20261101}"
unit=autolabor-conbine-four-camera-view.service
case "${1:-start}" in
  start)
    if systemctl --user is-active --quiet "$unit"; then
      echo '四路相机画面已经打开'; exit 0
    fi
    mkdir -p "$preview_root"
    preview_args=()
    if [[ "${CAMERA_PREVIEW_HEADLESS:-0}" == 1 ]]; then preview_args+=(--headless); fi
    if [[ ! -f "$preview_root/libzed_preview.so" || "$project_root/scripts/zed_preview_native.cpp" -nt "$preview_root/libzed_preview.so" ]]; then
      g++ -std=c++14 -O2 -shared -fPIC -Wall -Wextra \
        -I/usr/local/zed/include -I/usr/local/cuda/include \
        "$project_root/scripts/zed_preview_native.cpp" -o "$preview_root/libzed_preview.so" \
        -L/usr/local/zed/lib -Wl,-rpath,/usr/local/zed/lib -lsl_zed
    fi
    systemd-run --user --unit="$unit" --collect --working-directory="$project_root" \
      --setenv="DISPLAY=${DISPLAY:-:0}" --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      --setenv=PYTHONUTF8=1 --setenv="CAMERA_VIEW_ASSETS=$preview_root" \
      --setenv="CAMERA_BEV_BOARD_ROOT=$board_root" \
      --property=KillMode=mixed --property=TimeoutStopSec=45 \
      --property="StandardOutput=append:$preview_root/viewer.log" \
      --property="StandardError=append:$preview_root/viewer.log" \
      /usr/bin/python3 -B -s "$project_root/scripts/four_camera_view.py" "${preview_args[@]}"
    ;;
  stop)
    if systemctl --user is-active --quiet "$unit"; then systemctl --user stop "$unit"; fi
    ;;
  status)
    systemctl --user show "$unit" -p ActiveState -p MainPID
    if [[ -f "$preview_root/state.json" ]]; then cat "$preview_root/state.json"; fi
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
