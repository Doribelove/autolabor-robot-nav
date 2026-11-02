#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "$0")/.." && pwd)
unit=autolabor-conbine-heal-bev.service
output="$project/runtime/heal_bpu_live_20261101"
python=/home/slam/robot_ws/.venv/fod_yolo/bin/python3
case "${1:-start}" in
  start)
    if systemctl --user is-active --quiet "$unit"; then
      systemctl --user show "$unit" -p ActiveState -p MainPID
      exit 0
    fi
    # Reuse the existing capture/BEV packet server. Camera startup stays with its
    # usual entry; this experiment never starts navigation or a cleaning action.
    curl --fail --silent --max-time 5 http://127.0.0.1:18883/views -o /dev/null
    mkdir -p "$output"
    systemd-run --user --unit="$unit" --collect --working-directory="$project" \
      --setenv="DISPLAY=${DISPLAY:-:0}" \
      --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      --property=PartOf=autolabor-conbine-camera-bev.service \
      --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$output/preview.log" \
      --property="StandardError=append:$output/preview.log" \
      "$python" -s "$project/tools/bev_feature/heal_preview.py" \
      --deploy --output "$output" --frames 0 --display
    ;;
  stop)
    if systemctl --user is-active --quiet "$unit"; then
      systemctl --user stop "$unit"
    fi
    ;;
  status)
    systemctl --user show "$unit" -p ActiveState -p MainPID
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
