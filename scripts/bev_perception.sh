#!/usr/bin/env bash
# Three-camera capture + CUDA detection without opening a second Qt/master.
set -euo pipefail
bev_project=$(cd -- "$(dirname -- "$0")/.." && pwd)
bev_unit=autolabor-conbine-bev-yolo.service
case "${1:-status}" in
  start)
    bash "$bev_project/scripts/camera_bev.sh" start
    if curl --fail --silent --max-time 2 http://127.0.0.1:18884/state >/dev/null; then
      exit 0
    fi
    if systemctl --user is-active --quiet "$bev_unit"; then exit 0; fi
    mkdir -p "$bev_project/runtime/bev_cleaning/yolo-config"
    systemd-run --user --unit="$bev_unit" --collect --working-directory="$bev_project" \
      --setenv="YOLO_CONFIG_DIR=$bev_project/runtime/bev_cleaning/yolo-config" \
      --setenv=OMP_NUM_THREADS=2 --setenv=OPENBLAS_NUM_THREADS=2 \
      --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$bev_project/runtime/bev_cleaning/yolo.log" \
      --property="StandardError=append:$bev_project/runtime/bev_cleaning/yolo.log" \
      /home/slam/robot_ws/.venv/fod_yolo/bin/python3 -s "$bev_project/scripts/bev_yolo_server.py" \
      --output "$bev_project/runtime/bev_cleaning"
    ;;
  stop)
    if systemctl --user is-active --quiet "$bev_unit"; then systemctl --user stop "$bev_unit"; fi
    ;;
  status)
    systemctl --user show "$bev_unit" -p ActiveState -p MainPID
    curl --fail --silent --max-time 2 http://127.0.0.1:18884/state || true
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
