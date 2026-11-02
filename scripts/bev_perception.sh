#!/usr/bin/env bash
# Surround capture + original-view detection without a second Qt/master.
set -euo pipefail
bev_project=$(cd -- "$(dirname -- "$0")/.." && pwd)
bev_unit=autolabor-conbine-bev-yolo.service
# Cleaning uses the current four-camera rig. camera_bev.sh keeps its explicit
# three-camera default for the separate legacy entry.
export CAMERA_BEV_CONFIG="${CAMERA_BEV_CONFIG:-$bev_project/config/four_camera_bev.json}"
export CAMERA_BEV_ASSETS="${CAMERA_BEV_ASSETS:-$bev_project/runtime/surround_recalibrated_20261101}"
export CAMERA_BEV_BOARD_ROOT="${CAMERA_BEV_BOARD_ROOT:-/map/conbine_camera/surround_recalibrated_20261101}"
# Full navigation renders in Qt; capture and BEV stay in the background.
export CAMERA_PREVIEW_HEADLESS="${CAMERA_PREVIEW_HEADLESS:-1}"
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
      --setenv="BEV_DETECTOR_BACKEND=${BEV_DETECTOR_BACKEND:-cuda}" \
      --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$bev_project/runtime/bev_cleaning/yolo.log" \
      --property="StandardError=append:$bev_project/runtime/bev_cleaning/yolo.log" \
      /bin/bash "$bev_project/scripts/bev_perception.sh" run
    ;;
  run)
    # systemd does not inherit the Qt session's ROS Python dependencies.
    # The project YOLO confidence helper imports dynamic_reconfigure messages.
    set +u
    source /home/slam/robot_ws/.deps/setup.bash
    source "$bev_project/devel/setup.bash"
    set -u
    export PYTHONPATH="$bev_project/devel/lib/python3/dist-packages:/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/python3/dist-packages:/home/slam/robot_ws/.deps/sysroot/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
    exec /home/slam/robot_ws/.venv/fod_yolo/bin/python3 -s "$bev_project/scripts/bev_yolo_server.py" \
      --output "$bev_project/runtime/bev_cleaning"
    ;;
  stop)
    if systemctl --user is-active --quiet "$bev_unit"; then systemctl --user stop "$bev_unit"; fi
    ;;
  status)
    systemctl --user show "$bev_unit" -p ActiveState -p MainPID
    curl --fail --silent --max-time 2 http://127.0.0.1:18884/state || true
    ;;
  *) echo "Usage: $0 {start|stop|status|run}" >&2; exit 2 ;;
esac
