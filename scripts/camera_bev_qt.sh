#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
unit=autolabor-conbine-camera-bev-qt.service
case "${1:-start}" in
  start)
    bash "$project_root/scripts/camera_bev.sh" start
    if systemctl --user is-active --quiet "$unit"; then
      echo 'BEV / 识别 Qt 已运行'
      exit 0
    fi
    mkdir -p "$project_root/runtime/bev_qt"
    systemd-run --user --unit="$unit" --collect --working-directory="$project_root" \
      --setenv="DISPLAY=${DISPLAY:-:0}" --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      --setenv="BEV_DETECTOR_BACKEND=${BEV_DETECTOR_BACKEND:-cuda}" \
      --property=KillMode=mixed --property=TimeoutStopSec=40 \
      --property="StandardOutput=append:$project_root/runtime/bev_qt/session.log" \
      --property="StandardError=append:$project_root/runtime/bev_qt/session.log" \
      /bin/bash "$project_root/scripts/camera_bev_qt.sh" run
    ;;
  run)
    # Read-only dependency setup; current workspace files take precedence.
    set +u
    source /home/slam/robot_ws/.deps/setup.bash
    source "$project_root/devel/setup.bash"
    set -u
    export LD_LIBRARY_PATH="$project_root/devel/lib:/home/slam/robot_j6m_ws_navigation_20260907/devel/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$project_root/src/application/autolabor_fod_vision/src:$project_root/devel/lib/python3/dist-packages:/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/python3/dist-packages:/home/slam/robot_ws/.deps/sysroot/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
    export ROS_PACKAGE_PATH="$project_root/src:/home/slam/robot_j6m_ws_navigation_20260907/src:${ROS_PACKAGE_PATH:-}"
    unset QT_QPA_PLATFORM_PLUGIN_PATH
    exec /usr/bin/python3 -s "$project_root/scripts/bev_qt_session.py"
    ;;
  stop)
    if systemctl --user is-active --quiet "$unit"; then systemctl --user stop "$unit"; fi
    ;;
  status)
    systemctl --user show "$unit" -p ActiveState -p MainPID
    curl --fail --silent --max-time 3 http://127.0.0.1:18884/state
    echo
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
