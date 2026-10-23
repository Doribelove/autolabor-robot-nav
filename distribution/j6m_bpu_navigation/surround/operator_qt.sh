#!/usr/bin/env bash
set -euo pipefail
surround_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$surround_dir/operator_env.sh"
case "${1:-status}" in
  build)
    mkdir -p "$OPERATOR_RUNTIME/build" "$ROS_HOME" "$ROS_LOG_DIR"
    cd "$OPERATOR_WS"
    exec catkin_make --build "$OPERATOR_RUNTIME/build" \
      -DCATKIN_DEVEL_PREFIX="$OPERATOR_RUNTIME/devel" \
      -DCMAKE_INSTALL_PREFIX="$OPERATOR_RUNTIME/install" \
      -DCATKIN_WHITELIST_PACKAGES=autolabor_operator_gui \
      -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -j2 -l2
    ;;
  run-gui)
    export ROS_MASTER_URI=http://127.0.0.1:11342 ROS_IP=127.0.0.1
    export SURROUND_PREVIEW_URL=http://127.0.0.1:18781
    export XDG_CONFIG_HOME="$OPERATOR_RUNTIME/qt-config"
    mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" "$XDG_CONFIG_HOME"
    exec roslaunch -p 11342 autolabor_operator_gui operator_gui.launch \
      initial_vision_page:=true \
      configured_vision_backend:=j6m_surround_bpu \
      vision_backend_switch_script:=''
    ;;
  start|stop|status)
    exec python3 -B "$surround_dir/operator_session.py" "$1"
    ;;
  *) echo "Usage: $0 {build|start|stop|status}" >&2; exit 2 ;;
esac
