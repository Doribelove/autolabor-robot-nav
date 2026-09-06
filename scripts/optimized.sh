#!/usr/bin/env bash
# All candidate commands run with the original workspace and dependencies read-only.
set -euo pipefail
candidate_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "$candidate_ws" != /home/slam/robot_j6m_ws_optimized_20260905 ]]; then
  echo "Unexpected candidate directory; audit paths before relocating this version." >&2
  exit 2
fi
command -v bwrap >/dev/null || { echo 'bubblewrap is required for isolation.' >&2; exit 2; }
mkdir -p "$candidate_ws/runtime/isolated"/{config,cache,data,ros-home,tmp,cuda} "$candidate_ws/log/ros"
case "${1:-}" in
  build) shift; set -- "$candidate_ws/scripts/build_workspace.sh" "$@" ;;
  deploy) shift; set -- "$candidate_ws/scripts/deploy_j6m.sh" "$@" ;;
  start|stop|status|restart)
    operation="$1"; shift
    if [[ "$operation" == start || "$operation" == restart ]]; then
      original_state="$(systemctl --user show autolabor-dual-host.service -p ActiveState --value 2>/dev/null || true)"
      case "$original_state" in
        active|activating|deactivating)
          echo 'Original supervisor is active. Stop it using its own documented stop command first.' >&2
          exit 3 ;;
      esac
      # Inspect original PID records before the candidate launcher may attempt
      # remote cleanup. Never signal a process merely because a port is busy.
      ssh -o BatchMode=yes -o ConnectTimeout=4 root@192.168.10.100 '
        for record in /map/autolabor_runtime/dual_host/run/j6m_stack.pid /map/autolabor_runtime/dual_host/run/j6m_launcher.pid; do
          test -f "$record" || continue
          line=$(head -n 1 "$record")
          pid=${line%%:*}
          case "$pid" in ""|*[!0-9]*) exit 3;; esac
          if kill -0 "$pid" 2>/dev/null; then
            echo "Original J6M process is present: $pid; stop original stack first." >&2
            exit 3
          fi
        done
      ' || exit 3
      "$candidate_ws/scripts/verify_master_owner.sh" --allow-absent || exit 3
    fi
    set -- "$candidate_ws/scripts/start_dual_host.sh" "--$operation" "$@" ;;
  check) shift; set -- "$candidate_ws/scripts/health_check.sh" "$@" ;;
  run) shift ;;
  *) echo 'Usage: optimized.sh {build|deploy|start|stop|status|restart|check|run COMMAND} [args...]'; exit 2 ;;
esac
(( $# > 0 )) || exit 2
socket_mounts=()
if [[ -d /tmp/.X11-unix ]]; then
  socket_mounts+=(--ro-bind /tmp/.X11-unix /tmp/.X11-unix)
fi
exec bwrap --die-with-parent --ro-bind / / --bind "$candidate_ws" "$candidate_ws" \
  --dev-bind /dev /dev --proc /proc \
  --bind "$candidate_ws/runtime/isolated/tmp" /tmp \
  "${socket_mounts[@]}" \
  --chdir "$candidate_ws" \
  --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
  --setenv DUAL_HOST_WS "$candidate_ws" \
  --setenv DUAL_HOST_CONFIG "$candidate_ws/config/dual_host.env" \
  --setenv XDG_CONFIG_HOME "$candidate_ws/runtime/isolated/config" \
  --setenv XDG_CACHE_HOME "$candidate_ws/runtime/isolated/cache" \
  --setenv XDG_DATA_HOME "$candidate_ws/runtime/isolated/data" \
  --setenv CUDA_CACHE_PATH "$candidate_ws/runtime/isolated/cuda" \
  --setenv LOCATEANYTHING_CACHE_ROOT "$candidate_ws/runtime/isolated/cache/locateanything" \
  --setenv LOCATEANYTHING_RUNTIME_ROOT "$candidate_ws/runtime/isolated/locateanything" \
  --setenv MPLCONFIGDIR "$candidate_ws/runtime/isolated/config/matplotlib" \
  --setenv YOLO_CONFIG_DIR "$candidate_ws/runtime/isolated/config/ultralytics" \
  --setenv ROS_HOME "$candidate_ws/runtime/isolated/ros-home" \
  --setenv ROS_LOG_DIR "$candidate_ws/log/ros" \
  --setenv PYTHONDONTWRITEBYTECODE 1 \
  --setenv NO_ALBUMENTATIONS_UPDATE 1 \
  --unsetenv ROS_HOSTNAME --unsetenv CMAKE_PREFIX_PATH --unsetenv ROS_PACKAGE_PATH \
  --unsetenv PYTHONPATH --unsetenv LD_LIBRARY_PATH \
  -- "$@"
