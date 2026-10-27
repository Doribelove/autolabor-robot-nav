#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
preview_root="$project_root/runtime/camera_compare_20261024"
case "${1:-start}" in
  start)
    if [[ -f "$preview_root/viewer.pid" ]]; then
      preview_pid=$(cat "$preview_root/viewer.pid")
      if [[ "$preview_pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$preview_pid/cmdline" ]] &&
          tr '\0' '\n' < "/proc/$preview_pid/cmdline" | rg -Fxq "$preview_root/viewer.py"; then
        echo "相机对比窗口已运行：PID $preview_pid"
        exit 0
      fi
    fi
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
    systemd-run --user --unit=autolabor-conbine-camera-compare --collect \
      --working-directory="$project_root" --setenv="DISPLAY=$DISPLAY" \
      --setenv="XAUTHORITY=$XAUTHORITY" --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$preview_root/viewer.log" \
      --property="StandardError=append:$preview_root/viewer.log" \
      /usr/bin/python3 -s "$preview_root/viewer.py"
    ;;
  stop)
    /usr/bin/python3 -s - "$preview_root" <<'PY'
from pathlib import Path
import os, signal, sys
root = Path(sys.argv[1])
file = root/'viewer.pid'
if file.exists():
    pid = int(file.read_text())
    proc = Path('/proc')/str(pid)
    if proc.exists():
        if str(root/'viewer.py').encode() not in (proc/'cmdline').read_bytes().split(b'\0'):
            raise RuntimeError('Viewer PID identity mismatch')
        os.kill(pid, signal.SIGTERM)
PY
    ;;
  status)
    curl --fail --silent --max-time 3 http://127.0.0.1:18881/state
    echo
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
