#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
preview_root="$project_root/runtime/camera_bev_check_20261025"
unit=autolabor-conbine-camera-bev.service
case "${1:-start}" in
  start)
    if systemctl --user is-active --quiet "$unit"; then
      echo '三路 BEV 预览已运行'
      exit 0
    fi
    if systemctl --user is-active --quiet autolabor-conbine-camera-bev-check.service; then
      systemctl --user stop autolabor-conbine-camera-bev-check.service
    fi
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
    systemd-run --user --unit="$unit" --collect --working-directory="$project_root" \
      --setenv="DISPLAY=$DISPLAY" --setenv="XAUTHORITY=$XAUTHORITY" \
      --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$preview_root/bev_viewer.log" \
      --property="StandardError=append:$preview_root/bev_viewer.log" \
      /usr/bin/python3 -s "$project_root/scripts/three_camera_bev.py"
    ;;
  stop)
    if systemctl --user is-active --quiet "$unit"; then
      systemctl --user stop "$unit"
    fi
    ;;
  status)
    systemctl --user show "$unit" -p ActiveState -p MainPID
    if [[ -f "$preview_root/bev/bev.json" ]]; then
      /usr/bin/python3 -s - "$preview_root/bev/bev.json" <<'PY'
import json, sys, time
s=json.load(open(sys.argv[1]))
print(json.dumps(dict(age_s=(time.time_ns()-s['at_epoch_ns'])/1e9,
    sequences={n:m['sequence'] for n,m in s['sources'].items()},
    processing_ms=s['processing_ms'],metric_accuracy_verified=s['metric_accuracy_verified'],
    source_exposure_synchronized=s['source_exposure_synchronized']),ensure_ascii=False))
PY
    fi
    ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
