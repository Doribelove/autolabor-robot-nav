#!/usr/bin/env bash
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_root="$(cd "$script_dir/.." && pwd)"
cd "$robot_root"
source devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
case "${1:-status}" in
  start)
    if timeout -s KILL 3s python3 -c 'import rosnode,sys; sys.exit(0 if rosnode.rosnode_ping("/fod_platform_link",max_count=1,verbose=False) else 1)' >/dev/null 2>&1; then
      echo 'FOD platform link already running.'
      exit 0
    fi
    link_log_dir="$robot_root/log/platform_link_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$link_log_dir"
    python3 - "$script_dir/fod_platform_link.py" "$link_log_dir" <<'PY'
import subprocess, sys
from pathlib import Path
script, directory = sys.argv[1], Path(sys.argv[2])
with (directory/'process.log').open('a') as log:
    child = subprocess.Popen([sys.executable, '-u', script, '_log_directory:='+str(directory)],
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
(directory/'process.pid').write_text(str(child.pid)+'\n')
PY
    printf '%s\n' "$link_log_dir" > /tmp/agx_platform_current
    echo "Started communication process; use status to verify delivery. Logs: $link_log_dir"
    ;;
  stop)
    rosnode kill /fod_platform_link
    ;;
  status)
    timeout -s KILL 6s rostopic echo -n 1 /fod_platform/status
    ;;
  *) echo 'Usage: scripts/fod_platform_link.sh start|stop|status' >&2; exit 2;;
esac
