#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
MODE_SERVICE="/fod_navigation_mode/set_fod_enabled"
STATUS_TOPIC="/fod_navigation_mode/status"

usage() {
  echo "Usage: $0 start|stop|status|watch"
  echo
  echo "  start   pause the retained GPS route, stop, then enable FOD recovery"
  echo "  stop    disable FOD recovery and resume the retained GPS route"
  echo "  status  print one GPS/FOD mode-manager status message"
  echo "  watch   continuously display the mode-manager status"
}

call_mode_service() {
  local enabled="$1"
  local response
  response="$(rosservice call "$MODE_SERVICE" "data: $enabled")"
  printf '%s\n' "$response"
  if ! grep -Eq '^success:[[:space:]]+True$' <<<"$response"; then
    echo "Mode change was rejected; inspect $STATUS_TOPIC before retrying." >&2
    return 4
  fi
}

if (( $# != 1 )); then
  usage >&2
  exit 1
fi

if [[ ! -f "$ROS_SETUP" || ! -f "$ROBOT_WS/devel/setup.bash" ]]; then
  echo "ROS environment is not built or available under $ROBOT_WS." >&2
  exit 2
fi

source "$ROS_SETUP"
source "$ROBOT_WS/devel/setup.bash"

case "$1" in
  start)
    if ! rosservice info "$MODE_SERVICE" >/dev/null 2>&1; then
      echo "$MODE_SERVICE is unavailable. Start GPS bringup first." >&2
      exit 3
    fi
    echo "Requesting FOD recovery. GPS output is blocked before the vehicle-stop check."
    call_mode_service true
    ;;
  stop)
    if ! rosservice info "$MODE_SERVICE" >/dev/null 2>&1; then
      echo "$MODE_SERVICE is unavailable. The chassis mode cannot be changed." >&2
      exit 3
    fi
    echo "Requesting visual standby and GPS route resume."
    call_mode_service false
    ;;
  status)
    if ! timeout 3 rostopic echo -n 1 "$STATUS_TOPIC"; then
      echo "No status received from $STATUS_TOPIC. Start GPS bringup first." >&2
      exit 3
    fi
    ;;
  watch)
    rostopic echo "$STATUS_TOPIC"
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac
