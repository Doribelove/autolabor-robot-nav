#!/usr/bin/env bash
set -euo pipefail
conbine_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ "$conbine_ws" == /home/slam/robot_j6m_ws_conbine ]] || { echo 'Unexpected combined workspace' >&2; exit 2; }
case "${1:-status}" in
  build|test) exec bash "$conbine_ws/scripts/validate_navigation_tuning.sh" "$1" ;;
  start|stop|status) exec python3 -B "$conbine_ws/scripts/conbine_session.py" "$1" ;;
  live-start|live-stop|live-status) exec bash "$conbine_ws/scripts/conbine_live.sh" "${1#live-}" ;;
  mapping-start) exec bash "$conbine_ws/scripts/conbine_live.sh" start --mapping ;;
  *) echo 'Usage: conbine.sh {build|test|start|stop|status|live-start|live-stop|live-status|mapping-start}' >&2; exit 2 ;;
esac
