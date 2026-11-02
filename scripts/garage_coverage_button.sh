#!/usr/bin/env bash
set -eo pipefail
garage_button_ws=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
garage_button_unit=autolabor-conbine-coverage-button.service
case "${1:-start}" in
 start)
  if systemctl --user is-active --quiet "$garage_button_unit"; then exit 0; fi
  mkdir -p "$garage_button_ws/validation/garage_coverage66_start_20261101"
  exec systemd-run --user --unit="$garage_button_unit" --collect \
   --working-directory="$garage_button_ws" --property=PartOf=autolabor-conbine-live.service \
   --setenv="DISPLAY=${DISPLAY:-:0}" --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
   --property="StandardOutput=append:$garage_button_ws/validation/garage_coverage66_start_20261101/button.log" \
   --property="StandardError=append:$garage_button_ws/validation/garage_coverage66_start_20261101/button.log" \
   /bin/bash "$garage_button_ws/scripts/garage_coverage_button.sh" run
  ;;
 run)
  source /home/slam/robot_ws/.deps/setup.bash
  source "$garage_button_ws/devel/setup.bash"
  export ROS_MASTER_URI=http://192.168.10.100:11583 ROS_IP=192.168.10.50
  exec python3 "$garage_button_ws/scripts/garage_coverage_button.py"
  ;;
 stop) systemctl --user stop "$garage_button_unit" ;;
 *) echo 'Usage: garage_coverage_button.sh {start|stop|run}' >&2; exit 2 ;;
esac
