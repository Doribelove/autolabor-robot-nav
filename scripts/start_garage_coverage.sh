#!/usr/bin/env bash
# Prepare this map's verified route; optionally start it through existing services.
set -eo pipefail
garage_ws=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$garage_ws"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --plan-only ) ]]; then
  echo "Usage: $0 [--plan-only]" >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$garage_ws/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.10.100:11583
export ROS_IP=192.168.10.50
garage_output="$garage_ws/validation/garage_drivable_coverage_20261101/live_plan"
python3 "$garage_ws/tools/garage_coverage_mission.py" \
  --dataset "$garage_ws/rosbags/地下车库建图2" --output "$garage_output"
if [[ "${1:-}" != --plan-only ]]; then
  exec python3 "$garage_ws/tools/garage_coverage_mission.py" \
    --dataset "$garage_ws/rosbags/地下车库建图2" --output "$garage_output" --execute
fi
