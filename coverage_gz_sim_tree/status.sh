#!/usr/bin/env bash
set -euo pipefail
export ROS_MASTER_URI="http://127.0.0.1:11321"
export ROS_IP="127.0.0.1"
unset ROS_HOSTNAME || true
source /opt/ros/noetic/setup.bash
source /home/slam/robot_j6m_ws/devel/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws/devel/setup.bash"
if ! rosparam get /run_id >/dev/null 2>&1; then
  printf 'isolated coverage simulation is not running on %s\n' "${ROS_MASTER_URI}"
  exit 0
fi
rostopic echo -n 1 /coverage/status
