#!/usr/bin/env bash
set -eo pipefail
TRI_TEST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$TRI_TEST_WS/devel/setup.bash"
# Honor an explicitly supplied master (isolated verification uses another port).
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
exec /usr/bin/python3 "$TRI_TEST_WS/scripts/three_party_test.py" "$@"
