#!/usr/bin/env bash
set -euo pipefail
/tmp/install/bin/zed_capture_probe --version
/tmp/install/bin/zed_opencv_check
# The stock Noetic environment hooks read optional variables when unset.
set +u
source /opt/ros/noetic/setup.bash
set -u
python3 -B - <<'PY'
import json
from sensor_msgs.msg import Image, CameraInfo, Imu
print(json.dumps({'ros_messages_imported': [Image._type, CameraInfo._type, Imu._type],
                  'ros_nodes_started': False}))
PY
