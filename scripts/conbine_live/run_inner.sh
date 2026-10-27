#!/bin/bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/autolabor/ros/install/setup.bash
nav=/opt/autolabor/dual_host/releases/20260914_095005/install
source "$nav/setup.bash"
source /tmp/livox/setup.bash
export CMAKE_PREFIX_PATH="/tmp/overlay:/tmp/livox:$nav:/opt/autolabor/ros/install:/opt/ros/noetic"
export ROS_PACKAGE_PATH="/tmp/overlay/share:/tmp/livox/share:$nav/share:/opt/autolabor/ros/install/share:/opt/ros/noetic/share"
export PYTHONPATH="/tmp/overlay/lib/python3/dist-packages:/tmp/livox/lib/python3/dist-packages:$nav/lib/python3/dist-packages:/opt/autolabor/ros/install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages"
export LD_LIBRARY_PATH="/tmp/overlay/lib:/tmp/livox/lib:/tmp/sdk/lib:$nav/lib:/opt/autolabor/ros/install/lib:/opt/ros/noetic/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
cd /tmp
python3 -B - <<'PY'
from pathlib import Path
import time,yaml
config=Path('/tmp/persistent/coverage.yaml')
settings=yaml.safe_load(config.read_text())
if 'require_dual_lidar_for_coverage' in settings or 'dual_lidar_fresh_sec' in settings:
    config.with_name('coverage.before-mid360-only-'+str(time.time_ns())+'.yaml').write_bytes(config.read_bytes())
    settings.pop('require_dual_lidar_for_coverage',None);settings.pop('dual_lidar_fresh_sec',None)
    settings['mid360_status_fresh_sec']=1.0
    tmp=config.with_suffix('.tmp');tmp.write_text(yaml.safe_dump(settings,sort_keys=False));tmp.replace(config)
PY
if [[ "${1:-run}" == check ]]; then
  rospack find autolabor_coverage
  python3 -B -c 'import ctypes; ctypes.CDLL("/tmp/overlay/lib/libcoverage_global_planner.so"); ctypes.CDLL("/tmp/overlay/lib/libgps_geofence_layer.so"); from autolabor_coverage.cfg import HybridTuningConfig, TransitTuningConfig; from autolabor_coverage.srv import SetCoveragePlanningDefaults; print("Coverage/costmap ABI and generated interfaces: OK")'
  python3 -B - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
for package in Path('/tmp/overlay/share').iterdir():
    manifest=ET.parse(package/'package.xml').getroot()
    exports=manifest.find('export')
    for entry in exports if exports is not None else []:
        resource=entry.get('plugin')
        if resource:
            path=Path(resource.replace('${prefix}',str(package)))
            assert path.is_file(),str(path)
            ET.parse(path)
print('Every staged package plugin resource exists and parses: OK')
PY
  python3 -B -c 'import json; from autolabor_canbus_driver.msg import ChassisStatusInfo, ChassisMonitorInfo, ChassisParameter; from autolabor_coverage.msg import CoverageStatus; print("MESSAGE_MD5_JSON="+json.dumps({m._type:m._md5sum for m in (ChassisStatusInfo,ChassisMonitorInfo,ChassisParameter,CoverageStatus)}))'
  roslaunch --nodes /tmp/navigation.launch
  roslaunch --dump-params /tmp/navigation.launch > /tmp/expanded-params.yaml
  python3 -B -c 'import yaml,xmlrpc.client; p=yaml.safe_load(open("/tmp/expanded-params.yaml")); xmlrpc.client.dumps((list(p.values()),)); print("Expanded launch parameters serialize over XML-RPC: OK")'
  exit
fi
if [[ "${1:-run}" == recover-move-base ]]; then
  exec python3 -B -u /tmp/recover_move_base.py
fi
exec python3 -B -u /tmp/board_runtime.py
