#!/usr/bin/env bash
# Prepare only the candidate's private native BPU service; no firmware changes.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/load_config.sh"
[[ "${ROBOT_OPTIMIZED_SANDBOX:-}" == 1 && "$DUAL_HOST_WS" == /home/slam/robot_j6m_ws_optimized_20260905 ]] || exit 2
(( $# == 0 )) || { echo 'Usage: optimized.sh run scripts/deploy_perception_bpu.sh'; exit 2; }
cd "$DUAL_HOST_WS"
exec 9>runtime/deploy_perception_bpu.lock
flock -n 9 || { echo 'Another private BPU deployment is active.' >&2; exit 3; }
python3 - <<'PY'
import socket,subprocess
for unit in ('autolabor-dual-host.service','autolabor-optimized-20260905.service'):
    state=subprocess.check_output(['systemctl','--user','show',unit,'-p','ActiveState','--value'],text=True).strip()
    if state not in ('inactive','failed'):raise RuntimeError('Stop the owned navigation stack before BPU deployment')
for host,port in [('127.0.0.1',11311),('127.0.0.1',11571),('192.168.10.100',11311)]:
    with socket.socket() as probe:
        probe.settimeout(.5)
        if probe.connect_ex((host,port))==0:raise RuntimeError('An active ROS graph must be stopped before BPU deployment')
PY
python3 - <<'PY'
import hashlib,json
from pathlib import Path
import shutil,urllib.request
lab=Path('experiments/dual_perception_20260906')
manifest=json.loads((lab/'model_manifest.json').read_text());target=lab/manifest['path']
target.parent.mkdir(parents=True,exist_ok=True)
if not target.exists():
    temporary=target.with_suffix('.download')
    with urllib.request.urlopen(manifest['url'],timeout=30) as response,temporary.open('wb') as output:
        shutil.copyfileobj(response,output)
    if temporary.stat().st_size!=manifest['bytes'] or hashlib.sha256(temporary.read_bytes()).hexdigest()!=manifest['sha256']:
        raise RuntimeError('Downloaded CenterPoint model failed verification')
    temporary.replace(target)
if hashlib.sha256(target.read_bytes()).hexdigest()!=manifest['sha256']:raise RuntimeError('CenterPoint model SHA mismatch')
print('Pinned CenterPoint HBM verified:',manifest['revision'])
PY
remote=/map/robot_j6m_optimized_20260905/bpu_perception_20260906
target=root@192.168.10.100
ssh -o BatchMode=yes -o ConnectTimeout=4 "$target" "set -eu
  test -d /map/robot_j6m_optimized_20260905/bpu_lab_20260905/vendor/runtime_4_9_2/lib
  if test -f '$remote/tools/service_manager.py'; then python3 '$remote/tools/service_manager.py' stop; fi
  mkdir -p '$remote/tools' '$remote/python/autolabor_bpu_perception' '$remote/models/centerpoint_nash_m'"
rsync -a experiments/dual_perception_20260906/tools/ "$target:$remote/tools/"
rsync -a --exclude=__pycache__ src/application/autolabor_bpu_perception/src/autolabor_bpu_perception/ "$target:$remote/python/autolabor_bpu_perception/"
rsync -a experiments/dual_perception_20260906/models/centerpoint_nash_m/ "$target:$remote/models/centerpoint_nash_m/"
ssh "$target" bash "$remote/tools/build_native.sh"
echo 'Private BPU service deployed. The navigation companion starts it on demand.'
