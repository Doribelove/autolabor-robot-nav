#!/usr/bin/env bash
# Real online-mapping coverage, with full experiment recording before region planning.
set -euo pipefail
coverage_ws=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd "$coverage_ws"
coverage_name=${2:-覆盖清扫1}
case "${1:-status}" in
  start)
    if systemctl --user is-active --quiet autolabor-conbine-localization-record.service; then
      echo '已有录制正在进行，请先正常结束该录制。' >&2; exit 2
    fi
    [[ ! -e "$coverage_ws/rosbags/$coverage_name" && ! -L "$coverage_ws/rosbags/$coverage_name" ]] || {
      echo '实验名称已存在，请为新实验使用新名称。' >&2; exit 2;
    }
    if systemctl --user is-active --quiet autolabor-conbine-live.service; then
      bash scripts/conbine.sh live-stop
    fi
    BEV_DETECTOR_BACKEND=cuda bash scripts/bev_perception.sh start
    bash scripts/conbine_live.sh start --online-coverage
    python3 - <<'PY'
import json,os,socket,subprocess,time,xmlrpc.client
from pathlib import Path
socket.setdefaulttimeout(2)
deadline=time.monotonic()+55
while time.monotonic()<deadline:
    try:
        session=json.loads(Path('runtime/conbine_live/session.json').read_text())
        if not session.get('online_coverage'):raise RuntimeError('Waiting for online session')
        code,_,actual=xmlrpc.client.ServerProxy(session['master']).getParam('/coverage_prepare','/conbine/session')
        if code!=1 or actual['session']!=session['session']:raise RuntimeError('Waiting for ROS')
        env=dict(os.environ,DISPLAY=os.environ.get('DISPLAY',':0'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
        windows=subprocess.check_output(['xwininfo','-root','-tree'],env=env,text=True,stderr=subprocess.DEVNULL)
        if 'Autolabor 合并版 · 真实导航' not in windows:raise RuntimeError('Waiting for Qt')
        break
    except (OSError,ValueError,RuntimeError,xmlrpc.client.Error):time.sleep(.5)
else:raise SystemExit('Online session startup incomplete; inspect runtime/conbine_live/supervisor.log')
PY
    bash scripts/record_cleaning_experiment.sh start "$coverage_name"
    python3 - "$coverage_ws/rosbags/$coverage_name" <<'PY'
import pathlib,sys
root=pathlib.Path(sys.argv[1])
(root/'README.txt').write_text('在线建图覆盖清扫实验\n启动后即录制全ROS、四路识别/BEV和Qt录像，实际覆盖任务由用户在Qt点击开始。\nMID360 + FAST-LIO实时建图定位，无旧地图；map原点为本次camera_init原点。\n区域长10m（沿初始车头）、宽3m（横向），中心前8m/左3m。\n具体采样位姿与map顶点见region_request.json。\n作业宽度按已提供装置宽度设为0.80m，重叠率15%；实际有效刷宽可在Qt调整。\n结束录制：bash scripts/record_cleaning_experiment.sh stop\n')
PY
    set +u
    source /home/slam/robot_ws/.deps/setup.bash
    source devel/setup.bash
    set -u
    export ROS_MASTER_URI=http://192.168.10.100:11583 ROS_IP=192.168.10.50
    unset ROS_HOSTNAME
    coverage_prior_args=()
    [[ "${COVERAGE_CONFIRMED_FLAT:-0}" != 1 ]] || coverage_prior_args+=(--confirmed-flat)
    python3 -B scripts/coverage_region_preset.py --output "$coverage_ws/rosbags/$coverage_name/region_request.json" "${coverage_prior_args[@]}"
    ;;
  status) bash scripts/conbine.sh live-status; bash scripts/record_cleaning_experiment.sh status ;;
  stop-recording) bash scripts/record_cleaning_experiment.sh stop ;;
  *) echo 'Usage: online_coverage.sh {start [experiment-name]|status|stop-recording}' >&2; exit 2 ;;
esac
