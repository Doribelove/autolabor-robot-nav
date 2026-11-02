#!/usr/bin/env bash
# Record the dedicated live master independently of navigation lifecycle/control.
set -euo pipefail
record_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
record_unit=autolabor-conbine-localization-record.service
record_state="$record_ws/runtime/localization_recording"
case "${1:-status}" in
  start)
    if systemctl --user is-active --quiet "$record_unit"; then
      exec "$record_ws/scripts/record_localization_test.sh" status
    fi
    record_dir="$record_ws/rosbags/localization/$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$record_dir"/{tmp,ros-home,ros-logs} "$record_state"
    python3 - "$record_ws" "$record_dir" <<'PY'
import datetime,json,pathlib,socket,sys,xmlrpc.client
ws,out=map(pathlib.Path,sys.argv[1:]);socket.setdefaulttimeout(5)
session=json.loads((ws/'runtime/conbine_live/session.json').read_text())
master=xmlrpc.client.ServerProxy(session['master']);caller='/localization_record_metadata'
def query(method,*args):
    code,message,value=getattr(master,method)(caller,*args)
    if code!=1:raise RuntimeError(message)
    return value
actual=query('getParam','/conbine/session')
if actual['session']!=session['session']:raise RuntimeError('Live session changed; retry start')
(out/'session.json').write_text(json.dumps(session,ensure_ascii=False,indent=2)+'\n')
(out/'parameters_start.json').write_text(json.dumps(query('getParam','/'),ensure_ascii=False,indent=2)+'\n')
(out/'topics_start.json').write_text(json.dumps(query('getPublishedTopics',''),indent=2)+'\n')
(out/'graph_start.json').write_text(json.dumps(query('getSystemState'),indent=2)+'\n')
(out/'recording.json').write_text(json.dumps({
    'created_at':datetime.datetime.now().astimezone().isoformat(),
    'ros_master':session['master'],'session':session['session'],
    'selection':'all topics on dedicated live navigation master, including future topics',
    'compression':'lz4','split_seconds':60,'split_megabytes':2048,
    'buffer_megabytes':512,'control_commands_published':False},indent=2)+'\n')
PY
    ln -sfn "$record_dir" "$record_state/current"
    systemd-run --user --unit="$record_unit" --collect \
      --property=KillMode=control-group --property=KillSignal=SIGINT \
      --property=TimeoutStopSec=90 \
      /bin/bash "$record_ws/scripts/record_localization_test.sh" run "$record_dir"
    printf 'Recording directory: %s\n' "$record_dir"
    ;;
  run)
    record_dir="$2"
    exec >> "$record_dir/recorder.log" 2>&1
    exec bwrap --die-with-parent --unshare-pid --ro-bind / / \
      --bind "$record_dir" "$record_dir" --bind "$record_dir/tmp" /tmp \
      --dev /dev --proc /proc --chdir "$record_ws" \
      --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
      --setenv ROS_MASTER_URI http://192.168.10.100:11583 --setenv ROS_IP 192.168.10.50 \
      --setenv ROS_HOME "$record_dir/ros-home" --setenv ROS_LOG_DIR "$record_dir/ros-logs" \
      --unsetenv ROS_HOSTNAME \
      bash -c 'set -e; source scripts/setup_env.sh; exec rosbag record -a --lz4 --split --duration=60 --size=2048 --buffsize=512 --chunksize=768 -O "$1/localization" __name:=localization_accuracy_recorder' _ "$record_dir"
    ;;
  stop)
    # Ask the recorder itself to close its bag before stopping namespace wrappers.
    # Signalling the whole PID namespace at once can interrupt final index writes.
    if systemctl --user is-active --quiet "$record_unit"; then
      python3 - "$record_state/current" "$record_unit" <<'PY'
import json,pathlib,socket,subprocess,sys,time,xmlrpc.client
root=pathlib.Path(sys.argv[1]).resolve();socket.setdefaulttimeout(5)
metadata=json.loads((root/'recording.json').read_text())
caller='/localization_record_stop'
master=xmlrpc.client.ServerProxy(metadata['ros_master'])
code,message,uri=master.lookupNode(caller,'/localization_accuracy_recorder')
if code!=1:raise RuntimeError(message)
code,message,_=xmlrpc.client.ServerProxy(uri).shutdown(caller,'User ended localization recording')
if code!=1:raise RuntimeError(message)
deadline=time.monotonic()+60
while subprocess.run(['systemctl','--user','is-active','--quiet',sys.argv[2]]).returncode==0:
    if time.monotonic()>deadline:raise RuntimeError('Recorder has not finished closing; service left intact')
    time.sleep(.2)
if list(root.glob('*.bag.active')):raise RuntimeError('An unfinished bag remains; retain it for recovery')
PY
    fi
    # --collect may unload the unit as soon as the recorder closes normally.
    if systemctl --user is-active --quiet "$record_unit"; then
      systemctl --user stop "$record_unit"
    fi
    printf 'Recording stopped and bag files finalized: %s\n' "$(readlink -f "$record_state/current")"
    ;;
  status)
    systemctl --user show "$record_unit" -p ActiveState,SubState,MainPID
    if [[ -L "$record_state/current" ]]; then
      printf 'Recording directory: %s\n' "$(readlink -f "$record_state/current")"
      du -sh "$(readlink -f "$record_state/current")"
    fi
    ;;
  *) echo 'Usage: record_localization_test.sh {start|stop|status}' >&2; exit 2 ;;
esac
