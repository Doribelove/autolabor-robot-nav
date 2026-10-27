#!/usr/bin/env python3
"""Managed real navigation using the existing chassis command path."""
import argparse
import fcntl
import hashlib
import json
import os
import re
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import threading
import time
import xml.etree.ElementTree as ET

WS=Path(__file__).resolve().parents[1]
ROOT=WS/'runtime/conbine_live'
UNIT='autolabor-conbine-live.service'
SSH=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=3',
     '-o','ServerAliveCountMax=3','root@192.168.10.100']
CAN=Path('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0')
HELPERS=WS/'scripts/conbine_live'
PORT=11583
socket.setdefaulttimeout(3)

def command(args,check=True,**kwargs):
    return subprocess.run(args,check=check,text=True,capture_output=True,timeout=kwargs.pop('timeout',40),**kwargs)

def remote(script,check=True,timeout=40):
    return command(SSH+['sh','-s'],check,input=script,timeout=timeout)

def validate_configured_map(live_config):
    if live_config['sensor_layout']!='mid360_only':raise RuntimeError('Only the MID360 sensor layout is supported')
    map_dir=live_config['board_map_directory']
    if not map_dir.startswith('/map/robot_j6m_navigation_20260907/maps/releases/'):
        raise RuntimeError('Map must name an immutable board release directory')
    # Pin the requested historical prior; never fall back to the mutable September current link.
    verify='''import hashlib,json,pathlib
c=json.loads(CONFIG);root=pathlib.Path(c['board_map_directory'])
assert root.is_dir(), str(root)
text=(root/'manifest.yaml').read_text()
assert ('map_id: "'+c['default_map_id']+'"') in text and 'status: "complete"' in text
for name,expected in c['map_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
print('Pinned MID360 prior: '+c['default_map_id'])
'''.replace('CONFIG',repr(json.dumps(live_config)))
    command(SSH+['python3','-'],input=verify)
    return map_dir

def prepare(mapping=False):
    live_config=json.loads((WS/'config/conbine_live.json').read_text())
    map_dir=validate_configured_map(live_config)
    ROOT.mkdir(parents=True,exist_ok=True)
    session=ROOT/('session-'+time.strftime('%Y%m%d_%H%M%S')+'-'+str(os.getpid()))
    session.mkdir()
    board='/map/conbine_navigation/'+session.name
    payload=session/'payload';payload.mkdir()
    for name in ('namespace.sh','run_inner.sh','board_guardian.py','board_runtime.py','recover_move_base.py'):
        shutil.copy2(HELPERS/name,payload/name)
    for name in ('logs','ros-home','varlib','tmp','livox','sdk','map_set','persistent'):
        (payload/name).mkdir()
    (payload/'hosts').write_text('127.0.0.1 localhost\n192.168.10.100 hobot\n192.168.10.50 slam-desktop\n')
    identity={'workspace':str(WS),'session':session.name,'board':board,
              'master':'http://192.168.10.100:11583','motion_enabled':True,
              'sensor_layout':'mid360_only','mapping_mode':mapping,
              'map_id':live_config['default_map_id'],'board_map_directory':map_dir}
    (payload/'session.json').write_text(json.dumps(identity,indent=2)+'\n')
    shutil.copy2(WS/'src/platform/autolabor_dual_host/config/livox_mid360_nvidia.json',payload/'mid360.json')
    config=json.loads((payload/'mid360.json').read_text())
    host=config['MID360']['host_net_info']
    for key in ('cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip'):
        host[key]='192.168.1.100'
    config['lidar_configs'][0]['ip']='192.168.1.112'
    (payload/'mid360.json').write_text(json.dumps(config,indent=2)+'\n')
    packages={'autolabor_coverage':'src/application/autolabor_coverage',
              'teb_local_planner':'src/navigation_arena/forks/navigation/local_planner/teb',
              'autolabor_canbus_driver':'src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver',
              'autolabor_dual_host':'src/platform/autolabor_dual_host',
              'autolabor_fod_control':'src/application/autolabor_fod_control',
              'robot_bringup':'src/scripts/robot_bringup'}
    overlay=payload/'overlay'
    overlay.mkdir()
    (overlay/'.catkin').touch()
    for name,path in packages.items():
        source=WS/path;dest=overlay/'share'/name;dest.mkdir(parents=True)
        for item in ('package.xml','launch','config','msg','srv','action','coverage_global_planner_plugin.xml'):
            if (source/item).is_dir():
                shutil.copytree(source/item,dest/item,ignore=shutil.ignore_patterns('__pycache__'))
            elif (source/item).is_file():
                shutil.copy2(source/item,dest/item)
        for resource in source.glob('*.xml'):
            shutil.copy2(resource,dest/resource.name)
        lib=overlay/'lib'/name;lib.mkdir(parents=True)
        for file in (source/'scripts').glob('*.py'):
            shutil.copy2(file,lib/file.name);(lib/file.name).chmod(0o755)
    # This run's real tuning storage is separate from all offline preview values.
    launch=overlay/'share/autolabor_coverage/launch/coverage.launch'
    tree=ET.parse(launch)
    tree.getroot().find("arg[@name='config_file']").set('default','/tmp/persistent/coverage.yaml')
    tree.write(launch,encoding='unicode')
    python=overlay/'lib/python3/dist-packages/autolabor_coverage'
    shutil.copytree(WS/'devel/lib/python3/dist-packages/autolabor_coverage',python,
                    ignore=shutil.ignore_patterns('__pycache__'))
    for file in (WS/'src/application/autolabor_coverage/src/autolabor_coverage').glob('*.py'):
        shutil.copy2(file,python/file.name)
    shutil.copytree(WS/'devel/lib/python3/dist-packages/autolabor_canbus_driver',
                    overlay/'lib/python3/dist-packages/autolabor_canbus_driver',
                    ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('libcoverage_global_planner.so','libcoverage_hybrid_astar.so','libgps_geofence_layer.so',
                 'libteb_local_planner.so'):
        shutil.copy2(WS/'devel/lib'/name,overlay/'lib'/name)
    launch=ET.Element('launch')
    for key,value in {'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.,
        'output_data_type':0,'cmdline_str':'100000000000000','cmdline_file_path':'livox_test.lvx',
        'user_config_path':'/tmp/mid360.json','frame_id':'livox_frame',
        'enable_lidar_bag':False,'enable_imu_bag':False}.items():
        kind={str:'string',bool:'bool',int:'int',float:'double'}[type(value)]
        ET.SubElement(launch,'param',name=key,type=kind,
                      value=str(value).lower() if isinstance(value,bool) else str(value))
    ET.SubElement(launch,'node',pkg='livox_ros_driver2',type='livox_ros_driver2_node',
                  name='conbine_livox_driver',required='true',output='screen')
    inc=ET.SubElement(launch,'include',file='$(find autolabor_dual_host)/launch/j6m_fastlio_navigation.launch')
    for key,value in {'start_relays':'false','start_enhanced_cloud':'false',
                      'use_static_map':str(not mapping).lower(),'map_file':'/tmp/map_set/map_fused_2d/map.yaml',
                      'fast_lio_map_file':'/tmp/map_set/map_3d/map.pcd',
                      'fod_motion_enabled':'false','max_linear_speed':'0.8',
                      'max_angular_speed':'0.6'}.items():
        ET.SubElement(inc,'arg',name=key,value=value)
    ET.ElementTree(launch).write(payload/'navigation.launch',encoding='unicode')
    archive=session/'payload.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        for item in payload.iterdir():tar.add(item,arcname=item.name)
    manifest={str(p.relative_to(payload)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in payload.rglob('*') if p.is_file()}
    (session/'payload-sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    # Abort before uploading if this would interfere with any existing master or driver.
    pre=remote('''set -eu
ip -o -4 addr show dev eth1 | grep -q '192.168.1.100/24'
! ss -lntup | grep -E ':(11311|11583|56301|56401)[[:space:]]'
mkdir -p /map/conbine_navigation/persistent/ros-home
''')
    (session/'preflight.log').write_text(pre.stdout+pre.stderr)
    r=command(SSH+['mkdir','-p',board]);(session/'mkdir.log').write_text(r.stdout+r.stderr)
    with archive.open('rb') as stream:
        r=subprocess.run(SSH+['tar','-xzf','-','-C',board],stdin=stream,capture_output=True,timeout=30)
    if r.returncode:raise RuntimeError(r.stderr.decode())
    remote_script='''import pathlib,json,os
root=pathlib.Path(BOARD)
nav=pathlib.Path('/map/robot_j6m_navigation_20260907/rootfs/opt/autolabor/dual_host/releases/20260914_095005/install/lib')
for name in ('robot_bringup','autolabor_dual_host','autolabor_fod_control','autolabor_coverage'):
    for source in (nav/name).iterdir():
        dest=root/'overlay/lib'/name/source.name
        if not dest.exists():
            dest.symlink_to('/opt/autolabor/dual_host/releases/20260914_095005/install/lib/'+name+'/'+source.name)
config=pathlib.Path('/map/conbine_navigation/persistent/coverage.yaml')
if not config.exists():config.write_bytes((root/'overlay/share/autolabor_coverage/config/coverage.yaml').read_bytes())
'''.replace('BOARD',repr(board))
    command(SSH+['python3','-'],input=remote_script)
    check=command(SSH+['unshare','--mount','--pid','--fork','--kill-child=SIGTERM','bash',board+'/namespace.sh',board,'check'],False,timeout=45)
    (session/'board-preflight.log').write_text(check.stdout+check.stderr)
    if check.returncode:raise RuntimeError('Board preflight failed: '+check.stderr[-2500:])
    wire=json.loads(next(line.split('=',1)[1] for line in check.stdout.splitlines()
                         if line.startswith('MESSAGE_MD5_JSON=')))
    for typ,md5 in wire.items():
        package,name=typ.split('/')
        generated=WS/'devel/lib/python3/dist-packages'/package/'msg'/('_'+name+'.py')
        expected=re.search(r'_md5sum\s*=\s*"([0-9a-f]+)"',generated.read_text()).group(1)
        if expected!=md5:raise RuntimeError('Cross-host message mismatch: '+typ)
    # Fetch the immutable existing map for the GUI, without modifying its source.
    local_map=ROOT/'maps'/live_config['default_map_id'];local_map.mkdir(parents=True,exist_ok=True)
    r=subprocess.run(SSH+['tar','-czhf','-','-C',map_dir,
                          'manifest.yaml','map_2d','map_fused_2d'],capture_output=True,timeout=30)
    if r.returncode:raise RuntimeError(r.stderr.decode())
    import io
    with tarfile.open(fileobj=io.BytesIO(r.stdout),mode='r:gz') as tar:
        for item in tar.getmembers():
            if item.name.startswith('/') or '..' in Path(item.name).parts or item.issym() or item.islnk():
                raise RuntimeError('Unexpected map archive member')
        tar.extractall(local_map)
    identity['local_map_directory']=str(local_map)
    (ROOT/'session.json').write_text(json.dumps(dict(identity,local=str(session)),indent=2)+'\n')
    return session,board

def run():
    if WS != Path('/home/slam/robot_j6m_ws_conbine'):raise RuntimeError('Unexpected workspace')
    ROOT.mkdir(parents=True,exist_ok=True)
    with (ROOT/'session.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if not CAN.exists():raise RuntimeError('Confirmed chassis adapter absent')
        owners=command(['fuser',str(CAN.resolve())],False)
        if owners.stdout.strip():raise RuntimeError('CAN adapter occupied: '+owners.stdout)
        mapping=os.environ.get('CONBINE_LIVE_MODE','navigation')=='mapping'
        session,board=prepare(mapping)
        record=json.loads((ROOT/'session.json').read_text())
        stopping=threading.Event()
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopping.set())
        children=[]
        def spawn(name,args,pipe=False):
            stream=(session/(name+'.log')).open('a')
            p=subprocess.Popen(args,stdin=subprocess.PIPE if pipe else subprocess.DEVNULL,
                               stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,p,stream));return p
        try:
            board_process=spawn('board',SSH+['python3','-u',board+'/board_guardian.py',board],True)
            def heartbeat():
                while not stopping.wait(1):
                    try:board_process.stdin.write(b'live\n');board_process.stdin.flush()
                    except (OSError,ValueError):stopping.set();return
            threading.Thread(target=heartbeat,daemon=True).start()
            import xmlrpc.client
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and not stopping.is_set():
                try:
                    if xmlrpc.client.ServerProxy('http://192.168.10.100:11583').getParam('/conbine_launch','/conbine/session')[2].get('session')==session.name:break
                except (OSError,AttributeError):pass
                stopping.wait(.2)
            else:raise RuntimeError('Dedicated real ROS master did not start')
            spawn('gateway',['roslaunch',str(WS/'src/platform/autolabor_dual_host/launch/nvidia_gateway.launch'),
                'start_livox:=false','start_can:=true',
                'can_port:='+str(CAN),'motion_enabled:=true'])
            # This leaf makes only the map root visible; it never invents a robot pose.
            if not mapping:
                spawn('map_anchor',['rosrun','tf2_ros','static_transform_publisher',
                                    '0','0','0','0','0','0','1','map','autolabor_map_display_anchor'])
            spawn('qt',[str(WS/'devel/lib/autolabor_operator_gui/autolabor_operator_gui_node'),
                '__name:=autolabor_operator_gui','_static_map_mode:='+str(not mapping).lower(),
                '_rviz_startup_fixed_frame:='+('camera_init' if mapping else 'map'),
                '_rviz_navigation_fixed_frame:='+('camera_init' if mapping else 'map'),
                '_static_map_set:='+record['local_map_directory'],'_coverage_region_root:='+record['local_map_directory'],
                '_navigation_mode_label:=REAL_NAVIGATION',
                '_window_title:=Autolabor 合并版 · 真实导航',
                '_configured_vision_backend:=j6m_surround_bpu','_vision_backend_switch_script:=',
                '_rviz_config:='+str(WS/'src/application/autolabor_operator_gui/config/operator_navigation.rviz'),
                '_tuning_screenshot:='+str(session/'qt_live.png')])
            while not stopping.wait(.5):
                failed=[name for name,p,_ in children if p.poll() is not None]
                if failed:raise RuntimeError('Session child exited: '+','.join(failed))
        finally:
            stopping.set()
            # Stop the local gateway before closing the remote ownership lease.
            for name,p,stream in reversed(children):
                if p.poll() is None:
                    if name=='board' and p.stdin:
                        p.stdin.close()
                    else:os.killpg(p.pid,signal.SIGINT)
                    try:p.wait(timeout=25)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid,signal.SIGTERM)
                        try:p.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                stream.close()

def main():
    action=argparse.ArgumentParser();action.add_argument('action',choices=['start','stop','status','run']);action.add_argument('--mapping',action='store_true');args=action.parse_args()
    if args.action=='run':return run()
    existing=command(['systemctl','--user','show',UNIT,'-p','ExecStart','--value'],False).stdout
    if existing.strip() and str(WS/'scripts/conbine_live.sh') not in existing:raise RuntimeError('Service identity mismatch')
    if args.action=='start':
        if command(['systemctl','--user','is-active','--quiet',UNIT],False).returncode==0:
            if (ROOT/'session.json').exists() and json.loads((ROOT/'session.json').read_text()).get('mapping_mode',False)!=args.mapping:
                raise RuntimeError('Stop the current live session before changing mapping/navigation mode')
            return
        r=command(['systemd-run','--user','--unit='+UNIT,'--collect','--property=KillMode=mixed',
                  '--property=TimeoutStopSec=65','--setenv=DISPLAY='+os.environ.get('DISPLAY',':0'),
                  '--setenv=XAUTHORITY='+os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'),
                  '--setenv=CONBINE_LIVE_MODE='+('mapping' if args.mapping else 'navigation'),
                  '/bin/bash',str(WS/'scripts/conbine_live.sh'),'run'])
        print(r.stdout+r.stderr)
    elif args.action=='stop':
        print(command(['systemctl','--user','stop',UNIT]).stdout)
        # Local systemd can finish before the remote SSH guardian finishes teardown.
        if (ROOT/'session.json').exists():
            record=json.loads((ROOT/'session.json').read_text())
            deadline=time.monotonic()+35
            while time.monotonic()<deadline:
                r=command(SSH+['test','-f',record['board']+'/guardian_exit.json'],False,timeout=8)
                if r.returncode==0:break
                time.sleep(.2)
            else:raise RuntimeError('Owned remote session has not confirmed shutdown')
    else:
        print(command(['systemctl','--user','show',UNIT,'-p','ActiveState,SubState,MainPID,ExecStart'],False).stdout)
        if (ROOT/'session.json').exists():print((ROOT/'session.json').read_text())

if __name__=='__main__':main()
