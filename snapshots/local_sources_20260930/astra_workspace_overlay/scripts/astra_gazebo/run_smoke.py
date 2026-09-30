#!/usr/bin/env python3
"""New-identity four-method Gazebo bootstrap with durable attempts and images.

This is an environment/measurement diagnostic, not a performance experiment.
It imports the frozen R2 renderer without invoking any historical execution.
"""
import argparse
import csv
import datetime
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
STAGE = WORKSPACE.parent
METHODS = ('pct_scope_rl','analytic_prior','actual_teb','dwa')
CHECKPOINT = WORKSPACE/'artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def atomic(path,data):
    temp=Path(str(path)+'.tmp')
    with temp.open('w') as f:
        json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(str(temp),str(path))


def utc(): return datetime.datetime.now(datetime.timezone.utc).isoformat()


def append_csv(path,row):
    with path.open() as f:fields=next(csv.reader(f))
    with path.open('a',newline='') as f:
        csv.DictWriter(f,fieldnames=fields,extrasaction='ignore').writerow(row);f.flush();os.fsync(f.fileno())


def render(seed):
    spec=importlib.util.spec_from_file_location('frozen_renderer',WORKSPACE/'scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    text,scene=module.render('p02_static_corridor',seed,'development')
    tree=ET.fromstring(text);world=tree.find('world')
    ground=world.find("model[@name='ground_plane']/link")
    visual=ET.SubElement(ground,'visual',name='ground_visual')
    geom=ET.SubElement(visual,'geometry');plane=ET.SubElement(geom,'plane')
    ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='40 30'
    mat=ET.SubElement(visual,'material')
    ET.SubElement(mat,'ambient').text='.72 .76 .78 1';ET.SubElement(mat,'diffuse').text='.72 .76 .78 1'
    for m in world.findall('model'):
        if 'wall' in m.attrib['name']:
            mat=ET.SubElement(m.find('link/visual'),'material')
            ET.SubElement(mat,'ambient').text='.35 .42 .48 1';ET.SubElement(mat,'diffuse').text='.35 .42 .48 1'
    model=ET.SubElement(world,'model',name='astra_evidence_camera')
    ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text='0 0 18 0 1.5707963267948966 0'
    link=ET.SubElement(model,'link',name='camera_link')
    sensor=ET.SubElement(link,'sensor',name='overview',type='camera')
    ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='2'
    camera=ET.SubElement(sensor,'camera');ET.SubElement(camera,'horizontal_fov').text='1.05'
    img=ET.SubElement(camera,'image')
    for k,v in [('width','2560'),('height','1440'),('format','R8G8B8')]:ET.SubElement(img,k).text=v
    clip=ET.SubElement(camera,'clip');ET.SubElement(clip,'near').text='.1';ET.SubElement(clip,'far').text='100'
    plugin=ET.SubElement(sensor,'plugin',name='astra_camera_ros',filename='libgazebo_ros_camera.so')
    for k,v in [('robotNamespace','/astra'),('alwaysOn','true'),('updateRate','2'),('cameraName','camera'),('imageTopicName','image_raw'),('cameraInfoTopicName','camera_info'),('frameName','astra_camera_optical')]:ET.SubElement(plugin,k).text=v
    scene['expected_models']=[m.attrib['name'] for m in world.findall('model')]+['autolabor_m2']
    scene['evidence_camera']={'pose_xyz_rpy':[0,0,18,0,math.pi/2,0],'width':2560,'height':1440,'horizontal_fov_rad':1.05,'update_hz':2,'projection':'perspective','topic':'/astra/camera/image_raw'}
    # Initial horizontal compound footprint lies within chassis rectangle for
    # centered wheels. Tall walls and no actors make this smoke unambiguous.
    scene['initial_geometry_qc']={'robot_instances':1,'actors':0,'robot_xy':[-5.4,0],
        'robot_chassis_length_width_m':[1.04,.70],
        'minimum_initial_horizontal_clearance_m':2.35-.35,
        'illegal_initial_intersection':False,'wheel_ground_height':'runtime_required',
        'scope':'static corridor only; not a general geometric verifier'}
    module.indent(tree)
    return '<?xml version="1.0"?>\n'+ET.tostring(tree,encoding='unicode')+'\n',scene


def bindings(root):
    paths=[]
    for directory in ['scripts/astra_gazebo','src/application/v5_e2e_scope_rl','src/application/v3_scope_planner',
                      'src/simulation/m2_gazebo','src/navigation_arena/forks/navigation/local_planner/teb']:
        paths.extend(p for p in (WORKSPACE/directory).rglob('*') if p.is_file() and not p.is_symlink()
                     and not any(x in p.parts for x in ('.git','__pycache__','.pytest_cache')))
    paths.extend([WORKSPACE/'scripts/astra_env.sh',WORKSPACE/'scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py',CHECKPOINT])
    paths.extend((WORKSPACE/'devel/lib').glob('lib*.so'))
    paths.extend([Path('/opt/ros/noetic/lib/libdwa_local_planner.so'),Path('/opt/ros/noetic/lib/libgazebo_ros_camera.so'),Path('/usr/bin/gzserver')])
    paths.extend(Path('/opt/ros/noetic/lib')/name for name in
                 ('libgazebo_ros_api_plugin.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so'))
    paths.extend([Path('/opt/ros/noetic/lib/move_base/move_base'),Path(sys.executable).resolve(),
                  STAGE/'environment_manifest.yaml',WORKSPACE/'requirements/thesis-rl-lock.txt',
                  WORKSPACE/'requirements/thesis-rl-lock.yaml'])
    paths.extend((STAGE/'contracts').glob('*.yaml'))
    paths.append(STAGE/'contracts/initial_design_root_review.json')
    paths.extend(root/p for p in ('world.sdf','scene_manifest.yaml'))
    data={'snapshot_manifest_sha256':sha(STAGE/'bootstrap/content_manifest.jsonl'),
          'python_executable':sys.executable,'resources':[{'path':str(p),'sha256':sha(p)} for p in sorted(set(paths))]}
    atomic(root/'bindings.json',data)
    with (root/'dirty.patch').open('wb') as out:subprocess.run(['git','diff','--binary'],cwd=str(WORKSPACE),stdout=out,check=True)
    return sha(root/'bindings.json')


def owned_processes(root):
    marker=b'ROS_LOG_DIR='+os.fsencode(root/'roslog')
    found=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            if marker not in (entry/'environ').read_bytes().split(b'\0'):continue
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z':found.append((int(entry.name),int(fields[2]),fields[19]))
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    return found


def stop_group(proc,root):
    if proc.poll() is None:
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait(timeout=5)
    # roslaunch starts nodes with setsid: descendant PGIDs differ. Exact
    # per-run ROS_LOG_DIR plus PID start ticks establishes our ownership.
    for sig,seconds in ((signal.SIGTERM,5),(signal.SIGKILL,5)):
        owned=owned_processes(root)
        if not owned:break
        for pid,pgid,ticks in owned:
            try:
                current=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()
                if current[19]==ticks:os.kill(pid,sig)
            except (FileNotFoundError,ProcessLookupError):pass
        deadline=time.monotonic()+seconds
        while owned_processes(root) and time.monotonic()<deadline:time.sleep(.1)
    survivors=owned_processes(root)
    if survivors:raise RuntimeError('owned descendants did not terminate: '+str(survivors))
    for port in (14820,14821):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    atomic(root/'cleanup.json',{'utc':utc(),'owned_live_processes':[],
                              'ports_14820_14821_released':True,'scope':'exact run environment markers, not global process names'})


def _execute(row):
    root=STAGE/'runs'/row['run_id']
    if root.exists():
        if (root/'receipt.json').is_file():return json.loads((root/'receipt.json').read_text())
        raise RuntimeError('partial identity requires explicit live-process audit, never blindly restart: '+str(root))
    root.mkdir()
    atomic(root/'allocated.json',dict(row,started_utc=utc(),attempt=1,
                                    status='identity_consumed_preparing',retry_allowed=False))
    text,scene=render(row['seed']);(root/'world.sdf').write_text(text)
    (root/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
    bh=bindings(root)
    env=dict(os.environ)
    env.update(THESIS_WS=str(WORKSPACE),ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',
               ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_HOME=str(root/'ros_home'),ROS_LOG_DIR=str(root/'roslog'),
               GAZEBO_MODEL_DATABASE_URI='',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',MKL_NUM_THREADS='2')
    for port in (14820,14821):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    command=['roslaunch','m2_gazebo','astra_smoke_episode.launch']
    args=dict(world=root/'world.sdf',method=row['method'],scene_manifest=root/'scene_manifest.yaml',
              scene_id='p02_static_corridor',seed=row['seed'],report_path=root/'episode_report.yaml',
              actor_checkpoint_path=CHECKPOINT,evidence_dir=root/'evidence',run_id=row['run_id'])
    command.extend(k+':='+str(v) for k,v in args.items())
    started=utc();start=time.monotonic()
    atomic(root/'attempt.json',dict(row,started_utc=started,command=command,cwd=str(WORKSPACE),
                                   bindings_sha256=bh,wall_limit_s=180,environment_overrides={k:env[k] for k in ('ROS_MASTER_URI','GAZEBO_MASTER_URI','ROS_HOME','ROS_LOG_DIR','ROS_IP','ROS_HOSTNAME','THESIS_WS')}))
    append_csv(STAGE/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='pipeline_only',seed=row['seed'],run_id=row['run_id'],status='consumed',source_binding=bh))
    proc=None;reason=None;cleanup_pass=False
    try:
        with (root/'roslaunch.log').open('wb') as log:
            proc=subprocess.Popen(command,cwd=str(WORKSPACE),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().split()[21]})
            while proc.poll() is None:
                atomic(root/'heartbeat.json',{'utc':utc(),'pid':proc.pid,'wall_elapsed_s':time.monotonic()-start})
                if time.monotonic()-start>180:reason='infrastructure_wall_timeout';break
                time.sleep(2)
    except BaseException as error:
        reason=type(error).__name__+':'+str(error)
    finally:
        if proc is not None:stop_group(proc,root)
        cleanup_pass=True
    report=yaml.safe_load((root/'episode_report.yaml').read_text()) if (root/'episode_report.yaml').exists() else None
    result=report.get('result',{}) if report else {}
    evidence=json.loads((root/'evidence/status.json').read_text()) if (root/'evidence/status.json').exists() else {}
    receipt=dict(row,started_utc=started,ended_utc=utc(),wall_elapsed_s=time.monotonic()-start,
        exit_code=proc.returncode if proc else None,termination_detail=reason,bindings_sha256=bh,
        report_sha256=sha(root/'episode_report.yaml') if report else None,
        goal_reached=result.get('goal_reached',False),execution_integrity_pass=report.get('execution_integrity_pass',False) if report else False,
        legacy_restricted_elapsed_s=result.get('restricted_elapsed_s',22.),
        result=result,evidence=evidence,cleanup_pass=cleanup_pass,
        capture_complete=evidence.get('frames',0)>0 and evidence.get('state_samples',0)>0,
        status='completed' if report and reason is None else 'infrastructure_failure',
        scientific_eligibility='pipeline_only_never_performance_evidence')
    atomic(root/'receipt.json',receipt)
    append_csv(STAGE/'EXPERIMENTS.csv',dict(receipt,split='bootstrap_smoke',candidate_id='frozen_r2_baseline',scene_id='p02_static_corridor',layout_id='legacy_pipeline_corridor',code_sha256=bh,model_sha256=sha(CHECKPOINT),report_path=str(root/'episode_report.yaml'),raw_path=str(root)))
    append_csv(STAGE/'run_manifest.csv',dict(receipt,pid=proc.pid if proc else None,process_group=proc.pid if proc else None,command_json=json.dumps(command),cwd=str(WORKSPACE),heartbeat_path=str(root/'heartbeat.json'),receipt_path=str(root/'receipt.json')))
    return receipt


def execute(row):
    root=STAGE/'runs'/row['run_id']
    # Existing partial identities need a live PID/start-tick audit, not another
    # attempt. Keep this check outside the new-attempt error handler.
    if root.exists() and not (root/'receipt.json').is_file():
        raise RuntimeError('partial identity requires live-process audit: '+str(root))
    try:return _execute(row)
    except BaseException as error:
        if not root.exists():raise
        # This covers rendering, source/hash/port failures as well as cleanup.
        # Preparation failures are charged to the smoke identity conservatively.
        allocation=json.loads((root/'allocated.json').read_text()) if (root/'allocated.json').exists() else {}
        receipt=dict(row,status='infrastructure_failure',started_utc=allocation.get('started_utc'),ended_utc=utc(),
            exit_code=None,goal_reached=False,execution_integrity_pass=False,wall_elapsed_s=None,
            legacy_restricted_elapsed_s=22.,evidence={},termination_detail=type(error).__name__+':'+str(error),
            scientific_eligibility='pipeline_only_never_performance_evidence')
        atomic(root/'receipt.json',receipt)
        with (STAGE/'seed_registry.csv').open() as f:registered=any(r['run_id']==row['run_id'] for r in csv.DictReader(f))
        if not registered:append_csv(STAGE/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='pipeline_only',seed=row['seed'],run_id=row['run_id'],status='consumed_preparation_failure'))
        append_csv(STAGE/'EXPERIMENTS.csv',dict(receipt,candidate_id='frozen_r2_baseline',scene_id='p02_static_corridor',raw_path=str(root)))
        append_csv(STAGE/'run_manifest.csv',dict(receipt,receipt_path=str(root/'receipt.json')))
        return receipt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if Path(os.environ.get('THESIS_WS','')).resolve()!=WORKSPACE:raise RuntimeError('source scripts/astra_env.sh in the isolated workspace first')
    if not (STAGE/'contracts/acceptance.yaml').exists():raise RuntimeError('acceptance must be recorded before any simulation')
    review=json.loads((STAGE/'contracts/initial_design_root_review.json').read_text())
    for item in review['resources']:
        if sha(STAGE/item['path'])!=item['sha256']:raise RuntimeError('initial design bytes changed before simulation')
    lock=(STAGE/'queue/simulation.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    schedule=[dict(run_id='smoke_a01_{:02d}_{}'.format(i+1,m),method=m,seed=260926001,split='bootstrap_smoke') for i,m in enumerate(METHODS)]
    schedule_path=STAGE/'queue/smoke_a01_schedule.json'
    if schedule_path.exists():
        if json.loads(schedule_path.read_text())!=schedule:raise RuntimeError('frozen smoke schedule changed')
    else:atomic(schedule_path,schedule)
    if not args.execute:
        print(json.dumps(schedule,indent=2));return
    results=[]
    for row in schedule:
        print('START '+row['run_id'],flush=True);receipt=execute(row);results.append(receipt)
        print(json.dumps({k:receipt[k] for k in ('run_id','status','goal_reached','execution_integrity_pass','wall_elapsed_s','evidence')}),flush=True)
        atomic(STAGE/'queue/smoke_a01_status.json',{'updated_utc':utc(),'finished':len(results),'total':len(schedule),'receipts':[r['run_id'] for r in results]})
        if receipt['status']!='completed' or receipt['evidence'].get('machine_pass_frames',0)==0:break
    atomic(STAGE/'audits/smoke_a01_summary.json',{'results':results,'performance_evidence':False,'all_four_completed':len(results)==4 and all(r['status']=='completed' for r in results)})


if __name__=='__main__':main()
