#!/usr/bin/env python3
"""New immutable identity for a four-second physics evidence qualification."""
import csv
import fcntl
import json
import os
from pathlib import Path
import subprocess
import socket
import time
import xml.etree.ElementTree as ET
import yaml
import run_smoke_a02 as common


def _execute():
    stage=common.STAGE; workspace=common.WORKSPACE
    if Path(os.environ.get('THESIS_WS','')).resolve()!=workspace:raise RuntimeError('source isolated environment')
    rid='physics_evidence_q01';seed=260926003
    root=stage/'runs'/rid;root.mkdir()
    common.atomic(root/'allocated.json',{'run_id':rid,'seed':seed,'attempt':1,'started_utc':common.utc(),'purpose':'measurement_fixture','retry_allowed':False})
    world='''<?xml version="1.0"?><sdf version="1.6"><world name="astra_contact_fixture">
<gravity>0 0 -9.81</gravity><physics name="ode" type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate></physics>
<model name="ground_plane"><static>true</static><link name="body"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision></link></model>
<model name="falling_cube"><pose>0 0 1 0 0 0</pose><link name="body"><inertial><mass>1</mass><inertia><ixx>0.0266666667</ixx><iyy>0.0266666667</iyy><izz>0.0266666667</izz></inertia></inertial><collision name="cube"><geometry><box><size>0.4 0.4 0.4</size></box></geometry></collision></link></model>
<plugin name="astra_evidence" filename="libastra_evidence_plugin.so"><state_hz>50</state_hz></plugin>
</world></sdf>'''
    (root/'world.sdf').write_text(world)
    (root/'scene_manifest.yaml').write_text(yaml.safe_dump({'run_id':rid,'seed':seed,'expected_models':['ground_plane','falling_cube'],'purpose':'measurement_fixture','expected_contacts':'cube-ground after free fall','physics_hz':500,'state_hz':50,'duration_s':4.0,'gates_source':str(workspace/'src/simulation/m2_gazebo/scripts/astra_evidence_fixture_probe.py')}))
    launch=ET.Element('launch')
    inc=ET.SubElement(launch,'include',file='$(find gazebo_ros)/launch/empty_world.launch')
    for k,v in dict(world_name=str(root/'world.sdf'),paused='true',gui='false',use_sim_time='true',verbose='true',extra_gazebo_args='--seed '+str(seed)).items():ET.SubElement(inc,'arg',name=k,value=v)
    ET.SubElement(launch,'node',pkg='rosbag',type='record',name='fixture_bag',output='screen',args='--lz4 -O '+str(root/'fixture.bag')+' /clock /astra/world_models /astra/world_links /astra/physics_contacts /astra/physics_metadata')
    probe=ET.SubElement(launch,'node',pkg='m2_gazebo',type='astra_evidence_fixture_probe.py',name='fixture_probe',required='true',output='screen',**{'launch-prefix':str(workspace/'.venv/bin/python')})
    ET.SubElement(probe,'param',name='output',value=str(root))
    (root/'fixture.launch').write_text(ET.tostring(launch,encoding='unicode'))
    bh=common.bindings(root)
    bound=json.loads((root/'bindings.json').read_text())
    launch_sha=common.sha(root/'fixture.launch')
    bound['resources'].append({'path':str(root/'fixture.launch'),'sha256':launch_sha})
    blob=stage/'code_blobs'/launch_sha
    if not blob.exists():
        blob.write_bytes((root/'fixture.launch').read_bytes());blob.chmod(0o444)
    if common.sha(blob)!=launch_sha:raise RuntimeError('fixture launch archive mismatch')
    common.atomic(root/'bindings.json',bound);bh=common.sha(root/'bindings.json')
    env=dict(os.environ);env.update(ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_LOG_DIR=str(root/'roslog'),ROS_HOME=str(root/'ros_home'),GAZEBO_MODEL_DATABASE_URI='')
    command=['roslaunch',str(root/'fixture.launch')]
    for port in (14820,14821):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    common.atomic(root/'attempt.json',{'run_id':rid,'seed':seed,'command':command,'bindings_sha256':bh,'started_utc':common.utc(),'wall_cap_s':90})
    common.append_csv(stage/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='physics_measurement_fixture',seed=seed,run_id=rid,status='consumed',source_binding=bh))
    proc=None;error=None;start=time.monotonic()
    try:
        with (root/'roslaunch.log').open('wb') as log:
            proc=subprocess.Popen(command,cwd=str(workspace),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            common.atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')',1)[1].split()[19]})
            while proc.poll() is None:
                common.atomic(root/'heartbeat.json',{'utc':common.utc(),'pid':proc.pid,'elapsed_wall_s':time.monotonic()-start})
                if time.monotonic()-start>90:raise TimeoutError('fixture exceeded wall cap')
                time.sleep(1)
    except BaseException as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        if proc is not None:common.stop_group(proc,root)
    report=json.loads((root/'probe_report.json').read_text()) if (root/'probe_report.json').exists() else None
    exit_code=proc.returncode if proc else None
    bag_path=root/'fixture.bag';bag_topics=set()
    if bag_path.is_file():
        info=subprocess.run(['rosbag','info','--yaml',str(bag_path)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=20)
        (root/'bag_info.stdout.yaml').write_text(info.stdout);(root/'bag_info.stderr.log').write_text(info.stderr)
        if info.returncode==0:bag_topics={r['topic'] for r in yaml.safe_load(info.stdout).get('topics',[]) if r.get('messages',0)>0}
    missing=sorted({'/clock','/astra/world_models','/astra/world_links','/astra/physics_contacts','/astra/physics_metadata'}-bag_topics)
    receipt={'run_id':rid,'seed':seed,'method':'measurement_fixture','split':'bootstrap_smoke','status':'completed' if report and not error and exit_code==0 else 'infrastructure_failure','pass':bool(report and report['pass'] and not error and exit_code==0 and not missing),'report':report,'error':error,'exit_code':exit_code,'missing_bag_topics':missing,'bindings_sha256':bh,'ended_utc':common.utc(),'wall_elapsed_s':time.monotonic()-start,'performance_evidence':False}
    for name in ('probe_report.json','probe_raw.json','fixture.bag','fixture.launch'):
        if (root/name).exists():receipt[name+'_sha256']=common.sha(root/name)
    common.atomic(root/'receipt.json',receipt)
    for ledger in ('run_manifest.csv','EXPERIMENTS.csv'):common.append_csv(stage/ledger,dict(receipt,raw_path=str(root),receipt_path=str(root/'receipt.json'),command_json=json.dumps(command)))
    print(json.dumps(receipt,indent=2))
    return 0 if receipt['pass'] else 3


def main():
    stage=common.STAGE;rid='physics_evidence_q01';root=stage/'runs'/rid
    lock=(stage/'queue/simulation.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if root.exists():raise RuntimeError('immutable allocated identity exists; do not retry: '+str(root))
    try:return _execute()
    except BaseException as error:
        if not root.exists():raise
        receipt={'run_id':rid,'seed':260926003,'method':'measurement_fixture','split':'bootstrap_smoke',
                 'status':'infrastructure_failure','pass':False,'error':type(error).__name__+': '+str(error),
                 'ended_utc':common.utc(),'performance_evidence':False,'retry_allowed':False}
        common.atomic(root/'receipt.json',receipt)
        with (stage/'seed_registry.csv').open() as stream:
            registered=any(r['run_id']==rid for r in csv.DictReader(stream))
        if not registered:common.append_csv(stage/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='physics_measurement_fixture',seed=260926003,run_id=rid,status='consumed_preparation_failure'))
        for ledger in ('run_manifest.csv','EXPERIMENTS.csv'):common.append_csv(stage/ledger,dict(receipt,raw_path=str(root),receipt_path=str(root/'receipt.json')))
        raise


if __name__=='__main__':raise SystemExit(main())
