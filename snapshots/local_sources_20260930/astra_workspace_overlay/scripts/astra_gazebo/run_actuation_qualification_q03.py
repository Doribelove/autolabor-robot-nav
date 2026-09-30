#!/usr/bin/env python3
"""New immutable identity for a 36-second turning/braking qualification."""
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
    rid='actuation_q03';seed=260926006
    root=stage/'runs'/rid;root.mkdir()
    common.atomic(root/'allocated.json',{'run_id':rid,'seed':seed,'attempt':1,'started_utc':common.utc(),'purpose':'measurement_fixture','retry_allowed':False})
    contract_path=stage/'contracts/actuation_q03.yaml'
    contract=yaml.safe_load(contract_path.read_text())
    world=contract['world_fixture']['sdf']
    (root/'world.sdf').write_text(world)
    (root/'actuation_contract.yaml').write_bytes(contract_path.read_bytes())
    (root/'scene_manifest.yaml').write_text(yaml.safe_dump({'run_id':rid,'seed':seed,'expected_models':['ground_plane','autolabor_m2'],'purpose':'actuation_measurement_fixture','world_contract_sha256':common.sha(contract_path),'duration_s':contract['total_schedule_s']}))
    launch=ET.Element('launch')
    inc=ET.SubElement(launch,'include',file='$(find m2_gazebo)/launch/astra_spawn_hold_q03.launch')
    params=contract['effective_plugin_parameters']
    options=dict(world=str(root/'world.sdf'),paused='true',gui='false',use_sim_time='true',seed=str(seed),server_required='true',x='0',y='0',z='0',yaw='0',enable_actuator_dynamics='true')
    mapping={'speed_time_constant':'speedTimeConstant','steering_time_constant':'steeringTimeConstant',
             'max_acceleration':'maxAcceleration','max_deceleration':'maxDeceleration',
             'max_brake_deceleration':'maxBrakeDeceleration','max_emergency_deceleration':'maxEmergencyDeceleration',
             'max_steering_rate':'maxSteeringRate','command_delay':'commandDelay','command_jitter':'commandJitter'}
    options.update({name:str(params[key]) for name,key in mapping.items()})
    for k,v in options.items():ET.SubElement(inc,'arg',name=k,value=v)
    ET.SubElement(launch,'node',pkg='topic_tools',type='relay',name='actuation_command_relay',args='/cmd_vel_navigation /cmd_vel',output='screen')
    topics='/clock /astra/world_models /astra/world_links /astra/physics_contacts /astra/physics_metadata /odom /joint_states /cmd_vel_navigation /cmd_vel /scan /tf /tf_static /m2_driver/wheel_angle /m2_driver/applied_speed /m2_driver/control_timeout /m2_driver/command_activation_latency'
    ET.SubElement(launch,'node',pkg='rosbag',type='record',name='fixture_bag',output='screen',args='--lz4 -O '+str(root/'fixture.bag')+' '+topics)
    probe=ET.SubElement(launch,'node',pkg='m2_gazebo',type='astra_actuation_probe_q02.py',name='fixture_probe',required='true',output='screen',**{'launch-prefix':str(workspace/'.venv/bin/python')})
    for k,v in dict(output=str(root),contract_path=str(root/'actuation_contract.yaml'),world_path=str(root/'world.sdf'),seed=str(seed),run_id=rid).items():ET.SubElement(probe,'param',name=k,value=v)
    (root/'fixture.launch').write_text(ET.tostring(launch,encoding='unicode'))
    bh=common.bindings(root)
    bound=json.loads((root/'bindings.json').read_text())
    launch_sha=common.sha(root/'fixture.launch')
    bound['resources'].append({'path':str(root/'fixture.launch'),'sha256':launch_sha})
    blob=stage/'code_blobs'/launch_sha
    if not blob.exists():
        blob.write_bytes((root/'fixture.launch').read_bytes());blob.chmod(0o444)
    if common.sha(blob)!=launch_sha:raise RuntimeError('fixture launch archive mismatch')
    copy_path=root/'actuation_contract.yaml';copy_sha=common.sha(copy_path)
    bound['resources'].append({'path':str(copy_path),'sha256':copy_sha})
    copy_blob=stage/'code_blobs'/copy_sha
    if not copy_blob.exists():
        copy_blob.write_bytes(copy_path.read_bytes());copy_blob.chmod(0o444)
    if common.sha(copy_blob)!=copy_sha:raise RuntimeError('copied actuation contract archive mismatch')
    common.atomic(root/'bindings.json',bound);bh=common.sha(root/'bindings.json')
    env=dict(os.environ);env.update(ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_LOG_DIR=str(root/'roslog'),ROS_HOME=str(root/'ros_home'),GAZEBO_MODEL_DATABASE_URI='')
    command=['roslaunch',str(root/'fixture.launch')]
    for port in (14820,14821):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    common.atomic(root/'attempt.json',{'run_id':rid,'seed':seed,'command':command,'bindings_sha256':bh,'started_utc':common.utc(),'wall_cap_s':180})
    common.append_csv(stage/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='actuation_measurement_fixture',seed=seed,run_id=rid,status='consumed',source_binding=bh))
    proc=None;error=None;start=time.monotonic()
    try:
        with (root/'roslaunch.log').open('wb') as log:
            proc=subprocess.Popen(command,cwd=str(workspace),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            common.atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')',1)[1].split()[19]})
            while proc.poll() is None:
                common.atomic(root/'heartbeat.json',{'utc':common.utc(),'pid':proc.pid,'elapsed_wall_s':time.monotonic()-start})
                if time.monotonic()-start>180:raise TimeoutError('fixture exceeded wall cap')
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
    missing=sorted(set(topics.split())-bag_topics)
    receipt={'run_id':rid,'seed':seed,'method':'measurement_fixture','split':'bootstrap_smoke','status':'completed' if report and not report.get('terminal_exception') and not error and exit_code==0 else 'infrastructure_failure','pass':bool(report and report['pass'] and not error and exit_code==0 and not missing),'report':report,'error':error,'exit_code':exit_code,'missing_bag_topics':missing,'bindings_sha256':bh,'ended_utc':common.utc(),'wall_elapsed_s':time.monotonic()-start,'performance_evidence':False}
    for name in ('probe_report.json','probe_raw.json','probe_raw.jsonl','fixture.bag','fixture.launch','actuation_contract.yaml'):
        if (root/name).exists():receipt[name+'_sha256']=common.sha(root/name)
    common.atomic(root/'receipt.json',receipt)
    for ledger in ('run_manifest.csv','EXPERIMENTS.csv'):common.append_csv(stage/ledger,dict(receipt,raw_path=str(root),receipt_path=str(root/'receipt.json'),command_json=json.dumps(command)))
    print(json.dumps(receipt,indent=2))
    return 0 if receipt['pass'] else 3


def main():
    stage=common.STAGE;rid='actuation_q03';root=stage/'runs'/rid
    lock=(stage/'queue/simulation.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if root.exists():raise RuntimeError('immutable allocated identity exists; do not retry: '+str(root))
    try:return _execute()
    except BaseException as error:
        if not root.exists():raise
        receipt={'run_id':rid,'seed':260926006,'method':'measurement_fixture','split':'bootstrap_smoke',
                 'status':'infrastructure_failure','pass':False,'error':type(error).__name__+': '+str(error),
                 'ended_utc':common.utc(),'performance_evidence':False,'retry_allowed':False}
        common.atomic(root/'receipt.json',receipt)
        with (stage/'seed_registry.csv').open() as stream:
            registered=any(r['run_id']==rid for r in csv.DictReader(stream))
        if not registered:common.append_csv(stage/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='actuation_measurement_fixture',seed=260926006,run_id=rid,status='consumed_preparation_failure'))
        for ledger in ('run_manifest.csv','EXPERIMENTS.csv'):common.append_csv(stage/ledger,dict(receipt,raw_path=str(root),receipt_path=str(root/'receipt.json')))
        raise


if __name__=='__main__':raise SystemExit(main())
