#!/usr/bin/env python3
"""Fresh development matrix runner; no final-test or historical identity entry."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml
from run_smoke_a02 import (WORKSPACE, STAGE, CHECKPOINT, sha, atomic, utc,
                          append_csv, bindings, stop_group)


def load_scene(row):
    world,manifest=Path(row['world_path']),Path(row['manifest_path'])
    if sha(world)!=row['world_sha256'] or sha(manifest)!=row['manifest_sha256']:
        raise RuntimeError('frozen scene bytes changed')
    scene=yaml.safe_load(manifest.read_text())
    if scene['seed']!=row['seed'] or scene['layout_id']!=row['layout_id'] or scene['split']!='development':
        raise RuntimeError('scene identity or split mismatch')
    for key in ('scene_id','environment_version','input_variant_id','parent_geometry_id'):
        if row[key]!=scene[key]:raise RuntimeError('scene/schedule mismatch: '+key)
    if row['geometry_sha256']!=scene['canonical_geometry_sha256']:raise RuntimeError('geometry identity mismatch')
    if scene['environment_version']!='astra_common_environment_v4_routegate_ego_clock_fault_guard_z0_evidence':
        raise RuntimeError('unsupported common environment version')
    tree=ET.fromstring(world.read_text()).find('world')
    if tree is None or tree.find("plugin[@filename='libastra_evidence_plugin.so']") is None:
        raise RuntimeError('source-stamped physics evidence plugin missing')
    actual_actors=[]
    for model in tree.findall('model'):
        plugins=model.findall('plugin')
        if model.get('name').startswith('benchmark_actor_'):
            actual_actors.append(model.get('name'))
            if len(plugins)!=1 or plugins[0].get('filename')!='libastra_reactive_actor_plugin.so':
                raise RuntimeError('new actor plugin required')
            plugin=plugins[0]
            if plugin.find('start_delay') is not None or plugin.find('activation_ego_x') is not None:
                raise RuntimeError('legacy actor gate parameters forbidden')
            actor=next(a for a in scene['actors'] if a['model_name']==model.get('name'))
            if abs(float(plugin.findtext('gate_delay'))-actor['gate_delay_s'])>1e-12:
                raise RuntimeError('actor gate delay differs from declared scene')
    if sorted(actual_actors)!=sorted(a['model_name'] for a in scene['actors']):raise RuntimeError('actor roster mismatch')
    route=scene['route']
    if route['start']!=[-4.,0.,0.] or route['goal']!=[5.,0.,0.] or route['spawn']!=[-5.4,0.,0.]:
        raise RuntimeError('this runner requires the declared yaw-zero route; no implicit SE2 support')
    return world.read_text(),scene


def _execute(row):
    root=STAGE/'runs'/row['run_id']
    if root.exists():
        if (root/'receipt.json').is_file():
            receipt=json.loads((root/'receipt.json').read_text())
            if any(receipt.get(k)!=row.get(k) for k in ('run_id','method','seed','split','layout_id','scene_id','world_sha256','manifest_sha256','candidate_id','phase_id','input_sync_mode')):raise RuntimeError('existing receipt identity mismatch')
            if receipt.get('report_sha256') and sha(root/'episode_report.yaml')!=receipt['report_sha256']:raise RuntimeError('existing report changed')
            if receipt.get('bindings_sha256') and sha(root/'bindings.json')!=receipt['bindings_sha256']:raise RuntimeError('existing bindings changed')
            return receipt
        raise RuntimeError('partial identity requires explicit live-process audit, never blindly restart: '+str(root))
    root.mkdir()
    atomic(root/'allocated.json',dict(row,started_utc=utc(),attempt=1,
                                    status='identity_consumed_preparing',retry_allowed=False))
    text,scene=load_scene(row);(root/'world.sdf').write_text(text)
    (root/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
    bh=bindings(root)
    # Bind the exact schedule and stage contract actually selected by main.
    bound=json.loads((root/'bindings.json').read_text())
    for key in ('contract_path','schedule_path'):
        source=Path(row[key]);digest=sha(source)
        if digest!=row[key.replace('_path','_sha256')]:raise RuntimeError('selected protocol changed')
        bound['resources'].append({'path':str(source),'sha256':digest})
        blob=STAGE/'code_blobs'/digest
        if not blob.exists():blob.write_bytes(source.read_bytes());blob.chmod(0o444)
        if sha(blob)!=digest:raise RuntimeError('protocol archive mismatch')
    atomic(root/'bindings.json',bound);bh=sha(root/'bindings.json')
    env=dict(os.environ)
    env.update(THESIS_WS=str(WORKSPACE),ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',
               ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_HOME=str(root/'ros_home'),ROS_LOG_DIR=str(root/'roslog'),
               GAZEBO_MODEL_DATABASE_URI='',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',MKL_NUM_THREADS='2')
    for port in (14820,14821):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('127.0.0.1',port))
    command=['roslaunch','m2_gazebo','astra_diagnostic_episode.launch']
    args=dict(world=root/'world.sdf',method=row['method'],scene_manifest=root/'scene_manifest.yaml',
              scene_id=row['scene_id'],seed=row['seed'],report_path=root/'episode_report.yaml',
              stage=row['phase_id'],input_sync_mode=row.get('input_sync_mode','legacy'),
              actor_checkpoint_path=CHECKPOINT,evidence_dir=root/'evidence',run_id=row['run_id'])
    command.extend(k+':='+str(v) for k,v in args.items())
    started=utc();start=time.monotonic()
    atomic(root/'attempt.json',dict(row,started_utc=started,command=command,cwd=str(WORKSPACE),
                                   bindings_sha256=bh,wall_limit_s=180,environment_overrides={k:env[k] for k in ('ROS_MASTER_URI','GAZEBO_MASTER_URI','ROS_HOME','ROS_LOG_DIR','ROS_IP','ROS_HOSTNAME','THESIS_WS')}))
    append_csv(STAGE/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='development',purpose=row['phase_id'],seed=row['seed'],run_id=row['run_id'],status='consumed',source_binding=bh))
    proc=None;reason=None;cleanup_pass=False
    try:
        with (root/'roslaunch.log').open('wb') as log:
            proc=subprocess.Popen(command,cwd=str(WORKSPACE),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')',1)[1].split()[19]})
            while proc.poll() is None:
                atomic(root/'heartbeat.json',{'utc':utc(),'pid':proc.pid,'wall_elapsed_s':time.monotonic()-start})
                if time.monotonic()-start>180:reason='infrastructure_wall_timeout';break
                time.sleep(2)
    except BaseException as error:
        reason=type(error).__name__+':'+str(error)
    finally:
        if proc is not None:stop_group(proc,root)
        cleanup_pass=True
    # A stopped actor after a clock/ego fault is invalid infrastructure even if
    # navigation later reports an apparent goal. Preserve the log and outcome.
    if (root/'roslaunch.log').exists() and 'AstraReactiveActorPlugin gate_fault' in (root/'roslaunch.log').read_text(errors='replace'):
        reason='actor_gate_fault'
    report=yaml.safe_load((root/'episode_report.yaml').read_text()) if (root/'episode_report.yaml').exists() else None
    result=report.get('result',{}) if report else {}
    evidence=json.loads((root/'evidence/status.json').read_text()) if (root/'evidence/status.json').exists() else {}
    raw_bag=root/'episode_report.yaml.inputs.bag'
    bag_info=None
    if raw_bag.exists():
        info=subprocess.run(['rosbag','info','--yaml',str(raw_bag)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=20)
        if info.returncode==0:
            bag_info=yaml.safe_load(info.stdout)
            (root/'bag_info.yaml').write_text(info.stdout)
    evidence['bag_recorded']=bool(bag_info and bag_info.get('messages',0)>0)
    evidence['bag_sha256']=sha(raw_bag) if raw_bag.exists() else None
    required_topics={'/clock','/scan','/odom','/tf','/tf_static','/joint_states',
                     '/cmd_vel_navigation','/cmd_vel','/gazebo/model_states','/gazebo/link_states',
                     '/astra/world_models','/astra/world_links','/astra/physics_contacts','/astra/physics_metadata',
                     '/m2_driver/wheel_angle','/m2_driver/applied_speed','/m2_driver/control_timeout'}
    bag_topics={t['topic'] for t in (bag_info or {}).get('topics',[]) if t.get('messages',0)>0}
    evidence['bag_missing_required_topics']=sorted(required_topics-bag_topics)
    receipt=dict(row,started_utc=started,ended_utc=utc(),wall_elapsed_s=time.monotonic()-start,
        exit_code=proc.returncode if proc else None,termination_detail=reason,bindings_sha256=bh,
        report_sha256=sha(root/'episode_report.yaml') if report else None,
        goal_reached=result.get('goal_reached',False),execution_integrity_pass=report.get('execution_integrity_pass',False) if report else False,
        legacy_restricted_elapsed_s=result.get('restricted_elapsed_s',22.),
        result=result,evidence=evidence,cleanup_pass=cleanup_pass,
        capture_complete=evidence.get('frames',0)>0 and evidence.get('state_samples',0)>0 and evidence['bag_recorded'] and not evidence['bag_missing_required_topics'],
        status='completed' if report and reason is None and proc is not None and proc.returncode==0 else 'infrastructure_failure',
        scientific_eligibility='development_diagnostic_never_formal')
    for name in ('robot_description.xml','effective_parameters.yaml'):
        path=root/'evidence'/name
        receipt[name+'_sha256']=sha(path) if path.is_file() else None
    atomic(root/'receipt.json',receipt)
    append_csv(STAGE/'EXPERIMENTS.csv',dict(receipt,split='development',candidate_id=row['candidate_id'],scene_id=row['scene_id'],layout_id=row['layout_id'],code_sha256=bh,model_sha256=sha(CHECKPOINT),report_path=str(root/'episode_report.yaml'),raw_path=str(root)))
    append_csv(STAGE/'run_manifest.csv',dict(receipt,pid=proc.pid if proc else None,process_group=proc.pid if proc else None,command_json=json.dumps(command),cwd=str(WORKSPACE),heartbeat_path=str(root/'heartbeat.json'),receipt_path=str(root/'receipt.json')))
    return receipt


def execute(row):
    root=STAGE/'runs'/row['run_id']
    # Existing partial identities need a live PID/start-tick audit, not another
    # attempt. Keep this check outside the new-attempt error handler.
    if root.exists() and not (root/'receipt.json').is_file():
        raise RuntimeError('partial identity requires live-process audit: '+str(root))
    existed=root.exists()
    try:return _execute(row)
    except BaseException as error:
        if existed:raise  # Never overwrite a corrupted retained receipt.
        if not root.exists():raise
        # This covers rendering, source/hash/port failures as well as cleanup.
        # Preparation failures are charged to the smoke identity conservatively.
        allocation=json.loads((root/'allocated.json').read_text()) if (root/'allocated.json').exists() else {}
        receipt=dict(row,status='infrastructure_failure',started_utc=allocation.get('started_utc'),ended_utc=utc(),
            exit_code=None,goal_reached=False,execution_integrity_pass=False,wall_elapsed_s=None,
            legacy_restricted_elapsed_s=22.,evidence={},termination_detail=type(error).__name__+':'+str(error),
            scientific_eligibility='development_diagnostic_never_formal')
        atomic(root/'receipt.json',receipt)
        with (STAGE/'seed_registry.csv').open() as f:registered=any(r['run_id']==row['run_id'] for r in csv.DictReader(f))
        if not registered:append_csv(STAGE/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='development',purpose=row['phase_id'],seed=row['seed'],run_id=row['run_id'],status='consumed_preparation_failure'))
        append_csv(STAGE/'EXPERIMENTS.csv',dict(receipt,candidate_id=row['candidate_id'],scene_id=row['scene_id'],raw_path=str(root)))
        append_csv(STAGE/'run_manifest.csv',dict(receipt,receipt_path=str(root/'receipt.json')))
        return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if Path(os.environ.get('THESIS_WS','')).resolve()!=WORKSPACE:raise RuntimeError('source isolated astra_env.sh')
    contract=json.loads(args.contract.read_text())
    if contract['split']!='development' or contract['phase_id'].startswith('formal'):
        raise ValueError('only new development is supported by this runner')
    if not contract.get('ready_for_diagnostic_execution'):raise ValueError('diagnostic preflight not sealed')
    if sha(Path(contract['schedule_path']))!=contract['schedule_sha256']:raise ValueError('schedule changed')
    rows=json.loads(Path(contract['schedule_path']).read_text())
    if len(rows)!=contract['episode_count'] or len({r['run_id'] for r in rows})!=len(rows):raise ValueError('schedule identity count mismatch')
    for row in rows:
        if not re.fullmatch(r'[a-z0-9_]+',row['run_id']) or not row['run_id'].startswith(contract['phase_id']+'_'):raise ValueError('invalid run identity')
        if row['split']!='development' or row['phase_id']!=contract['phase_id'] or type(row['seed']) is not int or not 263000000<=row['seed']<=263999999:raise ValueError('fresh development namespace required')
        if row['method'] not in ('pct_scope_rl','analytic_prior','actual_teb','dwa'):raise ValueError('unknown diagnostic method')
        if row.get('input_sync_mode','legacy') not in ('legacy','synchronized'):raise ValueError('unknown input factor')
        if row['method'] in ('actual_teb','dwa') and row.get('input_sync_mode','legacy')!='legacy':raise ValueError('M02 cannot gate external move_base publisher')
        load_scene(row)
    def rehash():
        for item in contract['resources']:
            if sha(Path(item['path']))!=item['sha256']:raise RuntimeError('contract resource changed: '+item['path'])
    rehash()
    if not args.execute:
        print(json.dumps({'phase':contract['phase_id'],'episodes':len(rows),'preflight':'pass_not_executed'}));return
    lock=(STAGE/'queue/simulation.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    results=[]
    for row in rows:
        rehash()
        row.update(contract_path=str(args.contract.resolve()),contract_sha256=sha(args.contract),
                   schedule_path=contract['schedule_path'],schedule_sha256=contract['schedule_sha256'])
        print('START '+row['run_id'],flush=True)
        receipt=execute(row);results.append(receipt)
        print(json.dumps({k:receipt.get(k) for k in ('run_id','status','goal_reached','execution_integrity_pass','capture_complete','termination_detail')}),flush=True)
        atomic(STAGE/'queue'/(contract['phase_id']+'_status.json'),{'utc':utc(),'finished':len(results),'total':len(rows),'last_run_id':row['run_id'],'last_status':receipt['status']})
        # Algorithmic failures are scientific observations, never removed.
        # Only pipeline/capture failure halts unconsumed rows for diagnosis.
        if receipt['status']!='completed' or not receipt.get('capture_complete') or receipt['evidence'].get('machine_pass_frames',0)==0:break
    atomic(STAGE/'audits'/(contract['phase_id']+'_runner_summary.json'),{'phase':contract['phase_id'],'results':results,'formal_result':False,'complete':len(results)==len(rows),'requires_independent_exact_evaluation':True})


if __name__=='__main__':main()
