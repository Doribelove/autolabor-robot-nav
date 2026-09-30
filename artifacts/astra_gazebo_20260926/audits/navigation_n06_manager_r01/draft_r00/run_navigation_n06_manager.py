#!/usr/bin/env python3
"""Future exact-release N06 manager. Default and --verify are read-only.

No prepare, seed-registration, release-writing, retry or resume command exists.
Supervisor child exit, manager return and host-shell status are separate facts.
"""
import argparse
import copy
import datetime
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

import navigation_n06_integrity as q
import navigation_n06_process as processes
import navigation_evidence_n06 as evidence

STAGE=q.STAGE;WORKSPACE=q.prepared_api.WORKSPACE;NAME=q.NAME
QUEUE=STAGE/'queue';LEDGER=QUEUE/'navigation_n06_ledger.json';EVENTS=QUEUE/'navigation_n06_events'

def utc():return datetime.datetime.now(datetime.timezone.utc).isoformat()

def persist(state,event):
    sequence=state['event_sequence']+1
    next_state=copy.deepcopy(state);next_state['event_sequence']=sequence
    previous=None if sequence==1 else q.binding(EVENTS/('%04d.json'%(sequence-1)))
    record={'schema':'astra_navigation_n06_state_event_v1','sequence':sequence,'previous_event':previous,'utc':utc(),'event':event,'state':next_state}
    q.publish(EVENTS/('%04d.json'%sequence),record)
    q.publish(LEDGER,next_state,replace=LEDGER.exists())
    state.clear();state.update(next_state)

def replay_state(data):
    files=sorted(EVENTS.glob('*.json')) if EVENTS.exists() else []
    state=None;previous=None
    for index,path in enumerate(files,1):
        event=q.strict_json(q.regular_bytes(path))
        if path.name!='%04d.json'%index or event['sequence']!=index or event['previous_event']!=previous or event['state']['event_sequence']!=index:raise ValueError('noncontiguous/broken state event chain')
        state=event['state']
        if state['stage']!=NAME or state['release_sha256']!=data['release_binding']['sha256']:raise ValueError('state event identity differs')
        if len(state['units'])!=8 or any(any(not q.same_typed(u[k],r[k]) for k in r) for u,r in zip(state['units'],data['rows'])):raise ValueError('state event roster differs')
        previous=q.binding(path)
    if LEDGER.exists():
        pointer=q.strict_json(q.regular_bytes(LEDGER))
        if state is None or pointer['event_sequence']>state['event_sequence']:raise ValueError('ledger has no immutable event')
        selected=q.strict_json(q.regular_bytes(EVENTS/('%04d.json'%pointer['event_sequence'])))['state']
        if not q.same_typed(pointer,selected):raise ValueError('ledger pointer differs from immutable event')
    return state

def live_preflight(data):
    if Path(os.environ.get('THESIS_WS','')).resolve()!=WORKSPACE:raise ValueError('source isolated astra_env first')
    if Path(sys.executable).resolve()!=(WORKSPACE/'.venv/bin/python').resolve():raise ValueError('manager interpreter differs from reviewed runtime')
    if LEDGER.exists() or EVENTS.exists() or any((STAGE/'runs'/u['run_id']).exists() for u in data['rows']):raise ValueError('N06 identity already exists: retry/resume prohibited')
    q.check_registry(q.regular_bytes(STAGE/'seed_registry.csv'),data['registry_baseline'],data['release_binding']['sha256'],data['rows'])
    import run_smoke_a02 as common
    predecessor=q.json_bound(data['release']['predecessor_ledger'])
    for unit in predecessor['units']:
        if common.owned_processes(STAGE/'runs'/unit['run_id']):raise ValueError('predecessor still has owned live processes')
    processes.assert_ports_free()

def terminal_receipt(unit,release_sha,*,report,error,exit_code,root,started,elapsed):
    root=Path(root);integrity=not error and report is not None and report.get('capture_integrity_pass')is True
    process=q.strict_json(q.regular_bytes(root/'process.json')) if (root/'process.json').exists() else {}
    assessment=q.binding(root/'assessment.json') if (root/'assessment.json').exists() else None
    metadata={'started_utc':started,'ended_utc':utc(),'exit_code':'' if exit_code is None else exit_code,
        'pid':process.get('pid',''),'process_group':process.get('pgid',''),'command_json':json.dumps(['/opt/ros/noetic/bin/roslaunch',str(root/'fixture.launch')]),
        'cwd':str(WORKSPACE),'heartbeat_path':str(root/'heartbeat.json'),'receipt_path':str(root/'receipt.json'),
        'bindings_sha256':release_sha,'code_sha256':q.binding(__file__)['sha256'],'config_sha256':unit['contract']['sha256'],
        'model_sha256':'','report_path':'' if assessment is None else assessment['path'],
        'raw_path':str(root/'collector_raw.json') if (root/'collector_raw.json').exists() else ''}
    return {'stage':NAME,'run_id':unit['run_id'],'seed':unit['seed'],'method':unit['method'],'attempt':1,'release_sha256':release_sha,
        'integrity_pass':bool(integrity),'task_outcome':None if report is None else (report.get('task') or {}).get('task_phase'),
        'error':error,'roslaunch_exit_code':exit_code,'wall_elapsed_s':elapsed,'assessment':assessment,
        'administrative_metadata':metadata,'retry_allowed':False,'host_shell_exit_status':'unknown'}

def execute_unit(unit,side,data,session,stop,parent_identity):
    import run_passive_sweep_q05 as passive
    root=STAGE/'runs'/unit['run_id'];started=utc();wall=time.monotonic();proc=None;report=None;error=None;code=None
    nonce=secrets.token_hex(24);root.mkdir()
    q.publish(root/'allocated.json',dict(run_id=unit['run_id'],seed=unit['seed'],method=unit['method'],attempt=1,started_utc=started,release_sha256=data['release_binding']['sha256']))
    q.publish(root/'cleanup_owner.json',{'worker_pid':os.getpid(),'worker_start_ticks':processes.proc_row(os.getpid())['start_ticks'],
        'child_nonce':nonce,'supervisor':parent_identity,'purpose':'noncredential ownership token; child environment only'})
    value=q.json_bound(unit['contract'])
    try:
        for key,name in (('contract','contract.json'),('launch','fixture.launch'),('world','world.sdf'),('robot_description','robot_description.xml'),('gate_expectation','gate_expectation.json')):
            q.publish(root/name,q.read_bound(unit[key]))
        q.publish(root/'scene_manifest.yaml',q.read_bound(value['scene_manifest']))
        q.publish(root/'shared_bundle_binding.json',value['shared_bundle'])
        q.publish(root/'conversion_manifest.json',q.read_bound(side['conversion_manifest']))
        q.publish(root/'converted_robot.sdf',q.read_bound(side['converted_sdf']))
        for item in data['release']['resources']:passive.archive(root,item)
        passive.archive(root,data['release_binding']);session.recheck()
        q.check_registry(q.regular_bytes(STAGE/'seed_registry.csv'),data['registry_baseline'],data['release_binding']['sha256'],data['rows'])
        if stop.signal or not processes.checked_parent(**parent_identity):raise RuntimeError('manager interrupted or supervisor lost before launch')
        processes.assert_ports_free()
        env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:14830',GAZEBO_MASTER_URI='http://127.0.0.1:14831',
            ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_LOG_DIR=str(root/'roslog'),
            ROS_HOME=str(root/'ros_home'),GAZEBO_MODEL_DATABASE_URI='',ASTRA_N06_CHILD_NONCE=nonce)
        with (root/'roslaunch.log').open('xb') as log:
            proc=subprocess.Popen(['/opt/ros/noetic/bin/roslaunch',str(root/'fixture.launch')],cwd=str(WORKSPACE),env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            identity=processes.proc_row(proc.pid)
            if identity is None:raise RuntimeError('launch process exited before identity capture')
            q.publish(root/'process.json',identity)
            while proc.poll() is None:
                if stop.signal:raise RuntimeError('manager_signal:'+str(stop.signal))
                if not processes.checked_parent(**parent_identity):raise RuntimeError('supervisor identity lost')
                for marker,ack in (('ready_for_maps.json','maps_pre_ready.json'),('ready_for_maps_final.json','maps_final_ready.json')):
                    if (root/marker).exists() and not (root/ack).exists():evidence.capture_maps(root,root/marker,proc)
                if time.monotonic()-wall>value['wall_cap_s']:raise TimeoutError('whole attempt wall cap')
                q.publish(root/'heartbeat.json',{'utc':utc(),'pid':proc.pid,'elapsed_wall_s':time.monotonic()-wall},replace=True);time.sleep(.1)
            code=proc.returncode
    except BaseException as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        try:processes.cleanup(proc,root,nonce)
        except BaseException as exc:error=(error or '')+'; cleanup:'+repr(exc)
    try:
        if error or code!=0:raise RuntimeError('launch/preparation/cleanup failed: '+str(error)+' returncode='+str(code))
        session.recheck();evidence.closure(root,data['release'],data['review'],value);report=evidence.assess(root,value);session.recheck()
        if stop.signal:raise RuntimeError('manager interrupted during assessment')
        if time.monotonic()-wall>value['wall_cap_s']:raise TimeoutError('assessment-inclusive wall cap')
    except BaseException as exc:error=(error or '')+'; assessment:'+repr(exc)
    receipt=terminal_receipt(unit,data['release_binding']['sha256'],report=report,error=error,exit_code=code,root=root,started=started,elapsed=time.monotonic()-wall)
    q.publish(root/'receipt.json',receipt);return q.binding(root/'receipt.json'),receipt

def worker(path,sha,parent_pid,parent_ticks,lock_fd):
    parent_identity={'pid':parent_pid,'start_ticks':parent_ticks}
    if os.getppid()!=parent_pid or not processes.checked_parent(**parent_identity):raise ValueError('actual supervisor parent required')
    lockstat=os.fstat(lock_fd);expected=(QUEUE/'simulation.lock').stat()
    if (lockstat.st_dev,lockstat.st_ino)!=(expected.st_dev,expected.st_ino):raise ValueError('inherited shared simulation lock differs')
    session=q.VerifiedSession(path,sha);data=session.recheck();live_preflight(data);processes.enable_subreaper()
    EVENTS.mkdir();q.fsync_dir(QUEUE);state=q.initial_state(sha,data['rows']);persist(state,'one_shot_allocation')
    failure=None
    with processes.StopRequest() as stop:
        try:
            for index,(unit,side) in enumerate(zip(data['prepared']['units'],data['conversions']['units'])):
                session.recheck()
                if stop.signal or not processes.checked_parent(**parent_identity):raise RuntimeError('interrupt/supervisor loss before next allocation')
                q.consume(state,index);persist(state,'consume_identity_before_any_launch')
                bound,receipt=execute_unit(unit,side,data,session,stop,parent_identity)
                q.close_unit(state,index,bound,receipt);persist(state,'unit_receipt_closed')
                if not receipt['integrity_pass']:break
        except BaseException as exc:failure=type(exc).__name__+': '+str(exc)
        finally:
            q.terminal(state,failure);persist(state,'terminal_retire_every_unrun_identity')
    complete=state['consumed_units']==8 and all(r.get('integrity_pass')is True for r in state['units']) and state['infrastructure_stop'] is None
    code=0 if complete else 3
    q.publish(QUEUE/'navigation_n06_manager_return.json',{'stage':NAME,'returned_code':code,'manager_pid':os.getpid(),
        'release_sha256':sha,'ledger':q.binding(LEDGER),'utc':utc(),'authority':'worker function returned after durable terminal ledger; not process exit','host_shell_exit_status':'unknown'})
    return code

def reconcile_worker_exit(data,observed_code):
    """Supervisor closes missing receipts after abnormal worker termination.

    This is an infrastructure-failure record only; it never invents physics
    results or resumes an allocated identity.
    """
    state=replay_state(data);actions=[]
    if state is None:
        EVENTS.mkdir(exist_ok=True);q.fsync_dir(QUEUE)
        state=q.initial_state(data['release_binding']['sha256'],data['rows'])
        q.terminal(state,'worker exited before durable allocation ledger; all identities retired without motion claim')
        persist(state,'supervisor_retire_unstarted_budget_after_observed_worker_exit')
    for index,row in enumerate(state['units']):
        if row['status']!='consumed':continue
        root=STAGE/'runs'/row['run_id'];owner=root/'cleanup_owner.json'
        if owner.exists():
            identity=q.strict_json(q.regular_bytes(owner))
            if processes.scan(os.getpid(),identity['child_nonce'],root):
                # Never replace worker cleanup evidence: write supervisor's
                # audit into a separate subdirectory with the same marker root.
                if (root/'cleanup.json').exists():raise RuntimeError('live survivor after saved cleanup; independent closure required')
                processes.cleanup(None,root,identity['child_nonce'])
        if row['receipt'] is None:
            root.mkdir(exist_ok=True);receipt_path=root/'receipt.json'
            if receipt_path.exists():receipt=q.strict_json(q.regular_bytes(receipt_path))
            else:
                unit=data['prepared']['units'][index]
                receipt=terminal_receipt(unit,data['release_binding']['sha256'],report=None,error='worker exited before durable receipt; supervisor observed '+str(observed_code),
                    exit_code=None,root=root,started='unknown',elapsed=0.)
                q.publish(receipt_path,receipt)
            q.close_unit(state,index,q.binding(receipt_path),receipt);actions.append(row['run_id'])
    pointer_stale=not LEDGER.exists() or not q.same_typed(q.strict_json(q.regular_bytes(LEDGER)),state)
    if observed_code!=0 or state['status']!='terminal' or actions or pointer_stale:
        q.terminal(state,'supervisor observed interrupted/incomplete worker '+str(observed_code));persist(state,'supervisor_terminal_infrastructure_closure')
    return {'ledger_present':True,'missing_receipts_closed':actions,'ledger':q.binding(LEDGER)}

def supervise(path,sha):
    data=q.verify_release(path,sha);live_preflight(data)
    if (QUEUE/'navigation_n06_supervisor_start.json').exists():raise ValueError('supervisor identity already consumed; no retry')
    with (QUEUE/'simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);live_preflight(data);processes.enable_subreaper()
        me=processes.proc_row(os.getpid());q.publish(QUEUE/'navigation_n06_supervisor_start.json',{'stage':NAME,'release_sha256':sha,'pid':os.getpid(),
            'start_ticks':me['start_ticks'],'utc':utc(),'host_shell_exit_status':'unknown'})
        command=[sys.executable,str(Path(__file__).resolve()),'--worker','--release',str(Path(path).resolve()),'--release-sha256',sha,
            '--parent-pid',str(os.getpid()),'--parent-start-ticks',me['start_ticks'],'--lock-fd',str(lock.fileno())]
        with (QUEUE/'navigation_n06_worker.log').open('xb') as log,processes.StopRequest() as stop:
            proc=subprocess.Popen(command,cwd=str(WORKSPACE),env=dict(os.environ),stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True,pass_fds=(lock.fileno(),))
            child=processes.proc_row(proc.pid);forwarded=False
            if child is None:code=proc.wait()
            else:
                q.publish(QUEUE/'navigation_n06_worker_identity.json',dict(child,argv=command,supervisor_pid=os.getpid()))
                while proc.poll() is None:
                    if stop.signal and not forwarded:
                        processes.signal_pinned(proc.pid,child['start_ticks'],stop.signal,
                            lambda row:processes.descendant_chain(row['pid'],os.getpid()) is not None)
                        forwarded=True
                    time.sleep(.1)
                code=proc.wait()
        reconciliation=None;reconcile_error=None
        try:reconciliation=reconcile_worker_exit(data,code)
        except BaseException as exc:reconcile_error=repr(exc)
        returned=None;return_error=None
        try:
            returned=q.strict_json(q.regular_bytes(QUEUE/'navigation_n06_manager_return.json'))
            if returned['stage']!=NAME or returned['release_sha256']!=sha or returned['manager_pid']!=proc.pid or returned['returned_code']!=code or returned['ledger']!=q.binding(LEDGER):raise ValueError('manager return/observed process exit/terminal ledger disagreement')
        except BaseException as exc:return_error=repr(exc)
        q.publish(QUEUE/'navigation_n06_supervisor_wait.json',{'stage':NAME,'release_sha256':sha,'worker_pid':proc.pid,
            'observed_child_returncode':code,'termination_signal':-code if code<0 else None,'forwarded_supervisor_signal':stop.signal,
            'reconciliation':reconciliation,'reconciliation_error':reconcile_error,'manager_return':returned,'manager_return_error':return_error,'utc':utc(),
            'authority':'Python Popen.wait observation of this child only','host_shell_exit_status':'unknown'})
        return 0 if code==0 and reconcile_error is None and return_error is None else 3

def main():
    parser=argparse.ArgumentParser(description=__doc__);mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--verify',action='store_true');mode.add_argument('--execute',action='store_true');mode.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--release',type=Path);parser.add_argument('--release-sha256');parser.add_argument('--parent-pid',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--parent-start-ticks',help=argparse.SUPPRESS);parser.add_argument('--lock-fd',type=int,help=argparse.SUPPRESS);args=parser.parse_args()
    if not (args.verify or args.execute or args.worker):print(json.dumps({'stage':NAME,'execution_authorized':False,'allocated_budget':0,'required_release_absent_by_default':True}));return 0
    if args.release is None or args.release_sha256 is None:raise ValueError('exact release path and SHA required')
    if args.verify:
        data=q.verify_release(args.release,args.release_sha256);print(json.dumps({'status':'release_bytes_verified_readonly','units':len(data['rows']),'allocation_performed':False}));return 0
    if args.execute:return supervise(args.release,args.release_sha256)
    if None in (args.parent_pid,args.parent_start_ticks,args.lock_fd):raise ValueError('supervisor-only worker identity required')
    return worker(args.release,args.release_sha256,args.parent_pid,args.parent_start_ticks,args.lock_fd)

if __name__=='__main__':raise SystemExit(main())
