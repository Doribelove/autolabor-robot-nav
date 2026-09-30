#!/usr/bin/env python3
"""Q04 isolated passive fixture runner. Default is read-only/offline preflight.

Execution requires a root-reviewed hash-bound release; this file creates no
authorization, registry reservation, training or implicit simulation action.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import passive_sweep_q04 as q
import run_smoke_a02 as common

def render_launch(root,value):
    launch=ET.Element('launch')
    ET.SubElement(launch,'param',name='/use_sim_time',value='true')
    ET.SubElement(launch,'param',name='/m2_gazebo/simulation_only',value='true')
    ET.SubElement(launch,'param',name='robot_description',textfile=str(root/'robot_description.xml'))
    inc=ET.SubElement(launch,'include',file='$(find gazebo_ros)/launch/empty_world.launch')
    for name,text in {'world_name':str(root/'world.sdf'),'gui':'false','paused':'true','use_sim_time':'true',
                     'server_required':'true','extra_gazebo_args':'--seed '+str(value['seed'])}.items():
        ET.SubElement(inc,'arg',name=name,value=text)
    ET.SubElement(launch,'node',pkg='robot_state_publisher',type='robot_state_publisher',name='m2_robot_state_publisher',output='screen')
    ET.SubElement(launch,'node',pkg='topic_tools',type='relay',name='passive_command_relay',args='/cmd_vel_navigation /cmd_vel',output='screen')
    for script,node in [('astra_passive_command_q04.py','passive_command'),('astra_passive_collector_q04.py','passive_collector')]:
        item=ET.SubElement(launch,'node',pkg='m2_gazebo',type=script,name=node,required='true',output='screen',
                          **{'launch-prefix':str(q.WORKSPACE/'.venv/bin/python')})
        ET.SubElement(item,'param',name='contract_path',value=str(root/'contract.json'))
        digest=__import__('hashlib').sha256((json.dumps(value,indent=2)+'\n').encode()).hexdigest()
        ET.SubElement(item,'param',name='contract_sha256',value=digest)
        if node=='passive_collector':ET.SubElement(item,'param',name='output',value=str(root))
    ET.SubElement(launch,'node',pkg='rosbag',type='record',name='fixture_bag',output='screen',
                  args='--lz4 --buffsize=512 -O '+str(root/'fixture.bag')+' '+' '.join(q.RECORD_TOPICS))
    return ET.tostring(launch,encoding='unicode')+'\n'

def prepare(output,kd,camera=True):
    output=Path(output)
    if output.exists():raise FileExistsError('prepared output exists; preserve exact earlier bytes')
    output.mkdir(parents=True)
    units=[]
    for run_id,seed,actor in q.FIXTURES:
        value=q.contract(run_id,kd,camera);q.validate_contract(value)
        directory=output/run_id;directory.mkdir();actual=q.STAGE/'runs'/run_id
        (directory/'contract.json').write_text(json.dumps(value,indent=2)+'\n')
        (directory/'world.sdf').write_text(q.world_sdf(value))
        (directory/'robot_description.xml').write_text(q.render_urdf(value))
        (directory/'fixture.launch').write_text(render_launch(actual,value))
        units.append({'run_id':run_id,'seed':seed,'attempt':1,'actor':actor,'steering_kd':kd,
                      'files':[q.binding(directory/name) for name in ('contract.json','world.sdf','robot_description.xml','fixture.launch')]})
    existing=[p for p in q.required_resources(camera) if p.is_file()]
    missing=[str(p) for p in q.required_resources(camera) if not p.is_file()]
    result={'stage':q.NAME,'status':'offline_prepared_no_authority','execution_authorized':False,
       'maximum_episodes':2,'steering_kd':kd,'units':units,'resources':[q.binding(p) for p in existing],
       'missing_resources':missing,'source_review_required':True,'runtime_qualification':False}
    common.atomic(output/'prepared.json',result)
    return result

def native_review_facts(review):
    native=json.loads(q.read_bound(review['native_engine_report']))
    rows=[json.loads(line) for line in q.read_bound(review['native_engine_records']).decode().splitlines()]
    source_manifest=json.loads(q.read_bound(review['upstream_source_manifest']))
    expected_ode=native['why_end_twist_is_not_a_bound']['native_library']['sha256']
    reviewed_ode=[x['sha256'] for x in review['engine_module_bindings'] if x['role']=='ode_integrator']
    native_pass=(reviewed_ode==[expected_ode] and len(rows)>=301 and
       all(x.get('finite_rotation_mode')==0 for x in rows[1:301]) and
       any(x.get('sha256')==review['native_engine_records']['sha256'] for x in native['native_probe_artifacts']))
    source_bindings=review['engine_source_bindings']
    for item in source_bindings:q.read_bound(item)
    upstream={x['sha256'] for x in source_manifest}
    source_pass=(native['upstream_source_manifest']['sha256']==review['upstream_source_manifest']['sha256'] and
      bool(source_bindings) and all(x['sha256'] in upstream for x in source_bindings) and
      review.get('gazebo_callback_order_review')=='BeforePhysicsUpdate then native integration then dirty-pose propagation then WorldUpdateEnd')
    return {'native_reference_pass':native_pass,'upstream_source_review_pass':source_pass,'expected_ode_sha256':expected_ode}


def verify_release(path,expected):
    release=json.loads(q.read_bound({'path':str(path),'sha256':expected}))
    if (release.get('stage')!=q.NAME or release.get('status')!='root_reviewed_release' or
        release.get('execution_authorized')is not True or type(release.get('maximum_episodes'))is not int or release['maximum_episodes']!=2):
        raise ValueError('root-reviewed Q04 release missing or invalid')
    prepared=json.loads(q.read_bound(release['prepared']))
    if prepared.get('missing_resources'):raise ValueError('prepared observer/other compiled resources missing')
    kd=prepared['steering_kd']
    decision=json.loads(q.read_bound(release['physical_parameter_decision']))
    if decision.get('resolved_steering_kd')!=kd or not decision.get('evidence_bindings'):
        raise ValueError('Kd must be resolved against actual predecessor evidence before release')
    for b in decision['evidence_bindings']:q.read_bound(b)
    resources=release['resources'];by_path={str(Path(b['path']).resolve()):b for b in resources}
    if len(by_path)!=len(resources):raise ValueError('duplicate release resources')
    required=q.required_resources(any(json.loads(q.read_bound(u['files'][0]))['camera'] for u in prepared['units']))
    if not {str(p.resolve()) for p in required}<=set(by_path):raise ValueError('active resource closure incomplete')
    for b in resources:q.read_bound(b)
    for b in prepared['resources']:
        if by_path.get(str(Path(b['path']).resolve()),{}).get('sha256')!=b['sha256']:raise ValueError('prepared/release resource SHA differs')
    review=json.loads(q.read_bound(release['source_review']))
    if review.get('schema')!='astra_passive_q04_source_review_v1' or review.get('status')!='independently_reviewed':
        raise ValueError('independent source review absent')
    reviewed={str(Path(x['path']).resolve()):x['sha256'] for x in review['resources']}
    for p in q.SOURCE_FILES:
        resolved=str((q.WORKSPACE/p).resolve())
        if reviewed.get(resolved)!=by_path[resolved]['sha256']:raise ValueError('source review does not bind '+p)
    if review.get('robot_pose_velocity_writes')!=[] or review.get('actor_updates')!='reviewed_bounded_Begin_updates':
        raise ValueError('authorized source writer closure not reviewed')
    module_roles=set()
    for item in review['engine_module_bindings']:
        q.read_bound(item)
        if item['role'] in module_roles:raise ValueError('duplicate reviewed ELF role')
        module_roles.add(item['role'])
        if by_path.get(str(Path(item['path']).resolve()),{}).get('sha256')!=item['sha256']:raise ValueError('reviewed ELF absent from exact release closure')
    expected_roles=set(q.ELF_ROLES.values())-({'camera'} if not any(json.loads(q.read_bound(u['files'][0]))['camera'] for u in prepared['units']) else set())
    if not expected_roles<=module_roles:raise ValueError('reviewed ELF roles incomplete')
    if review.get('gazebo_callback_order_review')!='BeforePhysicsUpdate then native integration then dirty-pose propagation then WorldUpdateEnd':raise ValueError('callback order review absent')
    if not review.get('engine_source_bindings'):raise ValueError('reviewed native source bindings absent')
    for item in review['engine_source_bindings']:q.read_bound(item)
    for key in ('native_engine_report','native_engine_records','upstream_source_manifest'):
        q.read_bound(review[key])
    native_facts=native_review_facts(review)
    if not native_facts['native_reference_pass'] or not native_facts['upstream_source_review_pass']:raise ValueError('native implementation review evidence incomplete')
    expected_units=[(n,s) for n,s,_ in q.FIXTURES]
    if [(u['run_id'],u['seed']) for u in prepared['units']]!=expected_units:raise ValueError('changed exact two-unit fixture identities')
    for unit in prepared['units']:
        value=json.loads(q.read_bound(unit['files'][0]));q.validate_contract(value)
        if value['steering_kd']!=kd:raise ValueError('unit physical parameter changed')
        if [Path(b['path']).name for b in unit['files']]!=['contract.json','world.sdf','robot_description.xml','fixture.launch']:raise ValueError('nonexact fixture resource list')
        for b in unit['files']:q.read_bound(b)
        if q.read_bound(unit['files'][1]).decode()!=q.world_sdf(value):raise ValueError('world differs from reviewed generator')
        if q.read_bound(unit['files'][3]).decode()!=render_launch(q.STAGE/'runs'/value['run_id'],value):raise ValueError('launch differs from reviewed generator')
        q.validate_robot_description(q.read_bound(unit['files'][2]).decode(),value)
        if q.read_bound(unit['files'][2]).decode()!=q.render_urdf(value):raise ValueError('URDF differs from bound physical generator')
    return release,prepared,review

def archive(root,item):
    payload=q.read_bound(item);path=root/'bound_blobs'/item['sha256'];path.parent.mkdir(exist_ok=True)
    if not path.exists():
        with path.open('xb') as stream:stream.write(payload);stream.flush();os.fsync(stream.fileno())
        path.chmod(0o444)
    if q.sha(path)!=item['sha256']:raise ValueError('archive hash mismatch')
    return dict(q.binding(path),original_path=item['path'])

def descendant(pid,ancestor):
    visited=set()
    while pid>1 and pid not in visited:
        if pid==ancestor:return True
        visited.add(pid)
        fields=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()
        pid=int(fields[1])
    return False

def capture_maps(root,marker,proc):
    request=json.loads(marker.read_text());pid=request['pid']
    if type(pid)is not int or not descendant(pid,proc.pid):raise ValueError('Gazebo PID not owned by launched tree')
    label='pre' if request['phase']=='bootstrap' else 'final'
    data=q.elf_inventory(pid,root/('elf_'+label+'.maps.txt'))
    data.update(source_ns=request['source_ns'],phase=request['phase'],wall_monotonic_ns=time.monotonic_ns(),
                proc_start_ticks=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()[19])
    for item in data['loaded_modules']:
        original=item['path'];copied=archive(root,item)
        item.update(path=copied['path'],original_path=original)
    common.atomic(root/('elf_'+label+'.json'),data)
    common.atomic(root/('maps_'+label+'_ready.json'),{'inventory':q.binding(root/('elf_'+label+'.json'))})
    return data

def closure(root,release,review,value):
    """Derive audit conclusions from authenticated review plus actual run facts."""
    raw=json.loads((root/'collector_raw.json').read_text());start,end=raw['start_ns'],raw['end_ns']
    before=json.loads((root/'elf_pre.json').read_text());after=json.loads((root/'elf_final.json').read_text())
    from mapped_elf_inventory import same_process
    process_identity_pass=same_process(before,after)
    facts=native_review_facts(review)
    expected_ode=facts['expected_ode_sha256']
    expected_modules={x['role']:x['sha256'] for x in review['engine_module_bindings']}
    pre={role:x for x in before['loaded_modules'] for role in x['roles']}
    post={role:x for x in after['loaded_modules'] for role in x['roles']}
    module_checks={role:role in pre and role in post and pre[role]['sha256']==post[role]['sha256']==digest
                   for role,digest in expected_modules.items()}
    required_roles=set(q.ELF_ROLES.values())-{'actor_binaries','camera'}
    if not value['camera']:required_roles-=set(q.GRAPHICS_ROLES.values())
    if value['actor']:required_roles.add('actor_binaries')
    if value['camera']:required_roles.add('camera')
    module_complete=process_identity_pass and required_roles<=set(module_checks) and all(module_checks[r] for r in required_roles)
    native_ok=facts['native_reference_pass'] and pre.get('ode_integrator',{}).get('sha256')==expected_ode
    source_ok=facts['upstream_source_review_pass']
    source_bindings=review['engine_source_bindings']
    roster_rows=[r for r in raw['node_roster'] if start-200_000_000<=r['source_ns']<=raw['final_drain']['source_watermark_ns']]
    roster_ok=bool(roster_rows) and all(r['pass'] for r in roster_rows) and roster_rows[0]['source_ns']<=start and roster_rows[-1]['source_ns']>=end
    service=q.validate_services(raw['services'],start,end)
    common_fields={'run_id':value['run_id'],'start_ns':start,'end_ns':end,'scope_limit':'authorized launch/nodes; not adversarial process proof'}
    audits={
      'engine_binary_maps':dict(common_fields,**{'pass':module_complete and native_ok and source_ok,
         'same_process_identity_pass':process_identity_pass,
         'external_graphics_dynamic_code':after.get('external_graphics_dynamic_code',[]),
         'graphics_driver_provenance':'exact binary bindings plus independent GLX mapping reproduction; no NVIDIA source correspondence or dynamic executable memory equality claim',
         'loaded_modules':after['loaded_modules'],'module_checks':module_checks,
         'runtime_inventory_bindings':[q.binding(root/'elf_pre.json'),q.binding(root/'elf_final.json')],
         'native_report_binding':review['native_engine_report'],'native_records_binding':review['native_engine_records'],
         'implementation_review':{'ode_normal_euler_update_verified':native_ok,
           'gazebo_pre_end_callback_order_verified':source_ok,
           'loaded_module_source_correspondence_reviewed':module_complete and source_ok,
           'source_bindings':source_bindings},
         'interpretation':'selected physics/ROS/custom mapped ELF SHA + native reference + reviewed versioned engine source; external graphics receives binary/pattern evidence only; no formal bit-for-bit source-build theorem'}),
      'node_roster_audit':dict(common_fields,**{'pass':roster_ok,'loopback_master_isolated':roster_ok,
          'unexpected_nodes':sorted({n for r in roster_rows for n in r['unexpected_nodes']}),
          'observations':q.binding(root/'node_roster.jsonl'),'sampling':'0.20wall s during active window; trusted authorized node scope'}),
      'service_call_audit':dict(common_fields,**service,scope='authorized_launch_and_nodes',journal=q.binding(root/'service_calls.jsonl')),
      'pose_writer_review':dict(common_fields,**{'pass':module_complete and service['pass'],
          'robot_pose_velocity_writes':review['robot_pose_velocity_writes'],
          'actor_updates':'reviewed_bounded_Begin_updates' if value['actor'] else 'none','independent_review':release['source_review']})}
    for name,data in audits.items():common.atomic(root/(name+'.json'),data)
    import dense_sweep_p03 as dense
    bindings={key:[q.binding(root/(key+'.json'))] for key in audits}
    bindings.update(launch=[q.binding(root/'fixture.launch')],world=[q.binding(root/'world.sdf')],
       robot_description=[q.binding(root/'robot_description_runtime.xml')],
       driver_source=[archive(root,q.binding(q.WORKSPACE/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp'))],
       evidence_source=[archive(root,q.binding(q.WORKSPACE/'src/simulation/m2_gazebo/src/astra_physics_metadata_p03_plugin.cpp'))],
       driver_binary=[post['driver_binary']],evidence_binary=[post['evidence_binary']],
       actor_sources=[archive(root,q.binding(q.WORKSPACE/'src/simulation/m2_gazebo/src/astra_reactive_actor_plugin.cpp'))] if value['actor'] else [],
       actor_binaries=[post['actor_binaries']] if value['actor'] else [])
    result={'schema':'astra_p03_run_closure_v1','run_id':value['run_id'],'scope':dense.SCOPE,
            'numeric_profile':dense.PROFILE,'bindings':bindings,'stage':q.NAME}
    common.atomic(root/'run_closure.json',result)
    return result

def assess(root,value):
    import dense_sweep_p03 as dense
    import evaluate_episode as evaluator
    raw=json.loads((root/'collector_raw.json').read_text());start,end=raw['start_ns'],raw['end_ns']
    out=root/'sweep_evaluation'
    command=[str(q.WORKSPACE/'.venv/bin/python'),str(Path(dense.__file__)),'--bag',str(root/'fixture.bag'),
      '--world',str(root/'world.sdf'),'--urdf',str(root/'robot_description_runtime.xml'),
      '--closure',str(root/'run_closure.json'),'--start-ns',str(start),'--end-ns',str(end),'--output',str(out)]
    with (root/'sweep_evaluation.log').open('w') as stream:
        process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=90)
    sweep=json.loads((out/'report.json').read_text())
    _models,_links,contacts,metadata=evaluator.read_bag(root/'fixture.bag')
    coverage=q.source_coverage([r['stamp_ns'] for r in contacts],start,end)
    errors=[];world=evaluator.WorldCollisions((root/'world.sdf').read_text())
    contact=evaluator.contact_audit({r['stamp_ns']:r for r in contacts},world,start,end,'autolabor_m2',set(world.actor_speed_bounds),errors)
    # Bag retains every contact; JSON keeps counts/examples to avoid duplicating dense evidence.
    contact_summary=dict(contact)
    for key in ('illegal_robot_contacts','wheel_ground_support_contacts','actor_environment_contacts'):
        contact_summary[key+'_count']=len(contact[key])
        contact_summary[key+'_examples']=contact[key][:10]
        del contact_summary[key]
    records=dense.read_p03_bag(root/'fixture.bag')
    states=[r for r in records['end_model_rows'] if start<=r['stamp_ns']<=end]
    robot=[];actors=[]
    for row in states:
        robot.append(row['poses'][row['names'].index('autolabor_m2')][:3])
        if value['actor']:actors.append(row['poses'][row['names'].index('q04_actor')][:3])
    distance=lambda rows:max((sum((b-a)**2 for a,b in zip(rows[0],p))**.5 for p in rows),default=0.) if rows else 0.
    command_result=q.command_audit(raw['command_events'],value,start)
    ingress=[x for x in raw['command_ingress'] if start<=x['stamp_ns']<=end]
    driver=[r for r in raw['status_messages'] if start<=r['stamp_ns']<=end]
    physical_audit=q.physical_status_audit(raw['status_messages'],start,end)
    checks={'collector_pass':json.loads((root/'collector_report.json').read_text())['pass'],
      'passive_source_geometry_pass':process.returncode==0 and sweep['integrity_pass'],
      'positive_engineering_clearance':sweep['clearance_status']=='pass_within_model_scope',
      'contact_packet_coverage':coverage['pass'],
      'contact_metadata_complete':metadata.get('contacts_every_physics_step')is True and metadata.get('all_collision_pairs_retained')is True,
      'no_illegal_robot_contact':not errors and not contact['illegal_robot_contacts'],
      'no_actor_environment_contact':not contact['actor_environment_contacts'],
      'exact_publisher_clock_command_slots':command_result['pass'],'robot_motion_observed':distance(robot)>.1,
      'actor_motion_observed_or_not_applicable':distance(actors)>.25 if value['actor'] else True,
      'physical_status_coverage':physical_audit['coverage']['pass'],
      'physical_status_message_contract':physical_audit['message_contract_pass'],
      'no_physical_backend_fault':physical_audit['no_fault_timeout_brake_emergency']}
    report={'stage':q.NAME,'run_id':value['run_id'],'seed':value['seed'],'checks':checks,'pass':all(checks.values()),
      'command_audit':command_result,'physical_status_audit':physical_audit,
      'driver_ingress_description':{'count_in_source_window':len(ingress),
         'first_source_ns':ingress[0]['stamp_ns'] if ingress else None,
         'last_source_ns':ingress[-1]['stamp_ns'] if ingress else None,
         'time_authority':'actual P05 Gazebo SimTime at acceptance; not relay publication or force activation',
         'extra_pass_threshold':None},
      'contact_audit':contact_summary,'contact_issues':errors,'sweep_report':q.binding(out/'report.json'),
      'robot_maximum_displacement_m':distance(robot),'actor_maximum_displacement_m':distance(actors),
      'visual_review':'pending independent inspection of native PNGs' if value['camera'] else 'not requested',
      'formal_safety':False,'navigation_performance_evidence':False}
    common.atomic(root/'assessment.json',report);return report

def execute(release_path,release_sha):
    if Path(os.environ.get('THESIS_WS','')).resolve()!=q.WORKSPACE:raise ValueError('source isolated astra_env first')
    guard=(q.STAGE/'queue/simulation.lock').open('a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    release,prepared,review=verify_release(release_path,release_sha)
    ledger=q.STAGE/'queue/passive_sweep_q04_ledger.json'
    if ledger.exists() or any((q.STAGE/'runs'/row[0]).exists() for row in q.FIXTURES):raise ValueError('immutable identity/ledger exists; retry prohibited')
    state={'stage':q.NAME,'release_sha256':release_sha,'status':'active','maximum_units':2,'units':[
      {'run_id':n,'seed':s,'status':'reserved'} for n,s,_ in q.FIXTURES]}
    with ledger.open('x') as stream:json.dump(state,stream,indent=2);stream.flush();os.fsync(stream.fileno())
    receipts=[]
    try:
        for index,unit in enumerate(prepared['units']):
            state['units'][index]['status']='consumed';common.atomic(ledger,state)
            root=q.STAGE/'runs'/unit['run_id'];proc=None;error=None;report=None;started=time.monotonic();code=None;value=None
            try:
                root.mkdir()
                for file in unit['files']:(root/Path(file['path']).name).write_bytes(q.read_bound(file))
                value=json.loads((root/'contract.json').read_text());q.validate_contract(value)
                common.atomic(root/'allocated.json',dict(state['units'][index],attempt=1,retry_allowed=False,started_utc=common.utc()))
                for item in release['resources']:archive(root,item)
                for name in ('prepared','source_review','physical_parameter_decision'):archive(root,release[name])
                verify_release(release_path,release_sha)
                for port in (14820,14821):
                    with socket.socket() as sock:
                        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                        sock.bind(('127.0.0.1',port))
                env=dict(os.environ);env.update(ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',
                  ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',
                  ROS_LOG_DIR=str(root/'roslog'),ROS_HOME=str(root/'ros_home'),GAZEBO_MODEL_DATABASE_URI='')
                with (root/'roslaunch.log').open('wb') as log:
                    proc=subprocess.Popen(['roslaunch',str(root/'fixture.launch')],cwd=str(q.WORKSPACE),env=env,
                                          stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    common.atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')',1)[1].split()[19]})
                    while proc.poll() is None:
                        for name,ack in [('ready_for_maps.json','maps_pre_ready.json'),('ready_for_maps_final.json','maps_final_ready.json')]:
                            if (root/name).exists() and not (root/ack).exists():capture_maps(root,root/name,proc)
                        if time.monotonic()-started>value['wall_cap_s']:raise TimeoutError('fixture wall cap')
                        common.atomic(root/'heartbeat.json',{'utc':common.utc(),'elapsed_wall_s':time.monotonic()-started,'pid':proc.pid})
                        time.sleep(.1)
                    code=proc.returncode
            except BaseException as exc:error=type(exc).__name__+': '+str(exc)
            finally:
                if proc is not None:
                    try:common.stop_group(proc,root)
                    except BaseException as exc:error=(error or '')+'; cleanup '+repr(exc)
            try:
                if error or code!=0:raise RuntimeError('process/preparation failure: '+str(error)+' exit='+str(code))
                closure(root,release,review,value);report=assess(root,value)
                verify_release(release_path,release_sha)
                if time.monotonic()-started>value['wall_cap_s']:raise TimeoutError('complete attempt including assessment exceeded wall cap')
            except BaseException as exc:error=(error or '')+'; assessment '+repr(exc)
            receipt={'stage':q.NAME,'run_id':unit['run_id'],'seed':unit['seed'],'attempt':1,'retry_allowed':False,
              'pass':bool(not error and report and report['pass']),'error':error,'exit_code':code,
              'wall_elapsed_s':time.monotonic()-started,'report':q.binding(root/'assessment.json') if (root/'assessment.json').is_file() else None,'release_sha256':release_sha,
              'artifacts':[q.binding(p) for p in root.iterdir() if p.is_file() and p.suffix in ('.json','.sdf','.xml','.bag','.launch')] if root.is_dir() else []}
            receipt_path=root/'receipt.json' if root.is_dir() else q.STAGE/'queue'/(unit['run_id']+'_preparation_failure.json')
            common.atomic(receipt_path,receipt);receipts.append(receipt)
            state['units'][index]['receipt']=q.binding(receipt_path)
            state['units'][index]['pass']=receipt['pass'];common.atomic(ledger,state)
            if not receipt['pass']:break
    except BaseException as terminal_error:
        state['terminal_failure']=type(terminal_error).__name__+': '+str(terminal_error)
        raise
    finally:
        for row in state['units']:
            if row['status']=='reserved':row['status']='retired_unrun'
        state['status']='terminal';state['consumed_units']=sum(r['status']=='consumed' for r in state['units'])
        common.atomic(ledger,state)
    return 0 if len(receipts)==2 and all(r['pass'] for r in receipts) else 3

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare',type=Path);parser.add_argument('--steering-kd',type=float,choices=(4.,6.))
    parser.add_argument('--no-camera',action='store_true');parser.add_argument('--execute',action='store_true')
    parser.add_argument('--release',type=Path);parser.add_argument('--release-sha256')
    args=parser.parse_args()
    if args.execute:
        if args.prepare or not args.release or not args.release_sha256:raise ValueError('execute needs exact root release path+SHA only')
        return execute(args.release,args.release_sha256)
    if args.prepare:
        if args.steering_kd is None:raise ValueError('prepare requires explicit resolved steering Kd')
        print(json.dumps(prepare(args.prepare,args.steering_kd,not args.no_camera),indent=2));return 0
    print(json.dumps({'stage':q.NAME,'execution_authorized':False,'reserved_design_identities':q.FIXTURES,
        'physical_kd':'unresolved until explicit root decision','missing_resources':[str(p) for p in q.required_resources() if not p.is_file()]}))
    return 0
if __name__=='__main__':raise SystemExit(main())
