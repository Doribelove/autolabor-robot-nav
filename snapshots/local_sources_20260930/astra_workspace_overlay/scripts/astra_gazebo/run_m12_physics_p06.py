#!/usr/bin/env python3
"""P06 six-unit paired Kd pilot. Default is offline preflight; no implicit ROS."""
import argparse
import copy
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml
import run_smoke_a02 as common

STAGE=common.STAGE
WORKSPACE=common.WORKSPACE
DESIGN=STAGE/'contracts/m12_physics_p06_design.yaml'
RUNTIME_LEDGER=STAGE/'queue/m12_physics_p06_budget_runtime.json'
SOURCE=WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_actuation_command_source_p06.py'
PROBE=WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p06.py'
TOPICS=('/clock /astra/world_models /astra/world_links /astra/physics_models /astra/physics_links '
 '/m2_driver/actuator_target /m2_driver/physical_backend_fault /m2_driver/physical_metadata '
 '/m2_driver/actuator_commanded_effort /astra/physics_contacts /astra/physics_metadata /odom '
 '/joint_states /cmd_vel_navigation /cmd_vel /scan /tf /tf_static /m2_driver/wheel_angle '
 '/m2_driver/applied_speed /m2_driver/control_timeout /m2_driver/command_activation_latency '
 '/m2_driver/physical_status /astra_fixture/schedule_start_ns /astra_fixture/command_event '
 '/astra_fixture/command_source_status /m2_driver/command_ingress').split()
ARG_MAP={'speed_time_constant':'speedTimeConstant','steering_time_constant':'steeringTimeConstant',
 'max_acceleration':'maxAcceleration','max_deceleration':'maxDeceleration',
 'max_brake_deceleration':'maxBrakeDeceleration','max_emergency_deceleration':'maxEmergencyDeceleration',
 'max_steering_rate':'maxSteeringRate','command_delay':'commandDelay','command_jitter':'commandJitter',
 'wheel_kp':'wheelKp','wheel_ki':'wheelKi','wheel_integral_torque_limit':'wheelIntegralTorqueLimit',
 'wheel_drive_torque_limit':'wheelDriveTorqueLimit','wheel_brake_torque_limit':'wheelBrakeTorqueLimit',
 'steering_kp':'steeringKp','steering_kd':'steeringKd','steering_torque_limit':'steeringTorqueLimit'}


def read_bound(path,expected):
    payload=Path(path).read_bytes()
    if hashlib.sha256(payload).hexdigest()!=expected:raise ValueError('changed bound input '+str(path))
    return payload


def load_design():
    design=yaml.safe_load(DESIGN.read_bytes())
    if design['stage']!='astra_m12_physics_p06' or design['maximum_episodes']!=6:raise ValueError('wrong matrix design')
    parent=yaml.safe_load(read_bound(STAGE/design['parent_contract']['path'],design['parent_contract']['sha256']))
    rows=json.loads(read_bound(STAGE/design['schedule']['path'],design['schedule']['sha256']))
    budget=json.loads(read_bound(STAGE/design['budget_design']['path'],design['budget_design']['sha256']))
    expected=[(260926020,4),(260926020,6),(260926021,6),(260926021,4),(260926022,4),(260926022,6)]
    if len(rows)!=6 or budget['identities']!=[r['run_id'] for r in rows] or budget['maximum_units']!=6:raise ValueError('budget roster mismatch')
    parent_source=read_bound(STAGE/design['command_source_parent']['path'],design['command_source_parent']['sha256']).decode()
    if SOURCE.read_text()!=parent_source.replace("'astra_m12_physics_p02'","'astra_m12_physics_p06'").replace("libastra_physical_ackermann_p02_plugin.so","libastra_physical_ackermann_p05_plugin.so"):
        raise ValueError('P06 source changed beyond exact stage and plugin identity constants')
    for index,(row,(seed,kd)) in enumerate(zip(rows,expected),1):
        expected_id='m12_physics_p06_s{:03d}_kd{}'.format(seed-260926000,kd)
        if row['sequence']!=index or row['run_id']!=expected_id or row['seed']!=seed or row['steering_kd']!=kd or row['attempt']!=1:
            raise ValueError('schedule identity/factor/order mismatch')
        if row['arm']!=('control' if kd==4 else 'treatment') or row['pair']!=(index-1)//2+1 or row['pair_position']!=(index-1)%2+1:
            raise ValueError('schedule arm/pair label mismatch')
        contract=yaml.safe_load(read_bound(STAGE/row['contract_path'],row['contract_sha256']))
        if contract['stage']!='astra_m12_physics_p06' or contract['run_id']!=expected_id or type(contract['seed'])is not int or contract['seed']!=seed or contract['maximum_episodes']!=1:
            raise ValueError('per-run identity guard mismatch')
        if contract['effective_plugin_parameters']['steeringKd']!=kd:raise ValueError('Kd contract mismatch')
        normalized=copy.deepcopy(contract)
        for key in ('stage','purpose','run_id','seed'):normalized[key]=parent[key]
        normalized['effective_plugin_parameters']['steeringKd']=parent['effective_plugin_parameters']['steeringKd']
        normalized['command_source_protocol']['node_script']=parent['command_source_protocol']['node_script']
        normalized.pop('paired_pilot')
        if normalized.pop('terminal_collection_protocol')!=design['terminal_collection_protocol']:
            raise ValueError('terminal watermark protocol mismatch')
        if normalized.pop('command_ingress_protocol')!=design['command_ingress_protocol']:raise ValueError('driver ingress protocol mismatch')
        normalized['required_topics'].remove('/m2_driver/command_ingress')
        if normalized!=parent:raise ValueError('nonfactor per-run contract change')
    protocol=design['terminal_collection_protocol']
    if protocol['post_pause_drain_wall_s']!=parent['recorder_protocol']['post_pause_drain_wall_s'] or protocol['post_pause_drain_wall_s']!=5.:
        raise ValueError('terminal repair cannot expand frozen drain budget')
    old=set(design['prerequisite']['inherited_check_names']);new=set(design['prerequisite']['full_check_roster'])
    if len(old)!=120 or len(new)!=122 or new-old!={'final_paused_source_watermark_drained','physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain'} or old-new!={'command_loss_no_relay_publication_after_drain','command_relay_observed'}:
        raise ValueError('P06 must retain numeric gates and replace relay receipt-clock loss test with driver source evidence')
    return design,parent,rows


def render_launch(row,root,contract):
    launch=ET.Element('launch')
    include=ET.SubElement(launch,'include',file='$(find m2_gazebo)/launch/spawn_m2_physical_p05.launch')
    options={'world':str(root/'world.sdf'),'paused':'true','gui':'false','use_sim_time':'true','seed':str(row['seed']),
        'server_required':'true','x':'0','y':'0','z':'0','yaw':'0','wheel_mu':'1.0','laser_noise_stddev':'0.01'}
    options.update({name:str(contract['effective_plugin_parameters'][key]) for name,key in ARG_MAP.items()})
    for name,value in options.items():ET.SubElement(include,'arg',name=name,value=value)
    ET.SubElement(launch,'node',pkg='topic_tools',type='relay',name='actuation_command_relay',args='/cmd_vel_navigation /cmd_vel',output='screen')
    source=ET.SubElement(launch,'node',pkg='m2_gazebo',type=SOURCE.name,name='actuation_command_source',required='true',output='screen',**{'launch-prefix':str(WORKSPACE/'.venv/bin/python')})
    for name,value in {'contract_path':str(root/'actuation_contract.yaml'),'contract_sha256':row['contract_sha256'],
                       'seed':str(row['seed']),'run_id':row['run_id']}.items():ET.SubElement(source,'param',name=name,value=value)
    ET.SubElement(launch,'node',pkg='rosbag',type='record',name='fixture_bag',output='screen',args='--lz4 -O '+str(root/'fixture.bag')+' '+' '.join(TOPICS))
    probe=ET.SubElement(launch,'node',pkg='m2_gazebo',type=PROBE.name,name='fixture_probe',required='true',output='screen',**{'launch-prefix':str(WORKSPACE/'.venv/bin/python')})
    for name,value in {'output':str(root),'contract_path':str(root/'actuation_contract.yaml'),'world_path':str(root/'world.sdf'),
                       'seed':str(row['seed']),'run_id':row['run_id']}.items():ET.SubElement(probe,'param',name=name,value=value)
    ET.SubElement(probe,'rosparam',param='recording_topics').text=yaml.safe_dump(TOPICS,default_flow_style=True)
    return ET.tostring(launch,encoding='unicode')


def classify_report(report,design,expected_names=None):
    if not isinstance(report,dict) or report.get('terminal_exception') or report.get('assessment_exception') or report.get('assessment_incomplete') is not False:
        return 'infrastructure_failure',['missing_incomplete_or_exception_report']
    checks=report.get('checks',{})
    if not isinstance(checks,dict) or not checks or any(type(v)is not bool for v in checks.values()):
        return 'infrastructure_failure',['invalid_check_map']
    minimum=set(design['prerequisite']['minimum_measurement_checks'])
    if not minimum<=set(checks):return 'infrastructure_failure',['missing_measurement_checks']
    if expected_names is None:expected_names=design['prerequisite']['full_check_roster']
    if expected_names is not None and set(checks)!=set(expected_names):return 'infrastructure_failure',['changed_full_check_roster']
    failed=[key for key,value in checks.items() if not value]
    if report.get('pass')is not all(checks.values()):return 'infrastructure_failure',['inconsistent_overall_pass']
    physical=set(design['failure_policy']['physical_failures_allowed_to_continue'])
    if any(key not in physical for key in failed):return 'infrastructure_failure',failed
    return ('qualified' if not failed else 'physical_negative'),failed


def required_resources(design,rows):
    pkg=WORKSPACE/'src/simulation/m2_gazebo'
    paths=[Path(__file__).resolve(),Path(common.__file__).resolve(),DESIGN,SOURCE,PROBE,
      STAGE/design['parent_contract']['path'],STAGE/design['schedule']['path'],STAGE/design['budget_design']['path'],
      WORKSPACE/'scripts/astra_env.sh',STAGE/'environment_manifest.yaml',
      WORKSPACE/'devel/lib/libastra_physical_ackermann_p05_plugin.so',WORKSPACE/'devel/lib/libastra_dense_evidence_plugin.so']
    paths.extend(pkg/name for name in ['src/astra_physical_ackermann_p05_plugin.cpp','src/astra_dense_evidence_plugin.cpp',
      'include/m2_gazebo/astra_physical_control.h','include/m2_gazebo/actuator_dynamics.h','include/m2_gazebo/ackermann_kinematics.h',
      'launch/spawn_m2_physical_p05.launch','urdf/m2_physical_p05.urdf.xacro','urdf/m2_physical_p05_gazebo.xacro',
      'urdf/m2_physical_base.xacro','urdf/m2_sensors.xacro'])
    paths.extend(STAGE/r['contract_path'] for r in rows)
    # Reuse only the exact reviewed predecessor roster, never enumerate the
    # mutable workspace. New future components remain outside this closure.
    inherited=design['runtime_resource_parent']
    inherited_path=STAGE/inherited['path']
    inherited_release=json.loads(read_bound(inherited_path,inherited['sha256']))
    if len(inherited_release['resources'])!=inherited['resource_count']:raise ValueError('inherited runtime roster size changed')
    paths.append(inherited_path)
    paths.extend(Path(item['path']) for item in inherited_release['resources'])
    paths.extend(STAGE/item['path'] for key,item in design['prerequisite']['P03_terminal'].items() if key in ('ledger','summary','diagnosis'))
    paths.extend(STAGE/v['path'] for v in design['prerequisite']['P04_terminal'].values())
    paths.extend(STAGE/v['path'] for v in design['prerequisite']['P05_terminal'].values())
    paths.extend(STAGE/design['prerequisite'][key]['path'] for key in ('ingress_repair_review','driver_root_review'))
    return sorted(set(p.resolve() for p in paths))


def p02_protocol(design):
    root=STAGE/'runs/m12_physics_p02'
    for name in ('receipt.json','probe_report.json','probe_raw.json','fixture.bag','cleanup.json'):
        if not (root/name).is_file():raise ValueError('P02 complete measurement prerequisite absent: '+name)
    receipt=json.loads((root/'receipt.json').read_bytes());report=json.loads((root/'probe_report.json').read_bytes())
    inherited=copy.deepcopy(design)
    for key in ('final_paused_source_watermark_drained','physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain'):
        inherited['prerequisite']['minimum_measurement_checks'].remove(key)
    inherited['prerequisite']['minimum_measurement_checks'].extend(['command_loss_no_relay_publication_after_drain','command_relay_observed'])
    status,failed=classify_report(report,inherited,design['prerequisite']['inherited_check_names'])
    if status=='infrastructure_failure':raise ValueError('P02 measurement chain failed: '+str(failed))
    if receipt.get('status')!='completed' or receipt.get('error') or receipt.get('exit_code')!=0 or receipt.get('missing_bag_topics'):
        raise ValueError('P02 process/recorder completion failed')
    for name in ('probe_report.json','probe_raw.json','fixture.bag'):
        if receipt.get(name+'_sha256')!=common.sha(root/name):raise ValueError('P02 receipt hash mismatch '+name)
    cleanup=json.loads((root/'cleanup.json').read_bytes())
    if cleanup.get('owned_live_processes')!=[] or cleanup.get('ports_14820_14821_released')is not True:raise ValueError('P02 cleanup incomplete')
    return {'status':status,'physical_failed_checks':failed,'check_names':sorted(report['checks']),
            'resources':[{'path':str(root/name),'sha256':common.sha(root/name)} for name in ('receipt.json','probe_report.json','probe_raw.json','fixture.bag','cleanup.json')]}


def p03_terminal(design):
    requirement=design['prerequisite']['P03_terminal']
    documents={key:json.loads(read_bound(STAGE/requirement[key]['path'],requirement[key]['sha256'])) for key in ('ledger','summary','diagnosis')}
    ledger,summary,diagnosis=(documents[key] for key in ('ledger','summary','diagnosis'))
    if ledger.get('stage')!='astra_m12_physics_p03' or ledger.get('status')!='stopped_infrastructure' or ledger.get('consumed_units')!=2 or ledger.get('retired_units')!=4:
        raise ValueError('P03 terminal budget not preserved as2 consumed4 retired')
    expected=['m12_physics_p03_s009_kd4','m12_physics_p03_s009_kd6','m12_physics_p03_s010_kd6','m12_physics_p03_s010_kd4','m12_physics_p03_s011_kd4','m12_physics_p03_s011_kd6']
    if [unit['run_id'] for unit in ledger.get('units',[])]!=expected or [unit['budget_status'] for unit in ledger['units']]!=['consumed']*2+['retired']*4:
        raise ValueError('P03 terminal identity roster changed')
    if summary.get('measurement_valid_matrix_complete')is not False or summary.get('treatment_promoted')is not False:
        raise ValueError('P03 incomplete result was reinterpreted')
    if set(diagnosis.get('runs',{}))!=set(expected[:2]) or any(row.get('original_replay_exact')is not True for row in diagnosis['runs'].values()):
        raise ValueError('P03 terminal diagnosis does not replay both retained results')
    # Diagnosis records are prerequisites only; never count as P06 samples.
    for resource in diagnosis.get('inputs',[]):read_bound(resource['path'],resource['sha256'])
    return {'status':'terminal_negative_preserved','consumed_units':2,'retired_units':4,'new_P06_evidence':False,
            'resources':[{'path':str(STAGE/requirement[key]['path']),'sha256':requirement[key]['sha256']} for key in ('ledger','summary','diagnosis')]}


def ingress_review(design):
    spec=design['prerequisite']['ingress_repair_review'];path=STAGE/spec['path']
    review=json.loads(path.read_bytes())
    if review.get('status')!=spec['required_status'] or review.get('independent_probe_review')is not True:
        raise ValueError('P06 ingress review absent/pending')
    if review.get('reviewed_probe_sha256')!=common.sha(PROBE):raise ValueError('ingress review wrong probe hash')
    names=review.get('check_names')
    if review.get('numeric_thresholds_unchanged')is not True or review.get('total_gate_count')!=122 or not isinstance(names,list) or len(names)!=122 or set(names)!=set(design['prerequisite']['full_check_roster']):
        raise ValueError('P06 review numerical thresholds/122gate roster mismatch')
    driver=STAGE/design['prerequisite']['driver_root_review']['path'];d=json.loads(driver.read_bytes())
    if d.get('pass')is not True or d.get('independent_driver_review')is not True or d.get('controller_formula_unchanged')is not True or d.get('driver_sha256')!=common.sha(WORKSPACE/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp'):
        raise ValueError('passive driver review invalid')
    return {'status':review['status'],'resources':[{'path':str(p),'sha256':common.sha(p)} for p in (path,driver)]}


def p04_terminal(design):
    spec=design['prerequisite']['P04_terminal']
    docs={key:json.loads(read_bound(STAGE/item['path'],item['sha256'])) for key,item in spec.items()}
    ledger=docs['ledger'];summary=docs['summary'];receipt=docs['receipt']
    if ledger.get('status')!='stopped_infrastructure' or ledger.get('consumed_units')!=1 or ledger.get('retired_units')!=5:
        raise ValueError('P04 terminal1consumed5retired state changed')
    if summary.get('measurement_valid_matrix_complete')is not False or summary.get('treatment_promoted')is not False:
        raise ValueError('P04 terminal failure reinterpreted')
    if receipt.get('outcome')!='infrastructure_failure' or receipt['report']['checks'].get('final_paused_source_watermark_drained')is not True or receipt['report']['checks'].get('command_loss_no_relay_publication_after_drain')is not False:
        raise ValueError('P04 receipt no longer preserves original watermark pass and relay-clock failure')
    return {'status':'terminal1consumed5retired_preserved','resources':[{'path':str(STAGE/v['path']),'sha256':v['sha256']} for v in spec.values()]}


def p05_terminal(design):
    spec=design['prerequisite']['P05_terminal']
    docs={key:json.loads(read_bound(STAGE/item['path'],item['sha256'])) for key,item in spec.items()}
    ledger,summary,receipt,cleanup=(docs[key] for key in ('ledger','summary','receipt','cleanup'))
    if ledger.get('stage')!='astra_m12_physics_p05' or ledger.get('status')!='stopped_infrastructure' or ledger.get('consumed_units')!=1 or ledger.get('retired_units')!=5:
        raise ValueError('P05 terminal1consumed5retired state changed')
    expected=['m12_physics_p05_s017_kd4','m12_physics_p05_s017_kd6','m12_physics_p05_s018_kd6','m12_physics_p05_s018_kd4','m12_physics_p05_s019_kd4','m12_physics_p05_s019_kd6']
    if [unit['run_id'] for unit in ledger.get('units',[])]!=expected or [unit['budget_status'] for unit in ledger['units']]!=['consumed']+['retired']*5:
        raise ValueError('P05 terminal roster changed')
    if summary.get('measurement_valid_matrix_complete')is not False or summary.get('treatment_promoted')is not False:
        raise ValueError('P05 terminal startup failure reinterpreted')
    if receipt.get('outcome')!='infrastructure_failure' or receipt.get('report',{}).get('terminal_exception',{}).get('message')!='readiness wall deadline':
        raise ValueError('P05 original startup failure missing')
    if cleanup.get('owned_live_processes')!=[] or cleanup.get('ports_14820_14821_released')is not True:
        raise ValueError('P05 process cleanup incomplete')
    return {'status':'terminal_startup_failure_preserved','resources':[{'path':str(STAGE/item['path']),'sha256':item['sha256']} for item in spec.values()]}


def predecessor(design):
    return {'P02_protocol':p02_protocol(design),'P03_terminal':p03_terminal(design),
            'P04_terminal':p04_terminal(design),'P05_terminal':p05_terminal(design),'ingress_repair':ingress_review(design),
            'check_names':design['prerequisite']['full_check_roster']}


def active_inherited_digest(design,resource):
    approved=design.get('updated_build_dependencies',{})
    item=approved.get(resource['path'])
    if item is None:return resource['sha256']
    allowed={str(WORKSPACE/'src/simulation/m2_gazebo'/name) for name in ('CMakeLists.txt','package.xml')}
    if resource['path'] not in allowed or item['old_sha256']!=resource['sha256']:
        raise ValueError('unapproved inherited runtime source replacement')
    return item['new_sha256']


def check_seed_freshness(rows):
    with (STAGE/'seed_registry.csv').open() as f:registry=list(csv.DictReader(f))
    seeds={str(r['seed']) for r in rows};ids={r['run_id'] for r in rows}
    collisions=[r for r in registry if r.get('seed') in seeds or r.get('run_id') in ids]
    if collisions:raise ValueError('P06 planned seed/identity already consumed or retired')
    if RUNTIME_LEDGER.exists() or any((STAGE/'runs'/r['run_id']).exists() for r in rows):raise ValueError('P06 allocated matrix cannot be restarted/resumed')


def preflight():
    design,parent,rows=load_design();check_seed_freshness(rows)
    rendered=[]
    for row in rows:
        contract=yaml.safe_load(read_bound(STAGE/row['contract_path'],row['contract_sha256']))
        xml=ET.fromstring(render_launch(row,STAGE/'runs'/row['run_id'],contract));args={n.get('name'):n.get('value') for n in xml.findall('include/arg')}
        if float(args['steering_kd'])!=row['steering_kd']:raise ValueError('launch does not pass factor explicitly')
        rendered.append({'run_id':row['run_id'],'steering_kd':args['steering_kd'],'contract_sha256':row['contract_sha256']})
    paths=required_resources(design,rows)
    resources=[{'path':str(p),'sha256':common.sha(p)} for p in paths if p.is_file()]
    missing=[str(p) for p in paths if not p.is_file()]
    inherited=design['runtime_resource_parent'];parent=json.loads(read_bound(STAGE/inherited['path'],inherited['sha256']))
    current={item['path']:item['sha256'] for item in resources}
    changed=[item['path'] for item in parent['resources'] if current.get(str(Path(item['path']).resolve()))!=active_inherited_digest(design,item)]
    try:p02=p02_protocol(design);p02_ready=True
    except (OSError,ValueError) as exc:p02={'pending':str(exc)};p02_ready=False
    try:prior=predecessor(design);ready=True
    except (OSError,ValueError) as exc:prior={'pending':str(exc)};ready=False
    return {'status':'offline_static_preflight_pass','execution_ready':False,'all_prerequisites_ready':ready and not missing and not changed,
            'frozen_P02_protocol_ready':p02_ready,'frozen_P02_protocol':p02,'missing_resources':missing,
            'inherited_runtime_closure_unchanged':not changed,'changed_inherited_resources':changed,
            'root_release_required':True,'ros_started':False,'allocated_units':0,'runs':rendered,'resources':resources,'predecessor':prior}


def verify_release(path,expected_sha,design,rows):
    release=json.loads(read_bound(path,expected_sha))
    if release.get('stage')!=design['stage'] or release.get('status')!='root_reviewed_release' or release.get('execution_authorized')is not True or release.get('maximum_episodes')!=6:
        raise ValueError('reviewed P06 release absent/invalid')
    resources=release.get('resources',[]);bound={r['path']:r['sha256'] for r in resources}
    if len(bound)!=len(resources) or not {str(p) for p in required_resources(design,rows)}<=set(bound):raise ValueError('incomplete/duplicate release closure')
    for path,digest in bound.items():read_bound(path,digest)
    inherited=design['runtime_resource_parent'];parent=json.loads(read_bound(STAGE/inherited['path'],inherited['sha256']))
    for resource in parent['resources']:
        if bound.get(resource['path'])!=active_inherited_digest(design,resource):raise ValueError('frozen source/runtime closure changed')
    prior=predecessor(design)
    if release.get('predecessor_measurement_qualified')is not True or release.get('predecessor')!=prior:
        raise ValueError('release P02/P03/watermark prerequisite evidence mismatch')
    return release,prior


def append_registry(row,status,binding=''):
    common.append_csv(STAGE/'seed_registry.csv',dict(namespace='astra_gazebo_20260926',split='bootstrap_smoke',purpose='paired_steering_kd_pilot',
        seed=row['seed'],run_id=row['run_id'],status=status,source_binding=binding))


def write_receipt(row,receipt):
    root=STAGE/'runs'/row['run_id'];common.atomic(root/'receipt.json',receipt)
    for ledger in ('run_manifest.csv','EXPERIMENTS.csv'):
        common.append_csv(STAGE/ledger,dict(receipt,method='physical_kd'+str(int(row['steering_kd'])),split='bootstrap_smoke',raw_path=str(root),receipt_path=str(root/'receipt.json')))


def bind_from_release(root,release_path,release_sha,design,rows):
    """Archive only reviewed active dependencies and this row's exact inputs."""
    release,_=verify_release(release_path,release_sha,design,rows)
    resources={item['path']:item['sha256'] for item in release['resources']}
    for path in (root/'world.sdf',root/'scene_manifest.yaml',root/'fixture.launch',
                 root/'actuation_contract.yaml',release_path):
        key=str(path.resolve());digest=common.sha(path)
        if key in resources and resources[key]!=digest:raise ValueError('row binding conflicts with release')
        resources[key]=digest
    for path,digest in sorted(resources.items()):
        # Recheck active bytes even when a blob already exists from an earlier row.
        if common.sha(path)!=digest:raise ValueError('active dependency changed while binding '+path)
        target=STAGE/'code_blobs'/digest
        if not target.exists():
            temporary=target.with_name(target.name+'.p05.'+str(os.getpid())+'.tmp')
            try:
                shutil.copyfile(path,temporary)
                if common.sha(temporary)!=digest:raise ValueError('dependency changed while archiving '+path)
                temporary.chmod(0o444);os.replace(str(temporary),str(target))
            finally:
                if temporary.exists():temporary.unlink()
        if common.sha(target)!=digest:raise ValueError('archived dependency hash mismatch '+path)
    tracked=[str(Path(path).relative_to(WORKSPACE)) for path in resources if WORKSPACE in Path(path).parents]
    with (root/'dirty.patch').open('wb') as out:
        subprocess.run(['git','diff','--binary','--']+sorted(tracked),cwd=str(WORKSPACE),stdout=out,check=True)
    data={'scope':'reviewed P06 release dependencies plus immutable per-row inputs',
          'release_sha256':release_sha,'dirty_patch_sha256':common.sha(root/'dirty.patch'),
          'resources':[{'path':path,'sha256':digest} for path,digest in sorted(resources.items())]}
    common.atomic(root/'bindings.json',data)
    return common.sha(root/'bindings.json')


def execute_row(row,design,release_path,release_sha,expected_names):
    root=STAGE/'runs'/row['run_id'];root.mkdir()
    common.atomic(root/'allocated.json',dict(row,started_utc=common.utc(),retry_allowed=False,purpose='paired_steering_kd_pilot'))
    append_registry(row,'consumed')
    proc=None;error=None;started=time.monotonic();binding='';report=None;missing=list(TOPICS);exit_code=None
    try:
        contract_bytes=read_bound(STAGE/row['contract_path'],row['contract_sha256']);contract=yaml.safe_load(contract_bytes)
        (root/'actuation_contract.yaml').write_bytes(contract_bytes);(root/'world.sdf').write_text(contract['world_fixture']['sdf'])
        (root/'scene_manifest.yaml').write_text(yaml.safe_dump(dict(row,expected_models=['ground_plane','autolabor_m2'],performance_evidence=False)))
        (root/'fixture.launch').write_text(render_launch(row,root,contract))
        binding=bind_from_release(root,release_path,release_sha,design,load_design()[2])
        # Recheck the exact active closure immediately before starting this row.
        verify_release(release_path,release_sha,design,load_design()[2])
        for port in (14820,14821):
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                sock.bind(('127.0.0.1',port))
        env=dict(os.environ);env.update(ROS_MASTER_URI='http://127.0.0.1:14820',GAZEBO_MASTER_URI='http://127.0.0.1:14821',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',GAZEBO_IP='127.0.0.1',ROS_LOG_DIR=str(root/'roslog'),ROS_HOME=str(root/'ros_home'),GAZEBO_MODEL_DATABASE_URI='')
        command=['roslaunch',str(root/'fixture.launch')]
        common.atomic(root/'attempt.json',dict(row,command=command,bindings_sha256=binding,started_utc=common.utc(),wall_cap_s=contract['wall_cap_s']))
        with (root/'roslaunch.log').open('wb') as log:
            proc=subprocess.Popen(command,cwd=str(WORKSPACE),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            common.atomic(root/'process.json',{'pid':proc.pid,'pgid':proc.pid,'proc_start_ticks':Path('/proc/{}/stat'.format(proc.pid)).read_text().rsplit(')',1)[1].split()[19]})
            while proc.poll() is None:
                common.atomic(root/'heartbeat.json',{'utc':common.utc(),'pid':proc.pid,'elapsed_wall_s':time.monotonic()-started})
                if time.monotonic()-started>contract['wall_cap_s']:raise TimeoutError('row wall cap')
                time.sleep(1)
        exit_code=proc.returncode
    except BaseException as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        if proc is not None:
            try:common.stop_group(proc,root)
            except BaseException as exc:error=(error or '')+'; cleanup: '+str(exc)
    try:
        if (root/'probe_report.json').exists():report=json.loads((root/'probe_report.json').read_bytes())
        bag=root/'fixture.bag'
        if bag.is_file():
            info=subprocess.run(['rosbag','info','--yaml',str(bag)],capture_output=True,text=True,timeout=20)
            (root/'bag_info.stdout.yaml').write_text(info.stdout);(root/'bag_info.stderr.log').write_text(info.stderr)
            if info.returncode==0:missing=sorted(set(TOPICS)-{r['topic'] for r in yaml.safe_load(info.stdout).get('topics',[]) if r.get('messages',0)>0})
        outcome,failed=classify_report(report,design,expected_names)
        if report and (report.get('run_id')!=row['run_id'] or report.get('seed')!=row['seed']):raise ValueError('probe report identity mismatch')
        if error or exit_code!=0 or missing:outcome='infrastructure_failure'
        verify_release(release_path,release_sha,design,load_design()[2])
    except BaseException as exc:outcome='infrastructure_failure';failed=[];error=(error or '')+'; assessment/recording: '+str(exc)
    receipt=dict(row,status='infrastructure_failure' if outcome=='infrastructure_failure' else 'completed',outcome=outcome,
        pass_all_gates=outcome=='qualified',physical_failed_checks=failed,report=report,error=error,exit_code=exit_code,missing_bag_topics=missing,
        bindings_sha256=binding,ended_utc=common.utc(),wall_elapsed_s=time.monotonic()-started,performance_evidence=False,retry_allowed=False)
    for name in ('probe_report.json','probe_raw.json','probe_raw.jsonl','fixture.bag','fixture.launch','actuation_contract.yaml'):
        if (root/name).is_file():receipt[name+'_sha256']=common.sha(root/name)
    write_receipt(row,receipt);return receipt


def matrix_summary(rows,receipts,state):
    byid={r['run_id']:r for r in receipts};paired=[]
    for seed in (260926020,260926021,260926022):
        pair={r['arm']:byid.get(r['run_id']) for r in rows if r['seed']==seed};entry={'seed':seed,'results':{}}
        for arm,receipt in pair.items():
            if receipt:entry['results'][arm]={'run_id':receipt['run_id'],'outcome':receipt['outcome'],'failed_checks':receipt['physical_failed_checks'],'metrics':(receipt.get('report')or{}).get('metrics',{})}
        if all(pair.values()):
            metric=lambda arm:(pair[arm].get('report')or{}).get('metrics',{}).get('actual_center_rate_radps',{}).get('maximum_absolute')
            if metric('control')is not None and metric('treatment')is not None:entry['treatment_minus_control_peak_center_rate_radps']=metric('treatment')-metric('control')
        paired.append(entry)
    complete=len(receipts)==6 and all(r['outcome']!='infrastructure_failure' for r in receipts)
    treatment=[r for r in receipts if r['arm']=='treatment']
    promoted=complete and len(treatment)==3 and all(r['pass_all_gates'] for r in treatment)
    return {'stage':'astra_m12_physics_p06','measurement_valid_matrix_complete':complete,'treatment_promoted':promoted,'treatment_all_gate_pass_count':sum(r['pass_all_gates'] for r in treatment),'pairs':paired,'budget':state,'performance_evidence':False,'formal_safety_claim':False}


def execute(release_path,release_sha):
    if Path(os.environ.get('THESIS_WS','')).resolve()!=WORKSPACE:raise RuntimeError('source isolated environment')
    lock=(STAGE/'queue/simulation.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    design,parent,rows=load_design();check_seed_freshness(rows)
    release,prior=verify_release(release_path,release_sha,design,rows)
    state={'stage':design['stage'],'status':'active','maximum_units':6,'consumed_units':0,'retired_units':0,
        'units':[dict(row,budget_status='reserved') for row in rows],'started_utc':common.utc(),'release_sha256':release_sha}
    with RUNTIME_LEDGER.open('x') as f:json.dump(state,f,indent=2);f.flush();os.fsync(f.fileno())
    receipts=[];stop_reason=None
    try:
        for index,row in enumerate(rows):
            state['units'][index]['budget_status']='consumed';state['consumed_units']+=1;common.atomic(RUNTIME_LEDGER,state)
            receipt=execute_row(row,design,release_path,release_sha,prior['check_names']);receipts.append(receipt)
            state['units'][index]['outcome']=receipt['outcome'];common.atomic(RUNTIME_LEDGER,state)
            if receipt['outcome']=='infrastructure_failure':stop_reason='infrastructure failure at '+row['run_id'];break
    except BaseException as exc:
        stop_reason=type(exc).__name__+': '+str(exc)
        if state['consumed_units']>len(receipts):
            failed=state['units'][state['consumed_units']-1];failed['outcome']='infrastructure_failure'
            root=STAGE/'runs'/failed['run_id'];root.mkdir(exist_ok=True)
            receipt=dict(failed,status='infrastructure_failure',outcome='infrastructure_failure',pass_all_gates=False,
                physical_failed_checks=[],report=None,error=stop_reason,ended_utc=common.utc(),retry_allowed=False,performance_evidence=False)
            if not (root/'receipt.json').exists():write_receipt(failed,receipt)
            with (STAGE/'seed_registry.csv').open() as f:registered=any(r['run_id']==failed['run_id'] for r in csv.DictReader(f))
            if not registered:append_registry(failed,'consumed_preparation_failure',release_sha)
            receipts.append(receipt)
    finally:
        for unit in state['units']:
            if unit['budget_status']=='reserved':
                unit['budget_status']='retired';state['retired_units']+=1;append_registry(unit,'retired_unrun_after_infrastructure_failure',release_sha)
        state['status']='stopped_infrastructure' if stop_reason else 'terminal_complete';state['stop_reason']=stop_reason;state['ended_utc']=common.utc()
        common.atomic(RUNTIME_LEDGER,state)
        summary=matrix_summary(rows,receipts,state);common.atomic(STAGE/'audits/m12_physics_p06_summary.json',summary)
    print(json.dumps(summary,indent=2));return 0 if summary['treatment_promoted'] else 3


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    parser.add_argument('--preflight',action='store_true');parser.add_argument('--release',type=Path);parser.add_argument('--release-sha256');parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.execute:
        if args.preflight or args.release is None or args.release_sha256 is None:parser.error('execute needs exact reviewed release path+SHA, without preflight')
        return execute(args.release.resolve(),args.release_sha256)
    report=preflight()
    if args.output:common.atomic(args.output,report)
    print(json.dumps(report,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
