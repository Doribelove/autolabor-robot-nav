"""Strict N06 release bytes, identity, one-shot state, and durable I/O."""
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import tempfile

import prepare_navigation_n06_static as prepared_api
import navigation_evidence_n06 as evidence

STAGE=prepared_api.STAGE
NAME=prepared_api.NAME
PREPARED_SHA='a2eee7ec93770e3a2bd6df755d587465814cfa7aaf44cfb9bac2a8361f8ac9ad'
SCHEDULE_SHA='504924a8f96510f2b9de0d4cb4009a67cfd522312301afa0ed3ef00c1ed2a56b'
PREPARATION_MANIFEST_SHA='8d8c8909b3d2b6f2cc1b79185903506fc566dea3890da9d97c9b970840504fcf'
RELEASE_KEYS={'schema','stage','status','execution_authorized','training_authorized','real_vehicle_authorized','maximum_episodes',
 'retry_allowed','resume_allowed','prepared','schedule','preparation_manifest','source_review','d04_review',
 'passive_geometry_qualification','conversion_bundle','runtime_closure','registry_baseline','predecessor_ledger','resources'}
REGISTRY_FIELDS=('namespace','split','purpose','seed','run_id','status','source_binding')
ASSESSMENT_SOURCES=('fixture.bag','world.sdf','robot_description_runtime.xml','collector_raw.json','collector_report.json','run_closure.json')
ASSESSMENT_CHECKS={'collector_complete','passive_source_geometry_complete','contact_source_coverage','contact_message_contract','contact_metadata_complete','physical_status_message_contract','physical_status_source_coverage','goal_source_contract','shared_actor_gate_identity_timing'}
METADATA_FIELDS={'started_utc','ended_utc','exit_code','pid','process_group','command_json','cwd','heartbeat_path','receipt_path','bindings_sha256','code_sha256','config_sha256','model_sha256','report_path','raw_path'}

def digest(data):return hashlib.sha256(data).hexdigest()
def json_bytes(data):return (json.dumps(data,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
def same_typed(a,b):return type(a)is type(b) and (set(a)==set(b) and all(same_typed(a[k],b[k]) for k in a) if isinstance(a,dict) else len(a)==len(b) and all(same_typed(x,y) for x,y in zip(a,b)) if isinstance(a,list) else a==b)

def strict_json(data):
    def pairs(rows):
        result={}
        for k,v in rows:
            if k in result:raise ValueError('duplicate JSON key: '+k)
            result[k]=v
        return result
    return json.loads(data,object_pairs_hook=pairs,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite JSON')))

def regular_bytes(path):
    path=Path(path)
    if not path.is_absolute() or str(path.resolve(strict=True))!=str(path):raise ValueError('canonical absolute path required')
    fd=os.open(str(path),os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    try:
        before=os.fstat(fd)
        if not stat.S_ISREG(before.st_mode):raise ValueError('regular file required')
        with os.fdopen(fd,'rb',closefd=False) as f:data=f.read()
        after=os.fstat(fd);current=os.stat(str(path),follow_symlinks=False)
        key=lambda s:(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
        if key(before)!=key(after) or key(after)!=key(current):raise ValueError('file changed during bound read')
        return data
    finally:os.close(fd)

def read_bound(item):
    if type(item)is not dict or set(item)!={'path','sha256'} or type(item['path'])is not str or type(item['sha256'])is not str or not re.fullmatch('[0-9a-f]{64}',item['sha256']):raise ValueError('exact path/SHA binding required')
    data=regular_bytes(item['path'])
    if digest(data)!=item['sha256']:raise ValueError('bound bytes changed: '+item['path'])
    return data

def binding(path):return {'path':str(Path(path).resolve()),'sha256':digest(regular_bytes(Path(path).resolve()))}
def json_bound(item):return strict_json(read_bound(item))
def fsync_dir(path):
    fd=os.open(str(path),os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)

def publish(path,data,replace=False):
    path=Path(path);data=json_bytes(data) if not isinstance(data,bytes) else data
    fd,temp=tempfile.mkstemp(prefix='.n06-',dir=str(path.parent))
    try:
        with os.fdopen(fd,'wb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        if replace:os.replace(temp,str(path))
        else:os.link(temp,str(path),follow_symlinks=False)
        fsync_dir(path.parent)
    finally:
        if os.path.exists(temp):os.unlink(temp)

def csv_rows(data,fields):
    reader=csv.DictReader(io.StringIO(data.decode('utf-8'),newline=''))
    if tuple(reader.fieldnames or ())!=tuple(fields):raise ValueError('exact CSV header required')
    rows=list(reader)
    if any(set(r)!=set(fields) or any(v is None for v in r.values()) for r in rows):raise ValueError('malformed CSV row')
    return rows

def reserved_registry_bytes(baseline,release_sha,rows):
    before=csv_rows(baseline,REGISTRY_FIELDS);names={r['run_id'] for r in rows};seeds={str(r['seed']) for r in rows}
    if any(r['run_id'] in names or r['seed'] in seeds for r in before):raise ValueError('proposed identities/seeds already in registry baseline')
    if not baseline.endswith(b'\n'):raise ValueError('baseline CSV must end in newline')
    out=io.StringIO(newline='');writer=csv.DictWriter(out,fieldnames=REGISTRY_FIELDS,lineterminator='\r\n')
    for row in rows:writer.writerow(dict(namespace='astra_gazebo_20260926',split='development',purpose='n06_static_lookahead_pilot',
        seed=row['seed'],run_id=row['run_id'],status='reserved_exact_n06_release',source_binding=release_sha))
    return baseline+out.getvalue().encode()

def check_registry(actual,baseline,release_sha,rows):
    if actual!=reserved_registry_bytes(baseline,release_sha,rows):raise ValueError('entire live registry differs from baseline plus exact release reservation')

def initial_state(release_sha,rows):
    return {'schema':'astra_navigation_n06_ledger_v1','stage':NAME,'status':'active','release_sha256':release_sha,'maximum_units':8,
        'units':[dict(r,status='reserved',receipt=None) for r in rows],'event_sequence':0,'infrastructure_stop':None}

def consume(state,index):
    if state['status']!='active' or state['infrastructure_stop'] is not None or state['units'][index]['status']!='reserved':raise ValueError('identity not consumable')
    if any(r['status']!='consumed' or r.get('integrity_pass')is not True for r in state['units'][:index]):raise ValueError('prior unit not closed with integrity')
    state['units'][index]['status']='consumed'

def close_unit(state,index,receipt_binding,receipt):
    row=state['units'][index]
    if row['status']!='consumed' or row['receipt']is not None:raise ValueError('unit receipt is one-shot')
    validate_terminal_receipt(receipt,row,state['release_sha256'])
    row.update(receipt=receipt_binding,integrity_pass=receipt['integrity_pass'],task_outcome=receipt['task_outcome'])
    if not receipt['integrity_pass']:state['infrastructure_stop']='unit '+str(index)+' integrity failure'

def validate_terminal_receipt(receipt,row,release_sha):
    """Bind administrative/task meaning to the exact retained assessment.

    This checks receipt consistency; it does not independently replay physics.
    Return dependencies so administrative transactions rehash them throughout.
    """
    if type(receipt)is not dict or receipt.get('stage')!=NAME or any(not same_typed(receipt.get(k),row[k]) for k in ('run_id','seed','method','attempt')) or receipt.get('release_sha256')!=release_sha:raise ValueError('receipt stage/identity mismatch')
    if type(receipt.get('integrity_pass'))is not bool:raise ValueError('boolean integrity required')
    error=receipt.get('error');assessment=receipt.get('assessment');code=receipt.get('roslaunch_exit_code');task=receipt.get('task_outcome')
    if error is not None and (type(error)is not str or not error):raise ValueError('receipt error must be None or nonempty text')
    if code is not None and type(code)is not int:raise ValueError('receipt process code must be integer or absent')
    root=STAGE/'runs'/row['run_id'];dependencies=[];report=None
    contract_binding=binding(prepared_api.PREPARED/row['run_id']/'contract.json');contract=json_bound(contract_binding)
    if assessment is not None:
        if type(assessment)is not dict or assessment.get('path')!=str(root/'assessment.json'):raise ValueError('assessment canonical run path required')
        report=json_bound(assessment);dependencies.append(assessment)
        if type(report)is not dict or report.get('schema')!='astra_navigation_assessment_v1' or report.get('stage')!=NAME or any(not same_typed(report.get(k),row[k]) for k in ('run_id','seed','method')):raise ValueError('assessment identity mismatch')
        if type(report.get('capture_integrity_pass'))is not bool or type(report.get('task'))is not dict or report['task'].get('task_phase') not in evidence.TERMINAL_PHASES or task!=report['task']['task_phase']:raise ValueError('receipt/assessment task or integrity semantics disagree')
        checks=report.get('integrity_checks')
        if type(checks)is not dict or set(checks)!=ASSESSMENT_CHECKS or any(type(v)is not bool for v in checks.values()) or all(checks.values())!=report['capture_integrity_pass']:raise ValueError('assessment exact integrity checks disagree')
        sources=report.get('source_bindings')
        if type(sources)is not dict or set(sources)!=set(ASSESSMENT_SOURCES):raise ValueError('assessment complete source bindings required')
        for name,item in sources.items():
            if type(item)is not dict or item.get('path')!=str(root/name):raise ValueError('assessment source canonical run path required')
            read_bound(item);dependencies.append(item)
        if sources['world.sdf']['sha256']!=contract['world']['sha256'] or sources['robot_description_runtime.xml']['sha256']!=contract['robot_description']['sha256']:raise ValueError('assessment world/URDF differs from frozen contract')
        sweep=report.get('sweep_report')
        if type(sweep)is not dict or sweep.get('path')!=str(root/'sweep_evaluation/report.json'):raise ValueError('canonical bound sweep report required')
        sweep_value=json_bound(sweep);dependencies.append(sweep)
        if sweep_value.get('schema')!='astra_p04_engineering_sweep_v1' or type(sweep_value.get('integrity_pass'))is not bool or (checks['passive_source_geometry_complete'] and sweep_value['integrity_pass']is not True):raise ValueError('bound sweep integrity disagrees')
        sweep_sources=sweep_value.get('source_bindings');code_dir=prepared_api.WORKSPACE/'scripts/astra_gazebo'
        expected_paths={str(root/name) for name in ('fixture.bag','world.sdf','robot_description_runtime.xml','run_closure.json','converted_robot.sdf','conversion_manifest.json')}|{str(code_dir/name) for name in ('dense_sweep_n05.py','dense_sweep.py','geometry.py','evaluate_episode_n05.py','event_actor_contract_n05.py')}
        if type(sweep_sources)is not list or len(sweep_sources)!=len(expected_paths) or {item.get('path') for item in sweep_sources if type(item)is dict}!=expected_paths:raise ValueError('complete canonical sweep source roster required')
        for item in sweep_sources:
            read_bound(item);dependencies.append(item)
            name=Path(item['path']).name
            if name in ('fixture.bag','world.sdf','robot_description_runtime.xml','run_closure.json') and item!=sources[name]:raise ValueError('assessment/sweep source binding mismatch')
        for name in ('collector_raw.json','collector_report.json'):
            value=json_bound(sources[name])
            if value.get('stage')!=NAME or any(not same_typed(value.get(k),row[k]) for k in ('run_id','seed','method')):raise ValueError('bound collector identity disagrees')
            if name=='collector_report.json' and (type(value.get('pass'))is not bool or value['pass']!=checks['collector_complete']):raise ValueError('bound collector completion disagrees')
        closure=json_bound(sources['run_closure.json'])
        if closure.get('schema')!='astra_p04_run_closure_v1' or closure.get('stage')!=NAME or closure.get('run_id')!=row['run_id']:raise ValueError('bound run closure identity disagrees')
    elif task is not None:raise ValueError('task outcome without an assessment')
    if receipt['integrity_pass']:
        if error is not None or code!=0 or report is None or report['capture_integrity_pass']is not True:raise ValueError('successful integrity requires zero process exit, no error and bound passing assessment')
    elif error is None and (report is None or report['capture_integrity_pass']is not False):raise ValueError('failed integrity requires retained error or negative assessment')
    metadata=receipt.get('administrative_metadata')
    if type(metadata)is not dict or set(metadata)!=METADATA_FIELDS:raise ValueError('closed administrative metadata required; identity overrides forbidden')
    expected={'receipt_path':str(root/'receipt.json'),'report_path':'' if assessment is None else str(root/'assessment.json'),
        'raw_path':str(root/'collector_raw.json') if (root/'collector_raw.json').is_file() else '',
        'heartbeat_path':str(root/'heartbeat.json'),'cwd':str(prepared_api.WORKSPACE),'bindings_sha256':release_sha,
        'code_sha256':binding(prepared_api.WORKSPACE/'scripts/astra_gazebo/run_navigation_n06_manager_r03.py')['sha256'],
        'config_sha256':contract_binding['sha256'],'model_sha256':'',
        'exit_code':'' if code is None else code}
    if any(not same_typed(metadata.get(k),v) for k,v in expected.items()):raise ValueError('metadata canonical path/hash/value differs')
    if strict_json(metadata['command_json'])!=['/opt/ros/noetic/bin/roslaunch',str(root/'fixture.launch')]:raise ValueError('metadata launch command differs')
    if any(type(metadata[k])is not str or not metadata[k] for k in ('started_utc','ended_utc')):raise ValueError('metadata timestamps missing')
    process_path=root/'process.json';process={}
    if process_path.is_file():
        item=binding(process_path);process=json_bound(item);dependencies.append(item)
        if type(process.get('pid'))is not int or process['pid']<=1 or type(process.get('pgid'))is not int:raise ValueError('process identity malformed')
    if receipt['integrity_pass'] and not process:raise ValueError('successful receipt requires bound process identity')
    if not same_typed(metadata['pid'],process.get('pid','')) or not same_typed(metadata['process_group'],process.get('pgid','')):raise ValueError('metadata process identity differs')
    return dependencies

def terminal(state,reason=None):
    if reason:state['infrastructure_stop']=str(reason)
    for row in state['units']:
        if row['status']=='reserved':row['status']='retired_unrun'
    state['status']='terminal';state['consumed_units']=sum(r['status']=='consumed' for r in state['units'])
    return state

def required_manager_paths():
    p=prepared_api.WORKSPACE/'scripts/astra_gazebo'
    venv=prepared_api.WORKSPACE/'.venv';site=venv/'lib/python3.8/site-packages'
    return [p/name for name in ('run_navigation_n06_manager_r03.py','navigation_n06_integrity_r03.py','navigation_n06_process_r03.py','navigation_n06_admin_plan_r03.py')]+[
        Path('/usr/include/x86_64-linux-gnu/asm/unistd_64.h'),Path('/usr/include/linux/prctl.h'),venv/'pyvenv.cfg',
        site/'_virtualenv.py',site/'_distutils_hack/__init__.py',site/'_virtualenv.pth',site/'distutils-precedence.pth',site/'matplotlib-3.7.5-py3.8-nspkg.pth']

def verify_release(path,expected_sha):
    """Read-only full verifier. No ledger/lock/allocation is created here."""
    rb={'path':str(Path(path).resolve()),'sha256':expected_sha};release=json_bound(rb)
    if type(release)is not dict or set(release)!=RELEASE_KEYS:raise ValueError('closed exact N06 release schema required')
    expected={'schema':'astra_navigation_n06_exact_release_v1','stage':NAME,'status':'root_reviewed_release','execution_authorized':True,
        'training_authorized':False,'real_vehicle_authorized':False,'maximum_episodes':8,'retry_allowed':False,'resume_allowed':False}
    if any(not same_typed(release[k],v) for k,v in expected.items()):raise ValueError('N06 release authority/type differs')
    if type(release['resources'])is not list or not release['resources']:raise ValueError('nonempty exact resource roster required')
    resources={}
    for item in release['resources']:
        read_bound(item)
        if item['path'] in resources:raise ValueError('duplicate release resource path')
        resources[item['path']]=item['sha256']
    def required(item):
        read_bound(item)
        if resources.get(item['path'])!=item['sha256']:raise ValueError('required resource absent from release: '+item['path'])
    for key in RELEASE_KEYS-set(expected)-{'resources'}:required(release[key])
    if release['prepared']!={'path':str(prepared_api.PREPARED/'prepared.json'),'sha256':PREPARED_SHA} or release['schedule']!={'path':str(prepared_api.PREPARED/'schedule.json'),'sha256':SCHEDULE_SHA}:raise ValueError('frozen eight-unit package differs')
    if release['preparation_manifest']['sha256']!=PREPARATION_MANIFEST_SHA:raise ValueError('preparation source manifest differs')
    preparation=json_bound(release['preparation_manifest'])
    for key in ('owned_sources','root_owned_and_frozen_runtime_nodes','prepared_inputs','author_audits_including_failures','read_only_dependencies'):
        for item in preparation[key]:required({'path':item['path'],'sha256':item['sha256']})
    for p in required_manager_paths():required(binding(p))
    data=json_bound(release['prepared']);rows=prepared_api.schedule();schedule=json_bound(release['schedule'])
    if not same_typed(schedule['schedule'],rows) or len(data['units'])!=8:raise ValueError('exact schedule changed')
    for unit,row in zip(data['units'],rows):
        if any(not same_typed(unit[k],v) for k,v in row.items()):raise ValueError('unit identity/order/arm changed')
        for key in ('contract','launch','gate_expectation','robot_description','world'):required(unit[key])
        value=json_bound(unit['contract']);evidence.validate_contract(value)
        if any(not same_typed(value[k],unit[k]) for k in ('run_id','seed','method','attempt')) or value['stage']!=NAME:raise ValueError('contract identity changed')
        if read_bound(unit['launch']).decode()!=prepared_api.render_launch(value,Path(unit['contract']['path']),STAGE/'runs'/unit['run_id']):raise ValueError('launch is not exact prepared rendering')
    d04=json_bound(release['d04_review'])
    if d04.get('status')!='independent_offline_joint_split_dedup_and_four_static_raster_review_pass' or d04.get('author_report',{}).get('sha256')!=prepared_api.D04_REPORT_SHA:raise ValueError('D04 review missing/wrong revision')
    predecessor=json_bound(release['predecessor_ledger'])
    if predecessor.get('stage')!='astra_navigation_n05_preview_selection' or predecessor.get('status')!='terminal':raise ValueError('N05 predecessor not terminal')
    qualification=json_bound(release['passive_geometry_qualification'])
    if qualification.get('qualified_for_navigation_evidence')is not True or type(qualification.get('resolved_steering_kd'))not in (int,float) or qualification['resolved_steering_kd']!=6. or not qualification.get('evidence_bindings'):raise ValueError('P05 Kd6 passive qualification absent')
    for item in qualification['evidence_bindings']:required(item)
    review=json_bound(release['source_review'])
    if review.get('schema')!='astra_navigation_source_review_v1' or review.get('stage')!=NAME or review.get('status')!='independently_reviewed' or review.get('navigation_lifecycle_authority')!='collector_only_spawn_unpause_pause_fixed_gateway' or review.get('robot_pose_velocity_writes')!=[]:raise ValueError('N06 source/lifecycle review absent')
    reviewed={item['path']:item['sha256'] for item in review['resources']}
    for item in review['resources']:required(item)
    for name in resources:
        if Path(name).suffix in ('.py','.launch','.xacro','.cpp','.h','.yaml','.xml') and reviewed.get(name)!=resources[name]:raise ValueError('source/config outside independent review: '+name)
    from run_passive_sweep_q05 import native_review_facts
    facts=native_review_facts(review)
    if not facts['native_reference_pass'] or not facts['upstream_source_review_pass']:raise ValueError('native source/reference prerequisite failed')
    for item in review['engine_module_bindings']+review['engine_source_bindings']+[review[k] for k in ('native_engine_report','native_engine_records','upstream_source_manifest')]:required({'path':item['path'],'sha256':item['sha256']})
    runtime=json_bound(release['runtime_closure'])
    if runtime.get('schema')!='astra_navigation_n06_runtime_closure_v1' or runtime.get('stage')!=NAME or runtime.get('status')!='reviewed_disk_dependency_closure' or not runtime.get('resources'):raise ValueError('reviewed runtime dependency roster absent')
    for item in runtime['resources']:required(item)
    if runtime.get('python_executable')!=str((prepared_api.WORKSPACE/'.venv/bin/python').resolve()):raise ValueError('runtime interpreter differs from launch')
    if runtime.get('python_prefix')!=str((prepared_api.WORKSPACE/'.venv').resolve()) or runtime.get('python_no_user_site')is not True:raise ValueError('runtime virtualenv identity differs')
    # The installed command is a symlink to gzserver-11.15.1. Resource paths
    # are canonical, so never require the alias basename as if it were a file.
    gzserver=binding(Path('/usr/bin/gzserver').resolve());required(gzserver)
    if gzserver not in runtime['resources']:raise ValueError('runtime root omitted: canonical gzserver target')
    for name in ('libastra_physical_ackermann_p05_plugin.so','libastra_dense_evidence_plugin.so','libastra_physics_metadata_p03_plugin.so',
                 'libgazebo_ros_api_plugin.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so','libgazebo_ros_camera.so',
                 'record','relay','robot_state_publisher','roslaunch'):
        if not any(Path(item['path']).name==name for item in runtime['resources']):raise ValueError('runtime root omitted: '+name)
    conversions=json_bound(release['conversion_bundle'])
    if conversions.get('schema')!='astra_navigation_n06_conversion_bundle_v1' or conversions.get('stage')!=NAME or len(conversions.get('units',[]))!=8:raise ValueError('eight converted-model sidecars required')
    import dense_sweep_n05 as dense
    for side,unit in zip(conversions['units'],data['units']):
        if side.get('run_id')!=unit['run_id'] or side.get('input_urdf')!=unit['robot_description']:raise ValueError('conversion sidecar order/input differs')
        required(side['conversion_manifest']);required(side['converted_sdf'])
        converted=json_bound(side['conversion_manifest'])
        if converted['input_urdf']!=unit['robot_description'] or converted['converted_sdf']!=side['converted_sdf']:raise ValueError('conversion provenance differs')
        actual=dense.conversion_resources(side['conversion_manifest']['path'])
        if not same_typed(actual,side['conversion_resources']):raise ValueError('conversion resources differ')
        for item in actual:required(item)
        provenance=dense.authenticate_conversion(side['conversion_manifest']['path'],unit['robot_description']['path'],side['converted_sdf']['path'],{},diagnostic=True)
        if provenance.get('pass')is not True or provenance.get('diagnostic_only')is not True or provenance.get('runtime_converter_correspondence_qualified')is not False:raise ValueError('preallocation converter provenance failed: '+str(provenance.get('issues')))
    baseline=read_bound(release['registry_baseline']);reserved_registry_bytes(baseline,expected_sha,rows)
    return {'release':release,'release_binding':rb,'prepared':data,'review':review,'conversions':conversions,'rows':rows,'registry_baseline':baseline}

class VerifiedSession:
    def __init__(self,path,sha):
        self.data=verify_release(path,sha);self.closed=False
    def recheck(self):
        if self.closed:raise RuntimeError('release session permanently closed')
        try:
            read_bound(self.data['release_binding'])
            for item in self.data['release']['resources']:read_bound(item)
        except BaseException:self.closed=True;raise
        return self.data
