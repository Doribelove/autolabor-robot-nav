"""Independent N01 P04 manager integration tests. Conversion API is a marked stub. Real files + fake subprocesses; never ROS."""
import csv
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET
import pytest

SCRIPTS=Path(__file__).resolve().parents[1];sys.path.insert(0,str(SCRIPTS))
import run_navigation_n01 as manager


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value));return manager.evidence.binding(path)


def install_conversion_mock(monkeypatch):
    """Runner-only boundary stub: it does not qualify a real SDF conversion."""
    dense=types.ModuleType('dense_sweep_p04')
    def create(urdf,out):
        out=Path(out);out.mkdir();sdf=out/'robot.sdf';sdf.write_text('<sdf version="1.6"><model name="fixture"/></sdf>')
        manifest=out/'conversion.json';write(manifest,{'input_urdf':manager.evidence.binding(urdf),'converted_sdf':manager.evidence.binding(sdf)})
        return manifest
    def resources(path):
        data=json.loads(Path(path).read_bytes());return [manager.evidence.binding(path),data['input_urdf'],data['converted_sdf']]
    dense.create_conversion=create;dense.conversion_resources=resources;monkeypatch.setitem(sys.modules,'dense_sweep_p04',dense)
    return dense

def add_conversion(unit,dense):
    path=dense.create_conversion(unit['files'][1]['path'],Path(unit['files'][1]['path']).parent/'conversion')
    data=json.loads(path.read_bytes());unit.update(conversion_manifest=manager.evidence.binding(path),converted_sdf=data['converted_sdf'],conversion_resources=dense.conversion_resources(path))


@pytest.fixture
def execution_fixture(tmp_path,monkeypatch):
    stage=tmp_path/'stage';(stage/'queue').mkdir(parents=True);(stage/'runs').mkdir()
    workspace=tmp_path/'workspace';workspace.mkdir();monkeypatch.setattr(manager,'STAGE',stage);monkeypatch.setattr(manager,'WORKSPACE',workspace)
    monkeypatch.setenv('THESIS_WS',str(workspace))
    rows=[];dense=install_conversion_mock(monkeypatch)
    world=tmp_path/'world.sdf';world.write_text('<sdf/>');scene=tmp_path/'scene_manifest.yaml';scene.write_text('scene_id: fixture')
    for run_id,seed,method in manager.identities():
        folder=tmp_path/'prepared'/run_id;folder.mkdir(parents=True)
        value={'stage':manager.NAME,'run_id':run_id,'seed':seed,'method':method,'wall_cap_s':300.,'world':manager.evidence.binding(world),'scene_manifest':manager.evidence.binding(scene),'shared_bundle':{'path':'/fixture/bundle.json','sha256':'b'*64}}
        cb=write(folder/'contract.json',value);(folder/'robot_description.xml').write_text('<robot/>');(folder/'fixture.launch').write_text('<launch/>')
        rows.append(dict(run_id=run_id,seed=seed,method=method,attempt=1,files=[cb,manager.evidence.binding(folder/'robot_description.xml'),manager.evidence.binding(folder/'fixture.launch')]))
    for unit in rows:add_conversion(unit,dense)
    release={'resources':[],**{n:write(tmp_path/(n+'.json'),{}) for n in ('prepared','source_review','passive_geometry_qualification')}}
    prepared={'units':rows};release_sha='a'*64;release_path=tmp_path/'release.json';release_path.write_text('{}')
    monkeypatch.setattr(manager,'verify_release',lambda path,sha:(release,prepared,{}))
    registry=stage/'seed_registry.csv'
    def registry_rows(extra=None):
        with registry.open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=['run_id','seed','status','source_binding']);writer.writeheader()
            writer.writerows([dict(run_id=n,seed=s,status='reserved_exact_n01_release',source_binding=release_sha) for n,s,_ in manager.identities()]+(extra or []))
    registry_rows()
    calls={'popen':[],'cleanup':[],'assess':[],'archive':[]};settings={'mode':'success','outcomes':['timeout','complete','fault','complete']}
    passive=types.ModuleType('run_passive_sweep_q05')
    passive.archive=lambda root,item:calls['archive'].append((str(root),item))
    def capture_maps(root,marker,proc):raise RuntimeError('injected exact inventory failure')
    passive.capture_maps=capture_maps;monkeypatch.setitem(sys.modules,'run_passive_sweep_q05',passive)
    class FakeProcess:
        def __init__(self):self.pid=os.getpid();self.returncode=3 if settings['mode']=='exit_failure' else 0
        def poll(self):return None if settings['mode']in ('maps_failure','wall_timeout') else self.returncode
    def popen(args,**kwargs):
        calls['popen'].append((args,kwargs));root=Path(args[-1]).parent
        if settings['mode']=='launch_failure':raise OSError('injected launch failure')
        if settings['mode']=='maps_failure':write(root/'ready_for_maps.json',{'pid':os.getpid()})
        return FakeProcess()
    monkeypatch.setattr(manager.subprocess,'Popen',popen)
    def cleanup(proc,root):
        calls['cleanup'].append(str(root))
        if settings['mode']=='cleanup_failure':raise RuntimeError('injected surviving owned child')
    monkeypatch.setattr(manager.common,'stop_group',cleanup)
    monkeypatch.setattr(manager.evidence,'closure',lambda *args:None)
    def assess(root,value):
        index=len(calls['assess']);calls['assess'].append(str(root))
        if settings['mode']=='assessment_failure':raise RuntimeError('injected assessment failure')
        report={'capture_integrity_pass':settings['mode']!='integrity_failure','task':{'task_phase':settings['outcomes'][index]}}
        write(root/'assessment.json',report);return report
    monkeypatch.setattr(manager.evidence,'assess',assess)
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def setsockopt(self,*args):pass
        def bind(self,address):
            if settings['mode']=='port_failure':raise OSError('injected occupied port')
    monkeypatch.setattr(manager.socket,'socket',Socket)
    monkeypatch.setattr(manager.time,'sleep',lambda _:None)
    if_false_clock={'tick':0}
    def clock():
        if_false_clock['tick']+=1
        return if_false_clock['tick']*1000. if settings['mode']=='wall_timeout' else if_false_clock['tick']*.001
    monkeypatch.setattr(manager.time,'monotonic',clock)
    return types.SimpleNamespace(stage=stage,settings=settings,calls=calls,registry_rows=registry_rows,
        execute=lambda:manager.execute(release_path,release_sha),ledger=lambda:json.loads((stage/'queue/navigation_n01_ledger.json').read_text()))


def test_algorithm_failures_are_retained_and_all_four_units_consumed(execution_fixture):
    f=execution_fixture;assert f.execute()==0
    state=f.ledger();assert state['status']=='terminal' and state['consumed_units']==4
    assert [r['status'] for r in state['units']]==['consumed']*4
    assert [r['task_outcome'] for r in state['units']]==f.settings['outcomes']
    assert len(f.calls['popen'])==len(f.calls['cleanup'])==len(f.calls['assess'])==4
    assert all(options['start_new_session']is True for _,options in f.calls['popen'])
    assert all(options['env']['ROS_MASTER_URI']=='http://127.0.0.1:14820' for _,options in f.calls['popen'])


@pytest.mark.parametrize('mode',['launch_failure','port_failure','maps_failure','wall_timeout','exit_failure','cleanup_failure','assessment_failure','integrity_failure'])
def test_infrastructure_failure_consumes_one_and_retires_remaining(execution_fixture,mode):
    f=execution_fixture;f.settings['mode']=mode;assert f.execute()==3
    state=f.ledger();assert state['status']=='terminal' and state['consumed_units']==1
    assert [r['status'] for r in state['units']]==['consumed','retired_unrun','retired_unrun','retired_unrun']
    assert len(f.calls['popen'])<=1
    assert len(f.calls['cleanup'])==int(mode not in ('launch_failure','port_failure'))
    receipt=json.loads(Path(state['units'][0]['receipt']['path']).read_text())
    assert receipt['integrity_pass']is False
    if mode!='integrity_failure':assert receipt['error']
    before=len(f.calls['popen'])
    with pytest.raises(ValueError,match='identity already exists'):f.execute()
    assert len(f.calls['popen'])==before


def test_extra_historical_seed_collision_rejected_before_allocation(execution_fixture):
    f=execution_fixture;f.registry_rows([dict(run_id='unrelated_old_attempt',seed=manager.SEED,status='consumed',source_binding='f'*64)])
    with pytest.raises(ValueError):f.execute()
    assert not (f.stage/'queue/navigation_n01_ledger.json').exists() and not f.calls['popen']


def test_wrong_reservation_sha_rejected_before_allocation(execution_fixture):
    f=execution_fixture;p=f.stage/'seed_registry.csv';p.write_text(p.read_text().replace('a'*64,'b'*64))
    with pytest.raises(ValueError,match='reservation'):f.execute()
    assert not (f.stage/'queue/navigation_n01_ledger.json').exists() and not f.calls['popen']


def test_real_new_launch_ends_with_required_collector():
    root=ET.parse(manager.LAUNCH).getroot();nodes=[n for n in root.iter('node') if n.get('name')=='astra_navigation_collector']
    assert len(nodes)==1 and nodes[0].get('required')=='true'
    assert not any(n.get('type')=='spawn_model' for n in root.iter('node'))


def test_rendered_contract_path_points_to_copied_runtime_contract(tmp_path):
    value={'world':{'path':'/world.sdf'},'run_id':'fixture','seed':manager.SEED,'method':manager.METHODS[0],
      'shared_bundle':{'path':'/scene/common_bundle/common_route.json','sha256':'b'*64},'scene_manifest':{'path':'/scene/manifest.yaml'},
      'scene_id':'navigation_n01_clear','steering_kd':6.,'record_topics':['/clock','/odom']}
    root=ET.fromstring(manager.render_launch(value,'a'*64,tmp_path));include=root.find('include');args={x.get('name'):x.get('value') for x in include.findall('arg')}
    assert include.get('file')==str(manager.LAUNCH)
    assert args['contract_path']==str(tmp_path/'contract.json') and args['contract_sha256']=='a'*64
    assert args['bundle']=='/scene/common_bundle' and args['bag_topics']=='/clock /odom'
    assert args['steering_kd']=='6.0' and args['seed']=='260926026'


@pytest.fixture
def release_fixture(tmp_path,monkeypatch):
    stage=tmp_path/'stage';stage.mkdir();monkeypatch.setattr(manager,'STAGE',stage)
    launch=tmp_path/'real_include.launch';launch.write_text('<launch/>');monkeypatch.setattr(manager,'LAUNCH',launch)
    checkpoint=tmp_path/'actor.pt';checkpoint.write_bytes(b'fixture only');monkeypatch.setattr(manager,'CHECKPOINT',checkpoint)
    source=tmp_path/'active.py';source.write_text('pass\n');config=tmp_path/'active.yaml';config.write_text('x: 1\n')
    mandatory=[source,config];monkeypatch.setattr(manager,'required_resources',lambda:mandatory)
    monkeypatch.setattr(manager.evidence,'validate_contract',lambda value:value)
    units=[];canonical_contracts={};dense=install_conversion_mock(monkeypatch)
    for run_id,seed,method in manager.identities():
        directory=tmp_path/'prepared'/run_id;directory.mkdir(parents=True)
        value={'stage':manager.NAME,'run_id':run_id,'seed':seed,'method':method,'world':{'path':'/fixture/world.sdf'},
         'shared_bundle':{'path':'/fixture/common_bundle/common_route.json','sha256':'b'*64},'scene_manifest':{'path':'/fixture/manifest.yaml'},
         'scene_id':'navigation_n01_clear','steering_kd':6.,'record_topics':['/clock','/odom']}
        canonical_contracts[run_id]=json.loads(json.dumps(value))
        binding=write(directory/'contract.json',value);(directory/'robot_description.xml').write_text('<robot/>')
        (directory/'fixture.launch').write_text(manager.render_launch(value,binding['sha256'],stage/'runs'/run_id))
        units.append(dict(run_id=run_id,seed=seed,method=method,attempt=1,files=[binding,manager.evidence.binding(directory/'robot_description.xml'),manager.evidence.binding(directory/'fixture.launch')]))
    for unit in units:add_conversion(unit,dense)
    monkeypatch.setattr(manager.evidence,'make_contract',lambda **kwargs:json.loads(json.dumps(canonical_contracts[kwargs['run_id']])))
    prepared={'units':units};native_facts={'native_reference_pass':True,'upstream_source_review_pass':True}
    passive_manager=types.ModuleType('run_passive_sweep_q05');passive_manager.native_review_facts=lambda review:dict(native_facts)
    monkeypatch.setitem(sys.modules,'run_passive_sweep_q05',passive_manager)
    engine_rows=[]
    for role in sorted(set(manager.evidence.passive.ELF_ROLES.values())-{'actor_binaries'}):
        file=tmp_path/('engine_'+role+'.so');file.write_bytes(('synthetic runner boundary '+role).encode());engine_rows.append(dict(manager.evidence.binding(file),role=role))
    engine_source=tmp_path/'engine_source.cpp';engine_source.write_text('// synthetic source boundary')
    native_bindings={k:write(tmp_path/(k+'.json'),{'synthetic_runner_boundary':True})for k in ('native_engine_report','native_engine_records','upstream_source_manifest')}
    review={'stage':manager.NAME,'status':'independently_reviewed','schema':'astra_navigation_source_review_v1',
        'navigation_lifecycle_authority':'collector_only_spawn_unpause_pause_fixed_gateway','robot_pose_velocity_writes':[],
        'actor_updates':'reviewed_bounded_Begin_updates','engine_module_bindings':engine_rows,
        'engine_source_bindings':[manager.evidence.binding(engine_source)],'resources':[manager.evidence.binding(p) for p in mandatory],**native_bindings}
    prerequisite={'qualified_for_navigation_evidence':True,'resolved_steering_kd':6.,'evidence_bindings':[write(tmp_path/'qualified_fixture_evidence.json',{'fixture':True})]}
    release={'stage':manager.NAME,'status':'root_reviewed_release','execution_authorized':True,'maximum_episodes':4,'retry_allowed':False,'resume_allowed':False,
      'resources':[manager.evidence.binding(p) for p in mandatory]+list({item['path']:item for u in units for item in u['files']+u['conversion_resources']}.values())+engine_rows+[manager.evidence.binding(engine_source)]+list(native_bindings.values())}
    def seal():
        release['prepared']=write(tmp_path/'prepared.json',prepared);release['source_review']=write(tmp_path/'review.json',review)
        release['passive_geometry_qualification']=write(tmp_path/'qualification.json',prerequisite)
        b=write(tmp_path/'release.json',release);return manager.verify_release(Path(b['path']),b['sha256'])
    return types.SimpleNamespace(release=release,prepared=prepared,review=review,prerequisite=prerequisite,seal=seal,source=source,native_facts=native_facts)


def test_release_exact_four_identities_and_current_source_closure(release_fixture):
    f=release_fixture;release,prepared,review=f.seal()
    assert [(u['run_id'],u['seed'],u['method']) for u in prepared['units']]==manager.identities()
    assert len(prepared['units'])==4 and len({u['run_id'] for u in prepared['units']})==4


@pytest.mark.parametrize('case',['unit_order','attempt_bool','seed_change','missing_unit_binding','missing_active_resource','missing_source_review','changed_launch','prerequisite_unqualified','wrong_contract_identity','duplicate_resource'])
def test_release_mutation_is_rejected_before_allocation(release_fixture,case):
    f=release_fixture
    if case=='unit_order':f.prepared['units'].reverse()
    elif case=='attempt_bool':f.prepared['units'][0]['attempt']=True
    elif case=='seed_change':f.prepared['units'][0]['seed']+=1
    elif case=='missing_unit_binding':f.release['resources'].pop()
    elif case=='missing_active_resource':f.release['resources'].pop(0)
    elif case=='missing_source_review':f.review['resources']=[]
    elif case=='prerequisite_unqualified':f.prerequisite['qualified_for_navigation_evidence']=False
    elif case=='duplicate_resource':f.release['resources'].append(f.release['resources'][0])
    elif case in ('changed_launch','wrong_contract_identity'):
        index=2 if case=='changed_launch' else 0;item=f.prepared['units'][0]['files'][index]
        if case=='changed_launch':Path(item['path']).write_text('<launch/>')
        else:
            value=json.loads(Path(item['path']).read_text());value['method']='dwa_common_route';write(item['path'],value)
        replacement=manager.evidence.binding(item['path'])
        f.prepared['units'][0]['files'][index]=replacement
        f.release['resources']=[replacement if r['path']==item['path'] else r for r in f.release['resources']]
    with pytest.raises(ValueError):f.seal()


def test_changed_resource_bytes_rejected_not_only_manifest_labels(release_fixture):
    f=release_fixture;f.source.write_text('raise RuntimeError("changed")\n')
    with pytest.raises(ValueError):f.seal()


@pytest.mark.parametrize('case',['empty_qualification_evidence','mismatched_qualified_kd','unit_kd_drift'])
def test_qualification_and_fixed_physical_factor_are_bound(release_fixture,case):
    f=release_fixture
    if case=='empty_qualification_evidence':f.prerequisite['evidence_bindings']=[]
    elif case=='mismatched_qualified_kd':f.prerequisite['resolved_steering_kd']=4.
    else:
        unit=f.prepared['units'][0];old_contract=unit['files'][0];old_launch=unit['files'][2]
        value=json.loads(Path(old_contract['path']).read_text());value['steering_kd']=4.
        new_contract=write(old_contract['path'],value)
        Path(old_launch['path']).write_text(manager.render_launch(value,new_contract['sha256'],manager.STAGE/'runs'/unit['run_id']))
        new_launch=manager.evidence.binding(old_launch['path']);unit['files'][0]=new_contract;unit['files'][2]=new_launch
        replacements={new_contract['path']:new_contract,new_launch['path']:new_launch}
        f.release['resources']=[replacements.get(row['path'],row) for row in f.release['resources']]
    with pytest.raises(ValueError):f.seal()


def test_actual_prepare_real_contracts_no_run_allocation(tmp_path,monkeypatch):
    install_conversion_mock(monkeypatch)
    import yaml
    stage=SCRIPTS.parents[2]
    parsed=yaml.safe_load((stage/'audits/navigation_coordinator_components/launch_parse/astra_analytic_route_v1.yaml').read_bytes())
    xml=parsed['/robot_description'];calls=[]
    def xacro(args,**kwargs):calls.append(args);return xml
    monkeypatch.setattr(manager.subprocess,'check_output',xacro)
    fake_stage=tmp_path/'stage';fake_stage.mkdir();monkeypatch.setattr(manager,'STAGE',fake_stage)
    output=tmp_path/'prepared';result=manager.prepare(output)
    assert result['execution_authorized']is False and result['maximum_episodes']==4
    assert len(calls)==4 and all('dynamics_seed:=260926026'in row and 'steering_kd:=6.0'in row for row in calls)
    assert not (fake_stage/'runs').exists() and not (fake_stage/'queue').exists()
    for unit in result['units']:
        contract=json.loads(manager.evidence.read_bound(unit['files'][0]));manager.evidence.validate_contract(contract)
        assert contract['seed']==260926026 and contract['steering_kd']==6. and contract['method']==unit['method']
        launch=ET.fromstring(manager.evidence.read_bound(unit['files'][2]));params={a.get('name'):a.get('value') for a in launch.find('include').findall('arg')}
        assert params['contract_path']==str(fake_stage/'runs'/unit['run_id']/'contract.json')
    prior=(output/'prepared.json').read_bytes()
    with pytest.raises(FileExistsError):manager.prepare(output)
    assert (output/'prepared.json').read_bytes()==prior


@pytest.mark.parametrize('method',['actual_teb_common_route','dwa_common_route'])
@pytest.mark.parametrize('case',['valid','missing_process','duplicate_process','unowned','changed_pid','changed_start','inventory_start_differs','module_sha_changed','missing_active_plugin'])
def test_actual_baseline_inventory_identity_and_release_binding(tmp_path,monkeypatch,method,case):
    calls=[];root=tmp_path/'run';root.mkdir();marker=root/'marker.json';proc=types.SimpleNamespace(pid=100)
    process={'node':'/move_base','pid':200,'proc_start_ticks':'123'}
    module_path=tmp_path/'libactual.so';module_path.write_bytes(b'fake ELF fixture');item=manager.evidence.binding(module_path)
    release={'resources':[item]};value={'method':method};active='actual_teb' if method=='actual_teb_common_route' else 'actual_dwa'
    fake=types.ModuleType('mapped_elf_inventory')
    def inventory(pid,roles,required,raw_path):
        calls.append((pid,roles,required,str(raw_path)))
        assert required=={'move_base_executable','fixed_global_planner',active}
        if case=='missing_active_plugin':raise ValueError('required mapped ELF role absent')
        raw_path.write_text('fixture raw maps saved')
        return {'proc_start_ticks':'wrong' if case=='inventory_start_differs' else '123',
                'loaded_modules':[dict(item,sha256='0'*64 if case=='module_sha_changed' else item['sha256'],roles=[active])]}
    fake.inventory=inventory;monkeypatch.setitem(sys.modules,'mapped_elf_inventory',fake);monkeypatch.setitem(sys.modules,'mapped_elf_inventory_v2',fake)
    passive=types.SimpleNamespace(descendant=lambda pid,parent:case!='unowned',archive=lambda root,item:dict(item))
    request={'node_processes':[dict(process)],'phase':'bootstrap','source_ns':0,'planner_parameters':{'base_local_planner':'teb_local_planner/TebLocalPlannerROS'if method=='actual_teb_common_route'else'dwa_local_planner/DWAPlannerROS','base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'}}
    if case=='missing_process':request['node_processes']=[]
    if case=='duplicate_process':request['node_processes']*=2
    write(marker,request)
    if case in ('missing_process','duplicate_process','unowned'):
        with pytest.raises(ValueError):manager.capture_baseline_runtime(root,marker,proc,value,release,passive)
        assert not (root/'baseline_process_bootstrap.json').exists();return
    manager.capture_baseline_runtime(root,marker,proc,value,release,passive);assert not calls
    request['phase']='terminal';request['source_ns']=20_000_000_000
    if case=='changed_pid':request['node_processes'][0]['pid']+=1
    if case=='changed_start':request['node_processes'][0]['proc_start_ticks']='124'
    write(marker,request)
    if case!='valid':
        with pytest.raises(ValueError):manager.capture_baseline_runtime(root,marker,proc,value,release,passive)
        assert not (root/'baseline_runtime_inventory.json').exists();return
    manager.capture_baseline_runtime(root,marker,proc,value,release,passive)
    report=json.loads((root/'baseline_runtime_inventory.json').read_bytes())
    assert report['pass_']is True and report['method']==method and report['source_ns']==20_000_000_000
    assert report['loaded_modules'][0]['original_path']==str(module_path)


def test_method_recorder_topics_are_barriered_but_not_faked_as_route_consumption():
    for method in manager.METHODS:
        topics=manager.recorder_topics(method)
        assert len(topics)==len(set(topics)) and '/astra/common_route/identity'in topics and '/astra/common_route/served_path'in topics
        if method=='astra_analytic_route_v1':assert '/astra/route_runtime/candidates'in topics and '/astra/route_runtime/trace'in topics
        elif method=='frozen_r2_v060_shared_activation':assert '/astra/frozen_r2_execution_identity'in topics
        else:
            planner='TebLocalPlannerROS' if method=='actual_teb_common_route' else 'DWAPlannerROS'
            assert all('/move_base/'+planner+'/'+key in topics for key in ('local_plan','global_plan'))


@pytest.mark.parametrize('case',['missing_schema','wrong_schema','missing_lifecycle_authority','wrong_lifecycle_authority','unreviewed_pose_writer'])
def test_static_closure_review_requirements_rejected_before_allocation(release_fixture,case):
    """Actual verify_release must enforce static predicates used after the live run."""
    f=release_fixture
    f.review.update(schema='astra_navigation_source_review_v1',
        navigation_lifecycle_authority='collector_only_spawn_unpause_pause_fixed_gateway',
        robot_pose_velocity_writes=[])
    if case=='missing_schema':f.review.pop('schema')
    elif case=='wrong_schema':f.review['schema']='astra_passive_q05_source_review_v1'
    elif case=='missing_lifecycle_authority':f.review.pop('navigation_lifecycle_authority')
    elif case=='wrong_lifecycle_authority':f.review['navigation_lifecycle_authority']='unknown'
    else:f.review['robot_pose_velocity_writes']=['unreviewed SetWorldPose']
    with pytest.raises(ValueError):f.seal()


@pytest.mark.parametrize('case',['conversion_input_mismatch','conversion_output_mismatch','conversion_manifest_unbound','conversion_resource_roster_changed'])
def test_conversion_runtime_copy_inputs_bound_before_allocation(release_fixture,case):
    f=release_fixture;unit=f.prepared['units'][0]
    manifest=Path(unit['conversion_manifest']['path']);data=json.loads(manifest.read_bytes())
    if case=='conversion_manifest_unbound':
        f.release['resources']=[r for r in f.release['resources']if r['path']!=str(manifest)]
    elif case=='conversion_resource_roster_changed':unit['conversion_resources']=unit['conversion_resources'][:-1]
    else:
        key='input_urdf'if case=='conversion_input_mismatch'else'converted_sdf'
        changed=manifest.parent/'wrong.xml';changed.write_text('<different/>');data[key]=manager.evidence.binding(changed)
        replacement=write(manifest,data);unit['conversion_manifest']=replacement
        f.release['resources']=[replacement if r['path']==str(manifest)else r for r in f.release['resources']]
    with pytest.raises(ValueError):f.seal()


def test_runtime_world_scene_and_converter_artifacts_are_copied_before_fake_launch(execution_fixture):
    f=execution_fixture;assert f.execute()==0
    for row in f.ledger()['units']:
        root=f.stage/'runs'/row['run_id'];contract=json.loads((root/'contract.json').read_bytes())
        assert (root/'world.sdf').read_bytes()==manager.evidence.read_bound(contract['world'])
        assert (root/'scene_manifest.yaml').read_bytes()==manager.evidence.read_bound(contract['scene_manifest'])
        manifest=json.loads((root/'conversion_manifest.json').read_bytes())
        assert (root/'converted_robot.sdf').read_bytes()==manager.evidence.read_bound(manifest['converted_sdf'])
        assert json.loads((root/'shared_bundle_binding.json').read_bytes())==contract['shared_bundle']


@pytest.mark.parametrize('case',['native_reference_failed','native_source_failed','engine_role_missing','engine_role_duplicate','engine_unbound','source_unbound','native_record_unbound','native_wrong_hash','actor_review_missing'])
def test_static_native_and_engine_closure_enforced_preallocation(release_fixture,case):
    f=release_fixture
    if case=='native_reference_failed':f.native_facts['native_reference_pass']=False
    elif case=='native_source_failed':f.native_facts['upstream_source_review_pass']=False
    elif case=='engine_role_missing':f.review['engine_module_bindings'].pop()
    elif case=='engine_role_duplicate':f.review['engine_module_bindings'].append(f.review['engine_module_bindings'][0])
    elif case=='actor_review_missing':f.review.pop('actor_updates')
    elif case=='native_wrong_hash':f.review['native_engine_records']['sha256']='0'*64
    else:
        row=(f.review['engine_module_bindings'][0]if case=='engine_unbound'else f.review['engine_source_bindings'][0]if case=='source_unbound'else f.review['native_engine_records'])
        f.release['resources']=[r for r in f.release['resources']if r['path']!=row['path']]
    with pytest.raises(ValueError):f.seal()
