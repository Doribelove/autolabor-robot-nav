#!/usr/bin/env python3
"""Independent actual prepared-input audit. Offline ROS XML parser only."""
import ast, copy, csv, hashlib, json, os, pathlib, sys, xml.etree.ElementTree as ET
import yaml

S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
W=S/'workspace';G=W/'src/simulation/m2_gazebo';P=W/'scripts/astra_gazebo'
A=S/'audits/navigation_n06_preparation';D=pathlib.Path(__file__).parent
sys.path.insert(0,str(P))
import navigation_evidence_n06 as evidence
from route_candidate_core import RouteConfig,RouteAwareRuntime
from common_route import load_bundle
import roslaunch.config
import roslib.packages

READ={};checks=[]
def read(p):
    p=pathlib.Path(p).resolve();b=p.read_bytes();READ[str(p)]=hashlib.sha256(b).hexdigest();return b
def js(p):return json.loads(read(p))
def check(name,value,details=None):
    checks.append(dict(name=name,passed=bool(value),details=details))
    if not value:raise AssertionError((name,details))
def bindings(x):
    if isinstance(x,dict):
        if isinstance(x.get('path'),str) and isinstance(x.get('sha256'),str):yield x
        for value in x.values():yield from bindings(value)
    elif isinstance(x,list):
        for value in x:yield from bindings(value)
def bound(b):
    value=read(b['path']);assert hashlib.sha256(value).hexdigest()==b['sha256'],b;return value
def with_replacements(a,b,replacements):
    value=read(a).decode()
    for old,new in replacements:
        assert value.count(old)==1,(str(a),old,value.count(old));value=value.replace(old,new)
    assert value==read(b).decode(),(a,b)
def tree_signature(x):
    return [x.tag,dict(sorted(x.attrib.items())),(x.text or '').strip(),[tree_signature(c) for c in x]]
def run():
    author=js(A/'implementation_report.json');manifest=js(A/'source_evidence_manifest.json')
    refs=list(bindings(manifest))
    for item in refs:bound(item)
    check('all292_frozen_author_resources',len({r['path'] for r in refs})==292,dict(occurrences=len(refs),unique_paths=len({r['path'] for r in refs})))
    d04=js(S/'audits/development_geometry_d04_r05_independent/review.json')
    check('new_D04_independent_qualification_separately_bound',d04['status']=='independent_offline_joint_split_dedup_and_four_static_raster_review_pass' and d04['execution_authorized']is False)
    methods=['astra_analytic_route_v1','astra_analytic_lookahead_1p2_n06']
    oldmethods="METHODS=('astra_analytic_preview_eligible_n05','astra_analytic_route_v1','astra_pct_route_v1','frozen_r2_v060_sensor_lifecycle_v1','actual_teb_common_route','dwa_common_route')"
    newmethods="METHODS=('astra_analytic_route_v1','astra_analytic_lookahead_1p2_n06')"
    with_replacements(P/'navigation_coordinator_n05.py',P/'navigation_coordinator_n06.py',[(oldmethods,newmethods)])
    with_replacements(G/'scripts/astra_navigation_coordinator_n05.py',G/'scripts/astra_navigation_coordinator_n06.py',[('from navigation_coordinator_n05 import','from navigation_coordinator_n06 import')])
    with_replacements(G/'scripts/astra_navigation_collector_n05.py',G/'scripts/astra_navigation_collector_n06.py',[('import navigation_evidence_n05 as q','import navigation_evidence_n06 as q')])
    substitutions=[("SCHEMA='astra_navigation_evidence_n05_v1'","SCHEMA='astra_navigation_evidence_n06_v1'"),(oldmethods,newmethods)]
    for name in ('navigation_evidence','astra_navigation_collector','navigation_coordinator','astra_navigation_coordinator'):
        substitutions.append((name+'_n05.py',name+'_n06.py'))
    with_replacements(P/'navigation_evidence_n05.py',P/'navigation_evidence_n06.py',substitutions)
    with_replacements(G/'launch/astra_common_navigation_n05.launch',G/'launch/astra_common_navigation_n06.launch',[
        ('astra_navigation_coordinator_n05.py','astra_navigation_coordinator_n06.py'),('astra_analytic_preview_eligible_n05','astra_analytic_lookahead_1p2_n06'),('astra_route_preview_runtime_n05.py','astra_route_lookahead_runtime_n06.py')])
    with_replacements(G/'scripts/astra_route_coordinated_runtime.py',G/'scripts/astra_route_lookahead_runtime_n06.py',[
        ('Shared-gate adaptation of the frozen standalone route node. R2 is separate.','N06 analytic shared-gate route node with a 1.2 m lookahead factor.'),
        ("method not in ('astra_analytic_route_v1','astra_pct_route_v1'):raise ValueError('new method identity and positive seed required')","method!='astra_analytic_lookahead_1p2_n06':raise ValueError('N06 lookahead identity and positive seed required')"),
        ('config=RouteConfig(maximum_episode_duration_s=','config=RouteConfig(lookahead_m=1.2,maximum_episode_duration_s=')])
    check('all_runtime_deltas_exhaustively_whitelisted',True,'collector and coordinator control logic inherited exactly; only method/schema/module identities plus treatment lookahead=1.2')
    cfg0=vars(RouteConfig());cfg1=vars(RouteConfig(lookahead_m=1.2));delta={k:[cfg0[k],cfg1[k]]for k in cfg0 if cfg0[k]!=cfg1[k]}
    check('only_numerical_configuration_factor',delta=={'lookahead_m':[.7,1.2]},delta)
    prepared=js(S/'contracts/navigation_n06_prepared_r01/prepared.json');schedule=js(S/'contracts/navigation_n06_prepared_r01/schedule.json');design=js(S/'contracts/navigation_n06_lookahead_static_pilot_draft.json')
    for obj in (prepared,schedule,design):assert obj['execution_authorized']is False and type(obj['allocated_budget'])is int and obj['allocated_budget']==0
    assert len(prepared['blocks'])==4 and len(prepared['units'])==8 and not prepared['seeds_registered']
    assert schedule['retry_allowed']is False and schedule['resume_allowed']is False
    for i,(u,row) in enumerate(zip(prepared['units'],schedule['schedule'])):
        block=i//2;position=i%2;ordered=methods if block%2==0 else methods[::-1];method=ordered[position];arm='control' if method==methods[0] else 'treatment'
        expected=dict(sequence=i+1,block_index=block,pair_position=position+1,method=method,arm=arm,seed=263003001+block,attempt=1,seed_status='proposal_unallocated',run_id=f'astra_navigation_n06_b{block:03d}_{arm}',layout_id=f'd04r05_development_static_avoidance_{block:03d}',scene_id=f'navigation_n06_static_b{block:03d}',lookahead_m=.7 if arm=='control' else 1.2)
        assert row==expected and all(u[k]==v for k,v in expected.items())
    check('exact_four_blocks_counterbalanced_8attempt1_proposals_no_authority',True)
    original_registry=read(S/'seed_registry.csv');(D/'seed_registry_at_review.csv').write_bytes(original_registry)
    registry=list(csv.DictReader(original_registry.decode().splitlines()))
    proposed={str(x)for x in range(263003001,263003005)};collisions=[r for r in registry if r['seed']in proposed or r['run_id'].startswith('astra_navigation_n06_')]
    check('proposed_seeds_absent_from_current_registry',not collisions,dict(registry_rows=len(registry),collisions=collisions))
    parent_byid={s['layout_id']:s for s in js(S/'scenes/development_d04_geometry_r05/layouts.json')}
    pair_results=[]
    shell=ET.fromstring(read(S/'scenes/navigation_n01_clear_r03/world.sdf'));shellworld=shell.find('world')
    for block in prepared['blocks']:
        source=S/'scenes/development_d04_geometry_r05'/block['layout_id'];folder=pathlib.Path(block['manifest']['path']).parent
        original=parent_byid[block['layout_id']];scene=yaml.safe_load(bound(block['manifest']))
        assert scene['parent_geometry_id']==original['parent_geometry_id']==block['parent_geometry_id']
        for key in ('actor_geometry','actors','actor_parameter_cell_id','static_boxes','static_circles','corridor_half_width_m','route','joint_allocation'):assert scene[key]==original[key],key
        for p in (source/'raster_and_guide').rglob('*'):
            if p.is_file():assert read(p)==read(folder/'d04_raster_and_guide'/p.relative_to(source/'raster_and_guide'))
        for name in ('geometry_manifest.json','geometry_checks.json','reference_witness.json','reference_witness.csv'):assert read(source/name)==read(folder/('d04_'+name))
        for name in ('map.pgm','map.yaml','common_route.json'):assert read(source/'raster_and_guide/common_bundle'/name)==read(folder/'common_bundle'/name)
        assert bound(block['raster_witness'])==read(source/'raster_and_guide/raster_witness.csv')
        world=ET.fromstring(bound(block['world']));native=ET.fromstring(read(source/'raster_and_guide/static_geometry.sdf'))
        byname={m.get('name'):m for m in world.findall('world/model')};nativebyname={m.get('name'):m for m in native.findall('world/model')}
        assert set(byname)==set(nativebyname)|{'ground_plane','astra_n06_camera'}
        for name,m in nativebyname.items():assert tree_signature(m)==tree_signature(byname[name])
        assert tree_signature(byname['ground_plane'])==tree_signature(shellworld.find("model[@name='ground_plane']"))
        assert not byname['astra_n06_camera'].findall('.//collision')
        camera=copy.deepcopy(shellworld.find("model[@name='astra_n01_camera']"));camera.set('name','astra_n06_camera');plugin=camera.find('.//plugin');plugin.set('name','n06_camera');plugin.find('robotNamespace').text='/astra_n06';plugin.find('frameName').text='astra_n06_camera_optical'
        assert tree_signature(camera)==tree_signature(byname['astra_n06_camera'])
        for name in ('physics','gravity','scene','light','plugin'):
            assert [tree_signature(x)for x in world.findall('world/'+name)]==[tree_signature(x)for x in shellworld.findall(name)],name
        assert not world.findall('world/model/plugin')
        pair=[u for u in prepared['units'] if u['block_index']==block['block_index']];assert len(pair)==2
        contracts=[js(u['contract']['path'])for u in pair]
        assert pair[0]['robot_description']['sha256']==pair[1]['robot_description']['sha256'] and pair[0]['world']['sha256']==pair[1]['world']['sha256']==block['world']['sha256']
        for c in contracts:evidence.validate_contract(c)
        normalized=[]
        for c in contracts:
            item=copy.deepcopy(c)
            for k in ('run_id','method'):del item[k]
            for k in ('world','robot_description'):item[k]=item[k]['sha256']
            normalized.append(item)
        assert normalized[0]==normalized[1]
        grid,route,_=load_bundle(pathlib.Path(block['bundle']['path']).parent);urdf=bound(pair[0]['robot_description']).decode()
        a=RouteAwareRuntime(grid,route,urdf,RouteConfig());b=RouteAwareRuntime(grid,route,urdf,RouteConfig(lookahead_m=1.2))
        x,y=copy.deepcopy(a.identity),copy.deepcopy(b.identity);assert x['config'].pop('lookahead_m')==.7 and y['config'].pop('lookahead_m')==1.2
        assert x==y and a.semantics_sha256!=b.semantics_sha256 and a.actor is None and b.actor is None
        pair_results.append(dict(block_index=block['block_index'],parent_geometry_id=block['parent_geometry_id'],world_sha256=block['world']['sha256'],urdf_sha256=pair[0]['robot_description']['sha256'],control_semantics_sha256=a.semantics_sha256,treatment_semantics_sha256=b.semantics_sha256,remaining_contract_and_core_identity_identical=True))
    check('four_actual_D04_static_inputs_all_copies_shell_and_paired_contracts',len(pair_results)==4,pair_results)
    olddump=js(A/'launch_dump_report.json');expanded=[]
    for unit,prior in zip(prepared['units'],olddump['units']):
        assert unit['run_id']==prior['run_id'];launch=unit['launch'];bound(launch)
        c=js(unit['contract']['path']);cfg=roslaunch.config.load_config_default([launch['path']],None,verbose=False)
        params={key:value.value for key,value in cfg.params.items()};saved=yaml.safe_load(bound(prior['params']))
        assert params==saved,(unit['run_id'],set(params)^set(saved))
        nodes={n.name:n for n in cfg.nodes};expected_script='astra_route_coordinated_runtime.py'if unit['method']==methods[0]else'astra_route_lookahead_runtime_n06.py'
        assert nodes['astra_route_runtime'].type==expected_script and nodes['astra_route_runtime'].launch_prefix==str(W/'.venv/bin/python')
        assert [list(x)for x in nodes['astra_route_runtime'].remap_args]==[['/cmd_vel_navigation','/astra/method_cmd']]
        assert nodes['astra_navigation_collector'].type=='astra_navigation_collector_n06.py' and nodes['astra_navigation_collector'].required
        assert nodes['astra_navigation_coordinator'].type=='astra_navigation_coordinator_n06.py'
        assert set(nodes)==set(c['required_nodes'])-{'/rosout'} or {'/'+n for n in nodes}==set(c['required_nodes'])-{'/rosout'}
        assert nodes['navigation_command_relay'].args=='/cmd_vel_navigation /cmd_vel'
        assert params['/robot_description'].encode()==bound(unit['robot_description'])
        assert params['/astra_navigation_collector/contract_path']==unit['contract']['path'] and params['/astra_navigation_collector/contract_sha256']==unit['contract']['sha256']
        for node in ('astra_route_runtime','astra_navigation_coordinator'):
            for field in ('method','run_id','seed'):assert params['/'+node+'/'+field]==unit[field]
            assert params['/'+node+'/bundle_sha256']==c['shared_bundle']['sha256']
        assert params['/astra_route_runtime/maximum_episode_duration_s']==22. and params['/astra_route_runtime/checkpoint_metadata']==''
        assert params['/use_sim_time'] is True and params['/m2_gazebo/simulation_only']is True
        robot=ET.fromstring(params['/robot_description']);driver=robot.find(".//plugin[@filename='libastra_physical_ackermann_p05_plugin.so']")
        assert int(driver.findtext('dynamicsSeed'))==unit['seed'] and float(driver.findtext('steeringKd'))==6.
        args=nodes['gazebo'].args.split();assert '-u'in args and args[args.index('--seed')+1]==str(unit['seed']) and c['world']['path']in args
        bag_args=nodes['fixture_bag'].args.split();assert '--lz4'in bag_args and '--buffsize=128'in bag_args
        assert bag_args[bag_args.index('-O')+2:]==c['record_topics']
        assert not (S/'runs'/unit['run_id']).exists()
        gate=js(unit['gate_expectation']['path']);assert gate['current_source_timestamp_available']is False and gate['execution_authorized']is False and gate['lookahead_m']==unit['lookahead_m']
        node_records=[]
        for n in cfg.nodes:
            resolved=roslib.packages.find_node(n.package,n.type);assert resolved
            if n.package=='m2_gazebo':assert [str(pathlib.Path(p).resolve())for p in resolved]==[str((G/'scripts'/n.type).resolve())]
            for p in resolved:assert os.access(p,os.X_OK);read(p)
            node_records.append(dict(name=n.name,package=n.package,type=n.type,args=n.args,required=n.required,remap_args=n.remap_args,launch_prefix=n.launch_prefix,resolved=resolved))
        for p in cfg.roslaunch_files:read(p)
        expanded.append(dict(run_id=unit['run_id'],method=unit['method'],seed=unit['seed'],parameters_match_saved_author_dump=True,nodes=node_records))
    check('eight_launches_reexpanded_offline_exact_params_nodes_remaps_and_identity',len(expanded)==8,expanded)
    check('no_allocations_or_run_directories_created',read(S/'seed_registry.csv')==original_registry and all(not(S/'runs'/u['run_id']).exists() for u in prepared['units']))
    # Actual local conditional import graph reconstruction is conservative over branches.
    queue=[G/'scripts/astra_route_coordinated_runtime.py',G/'scripts/astra_route_lookahead_runtime_n06.py',G/'scripts/astra_navigation_coordinator_n06.py',G/'scripts/astra_navigation_collector_n06.py',G/'scripts/astra_common_route_provider.py',P/'prepare_navigation_n06_static.py'];graph={}
    while queue:
        p=queue.pop().resolve()
        if str(p)in graph:continue
        src=read(p);graph[str(p)]=hashlib.sha256(src).hexdigest()
        for n in ast.walk(ast.parse(src)):
            names=[n.module] if isinstance(n,ast.ImportFrom) else [a.name for a in n.names] if isinstance(n,ast.Import) else []
            for name in names:
                if name:
                    for root in (P,G/'scripts'):
                        child=root/(name.replace('.','/')+'.py')
                        if child.is_file():queue.append(child)
    assert not any('route_preview' in p for p in graph)
    check('no_N05_preview_selector_in_reachable_local_source_graph',True,dict(local_files=len(graph)))
    return dict(schema='astra_n06_preparation_independent_check_v1',passed=True,checks=checks,launch_count=len(expanded),paired_identities=pair_results,source_graph=[dict(path=p,sha256=h)for p,h in sorted(graph.items())],environment=dict(executable=sys.executable,python=sys.version,THESIS_WS=os.environ.get('THESIS_WS'),user_site_disabled=bool(sys.flags.no_user_site)),resources=[dict(path=p,sha256=h)for p,h in sorted(READ.items())])

if __name__=='__main__':
    try:result=run()
    except Exception as e:
        import traceback
        result=dict(passed=False,exception=type(e).__name__+':'+str(e),traceback=traceback.format_exc(),checks=checks,resources=[dict(path=p,sha256=h)for p,h in sorted(READ.items())])
    with (D/sys.argv[1]).open('x')as f:json.dump(result,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
    print(json.dumps(dict(passed=result['passed'],checks=len(checks),exception=result.get('exception'))))
    sys.exit(0 if result['passed'] else 1)
