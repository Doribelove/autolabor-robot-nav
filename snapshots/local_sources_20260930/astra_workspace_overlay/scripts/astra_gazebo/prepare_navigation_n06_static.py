#!/usr/bin/env python3
"""N06 four-block paired preparation only: no release, registry or simulator.

Copy the reviewed-input candidate D04 raster/guide exactly. The per-run world
adds the same frozen passive physical observation shell to each paired method.
"""
import argparse
import ast
import copy
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import yaml
import navigation_evidence_n06 as evidence

STAGE=evidence.STAGE
WORKSPACE=evidence.WORKSPACE
G=WORKSPACE/'src/simulation/m2_gazebo'
NAME='astra_navigation_n06_lookahead_static_pilot'
METHODS=('astra_analytic_route_v1','astra_analytic_lookahead_1p2_n06')
DESIGN=STAGE/'contracts/navigation_n06_lookahead_static_pilot_draft.json'
D04=STAGE/'scenes/development_d04_geometry_r05'
D04_REPORT=STAGE/'audits/development_geometry_d04_r05/implementation_report.json'
D04_REPORT_SHA='8cadc1a752ea0d08791b6d035d454f92fa11f5510e2473802278fe2892b955f2'
D04_MANIFEST_SHA='538b4b2504ea7a25db0f2bdd0d1f45d90429c490e1068a3e47aab3c8aba25d9e'
SHELL=STAGE/'scenes/navigation_n01_clear_r03/world.sdf'
LAUNCH=G/'launch/astra_common_navigation_n06.launch'
SCENES=STAGE/'scenes/navigation_n06_static_prepared_r01'
PREPARED=STAGE/'contracts/navigation_n06_prepared_r01'

def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def binding(path):return {'path':str(Path(path).resolve()),'sha256':sha(path)}
def write_new(path,data):
    with Path(path).open('x') as f:json.dump(data,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')

def validate_design(value):
    factor=value['single_behavioral_factor'];pop=value['proposed_population']
    if (value['stage']!=NAME or value['execution_authorized']is not False or type(value['allocated_budget'])is not int or value['allocated_budget']!=0
        or value['proposed_maximum_episodes']!=8 or value['methods']!=list(METHODS)
        or factor['control_m']!=.7 or factor['treatment_m']!=1.2
        or factor['N05_preview_eligibility_used']is not False
        or factor['control_and_treatment_use_original_backup_only_analytic_selector']is not True
        or pop['execution_seed_proposal']!=list(range(263003001,263003005))
        or pop['paired_blocks']!=4 or pop['independent_new_parent_geometries']!=4
        or pop['scene_family']!='static_avoidance_only' or pop['split']!='development'):
        raise ValueError('exact zero-authority N06 single-factor design required')
    return value

def schedule():
    rows=[]
    for block in range(4):
        order=METHODS if block%2==0 else tuple(reversed(METHODS))
        for position,method in enumerate(order):
            arm='control' if method==METHODS[0] else 'treatment'
            rows.append({'sequence':len(rows)+1,'block_index':block,'pair_position':position+1,
                'run_id':'astra_navigation_n06_b%03d_%s'%(block,arm),'method':method,'arm':arm,
                'seed':263003001+block,'seed_status':'proposal_unallocated','attempt':1,
                'layout_id':'d04r05_development_static_avoidance_%03d'%block,
                'scene_id':'navigation_n06_static_b%03d'%block,'lookahead_m':.7 if arm=='control' else 1.2})
    return rows

def verify_d04():
    if sha(D04_REPORT)!=D04_REPORT_SHA:raise ValueError('frozen D04 author report changed')
    report=read(D04_REPORT);mb=report['source_evidence_manifest']
    if mb['sha256']!=D04_MANIFEST_SHA or sha(mb['path'])!=D04_MANIFEST_SHA:raise ValueError('D04 manifest changed')
    manifest=read(mb['path'])
    for key in ('owned_generator_and_tests','final_geometry_inputs','retained_author_audits','read_only_dependencies'):
        for item in manifest[key]:
            if sha(item['path'])!=item['sha256']:raise ValueError('D04 bound input changed: '+item['path'])
    layouts=[s for s in read(D04/'layouts.json') if s['family']=='static_avoidance']
    if [s['layout_id'] for s in layouts]!=[x['layout_id'] for x in schedule()[::2]]:raise ValueError('exact four parent roster changed')
    if len({s['parent_geometry_id'] for s in layouts})!=4:raise ValueError('four distinct parent geometries required')
    for s in layouts:
        if s['split']!='development' or s['actors'] or s['actor_parameter_cell_id']!='actor_none_v1' or s['parent_geometry_retained_from_r04']is not False:raise ValueError('exact new static development parent required')
        qc=read(D04/s['layout_id']/'raster_and_guide/raster_guide_checks.json')
        if not qc['passed'] or qc['raster_witness']['selected']['continuous_conservative_lower_m']<=0 or qc['guide']['minimum_full_footprint_conservative_gap_m']<=0:raise ValueError('D04 raster/guide check failed')
    return layouts

def treatment_equivalence():
    control=G/'scripts/astra_route_coordinated_runtime.py';treatment=G/'scripts/astra_route_lookahead_runtime_n06.py'
    expected=control.read_text()
    substitutions=[('Shared-gate adaptation of the frozen standalone route node. R2 is separate.','N06 analytic shared-gate route node with a 1.2 m lookahead factor.'),
        ("method not in ('astra_analytic_route_v1','astra_pct_route_v1'):raise ValueError('new method identity and positive seed required')","method!='astra_analytic_lookahead_1p2_n06':raise ValueError('N06 lookahead identity and positive seed required')"),
        ('config=RouteConfig(maximum_episode_duration_s=','config=RouteConfig(lookahead_m=1.2,maximum_episode_duration_s=')]
    for old,new in substitutions:
        if expected.count(old)!=1:raise ValueError('control source context changed')
        expected=expected.replace(old,new)
    if expected!=treatment.read_text():raise ValueError('treatment has a nonfactor source change')
    from route_candidate_core import RouteConfig
    a=vars(RouteConfig());b=vars(RouteConfig(lookahead_m=1.2))
    delta={k:[a[k],b[k]] for k in a if a[k]!=b[k]}
    if delta!={'lookahead_m':[.7,1.2]}:raise ValueError('configuration changes beyond lookahead')
    for p in (control,treatment):
        tree=ast.parse(p.read_text())
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        if 'route_preview_selection_n05' in imports:raise ValueError('N05 selection adapter forbidden')
    return {'control':binding(control),'treatment':binding(treatment),'exact_allowed_text_substitutions':substitutions,
        'configuration_delta':delta,'candidate_generator_mask_selection_command_code_unchanged':True,
        'N05_preview_selector_used':False,'same_observation_schema':True,'same_core_class':True,
        'new_semantics_hash_expected_due_to_config':True}

def collision_signature(xml,static_only=True):
    root=ET.fromstring(xml);rows=[]
    for model in root.findall('world/model'):
        if static_only and model.get('name') in ('ground_plane','astra_n01_camera','astra_n06_camera'):continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                rows.append([model.get('name'),model.findtext('pose','0 0 0 0 0 0'),link.get('name'),link.findtext('pose','0 0 0 0 0 0'),collision.get('name'),collision.findtext('pose','0 0 0 0 0 0'),ET.tostring(collision.find('geometry'),encoding='unicode')])
    return sorted(rows)

def world_from_parent(parent_dir):
    native=(parent_dir/'raster_and_guide/static_geometry.sdf').read_text()
    template=ET.fromstring(SHELL.read_text());world=template.find('world');world.set('name','astra_navigation_n06_static')
    for model in list(world.findall('model')):
        if model.get('name') not in ('ground_plane','astra_n01_camera'):world.remove(model)
    camera=world.find("model[@name='astra_n01_camera']")
    if camera is None:raise ValueError('frozen shell camera missing')
    camera.set('name','astra_n06_camera')
    plugin=camera.find('.//plugin');plugin.set('name','n06_camera');plugin.find('robotNamespace').text='/astra_n06';plugin.find('frameName').text='astra_n06_camera_optical'
    for model in ET.fromstring(native).findall('world/model'):world.append(copy.deepcopy(model))
    text=ET.tostring(template,encoding='unicode')+'\n'
    if collision_signature(text)!=collision_signature(native):raise ValueError('D04 static collision geometry changed')
    plugins={x.get('filename') for x in world.findall('plugin')}
    if plugins!={'libastra_dense_evidence_plugin.so','libastra_physics_metadata_p03_plugin.so'}:raise ValueError('physical observation shell changed')
    return text

def nodes():
    return ['/rosout','/gazebo','/fixture_bag','/astra_navigation_collector','/m2_robot_state_publisher',
        '/navigation_command_relay','/astra_common_route_provider','/astra_navigation_coordinator','/astra_route_runtime']

def topics():
    return list(dict.fromkeys(list(evidence.ESSENTIAL_BEFORE_ARM)+['/astra/common_route/identity','/astra/common_route/served_path',
        '/astra/route_runtime/identity','/astra/route_runtime/status','/astra/route_runtime/trace','/astra/route_runtime/candidates']))

def render_launch(contract,contract_path,output):
    root=ET.Element('launch');inc=ET.SubElement(root,'include',file=str(LAUNCH))
    args={'world':contract['world']['path'],'run_id':contract['run_id'],'seed':str(contract['seed']),'method':contract['method'],
        'bundle':str(Path(contract['shared_bundle']['path']).parent),'bundle_sha256':contract['shared_bundle']['sha256'],
        'scene_manifest':contract['scene_manifest']['path'],'scene_id':contract['scene_id'],'legacy_report_path':str(output/'unused_legacy_report.json'),
        'steering_kd':'6.0','collector_script':'astra_navigation_collector_n06.py','contract_path':str(contract_path),
        'contract_sha256':sha(contract_path),'output':str(output),'bag_path':str(output/'fixture.bag'),'bag_topics':' '.join(contract['record_topics'])}
    for key,value in args.items():ET.SubElement(inc,'arg',name=key,value=value)
    return ET.tostring(root,encoding='unicode')+'\n'

def prepare(scene_root=SCENES,prepared_root=PREPARED):
    scene_root,prepared_root=Path(scene_root).resolve(),Path(prepared_root).resolve()
    validate_design(read(DESIGN));parents=verify_d04();equivalence=treatment_equivalence()
    if scene_root.exists() or prepared_root.exists():raise FileExistsError('preserve previous N06 preparation, including failures')
    scene_root.mkdir(parents=True);prepared_root.mkdir(parents=True)
    rows=schedule();blocks=[];units=[]
    try:
        write_new(prepared_root/'schedule.json',{'stage':NAME,'execution_authorized':False,'allocated_budget':0,'proposed_maximum_episodes':8,
            'seeds_registered':False,'source_design':binding(DESIGN),'schedule':rows,'retry_allowed':False,'resume_allowed':False})
        write_new(prepared_root/'factor_equivalence.json',equivalence)
        for block,parent in enumerate(parents):
            source=D04/parent['layout_id'];folder=scene_root/parent['layout_id'];folder.mkdir()
            shutil.copytree(source/'raster_and_guide',folder/'d04_raster_and_guide')
            shutil.copytree(source/'raster_and_guide/common_bundle',folder/'common_bundle')
            for name in ('geometry_manifest.json','geometry_checks.json','reference_witness.json','reference_witness.csv'):
                shutil.copyfile(source/name,folder/('d04_'+name))
            (folder/'world.sdf').write_text(world_from_parent(source))
            scene=copy.deepcopy(parent)
            scene.update(schema='astra_navigation_n06_static_prepared_scene_v1',scene_id='navigation_n06_static_b%03d'%block,
                seed=263003001+block,seed_status='proposal_unallocated',paired_method_roster=list(METHODS),
                environment_version='astra_n06_static_physical_p05_kd6_p03_observer',
                camera={'topic':'/astra_n06/camera/image_raw','width':2560,'height':1440,'rate_hz':2},
                robot_collision_reference={'reference':'base_link','spawn_xyz_rpy':[-5.4,0.,0.,0.,0.,0.],'full_collision_count':9},
                d04_parent_manifest=binding(source/'geometry_manifest.json'),d04_author_report=binding(D04_REPORT),
                d04_raster_checks=binding(source/'raster_and_guide/raster_guide_checks.json'),
                d04_independent_review='pending_separate_root_binding_before_release',
                global_seed_registry_modified=False,new_seed_allocated=False,execution_authorized=False,
                source_derivation_scope='physical observation shell and proposed identity only; exact D04 static geometry/map/guide',
                independent_new_geometry_relative_to_D04=False)
            (folder/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
            blocks.append({'block_index':block,'layout_id':parent['layout_id'],'parent_geometry_id':parent['parent_geometry_id'],
                'seed':scene['seed'],'world':binding(folder/'world.sdf'),'manifest':binding(folder/'scene_manifest.yaml'),
                'bundle':binding(folder/'common_bundle/common_route.json'),'raster_witness':binding(folder/'d04_raster_and_guide/raster_witness.csv')})
        for row in rows:
            folder=prepared_root/row['run_id'];folder.mkdir();block=blocks[row['block_index']]
            shutil.copyfile(block['world']['path'],folder/'world.sdf')
            command=['/opt/ros/noetic/bin/xacro','--inorder',str(G/'urdf/m2_physical_p05.urdf.xacro'),'dynamics_seed:='+str(row['seed']),'steering_kd:=6.0']
            result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=30)
            (folder/'xacro.stderr.log').write_text(result.stderr);(folder/'robot_description.xml').write_text(result.stdout)
            write_new(folder/'xacro_command.json',{'argv':command,'returncode':result.returncode,'starts_ROS':False})
            if result.returncode:raise RuntimeError('xacro offline expansion failed')
            value=evidence.make_contract(stage=NAME,run_id=row['run_id'],seed=row['seed'],method=row['method'],scene_id=row['scene_id'],
                world=folder/'world.sdf',robot_description=folder/'robot_description.xml',scene_manifest=block['manifest']['path'],
                shared_bundle=Path(block['bundle']['path']).parent,spawn_world_xyz_yaw=[-5.4,0.,0.,0.],steering_kd=6.,
                allowed_nodes=nodes(),required_nodes=nodes(),record_topics=topics(),required_before_arm=topics(),camera_topic='/astra_n06/camera/image_raw')
            write_new(folder/'contract.json',value)
            gate={'schema':'astra_n06_shared_gate_expectation_v1','run_id':row['run_id'],'seed':row['seed'],'method':row['method'],
                'activation_ns':5_000_000_000,'gate':'first eligible actual odometry source crossing world x>=-4',
                'warmup_speed_mps':.8,'timeout_ns':22_000_000_000,'base_goal_xy_tolerance_m':.25,'base_goal_yaw_tolerance_rad':.25,
                'goal_authority':'shared coordinator actual base odometry','lookahead_m':row['lookahead_m'],
                'execution_authorized':False,'current_source_timestamp_available':False,'N05_preview_selector_used':False}
            write_new(folder/'gate_expectation.json',gate)
            (folder/'fixture.launch').write_text(render_launch(value,folder/'contract.json',STAGE/'runs'/row['run_id']))
            units.append(dict(row,contract=binding(folder/'contract.json'),launch=binding(folder/'fixture.launch'),
                gate_expectation=binding(folder/'gate_expectation.json'),robot_description=binding(folder/'robot_description.xml'),world=binding(folder/'world.sdf')))
        result={'stage':NAME,'status':'offline_prepared_no_authority_independent_review_pending','execution_authorized':False,
            'allocated_budget':0,'proposed_maximum_episodes':8,'seeds_registered':False,'ROS_started':False,
            'design':binding(DESIGN),'source':binding(__file__),'D04_author_report':binding(D04_REPORT),'D04_independent_review_pending':True,
            'single_factor_equivalence':binding(prepared_root/'factor_equivalence.json'),'blocks':blocks,'units':units,
            'future_prerequisites':['independent D04 split/raster review','independent N06 source/input/launch review','exact runtime/conversion closure','new root release and seed reservation'],
            'assessment':'paired static development diagnosis only; retain task/integrity failures; no performance promotion',
            'retry_allowed':False,'resume_allowed':False,'training_authorized':False,'formal_result':False}
        write_new(prepared_root/'prepared.json',result)
        return result
    except Exception as exc:
        write_new(prepared_root/'retained_preparation_failure.json',{'stage':NAME,'exception':repr(exc),'created_blocks':len(blocks),'created_units':len(units),
            'execution_authorized':False,'seeds_registered':False,'ROS_started':False})
        raise

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene-root',type=Path,default=SCENES);p.add_argument('--prepared-root',type=Path,default=PREPARED)
    a=p.parse_args();result=prepare(a.scene_root,a.prepared_root);print(json.dumps({'status':result['status'],'blocks':len(result['blocks']),'units':len(result['units'])}))
