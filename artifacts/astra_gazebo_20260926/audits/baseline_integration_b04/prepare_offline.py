"""Only B04 parameter expansion fixtures. Never launches nodes or allocates seeds."""
import hashlib,json,os,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
S=Path(__file__).resolve().parents[2];W=S/'workspace';G=W/'src/simulation/m2_gazebo';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import navigation_evidence_b04 as q
import baseline_rear_evidence_b04 as rear
ROOT=Path(sys.argv[1]).resolve();ROOT.mkdir(exist_ok=False)
assert ROOT.parent==Path(__file__).resolve().parent
SCENE=S/'scenes/navigation_n03_dwa_r01';LAUNCH=G/'launch/astra_baseline_rear_b04.launch'
def bound(p):return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def jsonwrite(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def launchfile(path,args):
    root=ET.Element('launch');inc=ET.SubElement(root,'include',file=str(LAUNCH))
    for k,v in args.items():ET.SubElement(inc,'arg',name=k,value=str(v))
    path.write_text(ET.tostring(root,encoding='unicode')+'\n')
def dump(path,out):
    result=subprocess.run(['roslaunch','--dump-params',str(path)],capture_output=True,timeout=40)
    (out.with_suffix('.yaml')).write_bytes(result.stdout);out.with_suffix('.stderr.log').write_bytes(result.stderr)
    if result.returncode:raise RuntimeError('parameter expansion failed '+str(result.returncode))
    return yaml.safe_load(result.stdout)
def nested(flat,prefix):
    root={}
    for key,value in flat.items():
        if not key.startswith(prefix+'/'):continue
        parts=key[len(prefix)+1:].split('/');current=root
        for part in parts[:-1]:current=current.setdefault(part,{})
        current[parts[-1]]=value
    return root
rows=[]
for method in rear.METHODS:
    out=ROOT/method;out.mkdir();scene=yaml.safe_load((SCENE/'scene_manifest.yaml').read_bytes())
    scene.update(seed=1,scene_id='b04_offline_static_fixture',split='offline_parameter_expansion_only',execution_authorized=False)
    (out/'scene_manifest.yaml').write_text(yaml.safe_dump(scene,sort_keys=False))
    # seed=1 is synthetic syntax input, not a registered experimental identity.
    run_id='b04_offline_'+('teb' if method==rear.METHODS[0] else 'dwa')
    nodes=['/rosout','/gazebo','/fixture_bag','/astra_navigation_collector','/navigation_command_relay','/astra_navigation_coordinator','/astra_common_route_provider','/m2_robot_state_publisher','/astra_baseline_rear_adapter_b03','/move_base']
    args={'world':SCENE/'world.sdf','run_id':run_id,'seed':1,'method':method,'bundle':SCENE/'common_bundle','bundle_sha256':bound(SCENE/'common_bundle/common_route.json')['sha256'],'steering_kd':6.,'contract_path':out/'contract.json','contract_sha256':'0'*64,'output':S/'runs'/run_id,'bag_path':S/'runs'/run_id/'fixture.bag','bag_topics':' '.join(q.ESSENTIAL_BEFORE_ARM),'python_executable':W/'.venv_train_cuda/bin/python'}
    launchfile(out/'initial_expansion.launch',args);first=dump(out/'initial_expansion.launch',out/'initial_parameters')
    (out/'robot_description.xml').write_text(first['/robot_description'])
    value=q.make_contract(stage='astra_baseline_b04_offline',run_id=run_id,seed=1,method=method,scene_id=scene['scene_id'],world=SCENE/'world.sdf',robot_description=out/'robot_description.xml',scene_manifest=out/'scene_manifest.yaml',shared_bundle=SCENE/'common_bundle',spawn_world_xyz_yaw=[-5.4,0.,0.,0.],steering_kd=6.,allowed_nodes=nodes,required_nodes=nodes,camera_topic='/astra_n01/camera/image_raw')
    jsonwrite(out/'contract.json',value);args['contract_sha256']=bound(out/'contract.json')['sha256'];launchfile(out/'fixture.launch',args)
    actual=dump(out/'fixture.launch',out/'parameters');urdf=actual['/robot_description'];q.validate_robot_description(urdf,value)
    assert hashlib.sha256(urdf.encode()).hexdigest()==value['robot_description']['sha256']
    full=nested(actual,'/move_base');checked=rear.planner_parameters(full,method)
    assert actual['/astra_navigation_coordinator/method']==actual['/astra_baseline_rear_adapter_b03/method']==method
    assert actual['/astra_navigation_coordinator/bundle_sha256']==actual['/astra_baseline_rear_adapter_b03/bundle_sha256']==value['shared_bundle']['sha256']
    assert actual['/astra_navigation_collector/contract_sha256']==args['contract_sha256']
    # Parse actual launch objects; no ROS master or Gazebo process is started.
    import roslaunch
    config=roslaunch.config.load_config_default([str(out/'fixture.launch')],None)
    move=next(n for n in config.nodes if n.name=='move_base');collector=next(n for n in config.nodes if n.name=='astra_navigation_collector');server=next(n for n in config.nodes if n.name=='gazebo')
    assert ('cmd_vel','/astra/method_cmd') in [tuple(x) for x in move.remap_args] and ('odom','/astra/rear_odom') in [tuple(x) for x in move.remap_args]
    assert collector.required and collector.type=='astra_navigation_collector_b04.py';assert '-u' in server.args.split()
    assert all('spawn_model' not in n.type for n in config.nodes)
    config_summary=[{'package':n.package,'type':n.type,'name':n.name,'required':n.required,'args':n.args,'remaps':n.remap_args,'launch_prefix':n.launch_prefix} for n in config.nodes]
    jsonwrite(out/'resolved_nodes.json',config_summary);jsonwrite(out/'effective_move_base.json',full)
    selected=full['TebLocalPlannerROS' if method==rear.METHODS[0] else 'DWAPlannerROS']
    assert selected['max_vel_x']==1.2
    rows.append(dict(method=method,pass_=True,planner=checked,contract=bound(out/'contract.json'),launch=bound(out/'fixture.launch'),parameters=bound(out/'parameters.yaml'),nodes=bound(out/'resolved_nodes.json'),urdf=bound(out/'robot_description.xml'),goal_xy=selected['xy_goal_tolerance'],goal_yaw=selected['yaw_goal_tolerance'],source_boundaries='original base odom and physical P05 Kd6 unchanged'))
report={'schema':'astra_baseline_integration_b04_offline_launch_v1','status':'two_actual_launch_expansions_and_contract_validators_pass','author_check':True,'execution_authorized':False,'ROS_or_Gazebo_started':False,'seed_allocated':False,'release_written':False,'synthetic_seed':1,'rows':rows,'resources':[bound(Path(q.__file__)),bound(Path(rear.__file__)),bound(LAUNCH),bound(SCENE/'world.sdf'),bound(SCENE/'scene_manifest.yaml'),bound(SCENE/'common_bundle/common_route.json'),bound(Path(__file__))]}
jsonwrite(ROOT/'report.json',report);print(json.dumps({'status':report['status'],'report':bound(ROOT/'report.json')}))
