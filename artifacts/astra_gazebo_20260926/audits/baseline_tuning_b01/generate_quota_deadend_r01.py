#!/usr/bin/env python3
"""Outcome-free B01 proposal:12 distinct coarse configurations per baseline.

Only new YAML files and audit artifacts are generated. No ROS, simulator,
experiment seed, release, benchmark, final outcomes or training are accessed.
"""
import argparse,ast,collections,copy,csv,hashlib,io,itertools,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
O=Path(__file__).resolve().parent;S=O.parents[1];W=S/'workspace'
C=W/'src/simulation/m2_gazebo/config';OUT=C/'astra_baseline_b01'
T=W/'src/navigation_arena/forks/navigation/local_planner/teb'
D=S/'audits/navigation_n01_dwa_diagnosis/upstream_navigation_1.17.3'
R=Path('/opt/ros/noetic');INPUTS={}
def read(p):
 p=Path(p).resolve();b=p.read_bytes();INPUTS[str(p)]=hashlib.sha256(b).hexdigest();return b
def y(p):return yaml.safe_load(read(p))
def canonical(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False)
def digest(v):return hashlib.sha256(canonical(v).encode()).hexdigest()
def merged(a,b):
 a=copy.deepcopy(a)
 for k,v in b.items():a[k]=merged(a[k],v)if isinstance(v,dict)and isinstance(a.get(k),dict)else copy.deepcopy(v)
 return a
def cfg_fields(p):
 result={}
 for n in ast.walk(ast.parse(read(p).decode())):
  if not isinstance(n,ast.Call)or not isinstance(n.func,ast.Attribute)or n.func.attr!='add' or len(n.args)<5:continue
  try:
   key=ast.literal_eval(n.args[0]);default=ast.literal_eval(n.args[4]);minimum=ast.literal_eval(n.args[5])if len(n.args)>5 else None;maximum=ast.literal_eval(n.args[6])if len(n.args)>6 else None
  except (ValueError,TypeError):continue
  result[key]={'type':n.args[1].id,'default':default,'min':minimum,'max':maximum,'source_line':n.lineno}
 return result
def validate(field,value):
 if field['type']=='bool_t':assert type(value)is bool
 elif field['type']=='int_t':assert type(value)is int
 elif field['type']=='double_t':assert type(value)in(int,float)
 assert field['min']is None or value>=field['min'];assert field['max']is None or value<=field['max']
def space_fill(planner,axes,anchor,inactive):
 names=list(axes);active=[k for k in names if k not in inactive]
 allrows=[dict(zip(names,x))for x in itertools.product(*(axes[k]for k in names))]
 def fp(row):return canonical({k:row[k]for k in active})
 def dist(a,b):return sum(((float(a[k])-min(axes[k]))/(max(axes[k])-min(axes[k]))-(float(b[k])-min(axes[k]))/(max(axes[k])-min(axes[k])))**2 for k in active)/len(active)
 selected=[anchor];trace=[{'step':0,'selection':'current_effective_anchor','minimum_squared_active_distance':None}]
 while len(selected)<12:
  counts={k:collections.Counter(x[k]for x in selected)for k in names};used={fp(x)for x in selected};pool=[]
  for candidate in allrows:
   if fp(candidate)in used:continue
   if any(counts[k][candidate[k]]>=math.ceil(12/len(axes[k]))for k in names):continue
   distances=[dist(candidate,x)for x in selected]
   coverage=sum(counts[k][candidate[k]]for k in names)
   pool.append((min(distances),-coverage,sum(distances),digest([planner,candidate]),candidate))
  assert pool,'quota-constrained candidate pool empty'
  best=max(pool,key=lambda x:x[:4]);selected.append(best[4]);trace.append({'step':len(selected)-1,'selection':'deterministic_greedy_maximin','minimum_squared_active_distance':best[0],'marginal_occupancy_penalty':-best[1],'tie_sha256':best[3]})
 assert len({fp(x)for x in selected})==12
 assert all(set(axes[k])<=set(x[k]for x in selected)for k in names)
 return selected,{'space_size':len(allrows),'distance':'mean squared range-normalized distance over source-confirmed active axes; maximize nearest-selected distance, then lower marginal occupancy, then sumdistance, then SHA256 lexical max','active_uniqueness_axes':active,'excluded_from_distance_and_uniqueness':inactive,'level_counts':{k:{str(v):sum(x[k]==v for x in selected)for v in axes[k]}for k in names},'outside_grid_anchor_values':{k:anchor[k]for k in names if anchor[k]not in axes[k]},'minimum_squared_active_distance':min(dist(a,b)for i,a in enumerate(selected)for b in selected[:i]),'trace':trace}

def generate():
 read(__file__);mechanism=y(S/'contracts/mechanism_plan.yaml');tuning=mechanism['baseline_tuning'];assert tuning['minimum_coarse_configurations_per_planner']==12
 footprint=y(C/'astra_navigation_common_footprint.yaml');teb=merged(y(C/'benchmark_teb_v0.yaml')['TebLocalPlannerROS'],footprint['TebLocalPlannerROS']);dwa=y(C/'benchmark_dwa_nonholonomic_r1.yaml')['DWAPlannerROS']
 tf=cfg_fields(T/'cfg/TebLocalPlannerReconfigure.cfg');df=cfg_fields(D/'dwa_local_planner/cfg/DWAPlanner.cfg');cf=cfg_fields(R/'share/costmap_converter/cfg/dynamic_reconfigure/CostmapToDynamicObstacles.cfg')
 tcpp=read(T/'src/teb_config.cpp').decode();tros=read(T/'src/teb_local_planner_ros.cpp').decode();th=read(T/'include/teb_local_planner/teb_config.h').decode();dcpp=read(D/'dwa_local_planner/src/dwa_planner.cpp').decode()
 assert ET.fromstring(read(T/'package.xml')).findtext('version')=='0.8.4'
 assert ET.fromstring(read(R/'share/dwa_local_planner/package.xml')).findtext('version')=='1.17.3'
 assert ET.fromstring(read(R/'share/costmap_converter/package.xml')).findtext('version')=='0.0.13'
 assert 'costmap_converter_rate'not in tcpp and 'obstacles.costmap_converter_rate = 5;'in th and 'startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate)'in tros
 assert dcpp.count('stop_time_buffer_')==1 and 'stop_time_buffer_ = config.stop_time_buffer;'in dcpp
 assert 'dist_threshold *= 0.85;'in tros
 local=y(C/'local_costmap.yaml');assert local['local_costmap']['width']==local['local_costmap']['height']==12.
 for p in [T/'src/optimal_planner.cpp',R/'share/teb_local_planner/package.xml',R/'share/costmap_converter/plugins.xml',R/'include/pluginlib/class_loader_imp.hpp',R/'include/costmap_converter/costmap_converter_interface.h',R/'include/costmap_converter/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.h',R/'include/dwa_local_planner/dwa_planner.h',D/'dwa_local_planner/src/dwa_planner_ros.cpp',D/'base_local_planner/src/simple_trajectory_generator.cpp',D/'base_local_planner/src/local_planner_limits/__init__.py',C/'costmap_common.yaml',C/'benchmark_move_base.yaml',C/'astra_common_map_route.yaml',W/'src/simulation/m2_gazebo/launch/astra_common_navigation_n02.launch',W/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp',W/'src/simulation/m2_gazebo/include/m2_gazebo/ackermann_kinematics.h',W/'src/simulation/m2_gazebo/include/m2_gazebo/actuator_dynamics.h',W/'src/simulation/m2_gazebo/launch/spawn_m2_physical_p05.launch',W/'scripts/astra_gazebo/navigation_coordinator_n02.py',S/'scenes/navigation_n04_development_r01/reference_robot/converted_robot.sdf',S/'audits/navigation_n04_scenes_independent/review.json']:
  read(p)
 ta={'dt_ref':teb['dt_ref'],'min_obstacle_dist':teb['min_obstacle_dist'],'inflation_dist':teb['inflation_dist'],'weight_obstacle':teb['weight_obstacle'],'lookahead_m':teb['max_global_plan_lookahead_dist'],'homotopy':teb['enable_homotopy_class_planning'],'dynamic_scan_converter':False}
 da={k:dwa[k]for k in tuning['dwa_axes']}
 tc,tm=space_fill('teb',tuning['teb_axes'],ta,['lookahead_m']);dc,dm=space_fill('dwa',tuning['dwa_axes'],da,['stop_time_buffer'])
 tmap={'lookahead_m':'max_global_plan_lookahead_dist','homotopy':'enable_homotopy_class_planning'}
 configs={};records=[];schema_assertions=0
 converter={k:v['default']for k,v in cf.items()};assert converter['dt']==.2
 for planner,candidates,base,fields,namespace in [('teb',tc,teb,tf,'TebLocalPlannerROS'),('dwa',dc,dwa,df,'DWAPlannerROS')]:
  for i,candidate in enumerate(candidates):
   params=copy.deepcopy(base)
   if planner=='teb':
    for key,val in candidate.items():
     if key=='dynamic_scan_converter':continue
     target=tmap.get(key,key);params[target]=val
     if target in fields:validate(fields[target],val);schema_assertions+=1
     else:assert 'nh.param("'+target+'"'in tcpp
    if candidate['dynamic_scan_converter']:
     params.update(include_dynamic_obstacles=True,costmap_converter_plugin='costmap_converter::CostmapToDynamicObstacles',costmap_converter_spin_thread=True,dynamic_obstacle_inflation_dist=candidate['inflation_dist'],weight_dynamic_obstacle=candidate['weight_obstacle'],weight_dynamic_obstacle_inflation=base['weight_inflation'])
     params['costmap_converter/CostmapToDynamicObstacles']=converter
    elif i!=0:params.update(include_dynamic_obstacles=False,costmap_converter_plugin='')
    assert params['footprint_model']==footprint['TebLocalPlannerROS']['footprint_model'] and params['min_obstacle_dist']<params['inflation_dist'] and params['dt_ref']>params['dt_hysteresis']
   else:
    params.update(candidate)
    for k,v in candidate.items():validate(fields[k],v);schema_assertions+=1
   fixed=('max_vel_x','max_vel_theta','acc_lim_x','acc_lim_theta','xy_goal_tolerance','yaw_goal_tolerance')
   assert all(params[k]==base[k]for k in fixed) and params['max_vel_x']==1.2 and params['max_vel_y']==0
   if planner=='dwa':assert params['max_vel_trans']==1.2 and params['min_vel_y']==0 and params['acc_lim_y']==2.5 and params['vy_samples']==1
   else:assert params['wheelbase']==.65 and params['min_turning_radius']==1.2 and params['cmd_angle_instead_rotvel']is False
   cid='%s_b01_%02d%s'%(planner,i,'_anchor'if i==0 else '')
   text='# B01 offline candidate only; no execution authority. Load AFTER the shared footprint.\n'+yaml.safe_dump({namespace:params},sort_keys=False)
   configs[cid+'.yaml']=text
   active=(tm if planner=='teb'else dm)['active_uniqueness_axes']
   records.append({'candidate_id':cid,'planner':planner,'anchor':i==0,'candidate_index':i,'axes':candidate,'confirmed_active_fingerprint_sha256':digest({k:candidate[k]for k in active}),'config_path':str(OUT/(cid+'.yaml')),'config_sha256':hashlib.sha256(text.encode()).hexdigest(),'dynamic_converter_component_qualified':False if planner=='teb'and candidate['dynamic_scan_converter']else None,'runtime_ready':False,'changed_from_effective_anchor':{k:v for k,v in params.items()if k not in base or base[k]!=v},'removed_anchor_keys':sorted(set(base)-set(params))})
 assert len(records)==24 and schema_assertions>=144
 assert yaml.safe_load(configs['teb_b01_00_anchor.yaml'])['TebLocalPlannerROS']==teb and yaml.safe_load(configs['dwa_b01_00_anchor.yaml'])['DWAPlannerROS']==dwa
 for val in tuning['teb_axes']['min_obstacle_dist']:
  old=merged({'TebLocalPlannerROS':dict(teb,min_obstacle_dist=val)},footprint);new=merged(footprint,{'TebLocalPlannerROS':dict(teb,min_obstacle_dist=val)})
  assert old['TebLocalPlannerROS']['min_obstacle_dist']==.33 and new['TebLocalPlannerROS']['min_obstacle_dist']==val
 manifest={'schema':'astra_baseline_b01_offline_coarse_design_v1','stage':'baseline_tuning_b01','status':'offline_configs_generated_pending_independent_and_component_review','execution_authorized':False,'runtime_ready':False,'seeds_allocated':[],'authorized_episodes':0,'final_outcomes_read':False,'simulator_started':False,'candidate_count_per_planner':12,'candidate_counts':dict(collections.Counter(r['planner']for r in records)),'actual_versions':{'TEB_active_workspace_source':'0.8.4','TEB_system_apt_not_selected':ET.fromstring(read(R/'share/teb_local_planner/package.xml')).findtext('version'),'DWA_installed':'1.17.3','costmap_converter_installed':'0.0.13'},'matrix_design':{'teb':tm,'dwa':dm},'candidates':records,'schema_axis_value_checks':schema_assertions,'load_order':['existing shared move_base/costmap configuration','original compound footprint reference','exactly one full B01 planner YAML last'],'shared_footprint_binding':{'path':str(C/'astra_navigation_common_footprint.yaml'),'sha256':INPUTS[str((C/'astra_navigation_common_footprint.yaml').resolve())]},'converter_bundle':{'requested':True,'runtime_supported_or_qualified':False,'plugin':'costmap_converter::CostmapToDynamicObstacles','namespace_relative_to_TebLocalPlannerROS':'costmap_converter/CostmapToDynamicObstacles','source_effective_worker_rate_hz':5,'yaml_rate_key_written':False,'tracker_dt_s':.2,'parameters':converter,'input_scope':'TEB local Costmap2D from common static prior and actual /scan obstacle layer; /odom only for ego compensation; no actor/model truth topics','coupled_axes':{'include_dynamic_obstacles':'dynamic_scan_converter','plugin':'dynamic_scan_converter','dynamic_obstacle_inflation_dist':'inflation_dist','weight_dynamic_obstacle':'weight_obstacle','weight_dynamic_obstacle_inflation':'fixed anchor weight_inflation .20'}},'source_caveats':['TEB rate YAML is not read by this workspace0.8.4; effective source default5Hz, not legacyv7 requested10Hz.','TEB lookahead6/8/10 can be censored by common12m local-map radial cutoff5.1m; excluded from effective-uniqueness distance.','DWA stop_time_buffer is assigned but never used by this version planner source; excluded from effective-uniqueness distance.','DWA holonomic_robot anchor key has no verified source consumer; actual nonholonomic inputs are min/max_vel_y=0,vy_samples1,acc_lim_y2.5.','Current shared coordinator passes method Twist through; physicalP05 maps yaw to steering and limits reference rates. There is no claim that raw planner Twist already obeys Ackermann slew or that actualphysical acceleration equals reference limits.'], 'source_resources':[{'path':p,'sha256':h}for p,h in sorted(INPUTS.items())]}
 return configs,manifest

def main():
 p=argparse.ArgumentParser();p.add_argument('--check-only',action='store_true');args=p.parse_args();configs,manifest=generate()
 if args.check_only:
  assert all((OUT/n).read_text()==v for n,v in configs.items());saved=json.loads((O/'candidate_manifest.json').read_text());assert saved==manifest
  print(json.dumps({'status':'deterministic_regeneration_and_source_rehash_pass','configurations':len(configs),'source_bindings':len(INPUTS),'schema_axis_checks':manifest['schema_axis_value_checks']}));return
 if OUT.exists():raise FileExistsError('Preserve existing B01 configuration directory')
 OUT.mkdir()
 for name,text in configs.items():
  with(OUT/name).open('x')as f:f.write(text)
 with(O/'candidate_manifest.json').open('x')as f:json.dump(manifest,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
 for planner in ('teb','dwa'):
  records=[r for r in manifest['candidates']if r['planner']==planner];axes=list(records[0]['axes'])
  with(O/(planner+'_coarse.csv')).open('x')as f:
   writer=csv.DictWriter(f,fieldnames=['candidate_id','anchor']+axes+['confirmed_active_fingerprint_sha256','config_sha256']);writer.writeheader()
   for r in records:row={k:r[k]for k in ('candidate_id','anchor','confirmed_active_fingerprint_sha256','config_sha256')};row.update(r['axes']);writer.writerow(row)
 assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in INPUTS.items())
 print(json.dumps({'status':manifest['status'],'candidates':len(configs),'effective_unique_per_planner':12,'source_bindings':len(INPUTS),'schema_axis_checks':manifest['schema_axis_value_checks'],'matrix_design':manifest['matrix_design']},indent=2))
if __name__=='__main__':main()
