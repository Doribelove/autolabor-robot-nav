#!/usr/bin/env python3
"""Read closed bag, actual planner maps and saved launch parameters; no ROS runtime."""
import hashlib,json,math,sys,csv,collections
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/astra_navigation_n03_dwa_common_route'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag,mapped_elf_inventory as maps,yaml
read=lambda p:json.loads(Path(p).read_bytes());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();raw=read(R/'collector_raw.json');c=read(R/'contract.json');bundle=read(c['shared_bundle']['path']);first_gate=first_goal=None;ox,oy,theta=bundle['map_from_odom_xy_yaw'];gx,gy,gyaw=bundle['goal_xy_yaw'];odoms=[];commands=[];served=[]
with rosbag.Bag(str(R/'fixture.bag'))as bag:
 for topic,m,t,header in bag.read_messages(topics=['/odom','/astra/method_cmd','/cmd_vel_navigation','/astra/common_route/served_path'],return_connection_header=True):
  if topic=='/odom':
   ns=m.header.stamp.to_nsec();p=m.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+theta;x=ox+math.cos(theta)*p.position.x-math.sin(theta)*p.position.y;y=oy+math.sin(theta)*p.position.x+math.cos(theta)*p.position.y;odoms.append((ns,m.twist.twist.linear.y))
   if ns>=c['activation_source_ns']and x>=bundle['start_xy_yaw'][0]and first_gate is None:first_gate=ns
   if ns>=raw['start_ns']and math.hypot(x-gx,y-gy)<=.25 and abs(math.atan2(math.sin(yaw-gyaw),math.cos(yaw-gyaw)))<=.25 and first_goal is None:first_goal=ns
  elif topic.endswith('served_path'):served.append(m)
  else:
   caller=header.get('callerid',header.get(b'callerid'));caller=caller.decode()if isinstance(caller,bytes)else caller
   commands.append({'topic':topic,'bag_receipt_ns':t.to_nsec(),'callerid':caller,'vx':m.linear.x,'vy':m.linear.y,'vz':m.linear.z,'wx':m.angular.x,'wy':m.angular.y,'wz':m.angular.z})
inventory=read(R/'baseline_runtime_inventory.json');before=read(R/'baseline_process_bootstrap.json');roles={'move_base':'move_base_executable','libastra_common_route_global_planner.so':'fixed_global_planner','libteb_local_planner.so':'actual_teb','libdwa_local_planner.so':'actual_dwa'}
replay=maps.inventory_from_text((R/'baseline_final.maps.txt').read_text(),roles,{'move_base_executable','fixed_global_planner','actual_dwa'});key=lambda x:(x.get('original_path',x['path']),x['sha256'],x['build_id'],tuple(x['roles']),x['device_major'],x['device_minor'],x['inode'])
checks={'first_actual_odom_gate_and_goal':first_gate==raw['start_ns']and first_goal==raw['end_ns'],'full_20ms_odom_through_watermark':[x[0]for x in odoms]==list(range(20000000,raw['final_drain']['source_watermark_ns']+1,20000000)),'actual_dwa_fixed_global_movebase_maps_exact':sorted(map(key,replay['loaded_modules']))==sorted(map(key,inventory['loaded_modules'])),'raw_maps_hash':sha(R/'baseline_final.maps.txt')==inventory['raw_maps']['sha256'],'same_bootstrap_pid_startticks':inventory['pid']==before['pid']and inventory['proc_start_ticks']==str(before['proc_start_ticks']),'actual_planner_class_bootstrap_terminal':raw['planner_parameters']=={'bootstrap':{'base_local_planner':'dwa_local_planner/DWAPlannerROS','base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'},'terminal':{'base_local_planner':'dwa_local_planner/DWAPlannerROS','base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'}},'mapped_bound_binaries_exact':all(sha(x['path'])==x['sha256']for x in inventory['loaded_modules']),'all_actual_command_fields_finite':all(all(math.isfinite(x[k])for k in('vx','vy','vz','wx','wy','wz'))for x in commands),'all_actual_commands_planar_strict_1e12':all(abs(x[k])<=1e-12 for x in commands for k in('vy','vz','wx','wy')),'private_command_source_actual_movebase':set(x['callerid']for x in commands if x['topic']=='/astra/method_cmd')=={'/move_base'},'public_command_source_coordinator':set(x['callerid']for x in commands if x['topic']=='/cmd_vel_navigation')=={'/astra_navigation_coordinator'},'served_fixed_path_present':len(served)>0}
old=W/'src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml';new=W/'src/simulation/m2_gazebo/config/benchmark_dwa_nonholonomic_r1.yaml';a=yaml.safe_load(old.read_text());b=yaml.safe_load(new.read_text());expected=json.loads(json.dumps(a));expected['DWAPlannerROS']['acc_lim_y']=2.5
checks['configuration_only_acc_lim_y0_to2_5']=b==expected and a['DWAPlannerROS']['acc_lim_y']==0.
parameters={}
for line in(R/'roslaunch.log').read_text().splitlines():
 if line.strip().startswith('* /move_base/DWAPlannerROS/'):
  name,value=line.strip().split(': ',1);parameters[name.split('/')[-1]]=value
checks['actual_roslaunch_parameter_summary_2_5_and_zero_y']=all(parameters[k]==v for k,v in {'acc_lim_y':'2.5','min_vel_y':'0.0','max_vel_y':'0.0','vy_samples':'1'}.items())
checks['actual_odometry_within_declared_lateral_sampling_domain']=max(abs(y)for _,y in odoms)<=.125
with(D/'actual_commands.csv').open('x',newline='')as f:
 writer=csv.DictWriter(f,fieldnames=list(commands[0]));writer.writeheader();writer.writerows(commands)
with(D/'actual_lateral_odometry.csv').open('x',newline='')as f:
 writer=csv.writer(f);writer.writerow(['source_ns','actual_body_vy_mps']);writer.writerows(odoms)
out={'pass':all(checks.values()),'checks':checks,'actual_command_counts':dict(collections.Counter(x['topic']for x in commands)),'max_abs_actual_body_vy_mps':max(abs(y)for _,y in odoms),'max_abs_command_vy_mps':max(abs(x['vy'])for x in commands),'baseline_pid':inventory['pid'],'baseline_startticks':inventory['proc_start_ticks'],'loaded_modules':inventory['loaded_modules'],'startup_DWA_parameter_summary':parameters,'first_gate_ns':first_gate,'first_goal_ns':first_goal,'limits':['This is actual saved launch parameter summary and actual planner-class RPC evidence, not a full live DWA parameter dump at every timestep.','No exact headerless Twist generation-time/odom callback pairing or costmap winner replay.','The observed |actual body vy|<=.125 domain supports this finite run only; acc_lim_y2.5 cannot guarantee zero-vy commands outside that domain.','N01 negative remains unchanged; fresh N03 is single config integration evidence, not a paired performance comparison.'],'resources':[{'path':str(p.resolve()),'sha256':sha(p)}for p in(Path(__file__),R/'fixture.bag',R/'baseline_final.maps.txt',R/'baseline_runtime_inventory.json',R/'baseline_process_bootstrap.json',R/'roslaunch.log',old,new,D/'actual_commands.csv',D/'actual_lateral_odometry.csv')]}
(D/'dwa_actual_audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'commands':out['actual_command_counts'],'maxvy':out['max_abs_actual_body_vy_mps']},indent=2));assert out['pass']
