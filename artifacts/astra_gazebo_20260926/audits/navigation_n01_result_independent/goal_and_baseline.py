#!/usr/bin/env python3
"""Read actual source odometry and saved planner maps; no node/process access."""
import hashlib,json,math,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag,mapped_elf_inventory as maps
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
rows=[]
for method in ('astra_analytic_route_v1','actual_teb_common_route','frozen_r2_v060_shared_activation'):
 R=S/'runs'/('astra_navigation_n01_'+method);raw=read(R/'collector_raw.json');c=read(R/'contract.json');bundle=read(Path(c['shared_bundle']['path']));first_gate=None;first_goal=None;ox,oy,theta=bundle['map_from_odom_xy_yaw'];gx,gy,gyaw=bundle['goal_xy_yaw'];odomrows=[]
 with rosbag.Bag(str(R/'fixture.bag'))as bag:
  for _,m,_ in bag.read_messages(topics=['/odom']):
   t=m.header.stamp.to_nsec();p=m.pose.pose;z=p.orientation;yaw=math.atan2(2*(z.w*z.z+z.x*z.y),1-2*(z.y*z.y+z.z*z.z))+theta
   x=ox+math.cos(theta)*p.position.x-math.sin(theta)*p.position.y;y=oy+math.sin(theta)*p.position.x+math.cos(theta)*p.position.y;odomrows.append((t,x,y,yaw))
   if t>=c['activation_source_ns']and x>=bundle['start_xy_yaw'][0]and first_gate is None:first_gate=t
   if t>=raw['start_ns']and math.hypot(x-gx,y-gy)<=.25 and abs(math.atan2(math.sin(yaw-gyaw),math.cos(yaw-gyaw)))<=.25 and first_goal is None:first_goal=t
 rows.append({'method':method,'first_actual_odom_gate_ns':first_gate,'declared_gate_ns':raw['start_ns'],'gate_exact':first_gate==raw['start_ns'],'first_actual_odom_goal_ns':first_goal,'declared_terminal_ns':raw['end_ns'],'goal_exact':first_goal==raw['end_ns'],'odom_source_stamps_strict_20ms_full': [r[0]for r in odomrows]==list(range(20000000,raw['final_drain']['source_watermark_ns']+1,20000000))})
R=S/'runs/astra_navigation_n01_actual_teb_common_route';inventory=read(R/'baseline_runtime_inventory.json');before=read(R/'baseline_process_bootstrap.json')
roles={'move_base':'move_base_executable','libastra_common_route_global_planner.so':'fixed_global_planner','libteb_local_planner.so':'actual_teb','libdwa_local_planner.so':'actual_dwa'}
replay=maps.inventory_from_text((R/'baseline_final.maps.txt').read_text(),roles,{'move_base_executable','fixed_global_planner','actual_teb'})
key=lambda x:(x.get('original_path',x['path']),x['sha256'],x['build_id'],tuple(x['roles']),x['device_major'],x['device_minor'],x['inode'])
baseline={'actual_final_mapped_modules_exact':sorted(map(key,replay['loaded_modules']))==sorted(map(key,inventory['loaded_modules'])),'raw_map_binding_exact':sha(R/'baseline_final.maps.txt')==inventory['raw_maps']['sha256'],'same_bootstrap_pid_starttick':inventory['pid']==before['pid']and inventory['proc_start_ticks']==str(before['proc_start_ticks']),'actual_parameter_classes_exact':inventory['planner_parameters']=={'base_local_planner':'teb_local_planner/TebLocalPlannerROS','base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'},'mapped_archive_module_SHA_exact':all(sha(Path(x['path']))==x['sha256']for x in inventory['loaded_modules'])}
checks={'all_three_odometry_gates_first_actual_crossing':all(x['gate_exact']for x in rows),'all_three_terminal_first_actual_odom_goal':all(x['goal_exact']for x in rows),'all_three_20ms_odom_source_grids':all(x['odom_source_stamps_strict_20ms_full']for x in rows),'actual_TEB_saved_process_and_plugins_bound':all(baseline.values())}
out={'checks':checks,'pass':all(checks.values()),'rows':rows,'TEB_baseline':baseline,'DWA_limitation':'Final move_base ELF inventory absent in failed DWA attempt; no completed actual-plugin runtime closure. Bootstrap process and configured DWA parameter exist; they do not substitute final mappings.','scope':'First formal native geometry goal is 500Hz; coordinator goal is first qualifying actual 50Hz odometry source, and both times remain distinct.','script_SHA':sha(Path(__file__))}
with(D/'goal_and_baseline_audit.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps(out,indent=2));assert out['pass']
