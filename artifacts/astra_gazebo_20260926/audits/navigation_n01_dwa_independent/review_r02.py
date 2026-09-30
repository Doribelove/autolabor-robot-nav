#!/usr/bin/env python3
"""Independent raw-bag and native-library diagnosis review, no ROS initialisation."""
import ast,hashlib,json,math,struct,sys,datetime
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';E=S/'audits/navigation_n01_dwa_diagnosis';R=S/'runs/astra_navigation_n01_dwa_common_route'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag,yaml
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
f32=lambda x:struct.unpack('f',struct.pack('f',x))[0]
diag=read(E/'diagnosis.json');checks={}
checks['frozen_diagnosis_SHA']=sha(E/'diagnosis.json')=='0aeabfd90532cd635436c2936c4a141daeebb29bcfe481669e5f3ea33dc5c867'
checks['all_saved_diagnostic_input_source_artifacts_rehash']=all(sha(x['path'])==x['sha256']for x in diag['input_bindings']+diag['artifacts']+diag['upstream_sources'])
commands=[];odometry=[];callers=[]
with rosbag.Bag(str(R/'fixture.bag'))as bag:
 callers=sorted({c.header.get('callerid').decode('utf-8')if isinstance(c.header.get('callerid'),bytes)else c.header.get('callerid')for c in bag._get_connections(topics=['/astra/method_cmd'])})
 for topic,m,t in bag.read_messages(topics=['/astra/method_cmd','/odom']):
  if topic=='/astra/method_cmd':commands.append({'topic':topic,'bag_receipt_ns':t.to_nsec(),'vx':m.linear.x,'vy':m.linear.y,'vz':m.linear.z,'wx':m.angular.x,'wy':m.angular.y,'wz':m.angular.z,'nonplanar_or_nonfinite':not all(math.isfinite(v)for v in(m.linear.y,m.linear.z,m.angular.x,m.angular.y))or any(abs(v)>1e-12 for v in(m.linear.y,m.linear.z,m.angular.x,m.angular.y))})
  else:odometry.append({'source_ns':m.header.stamp.to_nsec(),'bag_receipt_ns':t.to_nsec(),'vx':m.twist.twist.linear.x,'vy':m.twist.twist.linear.y,'wz':m.twist.twist.angular.z})
checks['actual_bag_all_four_method_commands_exact_diagnosis']=commands==diag['raw_method_commands']and len(commands)==4 and callers==['/move_base']
checks['actual_command_failure_is_finite_lateral_only']=all(math.isfinite(x[k])for x in commands for k in ('vx','vy','vz','wx','wy','wz'))and all(x[k]==0 for x in commands for k in ('vz','wx','wy'))and all(abs(x['vy'])>1e-12 for x in commands[1:])
matches=[]
for cmd in commands[1:]:
 m=[x for x in odometry if x['source_ns']<=cmd['bag_receipt_ns']and f32(x['vy'])==cmd['vy']]
 matches.append({'command_bag_receipt_ns':cmd['bag_receipt_ns'],'command_vy':cmd['vy'],'matches':m})
checks['three_unique_causal_candidate_odom_float32_value_matches']=all(len(x['matches'])==1 for x in matches)and [x['matches'][0]for x in matches]==[x['matching_odom']for x in diag['actual_float32_matches']]
checks['all393_native_probe_odom_inputs_equal_actual_bag']=odometry==[json.loads(x)for x in(E/'source_odometry.jsonl').read_text().splitlines()]and len(odometry)==393
native=[json.loads(x)for x in(D/'native_replay_output.jsonl').read_text().splitlines()];bycase={x['case']:x for x in native};checks['actual1212_native_replay_byte_exact']=len(native)==len(bycase)==1212 and sha(D/'native_replay_output.jsonl')==sha(E/'native_probe_output.jsonl')
counts={str(a):sum(all(y['vy']==0 for y in x['y_samples'])for x in native if x['case'].startswith('odom_')and x['acc_lim_y']==a)for a in (0.,.6,2.5)}
checks['saved393_odom_zero_only_counts_0_392_393']=counts=={'0.0':0,'0.6':392,'2.5':393}
checks['zero_acc_nonzero_vy_inverts_native_bounds_and_expands_one_sample']=all(bycase[s+'_a0.0']['dynamic_lower']>bycase[s+'_a0.0']['dynamic_upper']and len(bycase[s+'_a0.0']['y_samples'])==2 and any(x['vy']!=0 and x['has_generator_accepted_trajectory']for x in bycase[s+'_a0.0']['y_samples'])for s in('positive','negative'))
boundary=[json.loads(x)for x in(D/'boundary_output.jsonl').read_text().splitlines()]
checks['own18_float32_boundary_cases_exact_plusminus0125_domain']=len(boundary)==18 and all((all(y['vy']==0 for y in x['y_samples']))==(abs(x['float32_vy'])<=.125)for x in boundary if x['acc_lim_y']==2.5)and all(any(y['vy']!=0 and y['has_generator_accepted_trajectory']for y in x['y_samples'])for x in boundary if x['acc_lim_y']==2.5 and abs(x['float32_vy'])>.125)
old=Path(diag['selected_config']['old']['path']);new=Path(diag['selected_config']['new']['path']);a=yaml.safe_load(old.read_text());b=yaml.safe_load(new.read_text());delta=[(k,a['DWAPlannerROS'][k],b['DWAPlannerROS'][k])for k in a['DWAPlannerROS']if a['DWAPlannerROS'][k]!=b['DWAPlannerROS'][k]]
checks['single_config_field_only_acc_lim_y_0_to_2point5']=set(a)==set(b)=={'DWAPlannerROS'}and set(a['DWAPlannerROS'])==set(b['DWAPlannerROS'])and delta==[('acc_lim_y',0.,2.5)]and new.read_text().replace('  acc_lim_y: 2.5\n','  acc_lim_y: 0.0\n')==old.read_text()
checks['raw_odom_topic_and_hard_lateral_zero_limits_unchanged']=b['DWAPlannerROS']['odom_topic']=='/odom'and b['DWAPlannerROS']['min_vel_y']==b['DWAPlannerROS']['max_vel_y']==0 and b['DWAPlannerROS']['vy_samples']==1
oldnode=W/'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator.py';newnode=W/'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator_n02.py'
body=lambda p:ast.dump(next(n for n in ast.walk(ast.parse(p.read_text()))if isinstance(n,ast.FunctionDef)and n.name=='on_method_command'),include_attributes=False)
checks['strict_command_guard_AST_unchanged']=body(oldnode)==body(newnode)
checks['no_historical_final_DWA_maps_or_assessment_claimed']=not(R/'baseline_runtime_inventory.json').exists()and not(R/'assessment.json').exists()and read(R/'receipt.json')['integrity_pass']is False and diag['native_probe']['run_dwa_final_mapped_elf_proof_available']is False
out={'status':'independent_DWA_native_window_cause_and_single_field_config_review_pass'if all(checks.values())else'independent_DWA_review_failed','pass':all(checks.values()),'independent_review':True,'checks':checks,'diagnosis':bind(E/'diagnosis.json'),'actual_float32_matches':matches,'native_cases_replayed':1212,'additional_independent_boundary_cases':18,'zero_only_counts_by_acceleration':counts,'configuration_delta':delta,'old_config':bind(old),'new_config':bind(new),
 'domain':'For fixed0.05s DWA window, acc_lim_y2.5 admits only zero lateral targets when abs(float32 measured vy)<=0.125m/s. Own next-representable-float32 outside both boundaries still emits accepted nonzero target; unchanged1e-12 guard rejects it.',
 'causal_strength':'Actual finite nonzero linear.y caused the strict method-command fault. Three raw command values equal unique earlier raw odometry afterfloat32 conversion. Installed native generator reproduces the inversion and endpoint acceptance; headerless messages do not prove the exact planner input callback join, and this probe does not reconstruct costmap scoring/winning trajectory.',
 'configuration_scope':'Virtual planner dynamic-window allowance; not real lateral actuation or a physical acceleration guarantee. It changes the candidate set and may change ranking, so only a fresh separately released DWA identity may qualify it.',
 'preserved_negative':'N01 source, raw, gate and failure remain unchanged. Missing historical final ELF closure is not replaced by current probe mappings. No live fix or performance claim.',
 'actual_commands':[str(E/'native_dynamic_window_probe')+' '+str(D/'native_replay.maps.txt')+' < '+str(E/'native_probe_inputs.txt')+' > '+str(D/'native_replay_output.jsonl'),str(E/'native_dynamic_window_probe')+' '+str(D/'boundary.maps.txt')+' < '+str(D/'boundary_inputs.txt')+' > '+str(D/'boundary_output.jsonl')],
 'resources':[bind(p)for p in [Path(__file__),oldnode,newnode,E/'native_dynamic_window_probe',E/'native_dynamic_window_probe.cpp',E/'native_probe_inputs.txt',D/'native_replay_output.jsonl',D/'native_replay.maps.txt',D/'boundary_inputs.txt',D/'boundary_output.jsonl',D/'boundary.maps.txt',R/'fixture.bag',D/'tool_notes.txt']],
 'ROS_started':False,'Gazebo_started':False,'new_seed_allocated':False,'live_qualification':False,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
with(D/'review_r02.json').open('x')as f:json.dump(out,f,indent=2,ensure_ascii=False);f.write('\n')
with(D/'review_r02.md').open('x')as f:f.write('DWA 根因与单字段配置独立审查通过。实际bag四条方法命令均为有限值；后三条仅横向vy不满足原严格平面接口，分别与唯一更早odom的float32值完全吻合。安装native generator的1212输入重执行输出逐字节一致；额外18个正负float32边界验证2.5仅覆盖|vy|≤0.125 m/s，紧邻域外仍输出非零横向样本。\n\n新配置仅acc_lim_y从0变2.5，/odom与min/max_vel_y=0、vy_samples=1及原命令检查AST均未改。此量是规划器虚拟速度窗裕量，不是实际侧向执行器能力或未来输入保证。headerless Twist不提供精确规划生成时刻；当前native库探针不补充历史缺失的DWA最终ELF闭包，也未复算costmap胜者。旧N01阴性保持，未来修补仍需新身份实跑资格。\n')
print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'SHA':sha(D/'review_r02.json')},indent=2));assert out['pass']
