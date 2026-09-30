#!/usr/bin/env python3
"""Actual closed N02 bag/registration/source/goal audit, no ROS services/nodes."""
from pathlib import Path
import sys,json,hashlib,collections,math,bisect
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/astra_navigation_n02_frozen_r2_v060_sensor_lifecycle_v1'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag
read=lambda p:json.loads(Path(p).read_bytes());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
raw=read(R/'collector_raw.json');contract=read(R/'contract.json');release=read(S/'contracts/navigation_n02_release.json');legacy=read(R/'legacy_report.json');data=collections.defaultdict(list);scans={};odoms={};tf=collections.defaultdict(list);methods=[];public=[]
with rosbag.Bag(str(R/'fixture.bag'))as b:
 for topic,m,t in b.read_messages():
  if topic.startswith('/astra/frozen_r2')or topic in('/astra/method/status','/astra/navigation/status','/astra/navigation/command_trace'):data[topic].append((t.to_nsec(),json.loads(m.data)))
  elif topic=='/scan':scans[m.header.stamp.to_nsec()]=m
  elif topic=='/odom':odoms[m.header.stamp.to_nsec()]=m
  elif topic in('/tf','/tf_static'):
   for v in m.transforms:tf[(v.header.frame_id.lstrip('/'),v.child_frame_id.lstrip('/'))].append(v.header.stamp.to_nsec())
  elif topic in('/astra/method_cmd','/cmd_vel_navigation'):
   row=(t.to_nsec(),m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
   (methods if topic=='/astra/method_cmd'else public).append(row)
prefix='/astra/frozen_r2_sensor_lifecycle/';identity=data[prefix+'identity'][0][1];receipt=data[prefix+'input_receipt'][0][1];traces=[r for _,r in data[prefix+'trace']];events=[r for _,r in data[prefix+'command_event']];report=data[prefix+'report'][0][1];checks={};resources=[]
bysha=collections.defaultdict(list)
for b in release['resources']:bysha[b['sha256']].append(b)
for b in identity['frozen_source_bindings']:
 resources.append(b)
checks['six_actual_identity_old_sources_exact_release_binding']=len(resources)==6 and all(sha(b['path'])==b['sha256']and b in release['resources']for b in resources)
weights=bysha[identity['checkpoint_sha256']];checks['actual_checkpoint_frozen_exact']=identity['checkpoint_sha256']=='deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8'and len(weights)>=1 and all(sha(b['path'])==b['sha256']for b in weights)
for k,p in [('node_sha256',W/'src/simulation/m2_gazebo/scripts/astra_frozen_r2_sensor_lifecycle.py'),('helper_sha256',W/'scripts/astra_gazebo/frozen_r2_sensor_lifecycle.py')]:checks[k+'_actual_bound']=sha(p)==identity[k] and bool(bysha[identity[k]])
expectedinputs={'/clock','/scan','/odom','/tf','/tf_static','/map','/astra/common_route/path','/astra/navigation/gate','/astra/navigation/status'}
rosters=[r for r in raw['node_roster']if raw['start_ns']<=r['source_ns']<=raw['final_drain']['source_watermark_ns']]
checks['actual_graph_exact_sensor_roster_and_command_authority']=bool(rosters)and all(set(r['sensor_method_graph']['inputs'])==expectedinputs and r['sensor_method_graph']['private_command_publishers']==['/astra_frozen_r2_method']and not r['sensor_method_graph']['unexpected_outputs'] and r['command_publishers']=={'/cmd_vel_navigation':['/astra_navigation_coordinator'],'/cmd_vel':['/navigation_command_relay']}for r in rosters)
checks['actual_identity_and_trace_no_oracle_fields']=identity['oracle_input']is False and all(t['actor_truth_available']is False and t['legacy_row']['actor_poses']is None and t['legacy_row']['conservative_clearance_m']is None for t in traces)
checks['report_matches_saved_and_counts']=report==legacy and len([x for x in traces if x['kind']=='decision'])==report['decision_count']and len([x for x in traces if x['kind']=='control'])==report['control_count']==len(events)
checks['all_actual_trace_sources_exist_at_or_before_publication']=all(t['source_scan_ns']in scans and t['source_odom_ns']in odoms and t['source_scan_ns']<=t['publication_clock_ns']and t['source_odom_ns']<=t['publication_clock_ns']for t in traces)
checks['six_real_distinct_scan_history_with_causal_order']=all(len(t['accepted_runtime_scan_ns'])==6 and len(set(t['accepted_runtime_scan_ns']))==6 and all(a<b for a,b in zip(t['accepted_runtime_scan_ns'],t['accepted_runtime_scan_ns'][1:])) and all(n in scans and n<=t['source_scan_ns']for n in t['accepted_runtime_scan_ns'])for t in traces)
checks['trace_sensor_and_committed_history_freshness_200ms']=all(0<=t['publication_clock_ns']-t['source_scan_ns']<=200000000 and 0<=t['publication_clock_ns']-t['source_odom_ns']<=200000000 and 0<=t['publication_clock_ns']-t['accepted_runtime_scan_ns'][-1]<=200000000 for t in traces)
checks['decision_source_health_and_masks_recorded']=all(not t['legacy_row']['input_health_failures'] and len(t['legacy_row']['action_mask'])==8 and t['legacy_row']['safe_action_count']==sum(t['legacy_row']['action_mask'])for t in traces if t['kind']=='decision')
checks['command_events_unique_ordered_gate_identity']= [e['sequence']for e in events]==list(range(1,len(events)+1)) and all(e['gate_source_ns']==raw['start_ns']and e['publication_source_ns']>=raw['start_ns']and e['time_authority']=='publication_ros_time_and_gate_identity_not_driver_acceptance'for e in events)and all(a['publication_source_ns']<=b['publication_source_ns']for a,b in zip(events,events[1:]))
# Ordered payload containment, allowing separately emitted pre/post/bootstrap zeros.
it=iter(methods);matched=[]
for e in events:
 found=next((m for m in it if m[1:]==(e['v'],0.,0.,0.,0.,e['w'])),None);matched.append(found)
checks['all_event_payloads_ordered_actual_private_Twist']=all(x is not None for x in matched)
checks['all_method_public_commands_finite_planar']=all(all(math.isfinite(x)for x in m[1:])and all(abs(m[i])<=1e-12 for i in (2,3,4,5))for m in methods+public)
checks['first_complete_method_terminal_preserved']=report['terminal']['terminal_source_ns']==raw['end_ns']and report['terminal']['phase']=='complete'and report['lifecycle_fault']is None and not report['late_faults']
terminal_visible=next(x['publisher_visible_clock_ns']for _,x in data['/astra/navigation/status']if x['phase']=='complete');post=[x for _,x in data['/astra/navigation/command_trace']if x['phase']=='complete'];checks['all_actual_coordinator_terminal_outputs_zero']=bool(post)and all(x['v']==x['w']==0 for x in post)
# Actual odometry independently identifies first source gate and first shared goal.
bundle=read(contract['shared_bundle']['path']);ox,oy,theta=bundle['map_from_odom_xy_yaw'];gx,gy,gyaw=bundle['goal_xy_yaw'];firstgate=firstgoal=None
for ns,m in sorted(odoms.items()):
 p=m.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+theta;x=ox+math.cos(theta)*p.position.x-math.sin(theta)*p.position.y;y=oy+math.sin(theta)*p.position.x+math.cos(theta)*p.position.y
 if ns>=contract['activation_source_ns']and x>=bundle['start_xy_yaw'][0]and firstgate is None:firstgate=ns
 if ns>=raw['start_ns']and math.hypot(x-gx,y-gy)<=.25 and abs(math.atan2(math.sin(yaw-gyaw),math.cos(yaw-gyaw)))<=.25 and firstgoal is None:firstgoal=ns
checks['first_actual_odom_gate_and_goal_exact']=firstgate==raw['start_ns']and firstgoal==raw['end_ns']
checks['all_odom_actual_source_20ms_through_watermark']=sorted(odoms)==list(range(20000000,raw['final_drain']['source_watermark_ns']+1,20000000))
# TF recording coverage at required scan source, not proof of original callback availability.
used=sorted({t['source_scan_ns']for t in traces}|{n for t in traces for n in t['accepted_runtime_scan_ns']});chains={f'{a}->{b}':{'count':len(v),'first':min(v),'last':max(v)}for (a,b),v in tf.items()};dynamic=sorted(set(tf[('odom','base_link')]));bracket=[]
for ns in used:
 i=bisect.bisect_left(dynamic,ns);bracket.append(i<len(dynamic)and (dynamic[i]==ns or i>0))
checks['saved_TF_source_brackets_all_used_scans']=all(bracket)and ('base_link','laser_link')in tf
checks['route_audit_only_limitation_explicit']=receipt['full_polyline_consumed']is False and identity['shared_route_consumer_semantics_equal']is False and receipt['static_prior_consumer_semantics_equal']is False
out={'schema':'independent_N02_actual_sensor_protocol_audit','pass':all(checks.values()),'checks':checks,'trace_counts':dict(collections.Counter(t['kind']for t in traces)),'private_command_event_count':len(events),'actual_method_twist_count':len(methods),'actual_public_twist_count':len(public),'roster_samples':len(rosters),'identity':identity,'input_receipt':receipt,'TF_recorded_chains':chains,'used_scan_count':len(used),'maximum_observation_ages_ns':{k:max(t['publication_clock_ns']-t[k]for t in traces)for k in('source_scan_ns','source_odom_ns')},'maximum_committed_history_age_ns':max(t['publication_clock_ns']-t['accepted_runtime_scan_ns'][-1]for t in traces),'terminal_times':{'actual_goal_odom_ns':firstgoal,'coordinator_first_terminal_visible_clock_ns':terminal_visible,'method_terminal_visible_clock_ns':report['terminal']['publisher_visible_clock_ns'],'last_private_method_event_clock_ns':events[-1]['publication_source_ns']},'limitations':['Observed trusted-launch registration graph, not proof against arbitrary unrecorded process/file channels.','TF bag brackets establish recorded transforms at required times; they do not reconstruct original callback arrival or exact tensor arrays.','Fixed global-x R2 consumes shared path only for audit; no fully shared-route or static-prior semantic fairness claim.','Headerless private Twist publication and ROS-clock events are distinct from physics driver acceptance; methods may issue after source goal before shared terminal reception while coordinator zeros after its terminal.','Single clear fixture; no performance improvement or general perception/safety qualification.'],'resources':[{'path':str(p.resolve()),'sha256':sha(p)}for p in [Path(__file__),R/'fixture.bag',R/'collector_raw.json',R/'legacy_report.json',S/'contracts/navigation_n02_release.json']]}
(D/'sensor_actual_audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'times':out['terminal_times']},indent=2));assert out['pass']
