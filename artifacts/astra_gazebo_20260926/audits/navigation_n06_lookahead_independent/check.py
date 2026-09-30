"""Independent old-bag coordinate/first-crossing/rear-point steering calculation."""
from pathlib import Path
import bisect,csv,hashlib,json,math,sys
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages');import rosbag
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;D=S/'runs/astra_navigation_n04_b00_astra_analytic_route_v1';R=S/'audits/navigation_n06_lookahead_hypothesis/report.json'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();bind=lambda p:dict(path=str(Path(p).resolve()),sha256=sha(p))
r=json.loads(R.read_bytes());resources=[bind(R),bind(S/'audits/navigation_n06_lookahead_probe.py')]
for f,k in [('fixture.bag','source_bag_sha256'),('assessment.json','source_assessment_sha256'),('contract.json','source_contract_sha256')]:
 p=D/f;assert sha(p)==r[k];resources.append(bind(p))
contract=json.loads((D/'contract.json').read_bytes());bp=Path(contract['shared_bundle']['path']);assert sha(bp)==r['shared_route_sha256']==contract['shared_bundle']['sha256'];bundle=json.loads(bp.read_bytes());resources.append(bind(bp));raw=json.loads((D/'collector_raw.json').read_bytes());resources.append(bind(D/'collector_raw.json'));assert raw['gate']['source_ns']==7_460_000_000
assessment=json.loads((D/'assessment.json').read_bytes());assert assessment['task']['task_phase']=='timeout' and assessment['capture_integrity_pass'] is True
points=bundle['points_xy'];lengths=[math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:])];arcs=[0.]
for length in lengths:arcs.append(arcs[-1]+length)
def project(xy):
 candidates=[]
 for i,(a,b,L) in enumerate(zip(points,points[1:],lengths)):
  dx,dy=b[0]-a[0],b[1]-a[1];u=max(0.,min(1.,((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/(L*L)))
  px,py=a[0]+u*dx,a[1]+u*dy;ex,ey=xy[0]-px,xy[1]-py
  candidates.append((ex*ex+ey*ey,i,arcs[i]+u*L,(-ex*dy+ey*dx)/L))
 return min(candidates,key=lambda x:(x[0],x[1]))
def sample(s):
 s=max(0.,min(arcs[-1],s));i=min(len(lengths)-1,bisect.bisect_right(arcs,s)-1);a,b=points[i:i+2];u=(s-arcs[i])/lengths[i]
 return (a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]),math.atan2(b[1]-a[1],b[0]-a[0]))
def rear(p):return (p[0]-.325*math.cos(p[2]),p[1]-.325*math.sin(p[2]))
def desired(base,look):
 pr=project(base[:2]);target=sample(pr[2]+look);rx,ry=rear(base);tx,ty=rear(target)
 bearing=math.atan2(ty-ry,tx-rx)-base[2]
 # sin is periodic; no dependency on author wrap_angle/helper.
 return max(-.4964,min(.4964,math.atan2(2*.65*math.sin(bearing),max(.05,math.hypot(tx-rx,ty-ry))))),pr,target
odom=[];joints={};map_tf=[]
with rosbag.Bag(str(D/'fixture.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/odom','/joint_states','/tf','/tf_static']):
  if topic=='/joint_states':
   assert len(m.name)==len(m.position)==len(set(m.name));ns=m.header.stamp.to_nsec();assert ns not in joints;joints[ns]=dict(zip(m.name,m.position))
  elif topic=='/odom':
   assert m.header.frame_id=='odom' and m.child_frame_id=='base_link';q=m.pose.pose.orientation;assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-8
   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));tx,ty,angle=bundle['map_from_odom_xy_yaw'];p=m.pose.pose.position
   odom.append((m.header.stamp.to_nsec(),(tx+math.cos(angle)*p.x-math.sin(angle)*p.y,ty+math.sin(angle)*p.x+math.cos(angle)*p.y,yaw+angle)))
  else:
   for x in m.transforms:
    if x.header.frame_id.lstrip('/')=='map' and x.child_frame_id.lstrip('/')=='odom':map_tf.append(x)
assert bundle['map_from_odom_xy_yaw']==[-5.4,0.,0.];assert map_tf and all(x.transform.translation.x==-5.4 and x.transform.translation.y==0 and x.transform.rotation.w==1 and x.transform.rotation.z==0 for x in map_tf)
results=[];maxerr=0.
for source in r['samples']:
 target=source['threshold_map_x_m'];eligible=[(i,ns,p) for i,(ns,p) in enumerate(odom) if p[0]>=target];i,ns,base=eligible[0]
 assert ns==source['source_ns'] and odom[i-1][1][0]<target and raw['gate']['source_ns']<ns<raw['gate']['deadline_source_ns'];assert (ns-raw['gate']['source_ns'])/1e9==source['elapsed_after_gate_s']
 assert all(abs(a-source[k])<1e-13 for a,k in zip(base,['base_map_x_m','base_map_y_m','base_yaw_rad']))
 js=joints[ns];left=js['front_left_steer_joint'];right=js['front_right_steer_joint'];assert left==source['actual_left_steer_rad'] and right==source['actual_right_steer_rad']
 old,projection,oldtarget=desired(base,.7);new,_,newtarget=desired(base,1.2);err=max(abs(old-source['original_desired_steer_rad']),abs(new-source['proposed_desired_steer_rad']),abs(projection[3]-source['route_lateral_m']));maxerr=max(maxerr,err);assert err<1e-12
 remaining=arcs[-1]-projection[2]-.125;assert remaining>1 # runtime terminal heading override not used here
 actual_center=math.atan(.65*.5*(math.tan(left)/(.65+.3*math.tan(left))+math.tan(right)/(.65-.3*math.tan(right))))
 results.append(dict(threshold_map_x_m=target,source_ns=ns,previous_source_ns=odom[i-1][0],previous_map_x=odom[i-1][1][0],base_map=base,rear_map=rear(base),arc_m=projection[2],remaining_m=remaining,original_steer=old,proposed_steer=new,delta=new-old,absolute_steer_delta=abs(new)-abs(old),actual_center_from_two_joints_rad=actual_center,recomputation_max_error=err))
assert len(results)==7;csvp=S/'audits/navigation_n06_lookahead_hypothesis/old_trace_steering_demand.csv';cs=list(csv.DictReader(csvp.open()));assert len(cs)==7 and all(all(v==str(j[k]) for k,v in c.items()) for c,j in zip(cs,r['samples']));resources.append(bind(csvp))
for p in [W/'scripts/astra_gazebo/route_candidate_core.py',W/'scripts/astra_gazebo/common_route.py',W/'scripts/astra_gazebo/axle_reference_adapter.py',W/'scripts/astra_gazebo/route_world_adapter.py',Path(__file__)]:resources.append(bind(p))
out=dict(status='independent_old_trace_lookahead_calculation_pass_hypothesis_only',independent_review=True,source_script=resources[1],reviewed_report=bind(R),source_bag_hash_verified=True,old_timeout_preserved=True,samples_recomputed=7,exact_same_header_joint_joins=7,first_threshold_crossing_samples_verified=True,map_transform_observed_in_bag=True,gate_hardcoded_value_verified_against_raw=True,independent_formula='project map base onto polyline; sample +lookahead; shift both base and target by yaw-rotated(-.325,0); atan2(2*.65*sin(bearing),max(.05,rear_target_distance)); clip±.4964',max_numeric_error=maxerr,rows=results,limitations=['Seven samples chosen from one completed old static-avoidance timeout; no independent episodes or intervention outcomes.', 'This matches zero-lateral-offset moving route-following actions1/2/3 before slew/acceleration/backup/selection. It does not reconstruct the actual chosen candidate or commanded/measured central steering.', 'Current state and same real joint measurements are held fixed. Changed lookahead does not predict future robot or actor states, mask, deadline, collision, completion or time.', 'Longer lookahead reduces early left demand in these samples, not steering magnitude everywhere: atx≈-1.5 sign reverses and absolute steering increases; atx≈-1.25 more negative; atx≈-1 both clamp equally.', 'Rear-point transform is the inherited planar approximation. Nonzero3D roll/pitch is not thereby removed from actual physical geometry.', 'Measured left/right joint angles are individual Ackermann joints, while desired value is central steering; raw differences between these quantities are not tracking errors.', 'No root probe main rerun/output mutation, ROS/Gazebo, newseed, learning, or N04/N05 result change.'],new_simulation_episodes=0,training_steps=0,execution_authorized=False,formal_result=False,resources=resources)
(A/'review.json').write_text(json.dumps(out,indent=2)+'\n');(A/'review.md').write_text('N06离线lookahead假设探针通过7点独立复算，最大数值差'+str(maxerr)+'。重新核验旧N04 bag/assessment/contract/bundle SHA，从原bag独立选取每个x阈值的首次越过，核对上一帧仍在阈值前、同header时刻真实左右转向关节与gate后时间。map_from_odom[-5.4,0,0]同时核对实际bag TF；后轴为base−0.325(cosψ,sinψ)，目标同样平移。独立polyline投影/采样与曲率公式吻合。\n\n仅适用于零横向offset移动路线跟随动作的目标中心角，尚未经过slew/acceleration/backup与候选选择，不代表旧实际命令。7点来自同一旧timeout且固定原状态，不能推断闭环收益、安全或成功。更大lookahead只在早期减少左转请求；x约−1.5处改为右转且绝对角更大，−1.25处更负，−1处同为负饱和。真实左右关节角不等于目标中心转角，不能直接相减当跟踪误差。没有ROS/仿真/训练/新身份或旧数据改动。\n')
print(json.dumps({'status':out['status'],'max_error':maxerr,'review':bind(A/'review.json'),'markdown':bind(A/'review.md'),'rows':[{'x':x['threshold_map_x_m'],'delta':x['delta'],'absolute_delta':x['absolute_steer_delta']} for x in results]},indent=2))
