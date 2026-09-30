#!/usr/bin/env python3
"""Read-only independent A02 raw audit. Never starts or connects to ROS/Gazebo.

Only completed receipts and closed bags are read. Output is confined to the
two declared stage audit files; failed invocations are retained in their JSON.
Model/link/Twist bag times are recorder receipt times, never source stamps.
"""
import argparse
import bisect
from collections import Counter, defaultdict
import datetime
import hashlib
import json
import io
import math
import os
from pathlib import Path
import subprocess
import sys
import traceback
import warnings
import xml.etree.ElementTree as ET

import numpy as np
import rosbag
import rospy
import tf2_py
import yaml
from geometry_msgs.msg import TransformStamped

from geometry import Pose3, RobotCollisionModel, compound_clearance, project_box

WORKSPACE = Path(__file__).resolve().parents[2]
STAGE = WORKSPACE.parent
OUTPUT = STAGE/'audits/smoke_a02_raw_audit.json'
METHODS = ('pct_scope_rl', 'analytic_prior', 'actual_teb', 'dwa')
REQUIRED = {'/clock','/scan','/odom','/tf','/tf_static','/joint_states',
            '/cmd_vel_navigation','/cmd_vel','/gazebo/model_states',
            '/gazebo/link_states','/m2_gazebo/contacts'}


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def json_scalar(value):
    if isinstance(value,bytes):return value.decode('utf-8',errors='replace')
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError('unsupported output type '+type(value).__name__)


def installed_transform(value):
    # rosbag dynamically generates classes; tf2 warns on their nested types.
    # Convert exact serialized bytes into installed TransformStamped classes.
    buffer=io.BytesIO();value.serialize(buffer)
    return TransformStamped().deserialize(buffer.getvalue())


def stats(values):
    a=np.array(values,dtype=float)
    if not len(a):return {'count':0}
    return {'count':len(a),'min':float(a.min()),'max':float(a.max()),
            'mean':float(a.mean()),'p50':float(np.percentile(a,50)),
            'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99))}


def stamp_summary(values,window=None):
    a=np.array([x for x in values if window is None or window[0]-1e-9<=x<=window[1]+1e-9])
    dt=np.diff(a)
    return {'count':len(a),'first_s':float(a[0]) if len(a) else None,
            'last_s':float(a[-1]) if len(a) else None,
            'regressions':int(np.sum(dt < -1e-9)),'duplicate_stamps':int(np.sum(np.abs(dt)<=1e-9)),
            'intervals_s':stats(dt),'interval_frequency_hz':float((len(a)-1)/(a[-1]-a[0])) if len(a)>1 and a[-1]>a[0] else None}


def pose(msg):return Pose3.from_ros(msg)
def yaw(p):return math.atan2(p.rotation[1][0],p.rotation[0][0])
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def vec(text,default=(0,0,0,0,0,0)):return tuple(map(float,text.split())) if text else default
def xml_pose(element):
    p=vec(element.findtext('pose'));return Pose3.from_xyz_rpy(p[:3],p[3:])


def parse_world(path):
    world=ET.parse(str(path)).getroot().find('world');physics=world.find('physics')
    obstacles=[];grounds=[]
    for model in world.findall('model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geom=collision.find('geometry');name='/'.join([model.get('name'),link.get('name'),collision.get('name')])
                p=xml_pose(model).compose(xml_pose(link)).compose(xml_pose(collision))
                if geom.find('plane') is not None:
                    grounds.append({'name':name,'pose_xyz':p.xyz,'normal':vec(geom.findtext('plane/normal'),(0,0,1))})
                elif geom.find('box') is not None:
                    size=vec(geom.findtext('box/size'),())
                    obstacles.append(project_box(name,size,p,link_name=link.get('name')))
                else:raise ValueError('A02 audit supports the actual static corridor box/plane world only')
    return obstacles, {'engine_declared_in_executed_world':physics.get('type'),
       'max_step_size_s':float(physics.findtext('max_step_size')),
       'real_time_update_rate_hz':float(physics.findtext('real_time_update_rate')),
       'gravity_mps2':vec(world.findtext('gravity'),()),'ground_planes':grounds,
       'runtime_engine_service_recorded':False,
       'interpretation':'World SDF declares ODE; no physics service response was recorded. Driver SetWorldPose is kinematic even in this physics world.'}


def expand_urdf(bindings,commands):
    directory=WORKSPACE/'src/simulation/m2_gazebo/urdf'
    expected={r['path']:r['sha256'] for r in bindings['resources']}
    for file in directory.glob('*.xacro'):
        if expected.get(str(file)) != sha(file):raise RuntimeError('bound URDF source changed: '+str(file))
    command=['/opt/ros/noetic/lib/xacro/xacro',str(directory/'m2.urdf.xacro'),
       'enable_actuator_dynamics:=true','dynamics_seed:=260926002',
       'max_acceleration:=0.60','max_deceleration:=0.90','max_brake_deceleration:=0.90',
       'max_emergency_deceleration:=0.90','max_steering_rate:=0.75']
    env=dict(os.environ);env['ROS_PACKAGE_PATH']=str(WORKSPACE/'src')+':/opt/ros/noetic/share'
    result=subprocess.run(command,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    record={'command':command,'purpose':'offline xacro expansion only','exit_code':result.returncode,
            'stderr':result.stderr,'stdout_sha256':hashlib.sha256(result.stdout.encode()).hexdigest()}
    commands.append(record)
    if result.returncode:raise RuntimeError('xacro expansion failed: '+result.stderr)
    return result.stdout


def collision_poses(model,actual_links):
    cache=dict(actual_links)
    def get(link):
        if link in cache:return cache[link]
        joint=model.by_child.get(link)
        if joint is None or joint.kind!='fixed':raise ValueError('missing actual moving-link pose '+link)
        cache[link]=get(joint.parent).compose(joint.origin);return cache[link]
    return [(c,get(c.link).compose(c.origin)) for c in model.collisions]


def exact_extents(c,p):
    r=p.rotation
    if c.kind=='box':half=[sum(abs(r[i][j])*c.dimensions[j]/2 for j in range(3)) for i in range(3)]
    elif c.kind=='cylinder':half=[c.dimensions[0]*math.hypot(r[i][0],r[i][1])+c.dimensions[1]/2*abs(r[i][2]) for i in range(3)]
    else:raise ValueError(c.kind)
    return tuple((p.xyz[i]-half[i],p.xyz[i]+half[i]) for i in range(3))


def geometry_audit(link_rows,model,obstacles,window):
    # This exact support calculation is deliberately narrow: actual A02 walls
    # are axis-aligned and every component is inside their finite X extent.
    walls=[]
    for obstacle in obstacles:
        xs=[p[0] for p in obstacle.polygon_xy];ys=[p[1] for p in obstacle.polygon_xy]
        if len(set(round(v,9) for v in xs))!=2 or len(set(round(v,9) for v in ys))!=2:raise ValueError('non-axis-aligned wall')
        walls.append((obstacle,min(xs),max(xs),min(ys),max(ys)))
    wheel_values=defaultdict(list);component_min={};phase_min={'all':None,'policy':None}
    potential_count=0;model_unknown=0;sampled=0;roster=set();first=None;worst=None
    for received,msg in link_rows:
        selected={n.split('::',1)[1]:pose(p) for n,p in zip(msg.name,msg.pose) if n.startswith('autolabor_m2::')}
        roster.update(selected);comps=collision_poses(model,selected);sampled+=1
        if first is None:first=(received,selected)
        for c,p in comps:
            ext=exact_extents(c,p)
            if c.link.endswith('_wheel_link'):wheel_values[c.link].append((received,ext[2][0]))
            for wall,xlo,xhi,ylo,yhi in walls:
                if not (xlo-1e-9<=ext[0][0] and ext[0][1]<=xhi+1e-9):raise ValueError('robot beyond finite wall span; general geometry needed')
                if ext[2][1]<wall.z_min_m or ext[2][0]>wall.z_max_m:continue
                gap=ylo-ext[1][1] if ylo>0 else ext[1][0]-yhi
                row={'recorder_receipt_sim_s':received,'component':c.name,'obstacle':wall.name,
                     'signed_projected_gap_m':gap,'conservative_gap_with_explicit_0p33m_margin_m':gap-.33,
                     'component_z_interval_m':ext[2], 'pose_source':'actual link message; fixed joints only reconstructed'}
                if gap<0:potential_count+=1
                if c.name not in component_min or gap<component_min[c.name]['signed_projected_gap_m']:component_min[c.name]=row
                for phase in ('all','policy'):
                    if phase=='policy' and not window[0]<=received<=window[1]:continue
                    if phase_min[phase] is None or gap<phase_min[phase]['signed_projected_gap_m']:
                        phase_min[phase]=row
                        if phase=='all':worst=(received,selected)
    library_checks=[]
    for label,sample in [('first',first),('minimum_gap',worst)]:
        if sample is None:continue
        received,selected=sample
        projections=model.project(link_world_poses=selected)
        library=compound_clearance(projections,obstacles,margin_m=.33)
        exact=min(min(ylo-exact_extents(c,p)[1][1] if ylo>0 else exact_extents(c,p)[1][0]-yhi
                      for wall,xlo,xhi,ylo,yhi in walls) for c,p in collision_poses(model,selected))
        error=max(p.approximation_error_m for p in projections)
        gap=library['minimum_z_conditioned_projected_clearance_m']
        library_checks.append({'sample':label,'receipt_sim_s':received,'exact_support_gap_m':exact,
           'geometry_library_outer_prism_gap_m':gap,'maximum_cylinder_outer_error_bound_m':error,
           'difference_within_geometry_error_bound':abs(exact-gap)<=error+1e-8})
    wheel_summary={}
    for name,values in wheel_values.items():
        heights=[v for _,v in values];pol=[v for t,v in values if window[0]<=t<=window[1]]
        wheel_summary[name]={'all':stats(heights),'policy':stats(pol),
           'all_abs_gap_gt_1cm_count':sum(abs(v)>.01 for v in heights),
           'policy_abs_gap_gt_1cm_count':sum(abs(v)>.01 for v in pol)}
    return {'actual_link_message_count':sampled,'robot_link_roster':sorted(roster),'collision_component_count':len(model.collisions),
       'wheel_ground_height_m':wheel_summary,'minimum_projected_gap':phase_min,'component_minima':component_min,
       'potential_XY_and_Z_overlap_component_pairs':potential_count,'geometry_library_crosscheck':library_checks,
       'method':'exact box/cylinder support to axis-aligned finite corridor walls; all recorded link messages; 0.33 m separately declared diagnostic margin',
       'scope':'wall-only geometry; ground handled by exact cylinder bottom. Actual link messages lack source timestamps. Projection overlap would be a potential intersection, never confirmed contact.'}


def kinematics(odom,joints,commands,window):
    rows=[r for r in odom if window[0]<=r[0]<=window[1]]
    times=np.array([r[0] for r in rows]);v=np.array([r[1].twist.twist.linear.x for r in rows]);w=np.array([r[1].twist.twist.angular.z for r in rows])
    dt=np.diff(times);valid=dt>1e-9;acc=np.diff(v)[valid]/dt[valid];at=(times[1:]+times[:-1])[valid]/2
    jerk=np.diff(acc)/np.diff(at) if len(acc)>1 else []
    center=np.arctan2(.65*w,v);moving=np.abs(v)>1e-4
    # atan2 has a pi branch for reversing; atan(w/v) is physical steering.
    center=np.array([math.atan(.65*b/a) if abs(a)>1e-4 else 0. for a,b in zip(v,w)])
    joint_centers=[];joint_consistency=[];joint_times=[]
    for stamp,msg in joints:
        if not window[0]<=stamp<=window[1]:continue
        d=dict(zip(msg.name,msg.position));left=math.tan(d['front_left_steer_joint']);right=math.tan(d['front_right_steer_joint'])
        kl=left/(.65+.30*left);kr=right/(.65-.30*right)
        joint_consistency.append(abs(kl-kr));joint_centers.append(math.atan(.65*(kl+kr)/2));joint_times.append(stamp)
    jdt=np.diff(joint_times);jr=np.diff(joint_centers)[jdt>1e-9]/jdt[jdt>1e-9]
    command_summary={}
    for topic,rr in commands.items():
        rr=[r for r in rr if window[0]<=r[0]<=window[1]];angles=[];pure_turn=0;nonplanar=0
        for received,m in rr:
            a,b=m.linear.x,m.angular.z
            if abs(a)<=1e-4:
                pure_turn+=int(abs(b)>1e-4)
            else:angles.append(math.atan(.65*b/a))
            nonplanar+=int(max(abs(m.linear.y),abs(m.linear.z),abs(m.angular.x),abs(m.angular.y))>1e-9)
        command_summary[topic]={'receipt_clock_frequency':stamp_summary([t for t,_ in rr]),
           'implied_unclipped_center_steering_rad':stats(angles),'outside_0p4964_rad_count':sum(abs(a)>.4964+1e-8 for a in angles),
           'zero_linear_nonzero_yaw_count':pure_turn,'nonplanar_command_count':nonplanar,
           'interpretation':'Twist command is headerless; requested steering may be clipped by driver and is not executed steering'}
    return {'source':'/odom header stamps and plugin-published filtered state, not an independently measured physical velocity',
       'source_dt_s':stats(dt),'speed_mps':stats(v),'yaw_rate_radps':stats(w),
       'acceleration_mps2':stats(acc),'jerk_mps3':stats(jerk),'jerk_definition':'difference of adjacent acceleration interval averages / distance between interval midpoints',
       'accel_over_0p60_tolerance_1e-6_count':sum(x>.600001 for x in acc),
       'decel_over_0p90_tolerance_1e-6_count':sum(x<-.900001 for x in acc),
       'odom_implied_center_steering_rad':stats(center[moving]),
       'published_joint_center_steering_rad':stats(joint_centers),'published_joint_center_rate_radps':stats(jr),
       'published_left_right_curvature_disagreement_inv_m':stats(joint_consistency),
       'steer_over_0p4964_tolerance_1e-6_count':sum(abs(x)>.496401 for x in joint_centers),
       'steer_rate_over_0p75_tolerance_1e-6_count':sum(abs(x)>.750001 for x in jr),
       'commands':command_summary,'jerk_preregistered_gate':None,
       'nonzero_steering_excitation_observed':any(abs(x)>1e-4 for x in joint_centers),
       'steering_dynamic_qualification_from_straight_corridor':False,
       'limitation':'Published joint positions contain commanded Ackermann geometry. Geometry audit separately uses actual link poses. No physical jerk or friction qualification follows from these internal states.'}


def audit_episode(root,invocation):
    receipt=json.loads((root/'receipt.json').read_text());report=yaml.load((root/'episode_report.yaml').read_text(),Loader=yaml.CSafeLoader)
    start=report['control_trace'][0]['time_s'];end=start+report['result']['elapsed_s'];window=(start,end)
    bindings=json.loads((root/'bindings.json').read_text())
    xml=expand_urdf(bindings,invocation['subprocess_commands']);model=RobotCollisionModel(xml)
    plugin=ET.fromstring(xml).find("gazebo/plugin[@name='m2_ackermann_interface']")
    actuator_config={name:plugin.findtext(name) for name in ('wheelbase','track','maxSpeed','maxSteer','updateRate','enableActuatorDynamics',
          'speedTimeConstant','steeringTimeConstant','maxAcceleration','maxDeceleration','maxBrakeDeceleration','maxEmergencyDeceleration','maxSteeringRate')}
    obstacles,physics=parse_world(root/'world.sdf')
    bagpath=root/'episode_report.yaml.inputs.bag'
    input_paths=[root/p for p in ('receipt.json','episode_report.yaml','bindings.json','world.sdf','scene_manifest.yaml','cleanup.json','evidence/states.jsonl','evidence/frames.jsonl','evidence/contacts.jsonl','episode_report.yaml.inputs.bag')]
    input_hashes={str(p.relative_to(STAGE)):sha(p) for p in input_paths if p.exists()}
    expected={r['path']:r['sha256'] for r in bindings['resources']}
    archive_mismatches=[r['path'] for r in bindings['resources'] if not (STAGE/'code_blobs'/r['sha256']).is_file() or sha(STAGE/'code_blobs'/r['sha256'])!=r['sha256']]
    declared_matches={'report':sha(root/'episode_report.yaml')==receipt['report_sha256'],
       'bindings':sha(root/'bindings.json')==receipt['bindings_sha256'],
       'bag':sha(bagpath)==receipt['evidence']['bag_sha256'],'archive_all_bound_resources':not archive_mismatches}
    source_stamps=defaultdict(list);receive_stamps=defaultdict(list);connections={};links=[];models=[];odom=[];joints=[];scans=[];commands=defaultdict(list)
    tf_events=[];static_events=[];tf_edge_stamps=defaultdict(list);contacts=[];model_duplicate=0;model_wrong_roster=0;clock=[]
    scene=yaml.safe_load((root/'scene_manifest.yaml').read_text());expected_models=set(scene['expected_models'])
    causal=tf2_py.BufferCore(rospy.Duration(1000));causal_lookup=[]
    with rosbag.Bag(str(bagpath),'r') as bag:
        for connection in bag._get_connections():
            connections.setdefault(connection.topic,[]).append({'callerid':connection.header.get('callerid'),
                  'type':connection.datatype,'md5':connection.md5sum,'latching':connection.header.get('latching')})
        for topic,msg,received in bag.read_messages():
            t=received.to_sec();receive_stamps[topic].append(t)
            if hasattr(msg,'header'):source_stamps[topic].append(msg.header.stamp.to_sec())
            if topic=='/clock':clock.append(msg.clock.to_sec());source_stamps[topic].append(msg.clock.to_sec())
            elif topic=='/gazebo/link_states':links.append((t,msg))
            elif topic=='/gazebo/model_states':
                model_duplicate+=int(len(set(msg.name))!=len(msg.name));model_wrong_roster+=int(set(msg.name)!=expected_models)
                models.append((t,msg))
            elif topic=='/odom':odom.append((msg.header.stamp.to_sec(),msg,t))
            elif topic=='/joint_states':joints.append((msg.header.stamp.to_sec(),msg))
            elif topic in ('/cmd_vel','/cmd_vel_navigation'):commands[topic].append((t,msg))
            elif topic in ('/tf','/tf_static'):
                for transform in msg.transforms:
                    transform=installed_transform(transform)
                    edge=transform.header.frame_id+'->'+transform.child_frame_id
                    tf_edge_stamps[edge].append(transform.header.stamp.to_sec())
                    if topic=='/tf_static':static_events.append(transform);causal.set_transform_static(transform,'recorded_static')
                    else:tf_events.append(transform);causal.set_transform(transform,'recorded_dynamic')
            elif topic=='/scan':
                stamp=msg.header.stamp.to_sec();scans.append((stamp,msg,t))
                try:causal.lookup_transform_core('odom',msg.header.frame_id,msg.header.stamp);ok=True;error=None
                except Exception as exc:ok=False;error=str(exc)
                causal_lookup.append({'stamp_s':stamp,'recorder_receipt_s':t,'available':ok,'error':error})
            elif topic=='/m2_gazebo/contacts':contacts.append((msg.header.stamp.to_sec(),len(msg.states),msg))
    complete=tf2_py.BufferCore(rospy.Duration(1000))
    for transform in static_events:complete.set_transform_static(transform,'recorded_static')
    for transform in sorted(tf_events,key=lambda x:x.header.stamp.to_sec()):complete.set_transform(transform,'recorded_dynamic')
    retrospective=[]
    for stamp,msg,t in scans:
        try:complete.lookup_transform_core('odom',msg.header.frame_id,msg.header.stamp);ok=True;error=None
        except Exception as exc:ok=False;error=str(exc)
        retrospective.append({'stamp_s':stamp,'available':ok,'error':error})
    def coverage(rows):
        out={}
        for phase in ('all','policy'):
            r=[x for x in rows if phase=='all' or start<=x['stamp_s']<=end]
            out[phase]={'total':len(r),'available':sum(x['available'] for x in r),
               'available_fraction':sum(x['available'] for x in r)/len(r) if r else None,
               'first_failures':[x for x in r if not x['available']][:5]}
        return out
    frame_rows=[json.loads(line) for line in (root/'evidence/frames.jsonl').read_text().splitlines()]
    frame_future=[];image_hash_errors=[];wheel_disagreements=[]
    for row in frame_rows:
        if sha(Path(row['image_path']))!=row['image_sha256']:image_hash_errors.append(row['frame_id'])
        for field in ('model_receipt_sim_time_s','link_receipt_sim_time_s','scan_stamp_s','odom_stamp_s','joint_stamp_s'):
            if row[field]>row['image_stamp_s']+1e-9:frame_future.append([row['frame_id'],field])
        for name,p in row['links'].items():
            if not name.endswith('_wheel_link'):continue
            q=Pose3((p['x'],p['y'],p['z']),(p['qx'],p['qy'],p['qz'],p['qw']))
            component=next(c for c in model.collisions if c.link==name.split('::')[1])
            bottom=exact_extents(component,q.compose(component.origin))[2][0]
            wheel_disagreements.append(abs(bottom-row['wheel_bottom_z_m'][name]))
    odom_map={t:msg for t,msg,_ in odom};tf_odom_errors=[]
    for transform in tf_events:
        if transform.header.frame_id!='odom' or transform.child_frame_id!='base_link':continue
        od=odom_map.get(transform.header.stamp.to_sec())
        if od:tf_odom_errors.append(math.hypot(transform.transform.translation.x-od.pose.pose.position.x,transform.transform.translation.y-od.pose.pose.position.y))
    model_times=[t for t,_ in models];world_odom=[]
    for stamp,msg,received in odom:
        i=bisect.bisect_right(model_times,stamp)-1
        if i<0:continue
        t,m=models[i];j=m.name.index('autolabor_m2');world=m.pose[j]
        if stamp-t<=.025:world_odom.append(math.hypot(world.position.x-(msg.pose.pose.position.x-5.4),world.position.y-msg.pose.pose.position.y))
    geometry=geometry_audit(links,model,obstacles,window)
    kine=kinematics(odom,joints,commands,window)
    legacy_replay=[float(scene['corridor_half_width_m'])-abs(r['ego_pose']['y_m'])-(1.15-.36) for r in report['control_trace']]
    legacy_error=[abs(a-r['conservative_clearance_m']) for a,r in zip(legacy_replay,report['control_trace'])]
    topic_stats={topic:{'receipt_clock':stamp_summary(times),'policy_receipt_clock':stamp_summary(times,window),
        'source_clock':stamp_summary(source_stamps[topic]) if topic in source_stamps else None,
        'policy_source_clock':stamp_summary(source_stamps[topic],window) if topic in source_stamps else None,
        'clock_semantics':'header source stamp' if topic in source_stamps else 'bag recorder receipt; source time absent'} for topic,times in receive_stamps.items()}
    result={'run_id':root.name,'method':receipt['method'],'receipt_status':receipt['status'],'pipeline_only':True,
       'input_sha256':input_hashes,'receipt_hash_checks':declared_matches,'archive_resource_count':len(bindings['resources']),
       'archive_mismatches':archive_mismatches,'expanded_URDF_sha256':hashlib.sha256(xml.encode()).hexdigest(),
       'policy_window_sim_s':list(window),'bag_topics':topic_stats,'bag_connections':connections,
       'missing_required_topics':sorted(REQUIRED-set(receive_stamps)),'source_tf_edge_stamps':{k:stamp_summary(v) for k,v in tf_edge_stamps.items()},
       'clock':stamp_summary(clock),'model_roster':{'expected':sorted(expected_models),'duplicate_name_messages':model_duplicate,'wrong_roster_messages':model_wrong_roster},
       'TF_at_scan_source_stamp':{'retrospective_complete_bag':coverage(retrospective),
           'recorder_order_availability':coverage(causal_lookup),
           'interpretation':'Full-bag availability is retrospective source-time coverage. Recorder-order availability cannot reproduce evaluator callback wait/queue behavior.'},
       'scan_intra_frame_timing':{'scan_time_s':stats([m.scan_time for _,m,_ in scans]),
           'time_increment_s':stats([m.time_increment for _,m,_ in scans]),'beam_motion_compensation_proved':False},
       'image_association':{'frames':len(frame_rows),'reported_machine_pass':sum(r['machine_qc']=='pass' for r in frame_rows),
           'future_associations':frame_future,'image_hash_failures':image_hash_errors,
           'wheel_bottom_formula_difference_m':stats(wheel_disagreements),
           'source_stamp_vs_model_link_receipt_time_caveat':True,'independent_visual_review_performed':False},
       'geometry':geometry,'kinematics':kine,'physics':physics,
       'actuator_configuration_from_bound_offline_URDF_expansion':actuator_config,
       'legacy_circle_clearance_replay':{'robot_radius_m':1.15-.36,'control_trace_values_m':stats(legacy_replay),
          'replay_absolute_error_m':stats(legacy_error),'reported_result_minimum_m':report['result']['minimum_conservative_clearance_m'],
          'scope':'unchanged historical wall-circle formula; separate from compound geometry with a declared 0.33m margin'},
       'control_trace_logged_cycle_stamps':stamp_summary([r['time_s'] for r in report['control_trace']]),
       'TF_odom_same_source_stamp_position_difference_m':stats(tf_odom_errors),
       'world_odom_approximate_at_before_position_difference_m':stats(world_odom),
       'world_odom_alignment_limitation':'World model messages selected at or before odom source stamp using bag receipt time; not synchronized source ground truth.',
       'contact_evidence':{'messages':len(contacts),'nonempty_messages':sum(n>0 for _,n,_ in contacts),
            'reported_collision_pairs':[[s.collision1_name,s.collision2_name] for _,_,m in contacts for s in m.states],
            'legacy_sensor_requested_collision':'chassis_collision',
            'coverage_fault_injection_performed':False,'whole_body_contact_coverage_proved':False,
            'zero_contacts_proves_no_collision':False,
            'interpretation':'Empty contact messages establish active publication only; fixed-joint collision naming and whole-body sensitivity need independent fault injection.'},
       'legacy_fixed_0p05s_acceleration_abs_max_mps2':report['result']['maximum_absolute_acceleration_mps2'],
       'legacy_fixed_0p05s_jerk_abs_max_mps3':report['result']['maximum_absolute_jerk_mps3'],
       'observed_goal_reached':report['result']['goal_reached'],'reported_execution_integrity_pass':report['execution_integrity_pass']}
    checks={'closed_bag_receipt_hashes_match':all(declared_matches.values()),'all_required_topics_present':not result['missing_required_topics'],
       'positive_monotonic_policy_scan_source_stamps':topic_stats['/scan']['policy_source_clock']['regressions']==0 and topic_stats['/scan']['policy_source_clock']['duplicate_stamps']==0,
       'scan_policy_frequency_at_least_14Hz':topic_stats['/scan']['policy_source_clock']['interval_frequency_hz']>=14,
       'odom_policy_source_stamps_unique_monotonic':topic_stats['/odom']['policy_source_clock']['regressions']==0 and topic_stats['/odom']['policy_source_clock']['duplicate_stamps']==0,
       'odom_policy_frequency_49_to_51Hz':49<=topic_stats['/odom']['policy_source_clock']['interval_frequency_hz']<=51,
       'complete_bag_policy_TF_coverage':result['TF_at_scan_source_stamp']['retrospective_complete_bag']['policy']['available_fraction']==1,
       'model_roster_unique_expected':model_duplicate==0 and model_wrong_roster==0,
       'source_clock_no_regression':result['clock']['regressions']==0,
       'image_sources_at_or_before_stamp':not frame_future,'image_files_match_recorded_sha':not image_hash_errors,
       'four_wheel_links_recorded':len(geometry['wheel_ground_height_m'])==4,
       'policy_wheel_bottom_within_1cm':all(v['policy_abs_gap_gt_1cm_count']==0 for v in geometry['wheel_ground_height_m'].values()),
       'positive_recorded_compound_wall_clearance':geometry['minimum_projected_gap']['all']['signed_projected_gap_m']>0,
       'geometry_library_crosscheck':all(c['difference_within_geometry_error_bound'] for c in geometry['geometry_library_crosscheck']),
       'published_acceleration_limit':kine['accel_over_0p60_tolerance_1e-6_count']==0,
       'published_deceleration_limit':kine['decel_over_0p90_tolerance_1e-6_count']==0,
       'published_center_steering_limit':kine['steer_over_0p4964_tolerance_1e-6_count']==0,
       'published_steering_rate_limit':kine['steer_rate_over_0p75_tolerance_1e-6_count']==0}
    result['diagnostic_checks']=checks
    result['status']='completed_raw_audit_checks_pass_with_declared_limits' if all(checks.values()) else 'completed_raw_audit_has_findings'
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run-id',action='append');args=parser.parse_args()
    previous=json.loads(OUTPUT.read_text()) if OUTPUT.exists() else {}
    invocation={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'command':sys.argv,
       'script_sha256':sha(Path(__file__).resolve()),'geometry_library_sha256':sha(Path(__file__).parent/'geometry.py'),
       'subprocess_commands':[],'errors':[],'ROS_started':False,'ROS_master_contacted':False}
    outcomes={r['run_id']:r for r in previous.get('episodes',[])}
    selected=args.run_id or ['smoke_a02_{:02d}_{}'.format(i+1,m) for i,m in enumerate(METHODS)]
    pending=[]
    for ident in selected:
        root=STAGE/'runs'/ident
        if not (root/'receipt.json').exists():pending.append(ident);continue
        try:
            if (root/'episode_report.yaml.inputs.bag.active').exists():raise RuntimeError('closed receipt has active bag; refused')
            receipt=json.loads((root/'receipt.json').read_text())
            if not (root/'episode_report.yaml').exists() or not (root/'episode_report.yaml.inputs.bag').exists():
                outcomes[ident]={'run_id':ident,'status':'consumed_missing_report_or_closed_bag','receipt':receipt,'pipeline_only':True}
            else:
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter('always')
                    outcomes[ident]=audit_episode(root,invocation)
                invocation.setdefault('warnings',[]).extend({'category':type(w.message).__name__,'message':str(w.message),'filename':w.filename,'line':w.lineno} for w in caught)
            print(ident+': '+outcomes[ident]['status'],flush=True)
        except Exception as exc:
            record={'run_id':ident,'error_type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
            invocation['errors'].append(record);print(json.dumps(record),file=sys.stderr,flush=True)
    records=previous.get('audit_invocations',[])+[invocation]
    result={'stage':'smoke_a02','status':'audit_invocation_errors_retained' if invocation['errors'] else 'independent_raw_audit_completed' if len(outcomes)==4 else 'partial_completed_receipts_audited',
       'performance_evidence':False,'pending_without_receipt':pending,'audit_invocations':records,
       'episodes':[outcomes[k] for k in sorted(outcomes)],
       'scope':'Read-only raw audit. No simulation or historical rerun. Headerless message times are receiver-clock approximations. No general physical safety or contact coverage claim.',
       'not_measured':['True rigid-body acceleration/jerk','contact detector sensitivity and fixed-joint collision-name coverage','true per-ray scan capture time','exact camera/model/link source synchronization','runtime physics service confirmation']}
    result['all_four_declared_raw_checks_pass']=len(outcomes)==4 and all(r.get('diagnostic_checks') and all(r['diagnostic_checks'].values()) for r in outcomes.values())
    OUTPUT.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False,default=json_scalar)+'\n')
    lines=['# A02 原始证据独立离线审计','',result['status'],'',result['scope'],'',
           '| 方法 | 原始核查 | scan Hz | odom Hz | 复算加速度范围 m/s² | 最小墙体投影净空 m |',
           '|---|---|---:|---:|---|---:|']
    for r in result['episodes']:
        if 'bag_topics' not in r:lines.append('| '+r['run_id']+' | '+r['status']+' | — | — | — | — |');continue
        a=r['kinematics']['acceleration_mps2']
        lines.append('| {} | {} | {:.4f} | {:.4f} | {:.6f}…{:.6f} | {:.6f} |'.format(r['method'],'所列检查通过' if all(r['diagnostic_checks'].values()) else '有待解释项',
           r['bag_topics']['/scan']['policy_source_clock']['interval_frequency_hz'],r['bag_topics']['/odom']['policy_source_clock']['interval_frequency_hz'],
           a['min'],a['max'],r['geometry']['minimum_projected_gap']['all']['signed_projected_gap_m']))
        failures=[k for k,v in r['diagnostic_checks'].items() if not v]
        if failures:lines+=['','发现：'+r['run_id']+'：'+', '.join(failures),'']
    good=[r for r in result['episodes'] if 'bag_topics' in r]
    if good:
        lines+=['','| 方法 | cmd 接收频率 Hz | 源 dt 复算最大绝对 jerk m/s³ | 四轮策略窗高度极值 mm | 策略窗 TF 事后覆盖 | recorder 当时已有 TF |',
                '|---|---:|---:|---|---:|---:|']
        for r in good:
            k=r['kinematics'];h=[v['policy'] for v in r['geometry']['wheel_ground_height_m'].values()]
            tf=r['TF_at_scan_source_stamp'];a=tf['retrospective_complete_bag']['policy'];b=tf['recorder_order_availability']['policy']
            lines.append('| {} | {:.4f} | {:.4f} | {:.6f}…{:.6f} | {}/{} | {}/{} |'.format(r['method'],
              k['commands']['/cmd_vel']['receipt_clock_frequency']['interval_frequency_hz'],max(abs(k['jerk_mps3']['min']),abs(k['jerk_mps3']['max'])),
              min(x['min'] for x in h)*1000,max(x['max'] for x in h)*1000,a['available'],a['total'],b['available'],b['total']))
        lines+=['', '这些回合的发布转向与请求 yaw 均为零；转向幅度/变化率检查在直走数据上成立，**转弯动态没有受到激励，不能据此资格认定**。',
          '', 'jerk 是相邻源时间加速度区间均值对区间中点时间的差分，未设置事后阈值。旧固定 0.05 s 统计出现的 1.08 m/s² 不代表源时间内部状态违反 0.90 m/s² 制动限值。',
          '', 'bag 首尾少数 scan 没有完整 TF，策略评价窗口内全部可用；最早期的 machine_qc 失败帧继续保留。无等待的 recorder 次序检查刻画异步到达，不能与节点 0.03 s 等待后的结果混为一谈。']
    lines+=['','完整结果、SHA、频率/单调性/TF 逐总体计数、四轮高度极值、compound 几何和全部初次工具错误见 JSON。',
       '', '解释边界：/odom 与 joint_states 是 SetWorldPose 运动学插件发布的内部状态；其源时间差复算比历史固定 0.05 s 导数可靠，但不等于接触/摩擦决定的真实物理加速度。无 header 的 model/link/Twist 使用 bag 接收时钟，不能假装精确源时间。全 bag TF 覆盖是事后结果；recorder 到达次序失败不能等同 evaluator 的等待队列失败。contacts 空消息只证明发布通道存在，零值不是无碰撞证明。0.33 m 是本诊断明确附加的几何裕量，不是替换旧保守圆评价器。',
       '', '没有写入任何 run 目录，没有修改 H01–H03，没有启动或连接 ROS/Gazebo。']
    OUTPUT.with_suffix('.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'status':result['status'],'episodes':len(outcomes),'pending':pending,'invocation_errors':len(invocation['errors'])}))
    if invocation['errors']:raise SystemExit(1)


if __name__=='__main__':main()
