#!/usr/bin/env python3
"""Bounded simulator-only turning/braking qualification; no planner or driver edits.

Pure assessment/helpers are importable without ROS. Runtime saves streaming raw
records before attempting assessment, and always preserves terminal failures.
Only /cmd_vel_navigation is published; /cmd_vel is observed after the relay.
"""
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from urllib.parse import urlparse

import yaml


def sha_bytes(value):return hashlib.sha256(value).hexdigest()
def phase_intervals(contract):
    at=0.;result=[]
    for item in contract['phase_schedule']:
        item=dict(item,start_s=at,end_s=at+item['duration_s']);result.append(item);at=item['end_s']
    if abs(at-contract['total_schedule_s'])>1e-9:raise ValueError('schedule duration mismatch')
    return result
def phase_at(elapsed,contract):
    return next((r for r in phase_intervals(contract) if r['start_s']<=elapsed<r['end_s']),None)
def command_for(phase,wheelbase):
    if phase.get('publish') is False:return None
    v=float(phase['speed_mps'])
    return v,float(phase['yaw_rate_request_radps']) if 'yaw_rate_request_radps' in phase else v*math.tan(phase['center_steering_request_rad'])/wheelbase
def angle_diff(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
def yaw(p):
    x,y,z,w=p[3:];return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def quat_relative(parent,child):
    ax,ay,az,aw=(-parent[0],-parent[1],-parent[2],parent[3]);bx,by,bz,bw=child
    return (aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz)
def pair_center(left,right,L,track):
    tl,tr=math.tan(left),math.tan(right);kl=tl/(L+track/2*tl);kr=tr/(L-track/2*tr)
    return math.atan(L*(kl+kr)/2),abs(kl-kr)
def summary(values):
    values=sorted(float(v) for v in values)
    if not values:return {'count':0}
    return {'count':len(values),'min':values[0],'max':values[-1],'mean':sum(values)/len(values),
            'p95':values[min(len(values)-1,int(.95*(len(values)-1)))],'maximum_absolute':max(abs(values[0]),abs(values[-1]))}
def derivative(times,values):
    return [(times[i]+times[i-1])/2 for i in range(1,len(times))],[(values[i]-values[i-1])/(times[i]-times[i-1]) for i in range(1,len(times))]
def finite(value):
    if isinstance(value,float):return math.isfinite(value)
    if isinstance(value,dict):return all(finite(v) for v in value.values())
    if isinstance(value,(tuple,list)):return all(finite(v) for v in value)
    return True


def validate_effective_urdf(xml,contract,seed):
    root=ET.fromstring(xml);plugin=root.find("gazebo/plugin[@name='m2_ackermann_interface']")
    if plugin is None:raise ValueError('M2 actuator plugin absent from effective robot_description')
    values={k:float(plugin.findtext(k)) for k in contract['effective_plugin_parameters']}
    for key,expected in contract['effective_plugin_parameters'].items():
        if abs(values[key]-expected)>1e-10:raise ValueError('effective actuator parameter mismatch '+key)
    for key,expected in contract['effective_boolean_parameters'].items():
        if (plugin.findtext(key).lower()=='true')!=expected:raise ValueError('effective boolean mismatch '+key)
    if int(plugin.findtext('dynamicsSeed'))!=seed:raise ValueError('effective actuator seed mismatch')
    limits={}
    for name in contract['front_joint_names']:
        joint=root.find("joint[@name='{}']".format(name));limit=joint.find('limit') if joint is not None else None
        if limit is None:raise ValueError('actual front joint limit absent '+name)
        limits[name]={'lower':float(limit.get('lower')),'upper':float(limit.get('upper')),'velocity':float(limit.get('velocity'))}
    return {'plugin':values,'front_joint_limits':limits,'robot_description_sha256':sha_bytes(xml.encode()),'seed':seed}


def validate_world(xml,contract):
    def structure(element):
        return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),tuple(structure(c) for c in element))
    if structure(ET.fromstring(xml))!=structure(ET.fromstring(contract['world_fixture']['sdf'])):
        raise ValueError('world differs structurally from preregistered empty-ground SDF')
    world=ET.fromstring(xml).find('world');spec=contract['world_fixture'];physics=world.find('physics')
    if physics.get('type')!=spec['physics_type'] or abs(float(physics.findtext('max_step_size'))-spec['max_step_s'])>1e-12:raise ValueError('world physics mismatch')
    if list(map(float,world.findtext('gravity').split()))!=spec['gravity_mps2']:raise ValueError('world gravity mismatch')
    if sorted(m.get('name') for m in world.findall('model'))!=sorted(spec['fixed_world_models']):raise ValueError('world fixture models changed')
    ground=world.find("model[@name='ground_plane']")
    if ground.findtext('static')!='true' or ground.find('pose') is not None:raise ValueError('ground must be static origin plane')
    collisions=ground.findall('link/collision')
    if len(collisions)!=1 or list(map(float,collisions[0].findtext('geometry/plane/size').split()))!=spec['ground_size_m']:raise ValueError('ground collision mismatch')
    plugin=world.find("plugin[@filename='libastra_evidence_plugin.so']")
    if plugin is None or float(plugin.findtext('state_hz'))!=spec['state_hz']:raise ValueError('source evidence plugin mismatch')
    return {'world_sha256':sha_bytes(xml.encode()),'expected_runtime_models':spec['fixed_world_models']+[contract['model_name']]}


def pose_derivatives(rows,stride):
    rows=rows[::stride]
    times=[];speeds=[];rates=[]
    for a,b in zip(rows,rows[1:]):
        dt=b['t']-a['t'];times.append((a['t']+b['t'])/2)
        speeds.append(math.hypot(b['p'][0]-a['p'][0],b['p'][1]-a['p'][1])/dt)
        rates.append(angle_diff(yaw(b['p']),yaw(a['p']))/dt)
    at,acc=derivative(times,speeds);jt,jerk=derivative(at,acc)
    return {'stride':stride,'interval_midpoint_speed_mps':summary(speeds),'interval_yaw_rate_radps':summary(rates),
            'acceleration_of_interval_speed_mps2':summary(acc),'jerk_of_interval_acceleration_mps3':summary(jerk),
            'interpretation':'Unsigned planar pose secants at explicit source times; impulsive pose-setting and discretization affect derivatives; no physical acceleration/jerk compliance claim.'}


def assess(raw,contract):
    g=contract['gates'];params=contract['effective_plugin_parameters'];L=params['wheelbase'];track=params['track'];start=raw['schedule_start_s'];end=start+contract['total_schedule_s']
    checks={};metrics={};errors=[]
    def gate(name,value):checks[name]=bool(value)
    def inside(r):return start<=r['t']<=end
    model_rows=[r for r in raw['models'] if inside(r)];link_rows=[r for r in raw['links'] if inside(r)]
    odom=[r for r in raw['odom'] if inside(r)];joints=[r for r in raw['joints'] if inside(r)];contacts=[r for r in raw['contacts'] if inside(r)]
    gate('all_numeric_finite',finite(raw));gate('no_callback_errors',not raw.get('callback_errors'))
    for name,rows in [('models',model_rows),('links',link_rows),('odom',odom),('joints',joints),('contacts',contacts)]:
        dt=[b['t']-a['t'] for a,b in zip(rows,rows[1:])];metrics[name+'_dt_s']=summary(dt)
        gate(name+'_source_strictly_monotonic',bool(dt) and min(dt)>0)
        gate(name+'_covers_schedule',bool(rows) and rows[0]['t']-start<=.025 and end-rows[-1]['t']<=.025)
        limit=g['maximum_contact_gap_s'] if name=='contacts' else g['maximum_state_gap_s']
        gate(name+'_bounded_source_gap',bool(dt) and max(dt)<=limit)
        if name!='contacts':gate(name+'_approximately_50Hz',bool(dt) and g['minimum_packet_frequency_hz']<=(len(rows)-1)/(rows[-1]['t']-rows[0]['t'])<=g['maximum_packet_frequency_hz'])
    # Missing/malformed rows remain raw; avoid laundering a partial result.
    if not all(checks.values()):
        return {'pass':False,'checks':checks,'metrics':metrics,'assessment_incomplete':True,'reason':'source coverage/finite gate failed before derivative assessment'}
    meta=raw['metadata'];gate('physics_source_metadata',meta['physics_type']=='ode' and abs(meta['max_step_s']-.002)<1e-12 and meta['phase']=='WorldUpdateEnd' and meta['all_collision_pairs_retained'])
    expected=set(contract['world_fixture']['fixed_world_models']+[contract['model_name']])
    gate('unique_expected_model_roster',all(len(set(r['names']))==len(r['names']) and set(r['names'])==expected for r in model_rows))
    links={r['ns']:r for r in link_rows};gate('models_links_exact_source_stamp_join',len(links)==len(link_rows)==len(model_rows) and all(r['ns'] in links for r in model_rows))
    robot=[];actual_angles=[];centers=[];curvature_error=[];angular_bounds=[]
    actual_limits=raw['effective_configuration']['front_joint_limits']
    for row in model_rows:
        p=row['poses'][row['names'].index(contract['model_name'])];robot.append({'t':row['t'],'p':p})
    gate('spawn_pose_matches_fixture',math.hypot(robot[0]['p'][0],robot[0]['p'][1])<=.01 and abs(robot[0]['p'][2])<=.01 and abs(yaw(robot[0]['p']))<=.01)
    gate('inside_empty_fixture',all(math.hypot(r['p'][0],r['p'][1])<contract['world_fixture']['runtime_xy_abort_radius_m'] for r in robot))
    for r in link_rows:
        byname=dict(zip(r['names'],r['poses']));base=byname[contract['model_name']+'::base_link'];angles=[]
        for joint_name,link in zip(contract['front_joint_names'],contract['front_link_names']):
            child=byname[contract['model_name']+'::'+link];q=quat_relative(base[3:],child[3:]);a=yaw([0,0,0]+list(q));angles.append(a)
            lim=actual_limits[joint_name];angular_bounds.append(lim['lower']-g['actual_front_joint_angle_tolerance_rad']<=a<=lim['upper']+g['actual_front_joint_angle_tolerance_rad'])
        c,err=pair_center(*angles,L,track);centers.append((r['t'],c));curvature_error.append(err);actual_angles.append(angles)
    metrics['actual_front_left_rad']=summary([x[0] for x in actual_angles]);metrics['actual_front_right_rad']=summary([x[1] for x in actual_angles]);metrics['actual_center_rad']=summary([v for _,v in centers]);metrics['actual_front_curvature_disagreement_inv_m']=summary(curvature_error)
    gate('actual_front_joint_URDF_angle_limits',all(angular_bounds));gate('actual_center_angle_limit',max(abs(v) for _,v in centers)<=params['maxSteer']+g['actual_center_steering_tolerance_rad'])
    gate('actual_front_Ackermann_pair_agreement',max(curvature_error)<=g['actual_front_pair_curvature_disagreement_inv_m'])
    for i,name in enumerate(contract['front_joint_names']):
        _,joint_rates=derivative([r['t'] for r in link_rows],[a[i] for a in actual_angles])
        metrics['actual_'+name+'_rate_radps']=summary(joint_rates)
        gate('actual_'+name+'_URDF_velocity_limit',max(abs(x) for x in joint_rates)<=actual_limits[name]['velocity']+g['actual_front_joint_rate_tolerance_radps'])
    _,actual_center_rates=derivative([t for t,_ in centers],[v for _,v in centers])
    metrics['actual_center_rate_radps']=summary(actual_center_rates)
    gate('actual_center_slew_with_declared_sampling_tolerance',max(abs(v) for v in actual_center_rates)<=params['maxSteeringRate']+g['actual_center_slew_tolerance_radps'])
    # Exact source stamps differ between 50Hz plugin phases. Select latest odom
    # at/before truth, retain its bounded age, and report physical pose agreement.
    all_odom=raw['odom'];ot=[r['t'] for r in all_odom];poserr=[];yawerr=[];ages=[]
    for r in robot:
        i=bisect.bisect_right(ot,r['t'])-1
        if i<0:raise ValueError('truth precedes all odom')
        o=all_odom[i];ages.append(r['t']-o['t']);poserr.append(math.hypot(r['p'][0]-o['p'][0],r['p'][1]-o['p'][1]));yawerr.append(abs(angle_diff(yaw(r['p']),yaw(o['p']))))
    metrics['truth_latest_odom_source_age_s']=summary(ages);metrics['truth_odom_xy_error_m']=summary(poserr);metrics['truth_odom_yaw_error_rad']=summary(yawerr)
    gate('odom_source_join_age',max(ages)<=g['latest_odom_source_age_s']);gate('model_odom_xy_agreement',max(poserr)<=g['model_to_latest_odom_xy_error_m']);gate('model_odom_yaw_agreement',max(yawerr)<=g['model_to_latest_odom_yaw_error_rad'])
    t=[r['t'] for r in odom];v=[r['v'] for r in odom];at,a=derivative(t,v);jt,j=derivative(at,a)
    published=[]
    for r in joints:
        d=r['positions'];c,_=pair_center(d[contract['front_joint_names'][0]],d[contract['front_joint_names'][1]],L,track);published.append(c)
    _,rates=derivative([r['t'] for r in joints],published);tol=g['published_limit_numeric_tolerance']
    metrics['internal_acceleration_mps2']=summary(a);metrics['internal_jerk_mps3']=summary(j);metrics['published_center_steering_rate_radps']=summary(rates)
    gate('internal_accel_limit',max(a)<=params['maxAcceleration']+tol);gate('internal_brake_limit',min(a)>=-params['maxDeceleration']-tol)
    gate('published_center_slew_limit',max(abs(v) for v in rates)<=params['maxSteeringRate']+tol);gate('published_center_angle_limit',max(abs(v) for v in published)<=params['maxSteer']+tol)
    metrics['actual_pose_derivatives_20ms']=pose_derivatives(robot,1);metrics['actual_pose_derivatives_100ms']=pose_derivatives(robot,5)
    ground=[];nonground=[]
    for r in contacts:
        for pair in r['pairs']:
            names=[pair['a'].split('::')[0],pair['b'].split('::')[0]]
            if contract['model_name'] not in names:continue
            (ground if contract['world_fixture']['ground_model'] in names else nonground).append({'t':r['t'],**pair})
    metrics['robot_ground_contact_pair_count']=len(ground);metrics['robot_non_ground_contact_pair_count']=len(nonground);metrics['first_non_ground_contacts']=nonground[:10]
    gate('no_non_ground_robot_contacts',len(nonground)<=g['maximum_non_ground_robot_contacts'])
    phase_metrics={}
    for phase in phase_intervals(contract):
        lo=start+phase['start_s'];hi=start+phase['end_s'];tail=hi-contract['settled_tail_s'];name=phase['name']
        rr=[r for r in robot if tail<=r['t']<hi];oo=[r for r in odom if tail<=r['t']<hi];cc=[v for ts,v in centers if tail<=ts<hi]
        metric={'tail_samples':len(oo),'actual_center_rad':summary(cc),'internal_tail_speed_mps':summary([r['v'] for r in oo])};phase_metrics[name]=metric
        sent=[r for r in raw['commands_sent'] if lo<=r['t']<hi]
        if phase.get('publish') is False:
            observed=[r for r in raw['commands_observed'] if lo+g['command_loss_relay_drain_grace_s']<=r['t']<hi]
            gate('command_loss_no_probe_publication',not sent);gate('command_loss_no_relay_publication_after_drain',not observed)
        else:
            expected_count=phase['duration_s']/contract['publish_period_s'];gate(name+'_command_count',len(sent)>=g['minimum_phase_command_count_fraction']*expected_count)
        if not oo or not rr or not cc:gate(name+'_tail_available',False);continue
        speed=phase.get('speed_mps',0)
        if speed:
            gate(name+'_steady_speed',max(abs(r['v']-speed) for r in oo)<=g['moving_tail_speed_error_mps'])
            desired=max(-params['maxSteer'],min(phase['center_steering_request_rad'],params['maxSteer']))
            gate(name+'_actual_steady_steering',max(abs(v-desired) for v in cc)<=g['steady_actual_steering_error_rad'])
            if 'turn_' in name or 'saturation_' in name:
                allp=[r for r in robot if lo<=r['t']<hi];change=sum(angle_diff(yaw(b['p']),yaw(a['p'])) for a,b in zip(allp,allp[1:]));metric['actual_phase_yaw_change_rad']=change
                gate(name+'_yaw_excitation',math.copysign(1,desired)*change>=g['minimum_turn_yaw_excitation_rad'])
        else:
            displacement=sum(math.hypot(b['p'][0]-a['p'][0],b['p'][1]-a['p'][1]) for a,b in zip(rr,rr[1:]));rotation=sum(abs(angle_diff(yaw(b['p']),yaw(a['p']))) for a,b in zip(rr,rr[1:]))
            metric.update(tail_model_path_m=displacement,tail_model_yaw_variation_rad=rotation)
            gate(name+'_internal_stopped',max(abs(r['v']) for r in oo)<=g['stopped_tail_speed_mps'])
            gate(name+'_model_stopped',displacement<=g['stopped_tail_model_displacement_m'] and rotation<=g['stopped_tail_model_yaw_change_rad'])
    metrics['phases']=phase_metrics
    sent=raw['commands_sent'];observed=raw['commands_observed'];matched=0
    for r in sent:
        if any(0<=o['t']-r['t']<=.12 and abs(o['v']-r['v'])<1e-9 and abs(o['w']-r['w'])<1e-9 for o in observed):matched+=1
    metrics['relay_match_fraction']=matched/len(sent) if sent else 0.;gate('command_relay_observed',metrics['relay_match_fraction']>=g['minimum_matched_relay_fraction'])
    gaps=[b['t']-a['t'] for a,b in zip(sent,sent[1:]) if b['phase']==a['phase']];gate('within_phase_command_deadline',bool(gaps) and max(gaps)<=g['maximum_publish_gap_s'])
    return {'pass':all(checks.values()),'checks':checks,'metrics':metrics,'assessment_incomplete':False,
       'scope':'one simulator-only actuator/execution qualification, not navigation performance or real-vehicle safety',
       'limits':['Actual model pose derivatives and Gazebo body twists differ under SetWorldPose.','Internal acceleration and slew bounds are not formal bounds on physical contact-driven motion.','Ground contacts are retained separately, not classified as obstacle collisions.','No physical jerk limit was preregistered or retrofitted.']}


def offline_fixtures(contract,effective):
    """Synthetic assessor/failure-injection fixtures; never simulator evidence."""
    import copy
    params=contract['effective_plugin_parameters'];L=params['wheelbase'];track=params['track']
    raw={'schedule_start_s':0.,'effective_configuration':effective,'models':[],'links':[],'odom':[],'joints':[],
         'contacts':[],'commands_sent':[],'commands_observed':[],'callback_errors':[],
         'metadata':{'physics_type':'ode','max_step_s':.002,'phase':'WorldUpdateEnd','all_collision_pairs_retained':True}}
    velocity=center=x=y=heading=0.
    def pp(x,y,h):return [x,y,0.,0.,0.,math.sin(h/2),math.cos(h/2)]
    def ack(c):
        k=math.tan(c)/L
        return math.atan2(L*k,1-track*k/2),math.atan2(L*k,1+track*k/2)
    for i in range(int(round(contract['total_schedule_s']/.02))+1):
        t=i*.02;phase=phase_at(max(0.,t-1e-8),contract);cmd=command_for(phase,L)
        if cmd is None:
            # Previous command persists until its 0.5s timeout, including last
            # 20Hz send at phase boundary minus one send period.
            target_v=1.2 if t < phase['start_s']+.45 else 0.;target_c=0.
        else:
            target_v=cmd[0];target_c=0. if abs(target_v)<.001 else max(-params['maxSteer'],min(math.atan(L*cmd[1]/target_v),params['maxSteer']))
        if i:
            desired=(target_v-velocity)*(1-math.exp(-.02/params['speedTimeConstant']))
            rate=params['maxAcceleration'] if target_v>velocity else params['maxDeceleration']
            velocity+=max(-rate*.02,min(desired,rate*.02))
            delta=(target_c-center)*(1-math.exp(-.02/params['steeringTimeConstant']))
            center+=max(-params['maxSteeringRate']*.02,min(delta,params['maxSteeringRate']*.02))
            x+=velocity*math.cos(heading)*.02;y+=velocity*math.sin(heading)*.02;heading+=velocity*math.tan(center)/L*.02
        left,right=ack(center);p=pp(x,y,heading);h={'t':t,'ns':int(round(t*1e9)),'frame':'world'}
        raw['models'].append(dict(h,names=['ground_plane','autolabor_m2'],poses=[pp(0,0,0),p]))
        raw['links'].append(dict(h,names=['autolabor_m2::base_link','autolabor_m2::front_left_steer_link','autolabor_m2::front_right_steer_link'],poses=[p,pp(x,y,heading+left),pp(x,y,heading+right)]))
        raw['odom'].append(dict(h,p=p,v=velocity,w=velocity*math.tan(center)/L))
        raw['joints'].append(dict(h,positions={'front_left_steer_joint':left,'front_right_steer_joint':right}))
    for i in range(int(round(contract['total_schedule_s']/.002))+1):raw['contacts'].append({'t':i*.002,'ns':i*2000000,'pairs':[]})
    for i in range(int(round(contract['total_schedule_s']/.05))):
        t=i*.05;phase=phase_at(t,contract);cmd=command_for(phase,L)
        if cmd is not None:
            row={'t':t,'phase':phase['name'],'v':cmd[0],'w':cmd[1]};raw['commands_sent'].append(row);raw['commands_observed'].append(dict(row))
    baseline=assess(raw,contract)
    cases={'ideal_synthetic_fixture_pass':baseline['pass']};details={'ideal_failed_checks':[k for k,v in baseline['checks'].items() if not v]}
    altered=copy.deepcopy(raw);altered['contacts'][50]['pairs']=[{'a':'autolabor_m2::base::collision','b':'obstacle::body::collision','depths':[.01]}]
    cases['non_ground_contact_fails']=not assess(altered,contract)['checks']['no_non_ground_robot_contacts']
    altered=copy.deepcopy(raw);altered['links'][100]['poses'][1]=pp(0,0,1.0)
    cases['actual_front_joint_overlimit_fails']=not assess(altered,contract)['checks']['actual_front_joint_URDF_angle_limits']
    altered=copy.deepcopy(raw);del altered['models'][100]
    cases['dropped_truth_packet_fails']=not assess(altered,contract)['pass']
    altered=copy.deepcopy(raw);altered['odom'][100]['t']=altered['odom'][99]['t']
    cases['duplicate_source_stamp_fails']=not assess(altered,contract)['checks']['odom_source_strictly_monotonic']
    altered=copy.deepcopy(raw);altered['commands_observed'].append({'t':33.,'v':1.2,'w':0.})
    cases['command_loss_extra_publisher_fails']=not assess(altered,contract)['checks']['command_loss_no_relay_publication_after_drain']
    altered=copy.deepcopy(raw);altered['odom'][100]['v']=float('nan')
    cases['nonfinite_state_fails']=not assess(altered,contract)['checks']['all_numeric_finite']
    actual=baseline['metrics'];details['synthetic_turn_yaw_changes']={k:v.get('actual_phase_yaw_change_rad') for k,v in actual.get('phases',{}).items() if 'turn' in k or 'saturation' in k}
    details['synthetic_actual_front_left_rad']=actual.get('actual_front_left_rad');details['synthetic_actual_center_slew_radps']=actual.get('actual_center_rate_radps')
    return {'purpose':'synthetic offline assessor/failure-injection fixtures only','checks':cases,'pass':all(cases.values()),'details':details}


def runtime():
    import rospy
    from gazebo_msgs.msg import WorldState,ContactsState
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    rospy.init_node('astra_actuation_probe')
    output=Path(rospy.get_param('~output'));output.mkdir(parents=True,exist_ok=True)
    if any((output/p).exists() for p in ('probe_raw.jsonl','probe_raw.json','probe_report.json')):raise RuntimeError('refusing to overwrite an existing probe identity')
    stream=(output/'probe_raw.jsonl').open('x',buffering=1);lock=threading.RLock()
    raw={'models':[],'links':[],'contacts':[],'odom':[],'joints':[],'commands_sent':[],'commands_observed':[],
         'metadata':None,'callback_errors':[],'pipeline_only':True,'vehicle_operation_authorized':False}
    subscribers=[];publisher=None;active=True;started=time.monotonic();failure=None;contract=None
    def record(kind,row):
        with lock:
            if not active:return
            stream.write(json.dumps({'kind':kind,'record':row},allow_nan=False)+'\n')
            if isinstance(raw.get(kind),list):raw[kind].append(row)
            else:raw[kind]=row
    def protected(fn):
        def callback(msg):
            try:fn(msg)
            except Exception as exc:
                with lock:raw['callback_errors'].append({'error':str(exc),'traceback':traceback.format_exc()})
        return callback
    def header(msg):return {'ns':msg.header.stamp.to_nsec(),'t':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'seq':msg.header.seq}
    def pp(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    def world(msg,kind):
        record(kind,dict(header(msg),names=list(msg.name),poses=[pp(p) for p in msg.pose],
          twists=[[p.linear.x,p.linear.y,p.linear.z,p.angular.x,p.angular.y,p.angular.z] for p in msg.twist]))
    def contact(msg):
        record('contacts',dict(header(msg),pairs=[{'a':p.collision1_name,'b':p.collision2_name,'depths':list(p.depths),
           'source_info':p.info,'positions':[[v.x,v.y,v.z] for v in p.contact_positions],'normals':[[v.x,v.y,v.z] for v in p.contact_normals]} for p in msg.states]))
    def service(name):
        done=threading.Event();error=[]
        def invoke():
            try:rospy.ServiceProxy(name,Empty)()
            except Exception as exc:error.append(exc)
            finally:done.set()
        threading.Thread(target=invoke,daemon=True).start()
        if not done.wait(5.):raise RuntimeError('service wall timeout '+name)
        if error:raise error[0]
    try:
        if not rospy.get_param('/use_sim_time',False) or not rospy.get_param('/m2_gazebo/simulation_only',False):raise RuntimeError('simulation-only guards required')
        if urlparse(os.environ.get('ROS_MASTER_URI','')).hostname not in ('127.0.0.1','localhost','::1'):raise RuntimeError('isolated loopback ROS master required')
        seed=rospy.get_param('~seed');run_id=rospy.get_param('~run_id')
        if isinstance(seed,bool) or not isinstance(seed,int) or seed<=0:raise ValueError('explicit positive fresh seed required')
        payload=Path(rospy.get_param('~contract_path')).read_bytes();contract=yaml.safe_load(payload)
        raw.update(seed=seed,run_id=run_id,contract_sha256=sha_bytes(payload),contract=contract,
                   script_sha256=sha_bytes(Path(__file__).read_bytes()))
        phase_intervals(contract)
        description=rospy.get_param('/robot_description');raw['effective_configuration']=validate_effective_urdf(description,contract,seed)
        raw['robot_description']=description;raw['world_validation']=validate_world(Path(rospy.get_param('~world_path')).read_text(),contract)
        record('startup',{'run_id':run_id,'seed':seed,'contract_sha256':raw['contract_sha256'],'effective_configuration':raw['effective_configuration']})
        publisher=rospy.Publisher('/cmd_vel_navigation',Twist,queue_size=1)
        subscribers=[rospy.Subscriber('/astra/world_models',WorldState,protected(lambda m:world(m,'models')),queue_size=5000),
          rospy.Subscriber('/astra/world_links',WorldState,protected(lambda m:world(m,'links')),queue_size=5000),
          rospy.Subscriber('/astra/physics_contacts',ContactsState,protected(contact),queue_size=20000),
          rospy.Subscriber('/astra/physics_metadata',String,protected(lambda m:record('metadata',json.loads(m.data))),queue_size=1),
          rospy.Subscriber('/odom',Odometry,protected(lambda m:record('odom',dict(header(m),p=pp(m.pose.pose),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))),queue_size=5000),
          rospy.Subscriber('/joint_states',JointState,protected(lambda m:record('joints',dict(header(m),positions=dict(zip(m.name,m.position))))),queue_size=5000),
          rospy.Subscriber('/cmd_vel',Twist,protected(lambda m:record('commands_observed',{'t':rospy.Time.now().to_sec(),'v':m.linear.x,'w':m.angular.z,'callerid':m._connection_header.get('callerid')})),queue_size=2000)]
        deadline=time.monotonic()+contract['readiness_wall_s']
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if all(s.get_num_connections()>0 for s in subscribers) and publisher.get_num_connections()>0 and raw['metadata'] is not None:break
            time.sleep(.02)
        else:raise RuntimeError('readiness wall deadline')
        code,message,system=rospy.get_master().getSystemState()
        pubs=dict(system[0]);raw['command_publishers_before_unpause']={k:pubs.get(k,[]) for k in ('/cmd_vel_navigation','/cmd_vel')}
        if pubs.get('/cmd_vel_navigation')!=[rospy.get_name()] or len(pubs.get('/cmd_vel',[]))!=1:raise RuntimeError('command-chain publisher uniqueness failed')
        rospy.wait_for_service('/gazebo/unpause_physics',timeout=5);service('/gazebo/unpause_physics')
        ready_deadline=time.monotonic()+contract['readiness_wall_s']
        while time.monotonic()<ready_deadline and not rospy.is_shutdown():
            if raw['models'] and raw['links'] and raw['odom'] and raw['joints'] and raw['contacts']:break
            time.sleep(.002)
        else:raise RuntimeError('source packets absent after unpause')
        start=rospy.Time.now().to_sec()+.1;raw['schedule_start_s']=start;next_send=start;last_time=rospy.Time.now().to_sec();last_progress=time.monotonic()
        while not rospy.is_shutdown():
            now=rospy.Time.now().to_sec();wall=time.monotonic()
            if wall-started>contract['wall_cap_s']:raise RuntimeError('probe wall cap')
            if now<last_time-1e-9:raise RuntimeError('simulation clock regressed')
            if now>last_time:last_progress=wall
            if wall-last_progress>contract['clock_stall_wall_s']:raise RuntimeError('simulation clock stalled')
            last_time=now
            if raw['callback_errors']:raise RuntimeError('raw callback failed')
            if raw['models']:
                row=raw['models'][-1];index=row['names'].index(contract['model_name']);p=row['poses'][index]
                if math.hypot(p[0],p[1])>contract['world_fixture']['runtime_xy_abort_radius_m']:raise RuntimeError('fixture range exceeded')
            if now>=start+contract['total_schedule_s']+.03:break
            phase=phase_at(now-start,contract)
            if phase is not None and now>=next_send-1e-9:
                command=command_for(phase,contract['effective_plugin_parameters']['wheelbase'])
                if command is not None:
                    m=Twist();m.linear.x,m.angular.z=command;publisher.publish(m)
                    record('commands_sent',{'t':now,'wall_monotonic_s':wall,'phase':phase['name'],'v':command[0],'w':command[1]})
                while next_send<=now+1e-9:next_send+=contract['publish_period_s']
            time.sleep(.001)
        if rospy.is_shutdown():raise RuntimeError('external ROS shutdown before successful terminal assessment')
    except BaseException as exc:
        failure={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    finally:
        if publisher is not None:
            try:publisher.publish(Twist())
            except Exception:pass
        # No ROS-time wait: shutdown/stall must still produce durable raw data.
        if contract is not None:
            try:service('/gazebo/pause_physics')
            except Exception as exc:raw['pause_error']=str(exc)
        with lock:
            active=False
            stream.flush();os.fsync(stream.fileno());stream.close()
        for subscriber in subscribers:subscriber.unregister()
        raw['wall_elapsed_s']=time.monotonic()-started;raw['terminal_exception']=failure
        (output/'probe_raw.json').write_text(json.dumps(raw,indent=2,allow_nan=False)+'\n')
        report={'pass':False,'pipeline_only':True,'vehicle_operation_authorized':False,'seed':raw.get('seed'),'run_id':raw.get('run_id'),
                'raw_sha256':sha_bytes((output/'probe_raw.json').read_bytes()),'raw_stream_sha256':sha_bytes((output/'probe_raw.jsonl').read_bytes()),'terminal_exception':failure}
        if failure is None:
            try:report.update(assess(raw,contract))
            except BaseException as exc:report['assessment_exception']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
        (output/'probe_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 0 if report['pass'] else 3


if __name__=='__main__':
    raise SystemExit(runtime())
