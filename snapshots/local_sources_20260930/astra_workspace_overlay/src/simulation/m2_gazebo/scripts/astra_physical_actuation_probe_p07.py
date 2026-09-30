#!/usr/bin/env python3
"""M12-P07 source-timed driver ingress and exact interval measurement.

Pure assessment/helpers are importable without ROS. Runtime saves streaming raw
records before attempting assessment, and always preserves terminal failures.
The isolated command source publishes; this collector observes without actuating.
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


def recorder_connections(bus_info,required_topics):
    """A master registration is not a completed TCPROS subscriber handshake."""
    required=set(required_topics)
    if not required or any(not isinstance(t,str) or not t.startswith('/') for t in required):
        raise ValueError('explicit absolute recorder topics required')
    connected=set()
    for row in bus_info:
        if not isinstance(row,(list,tuple)) or len(row)<6:raise ValueError('malformed ROS bus info')
        if row[2]=='i' and row[3]=='TCPROS' and row[5] is True:connected.add(row[4])
    return {'required':sorted(required),'connected':sorted(connected),'missing':sorted(required-connected)}


def final_stream_watermarks(raw,watermark_ns):
    """Expected final publisher slots after a stable paused world source time.

    Stream phases are measured at startup. No timeout increase, trimming or
    interpolation repairs an incomplete stream. Every published slot is required.
    """
    if type(watermark_ns) is not int or watermark_ns<=0:raise ValueError('positive final source watermark required')
    periods={key:20000000 for key in ('models','links','odom','joints','targets','backend_status','commanded_efforts')}
    periods.update({key:2000000 for key in ('dense_models','dense_links','contacts')})
    required={};observed={}
    for key,period in periods.items():
        rows=raw.get(key,[])
        if not rows:raise ValueError('final drain stream absent: '+key)
        stamps=[r.get('ns') for r in rows];first=stamps[0];last=stamps[-1]
        if type(first) is not int or type(last) is not int or not 0<first<=watermark_ns or last>watermark_ns:
            raise ValueError('stream stamp outside final source epoch: '+key)
        if any(type(ns) is not int or ns!=first+i*period for i,ns in enumerate(stamps)):
            raise ValueError('missing, duplicated or off-grid source slot: '+key)
        required[key]=first+((watermark_ns-first)//period)*period;observed[key]=last
    return {'required_last_ns':required,'observed_last_ns':observed,'complete':required==observed}


def paused_world_source_ns(physics,world,contract,end_ns):
    """Validate actual Gazebo service observations, not the ROS /clock receipt."""
    step_ns=int(round(contract['world_fixture']['max_step_s']*1e9))
    sim_time=world.get('sim_time')
    if (physics.get('success') is not True or physics.get('pause') is not True
            or physics.get('time_step')!=contract['world_fixture']['max_step_s']
            or world.get('success') is not True
            or not isinstance(world.get('model_names'),list)
            or sorted(world['model_names'])!=sorted(contract['world_fixture']['fixed_world_models']+[contract['model_name']])
            or type(sim_time) not in (float,int) or not math.isfinite(sim_time) or sim_time<=0):
        raise ValueError('invalid paused-world service observation')
    ns=int(round(sim_time*1e9))
    if abs(sim_time*1e9-ns)>1e-3 or ns%step_ns or ns<end_ns:
        raise ValueError('paused world time outside integer physics grid or formal end')
    return ns


def final_drain_integrity(raw,contract):
    try:
        drain=raw.get('final_drain',{});watermark=drain.get('source_watermark_ns')
        actual=final_stream_watermarks(raw,watermark)
        end_ns=raw['schedule_start_ns']+int(round(contract['total_schedule_s']*1e9))
        queries=drain.get('world_queries',[])
        observed=[paused_world_source_ns(drain.get('physics',{}),world,contract,end_ns) for world in queries]
        return (len(queries)==2 and observed==[watermark,watermark]
                and drain.get('stable_world_query_ns')==observed
                and actual['complete'] and drain.get('complete') is True
                and drain.get('required_last_ns')==actual['required_last_ns']
                and drain.get('observed_last_ns')==actual['observed_last_ns']
                and type(drain.get('wall_elapsed_s')) in (float,int)
                and 0<=drain['wall_elapsed_s']<=5. and not raw.get('pause_error'))
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError):return False


def sample_ns(row):
    """Source headers win; legacy receipt-clock rows remain diagnostic only."""
    if 'ns' in row:
        if type(row['ns']) is not int:raise ValueError('integer source ns required')
        return row['ns']
    return int(round(row['t']*1e9))


def in_ns_window(row,start_ns,end_ns,inclusive_end=False):
    ns=sample_ns(row)
    return start_ns<=ns and (ns<=end_ns if inclusive_end else ns<end_ns)


def parse_command_ingress(raw):
    """Actual callback entry in the physical driver, stamped by Gazebo time."""
    result=[];previous_ns=-1;previous_seq=None
    keys={'input_topic','requested_linear_x','requested_angular_z','accepted',
          'disposition','backend_fault','activation_time_s'}
    for row in raw.get('command_ingress',[]):
        ns=sample_ns(row);seq=row.get('seq');items=row.get('statuses',[])
        if (ns<0 or ns<previous_ns or row.get('frame')!='world' or type(seq) is not int
                or not 0<=seq<2**32 or (previous_seq is not None and seq!=(previous_seq+1)%2**32)
                or len(items)!=1):raise ValueError('invalid driver ingress source/order')
        status=items[0];values=status.get('values',{})
        if (status.get('name')!='astra_physical_command_ingress' or status.get('hardware_id')!='simulation-only'
                or status.get('level')!=0 or status.get('value_count')!=7 or set(values)!=keys
                or values.get('input_topic')!='cmd_vel' or values.get('accepted')!='true'
                or values.get('backend_fault')!='false' or values.get('disposition')!='accepted_queued'):
            raise ValueError('invalid or rejected driver command ingress')
        v=float(values['requested_linear_x']);w=float(values['requested_angular_z']);activation=float(values['activation_time_s'])
        if not all(math.isfinite(x) for x in (v,w,activation)) or int(round(activation*1e9))<ns:
            raise ValueError('invalid driver ingress payload/activation time')
        result.append({'ns':ns,'t':ns/1e9,'seq':seq,'v':v,'w':w,'activation_time_s':activation})
        previous_ns=ns;previous_seq=seq
    if not result:raise ValueError('driver ingress evidence absent')
    return result


def command_ingress_integrity(raw,contract):
    checks={'physical_command_ingress_source_schema_and_order':False,
            'physical_command_ingress_matches_source_commands':False}
    try:
        rows=parse_command_ingress(raw)
        watermark=raw.get('final_drain',{}).get('source_watermark_ns')
        step_ns=int(round(contract['world_fixture']['max_step_s']*1e9))
        if type(watermark) is not int or watermark<=0 or any(r['ns']>watermark or r['ns']%step_ns for r in rows):
            raise ValueError('driver ingress outside final source epoch or physics grid')
        checks['physical_command_ingress_source_schema_and_order']=True
        start=raw['schedule_start_ns'];end=start+int(round(contract['total_schedule_s']*1e9))
        sent=[r for r in raw['commands_sent'] if r['ns']>=start]
        received=[r for r in rows if r['ns']>=start]
        checks['physical_command_ingress_matches_source_commands']=(bool(sent) and len(sent)==len(received)
            and all(r['ns']>=s['ns'] and abs(r['v']-s['v'])<=1e-12 and abs(r['w']-s['w'])<=1e-12
                    for s,r in zip(sent,received))
            and len([r for r in received if r['ns']>=end and r['v']==r['w']==0.])==1)
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError):pass
    return checks


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
def world_vector_in_body(v,q):
    x,y,z,w=q
    return [
      (1-2*y*y-2*z*z)*v[0]+(2*x*y+2*w*z)*v[1]+(2*x*z-2*w*y)*v[2],
      (2*x*y-2*w*z)*v[0]+(1-2*x*x-2*z*z)*v[1]+(2*y*z+2*w*x)*v[2],
      (2*x*z+2*w*y)*v[0]+(2*y*z-2*w*x)*v[1]+(1-2*x*x-2*y*y)*v[2]]
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
    root=ET.fromstring(xml);plugin=root.find("gazebo/plugin[@filename='libastra_physical_ackermann_p05_plugin.so']")
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
    plugin=world.find("plugin[@filename='libastra_dense_evidence_plugin.so']")
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
            'interpretation':'Unsigned planar pose secants at explicit source times; finite-difference discretization affects derivatives; no acceleration/jerk compliance claim from target limits.'}


def source_packet_integrity(raw,contract):
    """Check dense bodies, references and actual joint fields before arithmetic."""
    model=contract['model_name'];expected_models=set(contract['world_fixture']['fixed_world_models']+[model])
    required_links={model+'::base_link'}|{model+'::'+n for n in contract['front_link_names']}
    required_links|={model+'::'+n+'_wheel_link' for n in ('front_left','front_right','rear_left','rear_right')}
    checks={'final_paused_source_watermark_drained':final_drain_integrity(raw,contract)}
    checks.update(command_ingress_integrity(raw,contract))
    stamped=('models','links','dense_models','dense_links','odom','joints','targets','contacts','commanded_efforts','backend_status','source_status','command_ingress')
    checks['all_source_stamp_ns_and_seconds_agree']=all(
        type(r.get('ns')) is int and r['ns']>=0 and abs(r['t']-r['ns']/1e9)<=1e-9
        for key in stamped for r in raw.get(key,[]))
    for key in ('models','links','dense_models','dense_links'):
        rows=raw[key];valid=bool(rows);first_names=set(rows[0].get('names',[])) if rows else set()
        for r in rows:
            names=r.get('names',[]);poses=r.get('poses',[]);twists=r.get('twists',[])
            valid=valid and r.get('frame')=='world'
            valid=valid and len(names)==len(set(names))==len(poses)==len(twists) and bool(names)
            valid=valid and all(len(p)==7 for p in poses) and all(len(v)==6 for v in twists)
            valid=valid and all(abs(sum(q*q for q in p[3:])-1.)<=1e-4 for p in poses if len(p)==7)
            if key.endswith('models'):valid=valid and set(names)==expected_models
            else:valid=valid and set(names)==first_names and required_links<=set(names)
        checks[key+'_complete_roster_and_shapes']=bool(valid)
    for low,dense in (('models','dense_models'),('links','dense_links')):
        lookup={r['ns']:r for r in raw[dense]}
        checks[low+'_matches_dense_same_stamp']=all(
            r['ns'] in lookup and all(r.get(k)==lookup[r['ns']].get(k) for k in ('names','poses','twists'))
            for r in raw[low]) and bool(raw[low])
    names={'speed','center_steering','front_left_steering','front_right_steering'}
    targets=raw['targets'];params=contract['effective_plugin_parameters'];L=params['wheelbase'];half=params['track']/2
    target_valid=bool(targets)
    for r in targets:
        d=r.get('positions',{});v=r.get('velocities',{})
        if (set(d)!=names or set(v)!=names or set(r.get('names',[]))!=names
                or len(r.get('names',[]))!=4 or r.get('position_count')!=4
                or r.get('velocity_count')!=4):target_valid=False;continue
        curvature=math.tan(d['center_steering'])/L
        left=math.atan2(L*curvature,1-half*curvature);right=math.atan2(L*curvature,1+half*curvature)
        target_valid=target_valid and abs(d['front_left_steering']-left)<=1e-9 and abs(d['front_right_steering']-right)<=1e-9
    checks['target_fields_and_Ackermann_geometry']=bool(target_valid)
    checks['target_speed_and_instantaneous_reference_limits']=bool(targets) and all(
        abs(r['positions'].get('speed',float('inf')))<=params['maxSpeed']+1e-6
        and -params['maxDeceleration']-1e-6<=r['velocities'].get('speed',float('inf'))<=params['maxAcceleration']+1e-6
        and abs(r['velocities'].get('center_steering',float('inf')))<=params['maxSteeringRate']+1e-6
        for r in targets)
    # Actual joints may precede the next WorldUpdateEnd packet by one step.
    # Compare with the newest dense packet at/before that explicit source time.
    dense=raw['dense_links'];times=[r['ns'] for r in dense];joint_ok=bool(raw['joints'])
    for r in raw['joints']:
        i=bisect.bisect_right(times,r['ns'])-1
        if i<0:continue  # Startup before first dense packet is outside the schedule.
        if r['ns']-times[i]>2000000:joint_ok=False;continue
        bodies=dict(zip(dense[i].get('names',[]),dense[i].get('poses',[])))
        if not required_links<=set(bodies):joint_ok=False;continue
        base=bodies[model+'::base_link'];positions=r.get('positions',{})
        for joint,link in zip(contract['front_joint_names'],contract['front_link_names']):
            child=bodies[model+'::'+link]
            angle=yaw([0,0,0]+list(quat_relative(base[3:],child[3:])))
            joint_ok=joint_ok and joint in positions and abs(angle_diff(positions.get(joint,0.),angle))<=contract['gates']['actual_front_joint_angle_tolerance_rad']
    checks['actual_joint_positions_agree_with_dense_source_pose']=bool(joint_ok)
    # Both publications occur at WorldUpdateEnd. Compare source-stamped actual
    # base-link velocity directly, including the nonzero lateral component.
    lookup={r['ns']:r for r in dense};odom_ok=bool(raw['odom'])
    for r in raw['odom']:
        if r['ns'] not in lookup:odom_ok=False;continue
        packet=lookup[r['ns']]
        if model+'::base_link' not in packet.get('names',[]):odom_ok=False;continue
        index=packet['names'].index(model+'::base_link')
        if len(packet.get('poses',[]))<=index or len(packet.get('twists',[]))<=index:odom_ok=False;continue
        pose=packet['poses'][index];twist=packet['twists'][index]
        if len(pose)!=7 or len(twist)!=6:odom_ok=False;continue
        linear=world_vector_in_body(twist[:3],pose[3:]);angular=world_vector_in_body(twist[3:],pose[3:])
        odom_ok=odom_ok and max(abs(r['v']-linear[0]),abs(r['vy']-linear[1]),abs(r['w']-angular[2]))<=1e-6
    checks['actual_odom_twist_agrees_with_dense_base_link']=bool(odom_ok)
    statuses=raw.get('backend_status',[]);status_valid=bool(statuses);any_fault=False
    for r in statuses:
        items=r.get('statuses',[])
        if len(items)!=1:status_valid=False;continue
        item=items[0];values=item.get('values',{})
        keys={'backend_fault','command_timeout','brake','emergency'}
        expected_level=2 if values.get('backend_fault')=='true' else 1 if values.get('command_timeout')=='true' else 0
        status_valid=status_valid and item.get('name')=='astra_physical_ackermann' and item.get('hardware_id')=='simulation-only' and set(values)==keys and item.get('value_count')==4 and all(v in ('true','false') for v in values.values()) and item.get('level')==expected_level
        any_fault=any_fault or values.get('backend_fault')=='true'
    checks['physical_status_source_schema']=bool(status_valid)
    legacy_fault=any(r.get('value') is True for r in raw.get('backend_fault',[]))
    checks['physical_backend_never_faulted']=bool(status_valid and not any_fault and not legacy_fault)
    start_ns=raw['schedule_start_ns'];end_ns=start_ns+int(round(contract['total_schedule_s']*1e9))
    formal_ns=lambda key:[r['ns'] for r in raw.get(key,[]) if start_ns<=r['ns']<=end_ns]
    checks['physical_status_source_exact_target_odom_join']=formal_ns('backend_status')==formal_ns('odom')==formal_ns('targets') and bool(formal_ns('backend_status'))
    source_rows=raw.get('source_status',[]);source_ok=bool(source_rows);states=[];previous_source_ns=-1
    for r in source_rows:
        items=r.get('statuses',[])
        if len(items)!=1:source_ok=False;continue
        item=items[0];values=item.get('values',{})
        state=values.get('state');ns=r['ns']
        source_ok=source_ok and item.get('name')=='astra_actuation_command_source' and item.get('hardware_id')=='simulation-only' and item.get('level')==0 and item.get('value_count')==6 and set(values)=={'state','run_id','seed','contract_sha256','reason','start_ns'} and values.get('reason')=='' and values.get('run_id')==raw['run_id'] and values.get('seed')==str(raw['seed']) and values.get('contract_sha256')==raw['contract_sha256'] and ns>=previous_source_ns
        if state=='bootstrap':source_ok=source_ok and ns<start_ns and values.get('start_ns')==''
        elif state=='running':source_ok=source_ok and ns<start_ns and values.get('start_ns')==str(start_ns)
        elif state=='complete':source_ok=source_ok and ns>=end_ns and values.get('start_ns')==str(start_ns)
        else:source_ok=False
        if not states or states[-1]!=state:states.append(state)
        previous_source_ns=ns
    checks['isolated_command_source_identity_and_completion']=bool(source_ok and states==['bootstrap','running','complete'])
    markers=raw.get('schedule_start_records',[])
    checks['schedule_start_marker_matches_trigger']=bool(markers) and all(r['data']==raw['schedule_start_ns'] for r in markers)
    events=raw.get('commands_sent',[]);events_ok=bool(events);slots=[];terminal=[]
    for row in events:
        ns=row.get('ns');scheduled=row.get('scheduled_ns');phase=row.get('phase')
        if type(ns) is not int or type(scheduled) is not int or type(row.get('missed_slots')) is not int:
            events_ok=False;continue
        events_ok=events_ok and row['missed_slots']>=0 and row.get('lateness_ns')==ns-scheduled and ns>=scheduled and abs(row['t']-ns/1e9)<=1e-9
        if ns<start_ns:
            events_ok=events_ok and phase=='bootstrap_zero' and row['v']==row['w']==0.
        elif ns>=end_ns:
            terminal.append(row)
            events_ok=events_ok and phase=='terminal_zero' and row['v']==row['w']==0. and scheduled==end_ns
        else:
            expected=next(p for p in phase_intervals(contract) if int(round(p['start_s']*1e9))<=ns-start_ns<int(round(p['end_s']*1e9)))
            command=command_for(expected,L)
            events_ok=events_ok and command is not None and phase==expected['name']
            if command is not None:events_ok=events_ok and abs(row['v']-command[0])<=1e-12 and abs(row['w']-command[1])<=1e-12
            slots.append(scheduled)
            events_ok=events_ok and scheduled==start_ns+((ns-start_ns)//50000000)*50000000
    checks['isolated_command_events_match_integer_schedule']=bool(events_ok and len(slots)==len(set(slots)) and len(terminal)==1)
    barrier=raw.get('recorder_connection_barrier',{})
    required=raw.get('recording_topics',[]);connected=barrier.get('connected',[])
    uri=urlparse(barrier.get('uri',''))
    checks['recorder_connections_ready_before_unpause']=bool(required) and len(required)==len(set(required)) and set(contract['required_topics'])<=set(required) and sorted(required)==barrier.get('required') and set(required)<=set(connected) and barrier.get('missing')==[] and barrier.get('node')=='/fixture_bag' and uri.scheme=='http' and uri.hostname in ('127.0.0.1','localhost','::1') and not uri.username and not uri.password
    warmup_start=raw.get('warmup_start_ns');warmup_end=raw.get('warmup_end_ns')
    running_stamps=[r['ns'] for r in source_rows if any(item.get('values',{}).get('state')=='running' for item in r.get('statuses',[]))]
    checks['recorder_warmup_before_start']=type(warmup_start) is int and type(warmup_end) is int and bool(running_stamps) and 0<=warmup_start<=warmup_end<=min(running_stamps)<start_ns and warmup_end-warmup_start>=int(round(contract['recorder_protocol']['warmup_sim_s']*1e9))

    metadata=raw.get('physical_metadata') or {}
    checks['physical_backend_metadata']=metadata.get('backend')=='astra_physical_ackermann_v1' and metadata.get('effort_only') is True and metadata.get('calibrated') is False
    for key,param in (('actual_mass_kg','expectedMass'),('wheelbase_m','wheelbase'),('track_m','track'),('wheel_radius_m','wheelRadius'),('wheel_drive_torque_limit_nm','wheelDriveTorqueLimit'),('wheel_brake_torque_limit_nm','wheelBrakeTorqueLimit'),('steering_torque_limit_nm','steeringTorqueLimit')):
        checks['physical_backend_metadata']=checks['physical_backend_metadata'] and isinstance(metadata.get(key),(int,float)) and abs(metadata[key]-params[param])<=1e-9
    effort_ok=bool(raw.get('commanded_efforts'));joint_names=set(contract['front_joint_names'])|{n+'_wheel_joint' for n in ('front_left','front_right','rear_left','rear_right')}
    for r in raw.get('commanded_efforts',[]):
        if set(r.get('names',[]))!=joint_names or len(r.get('names',[]))!=6 or len(r.get('effort',[]))!=6:effort_ok=False;continue
        for name,value in zip(r['names'],r['effort']):
            limit=params['steeringTorqueLimit'] if name in contract['front_joint_names'] else max(params['wheelDriveTorqueLimit'],params['wheelBrakeTorqueLimit'])
            effort_ok=effort_ok and abs(value)<=limit+1e-9
    checks['commanded_efforts_finite_limits_not_reaction']=bool(effort_ok)
    return checks


def assess(raw,contract):
    g=contract['gates'];params=contract['effective_plugin_parameters'];L=params['wheelbase'];track=params['track'];start=raw['schedule_start_s'];end=start+contract['total_schedule_s']
    start_ns=raw['schedule_start_ns'];end_ns=start_ns+int(round(contract['total_schedule_s']*1e9))
    checks={};metrics={};errors=[]
    def gate(name,value):checks[name]=bool(value)
    def inside(r):return in_ns_window(r,start_ns,end_ns,inclusive_end=True)
    model_rows=[r for r in raw['models'] if inside(r)];link_rows=[r for r in raw['links'] if inside(r)]
    odom=[r for r in raw['odom'] if inside(r)];joints=[r for r in raw['joints'] if inside(r)];targets=[r for r in raw['targets'] if inside(r)];contacts=[r for r in raw['contacts'] if inside(r)]
    gate('all_numeric_finite',finite(raw));gate('no_callback_errors',not raw.get('callback_errors'))
    if checks['all_numeric_finite']:
        checks.update(source_packet_integrity(raw,contract))
    # Legacy Bool receipt gaps remain descriptive. P02 qualification uses the
    # new source-stamped DiagnosticArray and retains P01's failed assessment.
    faults=[r for r in raw.get('backend_fault',[]) if inside(r)]
    fault_gaps=[b['t']-a['t'] for a,b in zip(faults,faults[1:])]
    metrics['legacy_backend_fault_receipt_gap_s']=summary(fault_gaps)
    metrics['backend_status_qualification_basis']='50Hz source-stamped DiagnosticArray; original Bool is diagnostic only'
    for name,rows in [('models',model_rows),('links',link_rows),('odom',odom),('joints',joints),('targets',targets),('contacts',contacts),('commanded_efforts',[r for r in raw['commanded_efforts'] if inside(r)]),('backend_status',[r for r in raw['backend_status'] if inside(r)])]:
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
    # Physical odom and joint_states are measured quantities; actuator_target
    # alone carries the bounded reference state. Never call actual odom internal.
    target_times=[r['t'] for r in targets]
    target_speed=[r['positions']['speed'] for r in targets]
    target_center=[r['positions']['center_steering'] for r in targets]
    at,a=derivative(target_times,target_speed);jt,j=derivative(at,a)
    _,rates=derivative(target_times,target_center);tol=g['published_limit_numeric_tolerance']
    metrics['internal_target_acceleration_mps2']=summary(a)
    metrics['internal_target_jerk_mps3']=summary(j)
    metrics['internal_target_center_slew_radps']=summary(rates)
    gate('internal_accel_limit',max(a)<=params['maxAcceleration']+tol)
    gate('internal_brake_limit',min(a)>=-params['maxDeceleration']-tol)
    gate('internal_target_center_slew_limit',max(abs(v) for v in rates)<=params['maxSteeringRate']+tol)
    gate('internal_target_center_angle_limit',max(abs(v) for v in target_center)<=params['maxSteer']+tol)
    actual_t=[r['t'] for r in odom]
    av=[r['v'] for r in odom]
    actual_at,actual_a=derivative(actual_t,av)
    _,actual_j=derivative(actual_at,actual_a)
    metrics['measured_odom_acceleration_mps2']=summary(actual_a)
    metrics['measured_odom_jerk_mps3']=summary(actual_j)
    metrics['measured_odom_lateral_speed_mps']=summary([r['vy'] for r in odom])
    # Dense source evidence is an additional integrity prerequisite; sampled
    # steering qualification above retains the original 50Hz observation grid.
    dense={key:[r for r in raw[key] if inside(r)] for key in ('dense_models','dense_links')}
    for key,rows in dense.items():
        ns=[r['ns'] for r in rows]
        gaps=[b-a for a,b in zip(ns,ns[1:])]
        metrics[key+'_source_gap_ns']=summary(gaps)
        gate(key+'_every_physics_step',bool(gaps) and all(v==2000000 for v in gaps))
        gate(key+'_covers_schedule',bool(rows) and rows[0]['t']-start<=.002000001 and end-rows[-1]['t']<=.002000001)
    gate('dense_model_link_exact_source_join',[r['ns'] for r in dense['dense_models']]==[r['ns'] for r in dense['dense_links']])
    gate('dense_source_metadata',bool(meta.get('all_body_states_every_physics_step')))
    metrics['actual_pose_derivatives_20ms']=pose_derivatives(robot,1);metrics['actual_pose_derivatives_100ms']=pose_derivatives(robot,5)
    ground=[];nonground=[];illegal_ground=[]
    support_links={contract['model_name']+'::'+name+'_wheel_link'
                   for name in ('front_left','front_right','rear_left','rear_right')}
    for r in contacts:
        for pair in r['pairs']:
            names=[pair['a'].split('::')[0],pair['b'].split('::')[0]]
            if contract['model_name'] not in names:continue
            if contract['world_fixture']['ground_model'] in names:
                ground.append({'t':r['t'],**pair})
                robot_collision=pair['a'] if names[0]==contract['model_name'] else pair['b']
                if '::'.join(robot_collision.split('::')[:2]) not in support_links:
                    illegal_ground.append({'t':r['t'],**pair})
            else:
                nonground.append({'t':r['t'],**pair})
    metrics['robot_ground_contact_pair_count']=len(ground);metrics['robot_non_ground_contact_pair_count']=len(nonground);metrics['first_non_ground_contacts']=nonground[:10]
    gate('no_non_ground_robot_contacts',len(nonground)<=g['maximum_non_ground_robot_contacts'])
    metrics['robot_nonwheel_ground_contact_pair_count']=len(illegal_ground)
    metrics['first_nonwheel_ground_contacts']=illegal_ground[:10]
    gate('only_wheels_may_contact_ground',not illegal_ground)
    phase_metrics={};ingress=parse_command_ingress(raw)
    metrics['command_loss_qualification_basis']='physical driver receipt at Gazebo World::SimTime; source order/payload joins, not probe-local headerless receipt clock'
    for phase in phase_intervals(contract):
        lo=start+phase['start_s'];hi=start+phase['end_s'];tail=hi-contract['settled_tail_s'];name=phase['name']
        lo_ns=start_ns+int(round(phase['start_s']*1e9));hi_ns=start_ns+int(round(phase['end_s']*1e9))
        tail_ns=hi_ns-int(round(contract['settled_tail_s']*1e9))
        rr=[r for r in robot if in_ns_window(r,tail_ns,hi_ns)];oo=[r for r in odom if in_ns_window(r,tail_ns,hi_ns)]
        cc=[v for ts,v in centers if in_ns_window({'t':ts},tail_ns,hi_ns)]
        metric={'tail_samples':len(oo),'actual_center_rad':summary(cc),'actual_tail_speed_mps':summary([r['v'] for r in oo])};phase_metrics[name]=metric
        lo_ns=raw['schedule_start_ns']+int(round(phase['start_s']*1e9));hi_ns=raw['schedule_start_ns']+int(round(phase['end_s']*1e9))
        sent=[r for r in raw['commands_sent'] if lo_ns<=r['ns']<hi_ns]
        if phase.get('publish') is False:
            lower=lo_ns+int(round(g['command_loss_relay_drain_grace_s']*1e9))
            observed=[r for r in ingress if in_ns_window(r,lower,hi_ns)]
            legacy=[r for r in raw['commands_observed'] if in_ns_window(r,lower,hi_ns)]
            metric['legacy_headerless_probe_callback_commands_in_loss']=legacy
            metric['driver_source_receipts_in_loss']=observed
            gate('command_loss_no_probe_publication',not sent);gate('command_loss_no_driver_receipt_after_drain',not observed)
        else:
            expected_count=phase['duration_s']/contract['publish_period_s'];gate(name+'_command_count',len(sent)>=g['minimum_phase_command_count_fraction']*expected_count)
        if not oo or not rr or not cc:gate(name+'_tail_available',False);continue
        speed=phase.get('speed_mps',0)
        if speed:
            gate(name+'_steady_speed',max(abs(r['v']-speed) for r in oo)<=g['moving_tail_speed_error_mps'])
            desired=max(-params['maxSteer'],min(phase['center_steering_request_rad'],params['maxSteer']))
            gate(name+'_actual_steady_steering',max(abs(v-desired) for v in cc)<=g['steady_actual_steering_error_rad'])
            if 'turn_' in name or 'saturation_' in name:
                allp=[r for r in robot if in_ns_window(r,lo_ns,hi_ns)];change=sum(angle_diff(yaw(b['p']),yaw(a['p'])) for a,b in zip(allp,allp[1:]));metric['actual_phase_yaw_change_rad']=change
                gate(name+'_yaw_excitation',math.copysign(1,desired)*change>=g['minimum_turn_yaw_excitation_rad'])
        else:
            displacement=sum(math.hypot(b['p'][0]-a['p'][0],b['p'][1]-a['p'][1]) for a,b in zip(rr,rr[1:]));rotation=sum(abs(angle_diff(yaw(b['p']),yaw(a['p']))) for a,b in zip(rr,rr[1:]))
            metric.update(tail_model_path_m=displacement,tail_model_yaw_variation_rad=rotation)
            gate(name+'_actual_stopped',max(abs(r['v']) for r in oo)<=g['stopped_tail_speed_mps'])
            gate(name+'_model_stopped',displacement<=g['stopped_tail_model_displacement_m'] and rotation<=g['stopped_tail_model_yaw_change_rad'])
    metrics['phases']=phase_metrics
    sent=[r for r in raw['commands_sent'] if raw['schedule_start_ns']<=r['ns']<raw['schedule_start_ns']+int(round(contract['total_schedule_s']*1e9))];observed=raw['commands_observed'];matched=0
    for r in sent:
        if any(0<=o['t']-r['t']<=.12 and abs(o['v']-r['v'])<1e-9 and abs(o['w']-r['w'])<1e-9 for o in observed):matched+=1
    metrics['legacy_probe_relay_match_fraction']=matched/len(sent) if sent else 0.
    metrics['legacy_probe_relay_match_threshold']=g['minimum_matched_relay_fraction']
    metrics['legacy_probe_relay_match_pass_diagnostic_only']=metrics['legacy_probe_relay_match_fraction']>=g['minimum_matched_relay_fraction']
    gaps=[b['t']-a['t'] for a,b in zip(sent,sent[1:]) if b['phase']==a['phase']];gate('within_phase_command_deadline',bool(gaps) and max(gaps)<=g['maximum_publish_gap_s'])
    return {'pass':all(checks.values()),'checks':checks,'metrics':metrics,'assessment_incomplete':False,
       'scope':'one simulator-only actuator/execution qualification, not navigation performance or real-vehicle safety',
       'limits':['Physical backend uses finite joint forces; body poses and twists are measured every2ms; this fixture alone does not establish a swept safety verifier.','Internal target acceleration and slew bounds are reported separately from measured physical motion.','Ground contacts are retained separately, not classified as obstacle collisions.','No physical jerk limit was preregistered or retrofitted.']}


def runtime():
    import rospy
    from gazebo_msgs.msg import WorldState,ContactsState
    from gazebo_msgs.srv import GetWorldProperties,GetPhysicsProperties
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String,Bool,Int64
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import Empty,Trigger
    rospy.init_node('astra_physical_actuation_probe_p07')
    output=Path(rospy.get_param('~output'));output.mkdir(parents=True,exist_ok=True)
    if any((output/p).exists() for p in ('probe_raw.jsonl','probe_raw.json','probe_report.json')):raise RuntimeError('refusing to overwrite an existing probe identity')
    stream=(output/'probe_raw.jsonl').open('x',buffering=1);lock=threading.RLock()
    raw={'models':[],'links':[],'contacts':[],'odom':[],'joints':[],'targets':[],'dense_models':[],'dense_links':[],'backend_fault':[],'commanded_efforts':[],'physical_metadata':None,'backend_status':[],'command_ingress':[],'source_status':[],'schedule_start_records':[],'commands_sent':[],'commands_observed':[],
         'metadata':None,'callback_errors':[],'pipeline_only':True,'vehicle_operation_authorized':False}
    subscribers=[];active=True;started=time.monotonic();failure=None;contract=None
    def record(kind,row):
        with lock:
            if not active:return
            stream.write(json.dumps({'kind':kind,'record':row},allow_nan=False)+'\n')
            if isinstance(raw.get(kind),list):raw[kind].append(row)
            else:raw[kind]=row
    def protected(fn):
        def callback(msg):
            with lock:
                if not active:return
                try:fn(msg)
                except Exception as exc:
                    raw['callback_errors'].append({'error':str(exc),'traceback':traceback.format_exc()})
        return callback
    def header(msg):return {'ns':msg.header.stamp.to_nsec(),'t':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'seq':msg.header.seq}
    def pp(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    def world(msg,kind):
        record(kind,dict(header(msg),names=list(msg.name),poses=[pp(p) for p in msg.pose],
          twists=[[p.linear.x,p.linear.y,p.linear.z,p.angular.x,p.angular.y,p.angular.z] for p in msg.twist]))
    def contact(msg):
        record('contacts',dict(header(msg),pairs=[{'a':p.collision1_name,'b':p.collision2_name,'depths':list(p.depths),
           'source_info':p.info,'positions':[[v.x,v.y,v.z] for v in p.contact_positions],'normals':[[v.x,v.y,v.z] for v in p.contact_normals]} for p in msg.states]))
    def diagnostic(message,kind):
        items=[{'name':s.name,'hardware_id':s.hardware_id,'level':int(s.level),'message':s.message,
                'values':{v.key:v.value for v in s.values},'value_count':len(s.values)} for s in message.status]
        record(kind,dict(header(message),statuses=items))
    def command_event(message):
        if len(message.status)!=1:raise ValueError('command event must have exactly one status')
        status=message.status[0];values={v.key:v.value for v in status.values}
        if status.name!='astra_fixture_command' or status.hardware_id!='simulation-only' or status.level!=0 or len(values)!=len(status.values):raise ValueError('invalid command event schema')
        if set(values)!={'phase','v','w','wall_monotonic_s','scheduled_ns','lateness_ns','missed_slots'}:raise ValueError('invalid command event fields')
        record('commands_sent',dict(header(message),phase=values['phase'],v=float(values['v']),w=float(values['w']),
              wall_monotonic_s=float(values['wall_monotonic_s']),scheduled_ns=int(values['scheduled_ns']),
              lateness_ns=int(values['lateness_ns']),missed_slots=int(values['missed_slots'])))
    def trigger_start():
        done=threading.Event();result=[];error=[]
        def invoke():
            try:result.append(rospy.ServiceProxy('/astra_fixture/start',Trigger)())
            except Exception as exc:error.append(exc)
            finally:done.set()
        threading.Thread(target=invoke,daemon=True).start()
        if not done.wait(5.):raise RuntimeError('command source trigger wall timeout')
        if error:raise error[0]
        if not result[0].success:raise RuntimeError('command source rejected start: '+result[0].message)
        value=json.loads(result[0].message)
        if value['run_id']!=run_id or value['seed']!=seed or value['contract_sha256']!=raw['contract_sha256']:raise RuntimeError('command source start identity mismatch')
        start_ns=value.get('start_ns',value.get('schedule_start_ns'))
        if type(start_ns) is not int or start_ns<=0:raise RuntimeError('invalid command source start ns')
        raw['command_source_start_response']=value
        return start_ns
    def call_service(name,kind,timeout=5.):
        done=threading.Event();error=[];result=[]
        def invoke():
            try:result.append(rospy.ServiceProxy(name,kind)())
            except Exception as exc:error.append(exc)
            finally:done.set()
        if timeout<=0.:raise RuntimeError('service wall deadline '+name)
        threading.Thread(target=invoke,daemon=True).start()
        if not done.wait(timeout):raise RuntimeError('service wall timeout '+name)
        if error:raise error[0]
        return result[0]
    def service(name):return call_service(name,Empty)
    def recorder_ready(required_topics):
        import http.client
        import xmlrpc.client
        class Transport(xmlrpc.client.Transport):
            def make_connection(self,host):return http.client.HTTPConnection(host,timeout=2.)
        code,message,uri=rospy.get_master().lookupNode('/fixture_bag')
        if code!=1:raise RuntimeError('recorder node lookup failed')
        parsed=urlparse(uri)
        if parsed.scheme!='http' or parsed.hostname not in ('localhost','127.0.0.1','::1') or parsed.username or parsed.password:raise RuntimeError('recorder URI is not isolated loopback')
        with xmlrpc.client.ServerProxy(uri,transport=Transport()) as proxy:
            code,message,rows=proxy.getBusInfo(rospy.get_name())
        if code!=1:raise RuntimeError('recorder bus lookup failed')
        return dict(recorder_connections(rows,required_topics),node='/fixture_bag',uri=uri,wall_monotonic_s=time.monotonic())
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
        subscribers=[rospy.Subscriber('/astra/world_models',WorldState,protected(lambda m:world(m,'models')),queue_size=5000),
          rospy.Subscriber('/astra/world_links',WorldState,protected(lambda m:world(m,'links')),queue_size=5000),
          rospy.Subscriber('/astra/physics_models',WorldState,protected(lambda m:world(m,'dense_models')),queue_size=30000),
          rospy.Subscriber('/astra/physics_links',WorldState,protected(lambda m:world(m,'dense_links')),queue_size=30000),
          rospy.Subscriber('/m2_driver/actuator_target',JointState,protected(lambda m:record('targets',dict(header(m),names=list(m.name),position_count=len(m.position),velocity_count=len(m.velocity),positions=dict(zip(m.name,m.position)),velocities=dict(zip(m.name,m.velocity))))),queue_size=5000),
          rospy.Subscriber('/m2_driver/physical_backend_fault',Bool,protected(lambda m:record('backend_fault',{'t':rospy.Time.now().to_sec(),'value':m.data})),queue_size=5000),
          rospy.Subscriber('/m2_driver/physical_metadata',String,protected(lambda m:record('physical_metadata',json.loads(m.data))),queue_size=1),
          rospy.Subscriber('/m2_driver/physical_status',DiagnosticArray,protected(lambda m:diagnostic(m,'backend_status')),queue_size=5000),
          rospy.Subscriber('/m2_driver/command_ingress',DiagnosticArray,protected(lambda m:diagnostic(m,'command_ingress')),queue_size=2000),
          rospy.Subscriber('/astra_fixture/command_source_status',DiagnosticArray,protected(lambda m:diagnostic(m,'source_status')),queue_size=1000),
          rospy.Subscriber('/astra_fixture/command_event',DiagnosticArray,protected(command_event),queue_size=2000),
          rospy.Subscriber('/astra_fixture/schedule_start_ns',Int64,protected(lambda m:record('schedule_start_records',{'data':m.data,'receipt_t':rospy.Time.now().to_sec()})),queue_size=1),
          rospy.Subscriber('/m2_driver/actuator_commanded_effort',JointState,protected(lambda m:record('commanded_efforts',dict(header(m),names=list(m.name),effort=list(m.effort)))),queue_size=5000),
          rospy.Subscriber('/astra/physics_contacts',ContactsState,protected(contact),queue_size=20000),
          rospy.Subscriber('/astra/physics_metadata',String,protected(lambda m:record('metadata',json.loads(m.data))),queue_size=1),
          rospy.Subscriber('/odom',Odometry,protected(lambda m:record('odom',dict(header(m),p=pp(m.pose.pose),v=m.twist.twist.linear.x,vy=m.twist.twist.linear.y,w=m.twist.twist.angular.z))),queue_size=5000),
          rospy.Subscriber('/joint_states',JointState,protected(lambda m:record('joints',dict(header(m),positions=dict(zip(m.name,m.position)),velocities=dict(zip(m.name,m.velocity)),efforts=dict(zip(m.name,m.effort))))),queue_size=5000),
          rospy.Subscriber('/cmd_vel',Twist,protected(lambda m:record('commands_observed',{'t':rospy.Time.now().to_sec(),'v':m.linear.x,'w':m.angular.z,'callerid':m._connection_header.get('callerid')})),queue_size=2000)]
        deadline=time.monotonic()+contract['readiness_wall_s']
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if all(s.get_num_connections()>0 for s in subscribers) and raw['metadata'] is not None and raw['source_status']:break
            time.sleep(.02)
        else:raise RuntimeError('readiness wall deadline')
        code,message,system=rospy.get_master().getSystemState()
        if code!=1:raise RuntimeError('publisher readiness query failed')
        pubs=dict(system[0]);raw['command_publishers_before_unpause']={k:pubs.get(k,[]) for k in ('/cmd_vel_navigation','/cmd_vel')}
        if pubs.get('/cmd_vel_navigation')!=['/actuation_command_source'] or len(pubs.get('/cmd_vel',[]))!=1:raise RuntimeError('isolated command-chain publisher uniqueness failed')
        bag_topics=rospy.get_param('~recording_topics')
        if not isinstance(bag_topics,list) or not set(contract['required_topics'])<=set(bag_topics):raise RuntimeError('explicit complete recorder topics required')
        raw['recording_topics']=bag_topics
        recorder_deadline=time.monotonic()+contract['readiness_wall_s'];last_recorder_error=None
        while time.monotonic()<recorder_deadline and not rospy.is_shutdown():
            try:
                barrier=recorder_ready(bag_topics);raw['recorder_connection_barrier']=barrier
                if not barrier['missing']:break
            except Exception as exc:last_recorder_error=str(exc)
            time.sleep(.05)
        else:raise RuntimeError('recorder TCPROS readiness deadline: '+str(last_recorder_error)+' '+str(raw.get('recorder_connection_barrier')))
        raw['warmup_start_ns']=rospy.Time.now().to_nsec()
        rospy.wait_for_service('/gazebo/unpause_physics',timeout=5);service('/gazebo/unpause_physics')
        ready_deadline=time.monotonic()+contract['readiness_wall_s']
        while time.monotonic()<ready_deadline and not rospy.is_shutdown():
            if rospy.Time.now().to_nsec()-raw['warmup_start_ns']>=int(round(contract['recorder_protocol']['warmup_sim_s']*1e9)) and all(raw[k] for k in ('models','links','odom','joints','targets','contacts','dense_models','dense_links','backend_fault','backend_status','commanded_efforts','physical_metadata')):break
            time.sleep(.002)
        else:raise RuntimeError('source packets absent after unpause')
        raw['warmup_end_ns']=rospy.Time.now().to_nsec()
        start_ns=trigger_start();start=start_ns/1e9;raw['schedule_start_ns']=start_ns;raw['schedule_start_s']=start;last_time=rospy.Time.now().to_sec();last_progress=time.monotonic()
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
            time.sleep(.001)
        if rospy.is_shutdown():raise RuntimeError('external ROS shutdown before successful terminal assessment')
    except BaseException as exc:
        failure={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    finally:
        # No ROS-time wait: shutdown/stall must still produce durable raw data.
        if contract is not None:
            try:service('/gazebo/pause_physics')
            except Exception as exc:
                raw['pause_error']=str(exc)
                if failure is None:failure={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
        # Drain queued source messages after pause without adding simulation
        # time; high-rate JSON serialization must not cut off terminal evidence.
        if failure is None:
            drain_started=time.monotonic();drain_deadline=drain_started+5.
            end_ns=raw['schedule_start_ns']+int(round(contract['total_schedule_s']*1e9))
            raw['final_drain']={'wall_budget_s':5.,'world_queries':[],'complete':False}
            try:
                physics=call_service('/gazebo/get_physics_properties',GetPhysicsProperties,drain_deadline-time.monotonic())
                raw['final_drain']['physics']={key:getattr(physics,key) for key in ('success','pause','time_step','status_message')}
                stamps=[]
                for index in range(2):
                    if index:
                        remaining=drain_deadline-time.monotonic()
                        if remaining<.02:raise RuntimeError('source evidence terminal drain deadline')
                        time.sleep(.02)
                    world_response=call_service('/gazebo/get_world_properties',GetWorldProperties,drain_deadline-time.monotonic())
                    observation={key:getattr(world_response,key) for key in ('success','sim_time','model_names','status_message')}
                    raw['final_drain']['world_queries'].append(observation)
                    stamps.append(paused_world_source_ns(raw['final_drain']['physics'],observation,contract,end_ns))
                raw['final_drain']['stable_world_query_ns']=stamps
                if stamps[0]!=stamps[1]:raise RuntimeError('paused world source time changed between queries')
                watermark=stamps[0];raw['final_drain']['source_watermark_ns']=watermark
                while time.monotonic()<drain_deadline:
                    with lock:
                        tail=final_stream_watermarks(raw,watermark)
                        complete=any(v.get('values',{}).get('state')=='complete' for r in raw['source_status'] for v in r['statuses'])
                        terminal=any(r['phase']=='terminal_zero' for r in raw['commands_sent'])
                        ingress=parse_command_ingress(raw)
                        terminal=terminal and any(r['ns']>=end_ns and r['v']==r['w']==0. for r in ingress)
                        if tail['complete'] and complete and terminal:
                            raw['final_drain'].update(tail,wall_elapsed_s=time.monotonic()-drain_started)
                            # Source joins and closure are checked under the same
                            # lock used by callbacks. The final captured epoch
                            # cannot change between verification and closing.
                            joins=source_packet_integrity(raw,contract)
                            keys=('models_matches_dense_same_stamp','links_matches_dense_same_stamp',
                                  'actual_joint_positions_agree_with_dense_source_pose',
                                  'actual_odom_twist_agrees_with_dense_base_link')
                            raw['final_drain']['preclose_source_joins']={key:joins[key] for key in keys}
                            raw['final_drain']['wall_elapsed_s']=time.monotonic()-drain_started
                            if raw['final_drain']['wall_elapsed_s']>5.:raise RuntimeError('source evidence terminal drain deadline')
                            active=False
                            break
                    time.sleep(.02)
                else:raise RuntimeError('source evidence terminal drain deadline')
            except Exception as exc:
                failure={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
                raw['final_drain']['wall_elapsed_s']=time.monotonic()-drain_started
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
