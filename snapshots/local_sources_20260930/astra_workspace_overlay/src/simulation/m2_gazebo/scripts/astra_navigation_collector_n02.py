#!/usr/bin/env python3
"""New navigation evidence collector; sole authorized Gazebo lifecycle gateway.

No navigation policy. Full dense evidence lives in the independently recorded
bag; this node retains source coverage, runtime roster and service observations.
"""
import hashlib
import http.client
import io
import math
import json
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse
import xmlrpc.client

WORKSPACE=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(WORKSPACE/'scripts/astra_gazebo'))
import navigation_evidence_n02 as q
import run_smoke_a02 as common

class Transport(xmlrpc.client.Transport):
    def __init__(self,timeout=2.):super().__init__();self.timeout=timeout
    def make_connection(self,host):return http.client.HTTPConnection(host,timeout=self.timeout)

def main():
    import rospy
    from gazebo_msgs.msg import WorldState,ContactsState
    from gazebo_msgs.srv import SpawnModel,GetWorldProperties,GetPhysicsProperties
    from geometry_msgs.msg import Pose
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Empty,Trigger
    rospy.init_node('astra_navigation_collector',anonymous=False)
    root=Path(rospy.get_param('~output')).resolve()
    value=q.validate_contract(json.loads(q.read_bound({'path':rospy.get_param('~contract_path'),'sha256':rospy.get_param('~contract_sha256')})))
    if root!=(q.STAGE/'runs'/value['run_id']).resolve():raise ValueError('output must be the new stage run identity directory')
    root.mkdir(exist_ok=True)
    master_uri=os.environ.get('ROS_MASTER_URI','');parsed=urlparse(master_uri)
    if parsed.hostname not in ('127.0.0.1','localhost') or parsed.port!=14820 or parsed.username or parsed.password:
        raise ValueError('isolated explicit loopback master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:
        raise ValueError('simulation-only guard required')
    phase=['bootstrap'];drain_deadline=[None];raw={'run_id':value['run_id'],'seed':value['seed'],
        'method':value['method'],'stage':value['stage'],'contract_sha256':rospy.get_param('~contract_sha256'),'source_stamps':{},'status_messages':[],
        'coordinator_status':[],'coordinator_identity':[],'gate':None,'command_trace_summary':[],'command_ingress':[],'node_processes':[],'metadata':[],'node_roster':[],'services':[],
        'errors':[],'images':[],'resource_samples':[],'source_wall_samples':[]}
    evidence=q.EvidenceStore(raw);lock=evidence.lock
    latest={};last_state={};last_image=[None];gazebo_pid=[None];source_wall=raw['source_wall_samples'];started_wall=time.monotonic()
    lifecycle_deadline=q.WallDeadline(value['wall_cap_s'])
    def durable_journal(name,row):
        with (root/name).open('a') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
    def parse_callback(topic,message):
        if isinstance(message,String):
            item=json.loads(message.data)
            if topic=='/astra/p03/step_status':stamp=item['stamp_ns']
            elif topic=='/astra/p03/runtime_metadata':raw['metadata'].append(item);return
            elif topic in q.COORDINATOR_TOPICS:
                q.coordinator_identity(item,value)
                if topic.endswith('/gate'):
                    q.gate_record(item,value)
                    if raw['gate'] is not None and raw['gate']!=item:raise ValueError('gate identity mutated')
                    raw['gate']=item
                elif topic.endswith('/identity'):raw['coordinator_identity'].append(item)
                elif topic.endswith('/status'):
                    if item.get('phase') not in {'bootstrap','warmup','activating','active'}|q.TERMINAL_PHASES:raise ValueError('unknown coordinator phase')
                    q.terminal_record(item,value);raw['coordinator_status'].append(item)
                else:
                    raw['command_trace_summary'].append({key:item.get(key) for key in
                      ('sequence','publisher_visible_clock_ns','actual_odom_source_ns','gate_source_ns','phase','v','w','reason','terminal_source_ns')})
                return
            else:raise ValueError('unrecognized evidence JSON topic')
        else:stamp=message.header.stamp.to_nsec()
        with lock:
            raw['source_stamps'].setdefault(topic,[]).append(stamp);latest[topic]=stamp
            if isinstance(message,WorldState):last_state[topic]=message
            if topic in ('/m2_driver/physical_status','/m2_driver/command_ingress'):
                raw['status_messages' if topic.endswith('physical_status') else 'command_ingress'].append({'stamp_ns':stamp,'sequence':message.header.seq,'statuses':[
                    {'name':r.name,'hardware_id':r.hardware_id,'level':r.level,'message':r.message,'keys':[x.key for x in r.values],'values':{x.key:x.value for x in r.values}} for r in message.status]})
            if topic=='/astra/p03/end_models':source_wall.append((stamp,time.monotonic_ns()))
    def callback(topic,message):
        evidence.ingest(lambda:parse_callback(topic,message),topic)
    subscribers=[]
    for topic in q.DENSE_TOPICS:
        kind=String if topic.endswith('step_status') else ContactsState if topic.endswith('physics_contacts') else WorldState
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:callback(t,msg),queue_size=10000,buff_size=16*1024*1024))
    for topic,kind in [('/astra/p03/runtime_metadata',String),('/m2_driver/physical_status',DiagnosticArray),
                       ('/m2_driver/command_ingress',DiagnosticArray)]+[(name,String) for name in q.COORDINATOR_TOPICS]:
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:callback(t,msg),queue_size=10000))
    if value['camera_topic'] is not None:
        subscribers.append(rospy.Subscriber(value['camera_topic'],Image,
            lambda message:evidence.ingest(lambda:last_image.__setitem__(0,message),'native_camera'),queue_size=1,buff_size=16*1024*1024))
    def source_now():
        # Latest actual End header, never call local /clock exact Gazebo service time.
        return latest.get('/astra/p03/end_models',0)
    def gateway(name,kind,*arguments,timeout=5.):
        if name not in q.SERVICE_PHASES or phase[0] not in q.SERVICE_PHASES[name]:raise ValueError('forbidden service/phase '+name)
        if phase[0]!='failure':timeout=min(timeout,lifecycle_deadline.remaining())
        if drain_deadline[0] is not None:timeout=min(timeout,drain_deadline[0].remaining())
        event={'sequence':len(raw['services']),'name':name,'phase':phase[0],'source_ns':source_now(),'publisher_clock_ns':rospy.Time.now().to_nsec(),
               'wall_monotonic_ns':time.monotonic_ns(),'completed':False,'error':None}
        raw['services'].append(event);durable_journal('service_calls.jsonl',dict(event,event='request'))
        done=threading.Event();response=[];errors=[]
        def invoke():
            try:response.append(rospy.ServiceProxy(name,kind)(*arguments))
            except BaseException as error:errors.append(error)
            finally:done.set()
        threading.Thread(target=invoke,daemon=True).start()
        if not done.wait(timeout):
            event['error']='wall timeout; actual completion unknown';durable_journal('service_calls.jsonl',dict(event,event='timeout'))
            raise TimeoutError(name)
        if errors:
            event['error']=str(errors[0]);durable_journal('service_calls.jsonl',dict(event,event='error'));raise errors[0]
        answer=response[0]
        event['completed']=True;event['response_publisher_clock_ns']=rospy.Time.now().to_nsec();event['response_wall_ns']=time.monotonic_ns()
        event['response']={key:getattr(answer,key) for key in ('success','status_message','message','sim_time','pause','time_step') if hasattr(answer,key)}
        if hasattr(answer,'success') and not answer.success:event['error']=getattr(answer,'status_message',getattr(answer,'message','service rejected'))
        if drain_deadline[0] is not None:
            try:drain_deadline[0].check()
            except TimeoutError as error:event['error']=str(error)
        if phase[0]!='failure':
            try:lifecycle_deadline.check()
            except TimeoutError as error:event['error']=str(error)
        durable_journal('service_calls.jsonl',dict(event,event='response'))
        if event['error']:raise RuntimeError(name+': '+event['error'])
        return answer
    def rpc(uri,method,*args):
        timeout=min(2.,lifecycle_deadline.remaining())
        if drain_deadline[0] is not None:timeout=min(timeout,drain_deadline[0].remaining())
        with xmlrpc.client.ServerProxy(uri,transport=Transport(timeout)) as server:
            code,message,result=getattr(server,method)(rospy.get_name(),*args)
        if drain_deadline[0] is not None:drain_deadline[0].check()
        lifecycle_deadline.check()
        if code!=1:raise RuntimeError(method+': '+message)
        return result
    def actual_planner_parameters():
        params={}
        if value['method'] in ('actual_teb_common_route','dwa_common_route'):
            params={name:rospy.get_param('/move_base/'+name) for name in ('base_local_planner','base_global_planner')}
        return q.planner_parameters(params,value['method'])
    def roster():
        system=rpc(master_uri,'getSystemState')
        nodes=sorted({node for category in system for _name,members in category for node in members})
        unexpected=sorted(set(nodes)-set(value['allowed_nodes']));pubs=dict(system[0])
        row={'source_ns':source_now(),'publisher_clock_ns':rospy.Time.now().to_nsec(),'wall_monotonic_ns':time.monotonic_ns(),'nodes':nodes,
             'unexpected_nodes':unexpected,'missing_nodes':sorted(set(value['required_nodes'])-set(nodes)),
             'command_publishers':{topic:sorted(pubs.get(topic,[])) for topic in ('/cmd_vel_navigation','/cmd_vel')}}
        row['pass']=(not unexpected and not row['missing_nodes'] and row['command_publishers']=={
            '/cmd_vel_navigation':[value['coordinator_node']],'/cmd_vel':[value['relay_node']]})
        if value['method']=='frozen_r2_v060_sensor_lifecycle_v1':
            row['sensor_method_graph']=q.sensor_method_graph(system)
            row['pass']=row['pass'] and row['sensor_method_graph']['pass_']
        raw['node_roster'].append(row);durable_journal('node_roster.jsonl',row)
        raw['resource_samples'].append({'source_ns':source_now(),'wall_monotonic_ns':time.monotonic_ns(),
           'processes':[q.proc_sample(p) for p in (os.getpid(),gazebo_pid[0]) if p is not None]})
        return row
    def require_roster():
        row=roster()
        if not row['pass']:raise RuntimeError('runtime node/publisher roster changed: '+str(row))
        return row
    def process_roster(label):
        result=[]
        for node in sorted(set(value['required_nodes'])):
            uri=rpc(master_uri,'lookupNode',node)
            if urlparse(uri).hostname not in ('127.0.0.1','localhost'):raise ValueError('node URI not loopback: '+node)
            pid=rpc(uri,'getPid');item=q.proc_sample(pid)
            item.update(node=node,uri=uri,executable=q.binding(Path('/proc/{}/exe'.format(pid)).resolve()))
            result.append(item)
        raw['node_processes'].append({'phase':label,'source_ns':source_now(),'processes':result})
        return result
    def recorder_ready(topics):
        uri=rpc(master_uri,'lookupNode','/fixture_bag')
        if urlparse(uri).hostname not in ('127.0.0.1','localhost'):raise ValueError('recorder is not loopback')
        result=q.recorder_connections(rpc(uri,'getBusInfo'),topics)
        result.update(uri=uri,source_ns=rospy.Time.now().to_nsec(),wall_monotonic_ns=time.monotonic_ns())
        return result
    def wait_until(predicate,seconds,label):
        def guard():
            if raw['errors']:raise q.LifecycleFault(raw['errors'][-1])
            lifecycle_deadline.check()
        return q.wait_condition(predicate,min(seconds,lifecycle_deadline.remaining()),label,shutdown=rospy.is_shutdown,guard=guard)
    def snapshot(label,minimum_stamp):
        message=last_image[0]
        if message is None:return False
        stamp=message.header.stamp.to_nsec()
        if stamp<minimum_stamp:return False
        if message.width!=2560 or message.height!=1440 or message.encoding not in ('rgb8','bgr8'):raise ValueError('camera image contract mismatch')
        import numpy as np
        from PIL import Image as PILImage
        pixels=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
        if message.encoding=='bgr8':pixels=pixels[:,:,::-1]
        if float(pixels.std())<1:raise ValueError('camera image lacks visible variation')
        directory=root/'images';directory.mkdir(exist_ok=True)
        path=directory/(label+'.png');PILImage.fromarray(pixels,'RGB').save(str(path))
        original=io.BytesIO();message.serialize(original);(directory/(label+'.rosmsg')).write_bytes(original.getvalue())
        raw['images'].append(dict(q.binding(path),source_ns=stamp,width=message.width,height=message.height,
                                  raw_message=q.binding(directory/(label+'.rosmsg')),visual_review='pending_human_or_image_inspection'))
        return True
    image_errors=[]
    def supplementary_snapshot(label,minimum_stamp):
        try:return snapshot(label,minimum_stamp)
        except Exception as error:
            image_errors.append(dict(label=label,minimum_source_ns=minimum_stamp,
                error=type(error).__name__+': '+str(error)))
            return False
    start=end=watermark=None;failure=None
    try:
        urdf=rospy.get_param('/robot_description')
        raw['robot_startup_validation']=q.validate_robot_description(urdf,value)
        (root/'robot_description_runtime.xml').write_text(urdf)
        if hashlib.sha256(urdf.encode()).hexdigest()!=value['robot_description']['sha256']:raise RuntimeError('runtime URDF differs from exact launch file')
        rospy.wait_for_service('/gazebo/spawn_urdf_model',timeout=min(value['readiness_wall_s'],lifecycle_deadline.remaining()))
        pose=Pose();x,y,z,yaw=value['spawn_world_xyz_yaw'];pose.position.x=x;pose.position.y=y;pose.position.z=z
        pose.orientation.z=math.sin(yaw/2);pose.orientation.w=math.cos(yaw/2)
        gateway('/gazebo/spawn_urdf_model',SpawnModel,'autolabor_m2',urdf,'/',pose,'world')
        wait_until(lambda:roster()['pass'],20.,'node readiness')
        raw['recorder_before_unpause']=wait_until(lambda:(lambda x:x if x['pass'] else None)(recorder_ready(value['required_before_unpause'])),20.,'recorder connections before unpause')
        gazebo_uri=rpc(master_uri,'lookupNode','/gazebo');pid=rpc(gazebo_uri,'getPid');gazebo_pid[0]=pid
        bootstrap_processes=process_roster('bootstrap')
        bootstrap_planners=actual_planner_parameters();raw['planner_parameters']={'bootstrap':bootstrap_planners}
        common.atomic(root/'ready_for_maps.json',{'pid':pid,'source_ns':rospy.Time.now().to_nsec(),'phase':'bootstrap',
          'node_processes':bootstrap_processes,'planner_parameters':bootstrap_planners})
        wait_until(lambda:(root/'maps_pre_ready.json').is_file(),30.,'parent actual ELF snapshot')
        gateway('/gazebo/unpause_physics',Empty);phase[0]='warmup'
        warmup_query=gateway('/gazebo/get_world_properties',GetWorldProperties)
        raw['positive_time_bootstrap_query_ns']=round(warmup_query.sim_time*1e9)
        def method_and_recorder_ready():
            if not q.readiness_guard(source_now(),raw['coordinator_status'][-1] if raw['coordinator_status'] else None,value):return False
            result=recorder_ready(value['required_before_arm'])
            if not result['pass']:return False
            if not all(t in latest for t in q.DENSE_TOPICS) or not raw['metadata']:return False
            raw['recorder_before_arm']=result;return True
        wait_until(method_and_recorder_ready,value['readiness_wall_s'],'positive-clock method/recorder readiness before5s')
        if source_now()>=value['activation_source_ns']:raise RuntimeError('arm requested after fixed activation')
        require_roster();answer=gateway('/astra/navigation/arm',Trigger)
        raw['arm_response']=json.loads(answer.message);phase[0]='active'
        if any(raw['arm_response'].get(k)!=value[k] for k in ('run_id','seed')) or raw['arm_response'].get('activation_ns')!=value['activation_source_ns']:raise ValueError('arm response identity/timing mismatch')
        last_roster=0.;last_source=source_now();progress_wall=time.monotonic();terminal=None
        while not rospy.is_shutdown():
            now=source_now();wall=time.monotonic()
            if now<last_source:raise RuntimeError('source clock regression')
            if now>last_source:progress_wall=wall
            if wall-progress_wall>value['clock_stall_wall_s']:raise RuntimeError('source clock stalled')
            last_source=now
            if wall-started_wall>value['wall_cap_s']:raise TimeoutError('collector wall cap')
            if wall-last_roster>=.20:
                require_roster();last_roster=wall
                model=last_state.get('/astra/p03/end_models')
                if model:
                    if set(model.name)!=set(value['expected_models']):raise RuntimeError('actual model roster differs from bound world')
                    robot=model.pose[model.name.index('autolabor_m2')]
                    if (robot.position.x**2+robot.position.y**2)**.5>value['maximum_robot_xy_radius_m']:raise RuntimeError('robot left declared domain')
            if raw['errors']:raise RuntimeError(raw['errors'][-1])
            if raw['gate'] is not None and start is None:start=raw['gate']['source_ns'];raw['start_ns']=start
            if raw['coordinator_status']:terminal=q.terminal_record(raw['coordinator_status'][-1],value)
            if value['camera_topic'] is not None and start is not None and now>=start+1_000_000_000 and not raw['images']:
                supplementary_snapshot('gate_plus1s',start+1_000_000_000)
            if terminal is not None and now>=terminal['terminal_source_ns']:
                end=terminal['terminal_source_ns'];raw.update(end_ns=end,terminal=terminal)
                if value['camera_topic'] is not None and start is not None:supplementary_snapshot('near_terminal_at_or_after_source_minus500ms',max(start,end-500_000_000))
                break
            time.sleep(.002)
        if terminal is None:raise RuntimeError('shutdown before coordinator terminal evidence')
        if start is None or end is None or end<=start:raise RuntimeError('no positive gate-to-terminal evidence interval')
        phase[0]='terminal';drain_deadline[0]=q.WallDeadline(value['drain_wall_s'])
        gateway('/astra/navigation/stop',Trigger)
        gateway('/gazebo/pause_physics',Empty)
        physics=gateway('/gazebo/get_physics_properties',GetPhysicsProperties)
        if physics.pause is not True or abs(physics.time_step-.002)>1e-12:raise ValueError('world not paused at fixed step')
        times=[]
        for _ in range(2):
            world=gateway('/gazebo/get_world_properties',GetWorldProperties)
            stamp=round(world.sim_time*1e9)
            if stamp%q.STEP_NS or abs(world.sim_time-stamp/1e9)>1e-9 or stamp<end:raise ValueError('invalid final source watermark')
            times.append(stamp)
            if drain_deadline[0].remaining()<.05:raise TimeoutError('drain deadline cannot accommodate stability observation')
            time.sleep(.05);drain_deadline[0].check()
        if times[0]!=times[1]:raise ValueError('paused source time changed')
        watermark=times[0]
        wait_until(lambda:all(latest.get(t)==watermark for t in q.DENSE_TOPICS),
                   drain_deadline[0].remaining(),'final source watermark drain')
        require_roster();terminal_processes=process_roster('terminal');drain_deadline[0].check()
        terminal_planners=actual_planner_parameters();raw['planner_parameters']['terminal']=terminal_planners;drain_deadline[0].check()
        raw['final_drain']={'source_watermark_ns':watermark,'queried_source_ns':times,'latest':dict(latest),
                            'wall_elapsed_s':drain_deadline[0].elapsed(),'deadline_scope':'coordinator stop request through pause, queries, joins, roster and process checks','physics_pause':physics.pause,'physics_step':physics.time_step}
        raw['image_coverage']=q.image_coverage(raw['images'],image_errors,value['camera_topic'])
        drain_deadline[0].check();drain_deadline[0]=None
        common.atomic(root/'ready_for_maps_final.json',{'pid':pid,'source_ns':watermark,'phase':'terminal',
          'node_processes':terminal_processes,'planner_parameters':terminal_planners})
        wait_until(lambda:(root/'maps_final_ready.json').is_file(),30.,'parent final actual ELF snapshot')
        lifecycle_deadline.check()
    except BaseException as error:
        failure=type(error).__name__+': '+str(error);raw['errors'].append(failure)
        phase[0]='failure';drain_deadline[0]=None
        try:gateway('/astra/navigation/stop',Trigger,timeout=2.)
        except BaseException as cleanup:raw['errors'].append('failure method stop: '+str(cleanup))
        try:gateway('/gazebo/pause_physics',Empty,timeout=2.)
        except BaseException as cleanup:raw['errors'].append('failure pause: '+str(cleanup))
    # Close callbacks before any final coverage/pass computation or file write.
    raw=evidence.freeze({'wall_elapsed_s':time.monotonic()-started_wall})
    raw['image_coverage']=q.image_coverage(raw['images'],image_errors,value['camera_topic'])
    source_wall=raw['source_wall_samples']
    if start is not None and end is not None:
        raw['coverage']={t:q.source_coverage(raw['source_stamps'].get(t,[]),start,end) for t in q.DENSE_TOPICS}
        raw['service_audit']=q.validate_services(raw['services'],start,end)
    passed=(failure is None and not raw['errors'] and bool(raw.get('coverage')) and
            all(x['pass'] for x in raw['coverage'].values()) and raw['service_audit']['pass'])
    # Only the detached frozen copy is used below.
    common.atomic(root/'collector_raw.json',raw)
    common.atomic(root/'collector_report.json',{'run_id':value['run_id'],'seed':value['seed'],'pass':passed,
        'method':value['method'],'stage':value['stage'],'task_outcome':None if raw.get('terminal') is None else raw['terminal']['phase'],
        'goal_or_performance_assessed':False,'errors':raw['errors'],'image_coverage':raw['image_coverage'],'source_window_ns':[start,end],'watermark_ns':watermark,
        'wall_elapsed_s':raw['wall_elapsed_s'],'raw_sha256':q.sha(root/'collector_raw.json'),
        'descriptive_overall_sim_seconds_per_wall_second':q.passive.source_window_wall_ratio(start,end,raw['wall_elapsed_s'],
            complete=bool(raw.get('coverage')) and all(x['pass'] for x in raw['coverage'].values())),
        'overall_ratio_definition':'complete gate-to-terminal source-window duration over total collector wall including bootstrap/drain; null if window incomplete',
        'descriptive_active_source_rtf':q.source_rtf(source_wall,start,end),'performance_claim':False,'runtime_source_closure_not_yet_assessed':True})
    return 0 if passed else 3
if __name__=='__main__':raise SystemExit(main())
