#!/usr/bin/env python3
"""Q04 evidence coordinator, audited service gateway and bounded collector.

No navigation policy. Full dense evidence lives in the independently recorded
bag; this node retains source coverage, runtime roster and service observations.
"""
import hashlib
import http.client
import io
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
import passive_sweep_q04 as q
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
    rospy.init_node('passive_collector',anonymous=False)
    root=Path(rospy.get_param('~output'));root.mkdir(exist_ok=True)
    value=q.validate_contract(json.loads(q.read_bound({'path':rospy.get_param('~contract_path'),'sha256':rospy.get_param('~contract_sha256')})))
    master_uri=os.environ.get('ROS_MASTER_URI','');parsed=urlparse(master_uri)
    if parsed.hostname not in ('127.0.0.1','localhost') or parsed.port!=14820 or parsed.username or parsed.password:
        raise ValueError('isolated explicit loopback master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:
        raise ValueError('simulation-only guard required')
    phase=['bootstrap'];drain_deadline=[None];raw={'run_id':value['run_id'],'seed':value['seed'],
        'contract_sha256':q.sha(root/'contract.json'),'source_stamps':{},'status_messages':[],
        'command_events':[],'command_status':[],'command_ingress':[],'node_processes':[],'metadata':[],'node_roster':[],'services':[],
        'errors':[],'images':[],'resource_samples':[],'source_wall_samples':[]}
    evidence=q.EvidenceStore(raw);lock=evidence.lock
    latest={};last_state={};last_image=[None];gazebo_pid=[None];source_wall=raw['source_wall_samples'];started_wall=time.monotonic()
    def durable_journal(name,row):
        with (root/name).open('a') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
    def parse_callback(topic,message):
        if topic in ('/astra/p03/step_status','/astra/p03/runtime_metadata','/astra_q04/command_event','/astra_q04/command_status'):
            item=json.loads(message.data)
            if topic.endswith('step_status'):stamp=item['stamp_ns']
            elif topic.endswith('runtime_metadata'):
                with lock:raw['metadata'].append(item)
                return
            elif topic.endswith('command_event'):
                with lock:raw['command_events'].append(item)
                return
            else:
                with lock:raw['command_status'].append(item)
                return
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
    for topic,kind in [('/astra/p03/runtime_metadata',String),('/astra_q04/command_event',String),
                       ('/astra_q04/command_status',String),('/m2_driver/physical_status',DiagnosticArray),('/m2_driver/command_ingress',DiagnosticArray)]:
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:callback(t,msg),queue_size=10000))
    if value['camera']:
        subscribers.append(rospy.Subscriber('/astra_q04/camera/image_raw',Image,
            lambda message:evidence.ingest(lambda:last_image.__setitem__(0,message),'native_camera'),queue_size=1,buff_size=16*1024*1024))
    def source_now():
        # Latest actual End header, never call local /clock exact Gazebo service time.
        return latest.get('/astra/p03/end_models',0)
    def gateway(name,kind,*arguments,timeout=5.):
        if name not in q.SERVICE_PHASES or phase[0] not in q.SERVICE_PHASES[name]:raise ValueError('forbidden service/phase '+name)
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
        event['completed']=True;event['response_source_ns']=rospy.Time.now().to_nsec();event['response_wall_ns']=time.monotonic_ns()
        event['response']={key:getattr(answer,key) for key in ('success','status_message','message','sim_time','pause','time_step') if hasattr(answer,key)}
        if hasattr(answer,'success') and not answer.success:event['error']=getattr(answer,'status_message',getattr(answer,'message','service rejected'))
        if drain_deadline[0] is not None:
            try:drain_deadline[0].check()
            except TimeoutError as error:event['error']=str(error)
        durable_journal('service_calls.jsonl',dict(event,event='response'))
        if event['error']:raise RuntimeError(name+': '+event['error'])
        return answer
    def rpc(uri,method,*args):
        with xmlrpc.client.ServerProxy(uri,transport=Transport(min(2.,drain_deadline[0].remaining()) if drain_deadline[0] is not None else 2.)) as server:
            code,message,result=getattr(server,method)(rospy.get_name(),*args)
        if drain_deadline[0] is not None:drain_deadline[0].check()
        if code!=1:raise RuntimeError(method+': '+message)
        return result
    def roster():
        system=rpc(master_uri,'getSystemState')
        nodes=sorted({node for category in system for _name,members in category for node in members})
        unexpected=sorted(set(nodes)-q.ALLOWED_NODES);pubs=dict(system[0])
        row={'source_ns':source_now(),'publisher_clock_ns':rospy.Time.now().to_nsec(),'wall_monotonic_ns':time.monotonic_ns(),'nodes':nodes,
             'unexpected_nodes':unexpected,'missing_nodes':sorted(q.REQUIRED_NODES-set(nodes)),
             'command_publishers':{topic:sorted(pubs.get(topic,[])) for topic in ('/cmd_vel_navigation','/cmd_vel')}}
        row['pass']=(not unexpected and not row['missing_nodes'] and row['command_publishers']=={
            '/cmd_vel_navigation':['/passive_command'],'/cmd_vel':['/passive_command_relay']})
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
        for node in sorted(q.REQUIRED_NODES):
            uri=rpc(master_uri,'lookupNode',node)
            if urlparse(uri).hostname not in ('127.0.0.1','localhost'):raise ValueError('node URI not loopback: '+node)
            pid=rpc(uri,'getPid');item=q.proc_sample(pid)
            item.update(node=node,uri=uri,executable=q.binding(Path('/proc/{}/exe'.format(pid)).resolve()))
            result.append(item)
        raw['node_processes'].append({'phase':label,'source_ns':source_now(),'processes':result})
        return result
    def recorder_ready():
        uri=rpc(master_uri,'lookupNode','/fixture_bag')
        if urlparse(uri).hostname not in ('127.0.0.1','localhost'):raise ValueError('recorder is not loopback')
        result=q.recorder_connections(rpc(uri,'getBusInfo'),q.RECORD_TOPICS)
        result.update(uri=uri,source_ns=rospy.Time.now().to_nsec(),wall_monotonic_ns=time.monotonic_ns())
        return result
    def wait_until(predicate,seconds,label):
        end=time.monotonic()+seconds;last_error=None
        while not rospy.is_shutdown() and time.monotonic()<end:
            if raw['errors']:raise RuntimeError(raw['errors'][-1])
            try:
                result=predicate()
                if result:return result
            except Exception as error:last_error=repr(error)
            time.sleep(.02)
        raise TimeoutError(label+': '+str(last_error))
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
    start=end=watermark=None;failure=None
    try:
        urdf=rospy.get_param('/robot_description')
        raw['robot_startup_validation']=q.validate_robot_description(urdf,value)
        (root/'robot_description_runtime.xml').write_text(urdf)
        if hashlib.sha256(urdf.encode()).hexdigest()!=q.sha(root/'robot_description.xml'):raise RuntimeError('runtime URDF differs from exact launch file')
        rospy.wait_for_service('/gazebo/spawn_urdf_model',timeout=value['readiness_wall_s'])
        pose=Pose();pose.orientation.w=1.
        gateway('/gazebo/spawn_urdf_model',SpawnModel,'autolabor_m2',urdf,'/',pose,'world')
        wait_until(lambda:roster()['pass'],20.,'node readiness')
        raw['recorder_before_unpause']=wait_until(lambda:(lambda x:x if x['pass'] else None)(recorder_ready()),20.,'recorder connections')
        gazebo_uri=rpc(master_uri,'lookupNode','/gazebo');pid=rpc(gazebo_uri,'getPid');gazebo_pid[0]=pid
        process_roster('bootstrap')
        common.atomic(root/'ready_for_maps.json',{'pid':pid,'source_ns':rospy.Time.now().to_nsec(),'phase':'bootstrap'})
        wait_until(lambda:(root/'maps_pre_ready.json').is_file(),30.,'parent actual ELF snapshot')
        gateway('/gazebo/unpause_physics',Empty);phase[0]='warmup'
        warmup_query=gateway('/gazebo/get_world_properties',GetWorldProperties)
        warmup_start=round(warmup_query.sim_time*1e9)
        wait_until(lambda:source_now()>=warmup_start+value['warmup_ns'] and all(t in latest for t in q.DENSE_TOPICS)
            and bool(raw['metadata']),value['readiness_wall_s'],'positive source clock/data warmup')
        require_roster();raw['warmup_end_ns']=source_now()
        answer=gateway('/astra_q04/start',Trigger)
        started=json.loads(answer.message)
        if started['run_id']!=value['run_id'] or started['seed']!=value['seed']:raise ValueError('command start identity mismatch')
        start=started['start_ns'];end=start+q.DURATION_NS;raw.update(start_ns=start,end_ns=end);phase[0]='active'
        last_roster=0.;last_source=source_now();progress_wall=time.monotonic()
        while not rospy.is_shutdown() and source_now()<end:
            now=source_now();wall=time.monotonic()
            if now<last_source:raise RuntimeError('source clock regression')
            if now>last_source:progress_wall=wall
            if wall-progress_wall>value['clock_stall_wall_s']:raise RuntimeError('source clock stalled')
            last_source=now
            if wall-started_wall>value['wall_cap_s']:raise TimeoutError('collector wall cap')
            if wall-last_roster>=.20:
                require_roster();last_roster=wall
                model=last_state.get('/astra/p03/end_models')
                if model and 'autolabor_m2' in model.name:
                    pose=model.pose[model.name.index('autolabor_m2')]
                    if (pose.position.x**2+pose.position.y**2)**.5>value['maximum_robot_xy_radius_m']:raise RuntimeError('robot left fixture domain')
            if raw['errors']:raise RuntimeError(raw['errors'][-1])
            if raw['command_status'] and raw['command_status'][-1].get('state')=='fault':raise RuntimeError('command source fault')
            if raw['status_messages'] and any(s['values'].get('backend_fault')=='true' for s in raw['status_messages'][-1]['statuses']):raise RuntimeError('physical backend fault')
            if value['camera']:
                if now>=start+1_000_000_000 and not raw['images']:snapshot('pulse_start',start+1_000_000_000)
                if now>=start+8_000_000_000 and len(raw['images'])==1:snapshot('stopped',start+8_000_000_000)
            time.sleep(.002)
        phase[0]='terminal';drain_deadline[0]=q.WallDeadline(value['drain_wall_s'])
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
        wait_until(lambda:bool(raw['command_status']) and raw['command_status'][-1].get('state')=='complete'
            and q.command_audit(raw['command_events'],value,start)['pass'],
            drain_deadline[0].remaining(),'final command event drain')
        require_roster();process_roster('terminal');drain_deadline[0].check()
        raw['final_drain']={'source_watermark_ns':watermark,'queried_source_ns':times,'latest':dict(latest),
                            'wall_elapsed_s':drain_deadline[0].elapsed(),'deadline_scope':'pause request through queries, joins, roster and process checks','physics_pause':physics.pause,'physics_step':physics.time_step}
        if value['camera'] and len(raw['images'])<2:raise RuntimeError('requested native image snapshots absent')
        drain_deadline[0].check();drain_deadline[0]=None
        common.atomic(root/'ready_for_maps_final.json',{'pid':pid,'source_ns':watermark,'phase':'terminal'})
        wait_until(lambda:(root/'maps_final_ready.json').is_file(),30.,'parent final actual ELF snapshot')
    except BaseException as error:
        failure=type(error).__name__+': '+str(error);raw['errors'].append(failure)
        phase[0]='failure';drain_deadline[0]=None
        try:gateway('/gazebo/pause_physics',Empty,timeout=2.)
        except BaseException as cleanup:raw['errors'].append('failure pause: '+str(cleanup))
    # Close callbacks before any final coverage/pass computation or file write.
    raw=evidence.freeze({'wall_elapsed_s':time.monotonic()-started_wall})
    source_wall=raw['source_wall_samples']
    if start is not None and end is not None:
        raw['coverage']={t:q.source_coverage(raw['source_stamps'].get(t,[]),start,end) for t in q.DENSE_TOPICS}
        raw['service_audit']=q.validate_services(raw['services'],start,end)
    passed=(failure is None and not raw['errors'] and bool(raw.get('coverage')) and
            all(x['pass'] for x in raw['coverage'].values()) and raw['service_audit']['pass'])
    # Only the detached frozen copy is used below.
    common.atomic(root/'collector_raw.json',raw)
    common.atomic(root/'collector_report.json',{'run_id':value['run_id'],'seed':value['seed'],'pass':passed,
        'errors':raw['errors'],'source_window_ns':[start,end],'watermark_ns':watermark,
        'wall_elapsed_s':raw['wall_elapsed_s'],'raw_sha256':q.sha(root/'collector_raw.json'),
        'descriptive_overall_sim_seconds_per_wall_second':q.source_window_wall_ratio(start,end,raw['wall_elapsed_s'],
            complete=bool(raw.get('coverage')) and all(x['pass'] for x in raw['coverage'].values())),
        'overall_ratio_definition':'complete formal source-window duration over total collector wall time including bootstrap/drain; not whole-run clock RTF; null if window incomplete',
        'descriptive_active_source_rtf':q.source_rtf(source_wall,start,end),'performance_claim':False,'runtime_source_closure_not_yet_assessed':True})
    return 0 if passed else 3
if __name__=='__main__':raise SystemExit(main())
