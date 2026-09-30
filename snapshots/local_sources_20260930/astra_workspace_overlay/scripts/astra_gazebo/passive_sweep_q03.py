#!/usr/bin/env python3
"""Independent Q03 fixture contracts, geometry and fail-closed evidence helpers.

Importing this module does not import ROS or start any process.
"""
import hashlib
import copy
import threading
import time
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

WORKSPACE=Path(__file__).resolve().parents[2]
STAGE=WORKSPACE.parent
NAME='astra_passive_sweep_q03'
STEP_NS=2_000_000
PERIOD_NS=50_000_000
DURATION_NS=10_000_000_000
PHASES=(('settle',0,1,0.,0.),('straight',1,3,.3,0.),
        ('left',3,5,.3,.1),('right',5,7,.3,-.1),
        ('straight_exit',7,8,.3,0.),('stop',8,10,0.,0.))
FIXTURES=(('passive_sweep_q03_static',260926027,False),('passive_sweep_q03_actor',260926028,True))
DENSE_TOPICS=tuple('/astra/p03/'+x for x in ('pre_models','pre_links','end_models','end_links','step_status'))+('/astra/physics_contacts',)
RECORD_TOPICS=DENSE_TOPICS+('/astra/p03/runtime_metadata','/astra/physics_metadata',
 '/astra/world_models','/astra/world_links','/clock','/odom','/joint_states',
 '/m2_driver/physical_status','/m2_driver/command_ingress','/m2_driver/physical_metadata','/m2_driver/actuator_target',
 '/m2_driver/actuator_commanded_effort','/cmd_vel_navigation','/cmd_vel',
 '/astra_q03/command_event','/astra_q03/command_status','/astra_q03/start_ns')
ALLOWED_NODES={'/rosout','/gazebo','/fixture_bag','/passive_collector',
 '/passive_command','/passive_command_relay','/m2_robot_state_publisher'}
REQUIRED_NODES=ALLOWED_NODES
SERVICE_PHASES={
 '/gazebo/spawn_urdf_model':{'bootstrap'},
 '/gazebo/unpause_physics':{'bootstrap'},
 '/astra_q03/start':{'warmup'},
 '/gazebo/pause_physics':{'terminal','failure'},
 '/gazebo/get_world_properties':{'bootstrap','warmup','terminal'},
 '/gazebo/get_physics_properties':{'bootstrap','terminal'}}
ELF_ROLES={'libgazebo_ode.so':'ode_integrator','libgazebo_physics.so':'gazebo_physics',
 'libgazebo_ros_api_plugin.so':'gazebo_ros_api',
 'libastra_physical_ackermann_p05_plugin.so':'driver_binary',
 'libastra_physics_metadata_p03_plugin.so':'evidence_binary',
 'libastra_dense_evidence_plugin.so':'whole_contact_observer',
 'libastra_reactive_actor_plugin.so':'actor_binaries',
 'libgazebo_ros_camera.so':'camera',
 'libgazebo_ros_laser.so':'laser_sensor','libgazebo_ros_bumper.so':'chassis_contact_sensor',
 'libgazebo_ros_imu_sensor.so':'imu_sensor'}
SOURCE_FILES=(
 'scripts/astra_gazebo/mapped_elf_inventory.py',
 'scripts/astra_gazebo/passive_sweep_q03.py','scripts/astra_gazebo/run_smoke_a02.py','scripts/astra_gazebo/run_passive_sweep_q03.py',
 'scripts/astra_gazebo/dense_sweep_p03.py','scripts/astra_gazebo/dense_sweep.py',
 'scripts/astra_gazebo/geometry.py','scripts/astra_gazebo/evaluate_episode.py',
 'src/simulation/m2_gazebo/scripts/astra_passive_command_q03.py',
 'src/simulation/m2_gazebo/scripts/astra_passive_collector_q03.py',
 'src/simulation/m2_gazebo/src/astra_physics_metadata_p03_plugin.cpp',
 'src/simulation/m2_gazebo/src/astra_dense_evidence_plugin.cpp',
 'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp',
 'src/simulation/m2_gazebo/include/m2_gazebo/astra_physical_control.h',
 'src/simulation/m2_gazebo/include/m2_gazebo/actuator_dynamics.h',
 'src/simulation/m2_gazebo/include/m2_gazebo/ackermann_kinematics.h',
 'src/simulation/m2_gazebo/src/astra_reactive_actor_plugin.cpp',
 'src/simulation/m2_gazebo/include/m2_gazebo/astra_actor_gate.h',
 'src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h',
 'src/simulation/m2_gazebo/urdf/m2_physical_p05.urdf.xacro',
 'src/simulation/m2_gazebo/urdf/m2_physical_p05_gazebo.xacro',
 'src/simulation/m2_gazebo/urdf/m2_physical_base.xacro',
 'src/simulation/m2_gazebo/urdf/m2_sensors.xacro',
 'scripts/astra_env.sh')

class EvidenceStore:
    """One lock owns callback parsing/errors and the terminal immutable copy."""
    def __init__(self,raw):
        self.raw=raw;self.lock=threading.RLock();self.active=True
    def ingest(self,callback,label):
        with self.lock:
            if not self.active:return False
            try:callback()
            except Exception as error:self.raw['errors'].append('callback '+label+': '+repr(error))
            return True
    def freeze(self,updates=None):
        with self.lock:
            if not self.active:raise RuntimeError('evidence already closed')
            self.active=False
            self.raw.update(updates or {})
            return copy.deepcopy(self.raw)

class WallDeadline:
    def __init__(self,seconds,clock=time.monotonic):
        if not math.isfinite(seconds) or seconds<=0:raise ValueError('positive wall budget required')
        self.clock=clock;self.started=clock();self.end=self.started+seconds
    def remaining(self):
        left=self.end-self.clock()
        if left<=0:raise TimeoutError('total drain wall deadline exhausted')
        return left
    def check(self):
        if self.clock()>self.end:raise TimeoutError('total drain wall deadline exceeded')
    def elapsed(self):return self.clock()-self.started


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def binding(path):
    p=Path(path).resolve();return {'path':str(p),'sha256':sha(p)}

def read_bound(item):
    payload=Path(item['path']).read_bytes()
    if hashlib.sha256(payload).hexdigest()!=item['sha256']:raise ValueError('bound resource changed: '+item['path'])
    return payload

def integer_ns(value):
    if type(value)is not int or not 0<=value<2**63:raise ValueError('nonnegative int64 nanoseconds required')
    return value

def contract(run_id,steering_kd=None,camera=True):
    row=next((r for r in FIXTURES if r[0]==run_id),None)
    if row is None:raise ValueError('unknown reserved fixture identity')
    if steering_kd is not None and (type(steering_kd) not in (int,float) or steering_kd not in (4,6)):
        raise ValueError('Kd must be root-resolved 4 or6, or unresolved None')
    if type(camera)is not bool:raise ValueError('camera flag must be boolean')
    return {'schema':NAME+'_contract_v1','stage':NAME,'run_id':row[0],'seed':row[1],'attempt':1,
      'actor':row[2],'camera':camera,'execution_authorized':False,'retry_allowed':False,
      'training_authorized':False,'navigation_performance_evidence':False,'real_vehicle_authorized':False,
      'driver_variant':'physical_p05','command_time_semantics':'integer schedule on publisher-visible /clock; driver ingress uses Gazebo SimTime and is reported separately',
      'duration_ns':DURATION_NS,'physics_step_ns':STEP_NS,'command_period_ns':PERIOD_NS,
      'wheelbase_m':.65,'steering_kd':steering_kd,'warmup_ns':1_000_000_000,
      'readiness_wall_s':90.,'wall_cap_s':240.,'clock_stall_wall_s':5.,'drain_wall_s':5.,
      'command_max_lateness_ns':20_000_000,'maximum_robot_xy_radius_m':8.,
      'phase_schedule':[{'name':n,'start_ns':a*10**9,'end_ns':b*10**9,'v_mps':v,'steering_rad':d} for n,a,b,v,d in PHASES],
      'required_topics':list(RECORD_TOPICS),
      'scope':'passive algorithmic sweep/component qualification only; no navigation, learning or real continuous safety claim'}

def same_typed(first,second):
    if type(first)is not type(second):return False
    if isinstance(first,dict):return set(first)==set(second) and all(same_typed(first[k],second[k]) for k in first)
    if isinstance(first,list):return len(first)==len(second) and all(same_typed(a,b) for a,b in zip(first,second))
    return first==second

def validate_contract(value,require_resolved=True):
    expected=contract(value.get('run_id'),value.get('steering_kd'),value.get('camera'))
    if not same_typed(value,expected):raise ValueError('closed Q03 contract differs from fixed design')
    if require_resolved and value['steering_kd'] is None:raise ValueError('physical Kd unresolved pending physical P05/root decision')
    return value

class CommandSchedule:
    """Integer source slots. Missed slots/clock regressions fault instead of replay."""
    def __init__(self,value):
        self.contract=validate_contract(value);self.start_ns=None;self.last_ns=None
        self.last_slot=-1;self.started=False;self.state='bootstrap';self.reason=None
    def start(self,now):
        integer_ns(now)
        if self.started or now<=0 or self.state!='bootstrap':raise ValueError('start is one-shot after positive clock')
        self.started=True
        # Both ends also lie on the50Hz source geometry/driver grid.
        self.start_ns=((now+19_999_999)//20_000_000)*20_000_000+100_000_000
        self.last_ns=now;self.state='running';return self.start_ns
    def poll(self,now):
        integer_ns(now)
        if self.last_ns is not None and now<self.last_ns:
            self.state='fault';self.reason='source clock reset';raise ValueError(self.reason)
        self.last_ns=now
        if self.state!='running' or now<self.start_ns:return None
        elapsed=now-self.start_ns
        if elapsed>=DURATION_NS:
            self.state='complete'
            return {'phase':'terminal_zero','actual_ns':now,'scheduled_ns':self.start_ns+DURATION_NS,
                    'slot':DURATION_NS//PERIOD_NS,'v':0.,'w':0.,'steering':0.,'lateness_ns':elapsed-DURATION_NS}
        slot=elapsed//PERIOD_NS
        if slot<=self.last_slot:return None
        late=elapsed-slot*PERIOD_NS
        if slot!=self.last_slot+1 or late>self.contract['command_max_lateness_ns']:
            self.state='fault';self.reason='missed or late source command slot';raise ValueError(self.reason)
        self.last_slot=slot
        phase=next(p for p in self.contract['phase_schedule'] if p['start_ns']<=elapsed<p['end_ns'])
        v,steer=phase['v_mps'],phase['steering_rad']
        return {'phase':phase['name'],'actual_ns':now,'scheduled_ns':self.start_ns+slot*PERIOD_NS,
                'slot':slot,'v':v,'w':v*math.tan(steer)/self.contract['wheelbase_m'],
                'steering':steer,'lateness_ns':late}

    def prepare_send(self,command,now):
        integer_ns(now)
        if now<command['actual_ns'] or (self.last_ns is not None and now<self.last_ns):
            self.state='fault';raise ValueError('source clock reset before publish')
        self.last_ns=now
        if command['phase']=='terminal_zero':
            return dict(command,actual_ns=now,lateness_ns=now-command['scheduled_ns'])
        if now>=self.start_ns+DURATION_NS:
            self.state='complete'
            return dict(command,phase='terminal_zero',actual_ns=now,scheduled_ns=self.start_ns+DURATION_NS,
                        slot=DURATION_NS//PERIOD_NS,v=0.,w=0.,steering=0.,lateness_ns=now-self.start_ns-DURATION_NS)
        if ((now-self.start_ns)//PERIOD_NS!=command['slot'] or
            now-command['scheduled_ns']>self.contract['command_max_lateness_ns']):
            self.state='fault';raise ValueError('source slot changed or became late before publish')
        return dict(command,actual_ns=now,lateness_ns=now-command['scheduled_ns'])

def command_audit(events,value,start):
    """Exact publisher-visible clock schedule; never substitute driver ingress time."""
    end=start+DURATION_NS
    rows=[x for x in events if type(x.get('actual_ns'))is int and start<=x['actual_ns']<end]
    reasons=[]
    if any(type(x.get('actual_ns'))is not int or x['actual_ns']<0 for x in events):reasons.append('invalid publisher clock stamp')
    if [x.get('slot') for x in rows]!=list(range(DURATION_NS//PERIOD_NS)):reasons.append('nonexact ordered command slots')
    for event in rows:
        slot=event.get('slot')
        if type(slot)is not int or not 0<=slot<200:reasons.append('invalid slot');continue
        elapsed=slot*PERIOD_NS
        phase=next(p for p in value['phase_schedule'] if p['start_ns']<=elapsed<p['end_ns'])
        expected={'phase':phase['name'],'scheduled_ns':start+elapsed,'run_id':value['run_id'],
          'v':phase['v_mps'],'steering':phase['steering_rad'],
          'w':phase['v_mps']*math.tan(phase['steering_rad'])/value['wheelbase_m']}
        if any(event.get(k)!=v for k,v in expected.items()):reasons.append('command differs from phase schedule')
        late=event['actual_ns']-expected['scheduled_ns']
        if event.get('lateness_ns')!=late or not 0<=late<=value['command_max_lateness_ns']:reasons.append('late/inconsistent publisher clock stamp')
    return {'pass':not reasons,'issues':sorted(set(reasons)),'observed_count':len(rows),
       'time_authority':'publisher-visible /clock, not unstamped Twist driver receipt',
       'maximum_publisher_lateness_ns':max((x.get('lateness_ns',0) for x in rows),default=None)}

def source_rtf(samples,start,end):
    rows=[(s,w) for s,w in samples if start is not None and start<=s<=end]
    if len(rows)<2 or rows[-1][1]<=rows[0][1]:return None
    return {'sim_seconds':(rows[-1][0]-rows[0][0])/1e9,
      'receipt_wall_seconds':(rows[-1][1]-rows[0][1])/1e9,
      'ratio':(rows[-1][0]-rows[0][0])/(rows[-1][1]-rows[0][1]),
      'interpretation':'descriptive within-run receipt throughput; no observer-off causal comparison'}


def proc_sample(pid):
    fields=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()
    import os
    return {'pid':pid,'proc_start_ticks':int(fields[19]),'user_cpu_ticks':int(fields[11]),
      'system_cpu_ticks':int(fields[12]),'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),
      'virtual_bytes':int(fields[20]),'resident_bytes':int(fields[21])*os.sysconf('SC_PAGE_SIZE')}


def world_sdf(value):
    validate_contract(value,False)
    root=ET.Element('sdf',version='1.6');world=ET.SubElement(root,'world',name=NAME)
    def text(node,tag,value):ET.SubElement(node,tag).text=str(value)
    text(world,'gravity','0 0 -9.81')
    physics=ET.SubElement(world,'physics',name='physics',type='ode');text(physics,'max_step_size',.002);text(physics,'real_time_update_rate',500)
    light=ET.SubElement(world,'light',name='sun',type='directional');text(light,'pose','0 0 10 0 0 0')
    text(light,'diffuse','.9 .9 .9 1');text(light,'direction','-.5 .1 -.9')
    def model(name,pose,kind,dimensions,color,local='0 0 0 0 0 0'):
        m=ET.SubElement(world,'model',name=name);text(m,'static','true');text(m,'pose',pose)
        link=ET.SubElement(m,'link',name='body')
        for category in ('collision','visual'):
            c=ET.SubElement(link,category,name='shape');text(c,'pose',local);g=ET.SubElement(ET.SubElement(c,'geometry'),kind)
            for k,v in dimensions.items():text(g,k,v)
            if category=='visual':
                mat=ET.SubElement(c,'material');text(mat,'ambient',color);text(mat,'diffuse',color)
        return m
    model('ground_plane','0 0 0 0 0 0','plane',{'normal':'0 0 1','size':'30 20'},'.75 .78 .8 1')
    model('rotated_box','1.4 2.8 .6 0 0 .6','box',{'size':'1.5 .5 1.2'},'.25 .45 .7 1')
    model('cylinder','1.2 -2.6 .6 0 0 0','cylinder',{'radius':'.45','length':'1.2'},'.7 .4 .2 1')
    if value['actor']:
        m=model('q03_actor','5 -2 0 0 0 1.5707963267948966','box',{'size':'.72 .72 1'},'.8 .25 .25 1','0 0 .5 0 0 0')
        plug=ET.SubElement(m,'plugin',name='bounded_actor',filename='libastra_reactive_actor_plugin.so')
        fields={'ego_model':'autolabor_m2','policy':'cautious','start':'5 -2','end':'5 2','conflict':'5 0',
                'route_origin':'0 0','route_yaw':'0','gate_route_x':'-10','gate_delay':'.5','base_speed':'.6',
                'initial_speed':'.6','response_distance':'1','response_time_margin':'1.35','seed':value['seed'],'z':'0'}
        for k,v in fields.items():text(plug,k,v)
    if value['camera']:
        m=ET.SubElement(world,'model',name='q03_camera');text(m,'static','true');text(m,'pose','2 0 14 0 1.5707963267948966 1.5707963267948966')
        link=ET.SubElement(m,'link',name='body');sensor=ET.SubElement(link,'sensor',name='overview',type='camera')
        text(sensor,'always_on','true');text(sensor,'update_rate',2)
        camera=ET.SubElement(sensor,'camera');text(camera,'horizontal_fov',1.05)
        img=ET.SubElement(camera,'image')
        for k,v in [('width',2560),('height',1440),('format','R8G8B8')]:text(img,k,v)
        clip=ET.SubElement(camera,'clip');text(clip,'near','.1');text(clip,'far',100)
        plugin=ET.SubElement(sensor,'plugin',name='q03_camera',filename='libgazebo_ros_camera.so')
        for k,v in {'robotNamespace':'/astra_q03','cameraName':'camera','imageTopicName':'image_raw','cameraInfoTopicName':'camera_info','frameName':'q03_camera_optical','updateRate':'2','alwaysOn':'true'}.items():text(plugin,k,v)
    dense=ET.SubElement(world,'plugin',name='whole_contact_evidence',filename='libastra_dense_evidence_plugin.so');text(dense,'state_hz',50)
    ET.SubElement(world,'plugin',name='passive_sweep_evidence',filename='libastra_physics_metadata_p03_plugin.so')
    return ET.tostring(root,encoding='unicode')+'\n'

def validate_robot_description(xml,value):
    """Shared offline/startup validator of the actual expanded URDF bytes."""
    validate_contract(value)
    root=ET.fromstring(xml)
    if root.tag!='robot' or root.get('name')!='autolabor_m2':raise ValueError('expanded physical robot identity mismatch')
    plugins=root.findall('.//plugin')
    required={'libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so','libastra_physical_ackermann_p05_plugin.so'}
    if len(plugins)!=4 or {p.get('filename') for p in plugins}!=required:raise ValueError('expected exact P05 driver and three sensor plugins')
    driver=next(p for p in plugins if p.get('filename')=='libastra_physical_ackermann_p05_plugin.so')
    expected={'wheelbase':.65,'track':.6,'wheelRadius':.15,'maxSpeed':2.778,'maxSteer':.4964,
      'expectedMass':79.6,'commandTimeout':.5,'publishRate':50.,'dynamicsSeed':value['seed'],
      'commandDelay':0.,'commandJitter':0.,'steeringKd':value['steering_kd'],
      'wheelKp':4.,'wheelKi':2.,'wheelIntegralTorqueLimit':1.5,'wheelDriveTorqueLimit':8.,
      'wheelBrakeTorqueLimit':12.,'steeringKp':60.,'steeringTorqueLimit':20.,
      'speedTimeConstant':.22,'steeringTimeConstant':.18,'maxAcceleration':.6,
      'maxDeceleration':.9,'maxBrakeDeceleration':.9,'maxEmergencyDeceleration':.9,'maxSteeringRate':.75}
    for key,wanted in expected.items():
        fields=driver.findall(key)
        if len(fields)!=1:raise ValueError('unique physical parameter required: '+key)
        actual=float(fields[0].text)
        if not math.isfinite(actual) or abs(actual-wanted)>1e-12:raise ValueError('physical parameter differs: '+key)
    links=root.findall('link');joints=root.findall('joint');collisions=root.findall('link/collision')
    if len(links)!=10 or len(joints)!=9 or len(collisions)!=9:raise ValueError('physical robot geometry/joint roster changed')
    if len({n.get('name') for n in links})!=len(links) or len({n.get('name') for n in joints})!=len(joints):raise ValueError('duplicate robot identity')
    return {'driver_filename':driver.get('filename'),'seed':value['seed'],'steering_kd':value['steering_kd'],
       'link_count':len(links),'joint_count':len(joints),'collision_count':len(collisions)}


def render_urdf(value):
    validate_contract(value)
    source=WORKSPACE/'src/simulation/m2_gazebo/urdf/m2_physical_p05.urdf.xacro'
    xml=subprocess.check_output(['xacro','--inorder',str(source),'dynamics_seed:='+str(value['seed']),
        'steering_kd:='+str(value['steering_kd'])],text=True,timeout=30)
    validate_robot_description(xml,value)
    return xml

def source_coverage(stamps,start,end,period=STEP_NS):
    invalid=sum(type(x)is not int or x<0 for x in stamps)
    selected=[x for x in stamps if type(x)is int and start<=x<=end]
    expected=list(range(start,end+1,period))
    return {'pass':selected==expected and not invalid,'invalid_source_stamp_count':invalid,'expected_count':len(expected),'observed_count':len(selected),
            'first_ns':selected[0] if selected else None,'last_ns':selected[-1] if selected else None,
            'missing_count':len(set(expected)-set(selected)),'duplicate_count':len(selected)-len(set(selected)),
            'source_order_increasing':all(b>a for a,b in zip(selected,selected[1:]))}

def physical_status_audit(rows,start,end):
    coverage=source_coverage([row.get('stamp_ns') for row in rows],start,end,20_000_000)
    selected=[r for r in rows if type(r.get('stamp_ns'))is int and start<=r['stamp_ns']<=end]
    issues=[];clean=True
    for row in selected:
        statuses=row.get('statuses')
        if not isinstance(statuses,list) or len(statuses)!=1:
            issues.append('exactly one physical status required');continue
        status=statuses[0];fields=status.get('values',{});keys=status.get('keys')
        expected={'backend_fault','command_timeout','brake','emergency'}
        if status.get('name')!='astra_physical_ackermann' or status.get('hardware_id')!='simulation-only':issues.append('physical status identity mismatch')
        if not isinstance(fields,dict) or set(fields)!=expected or not isinstance(keys,list) or len(keys)!=4 or set(keys)!=expected:
            issues.append('physical boolean keys absent, duplicated or unexpected');continue
        if any(type(v)is not str or v not in ('true','false') for v in fields.values()):issues.append('physical status nonboolean string');continue
        expected_level=2 if fields['backend_fault']=='true' else 1 if fields['command_timeout']=='true' else 0
        expected_message='backend_fault' if expected_level==2 else 'command_timeout' if expected_level==1 else 'ok'
        if type(status.get('level'))is not int or status['level']!=expected_level or status.get('message')!=expected_message:
            issues.append('physical status level/message inconsistent')
        if any(v!='false' for v in fields.values()):clean=False
    contract_pass=coverage['pass'] and bool(selected) and not issues
    return {'pass':contract_pass and clean,'message_contract_pass':contract_pass,
      'no_fault_timeout_brake_emergency':clean and contract_pass,'coverage':coverage,'issues':sorted(set(issues))}


def recorder_connections(rows,topics):
    connected={r[4] for r in rows if len(r)>=6 and r[2]=='i' and r[3]=='TCPROS' and r[5] is True}
    missing=sorted(set(topics)-connected)
    return {'pass':not missing,'connected':sorted(connected),'missing':missing}

def validate_services(rows,start,end):
    failures=[]
    for item in rows:
        name=item.get('name');phase=item.get('phase');stamp=item.get('source_ns')
        if name not in SERVICE_PHASES or phase not in SERVICE_PHASES.get(name,set()):failures.append(item)
        elif item.get('completed') is not True or item.get('error'):failures.append(item)
        elif type(stamp)is not int or stamp<0:failures.append(item)
        elif name in ('/gazebo/spawn_urdf_model','/gazebo/unpause_physics','/astra_q03/start') and stamp>=start:failures.append(item)
        elif name=='/gazebo/pause_physics' and stamp<end:failures.append(item)
    names=[r.get('name') for r in rows]
    required=['/gazebo/spawn_urdf_model','/gazebo/unpause_physics','/astra_q03/start','/gazebo/pause_physics']
    counts={n:names.count(n) for n in required}
    return {'pass':not failures and all(v==1 for v in counts.values()),'forbidden_calls':failures,'required_call_counts':counts}

def source_window_wall_ratio(start,end,wall_seconds,complete=False):
    """Descriptive actual source window only; absent bootstrap interval is null."""
    if complete is not True or type(start)is not int or type(end)is not int or end<=start or not math.isfinite(wall_seconds) or wall_seconds<=0:return None
    return (end-start)/1e9/wall_seconds


def elf_inventory(pid,raw_maps_path):
    """Q03 sole repair: durable raw proc maps plus classified ELF inventory."""
    from mapped_elf_inventory import inventory
    required=set(ELF_ROLES.values())-{'actor_binaries','camera'}
    return inventory(pid,ELF_ROLES,required,raw_maps_path)


def required_resources(camera=True):
    paths=[WORKSPACE/p for p in SOURCE_FILES]
    paths.extend(WORKSPACE/'devel/lib'/name for name in (
      'libastra_physical_ackermann_p05_plugin.so','libastra_physics_metadata_p03_plugin.so',
      'libastra_dense_evidence_plugin.so','libastra_reactive_actor_plugin.so'))
    paths.extend(Path('/opt/ros/noetic/lib')/n for n in ('libgazebo_ros_api_plugin.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so'))
    paths.extend(Path('/usr/lib/x86_64-linux-gnu')/n for n in ('libgazebo_ode.so.11.15.1','libgazebo_physics.so.11.15.1'))
    paths.append(Path('/usr/bin/python3.8'))
    if camera:paths.append(Path('/opt/ros/noetic/lib/libgazebo_ros_camera.so'))
    paths.extend(STAGE/'audits'/name for name in ('dense_sweep_component.json','dense_sweep_native_ode_steps.jsonl','dense_sweep_upstream_sources/manifest.json'))
    paths.extend(Path(p) for p in ('/opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch',
      '/opt/ros/noetic/lib/gazebo_ros/gzserver',
      '/usr/bin/gzserver','/opt/ros/noetic/bin/roslaunch','/opt/ros/noetic/lib/rosbag/record',
      '/opt/ros/noetic/lib/topic_tools/relay','/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher'))
    return paths
