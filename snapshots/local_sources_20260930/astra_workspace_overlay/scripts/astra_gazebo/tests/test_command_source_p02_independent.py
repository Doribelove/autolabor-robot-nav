"""Independent offline runtime fixtures with fake transports; no ROS imports/init."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import random
import sys
import types
import pytest
import yaml

S=Path(__file__).resolve().parents[4]
P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_actuation_command_source_p02.py'
C=S/'contracts/m12_physics_p02.yaml'
SPEC=importlib.util.spec_from_file_location('p02_independent_command',P)
m=importlib.util.module_from_spec(SPEC);sys.modules[SPEC.name]=m
exec(compile(P.read_bytes(),str(P),'exec'),vars(m))
CONTRACT=yaml.safe_load(C.read_text())
SHA=hashlib.sha256(C.read_bytes()).hexdigest()
SOURCE_SHA=hashlib.sha256(P.read_bytes()).hexdigest()
RESULTS={}

class Box:
    def __init__(self,**kwargs):self.__dict__.update(kwargs)
class Stamp:
    def __init__(self,ns):self.ns=ns
    def to_nsec(self):return self.ns
class Array:
    def __init__(self):self.header=Box(stamp=Stamp(0));self.status=[]
class Status:
    OK=0;ERROR=2
class Twist:
    def __init__(self):self.linear=Box(x=0.);self.angular=Box(z=0.)
class TriggerResponse:
    def __init__(self,success,message):self.success=success;self.message=message

class FakeRuntime:
    def __init__(self,monkeypatch,mode='normal'):
        self.mode=mode;self.clock=0;self.wall=0;self.pub=[];self.events=[];self.commands=[];self.status=[];self.start_stamps=[];self.callbacks={};self.responses=[];self.start=None;self.shutdown=False;self.state_seen_wall=None;self.action=False;self.paused=mode!='not_paused';self.logs=[];self.threads=[]
        self.params={'~seed':CONTRACT['seed'],'~run_id':CONTRACT['run_id'],'~contract_sha256':SHA,'~contract_path':str(C),'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'/robot_description':self.xml()}
        runtime=types.ModuleType('rospy')
        runtime.init_node=lambda name: self.logs.append(('init',name))
        runtime.get_param=lambda key,default=None:self.params.get(key,default)
        runtime.wait_for_service=lambda *a,**kw:self.logs.append(('wait_service',a[0]))
        runtime.ServiceProxy=lambda *a,**kw:lambda:Box(pause=self.paused)
        runtime.Publisher=self.publisher
        runtime.Service=self.service
        runtime.Time=Box(now=lambda:Stamp(self.clock))
        runtime.get_name=lambda:'/actuation_command_source'
        runtime.get_master=lambda:Box(getSystemState=self.master)
        runtime.is_shutdown=lambda:self.shutdown
        runtime.logerr=lambda *a:self.logs.append(('error',a))
        modules={'rospy':runtime,
                 'diagnostic_msgs.msg':Box(DiagnosticArray=Array,DiagnosticStatus=Status,KeyValue=Box),
                 'geometry_msgs.msg':Box(Twist=Twist),'std_msgs.msg':Box(Int64=Box),
                 'std_srvs.srv':Box(Trigger=object,TriggerResponse=TriggerResponse),
                 'gazebo_msgs.srv':Box(GetPhysicsProperties=object)}
        for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
        monkeypatch.setattr(m,'time',Box(monotonic_ns=lambda:self.wall,monotonic=lambda:self.wall/1e9,sleep=self.sleep))
        monkeypatch.setattr(m.threading,'Thread',lambda target,daemon:Box(start=lambda:self.threads.append(target)))
        monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820')

    def xml(self):
        params=''.join('<{0}>{1}</{0}>'.format(k,v) for k,v in CONTRACT['effective_plugin_parameters'].items())
        params+=''.join('<{0}>{1}</{0}>'.format(k,str(v).lower()) for k,v in CONTRACT['effective_boolean_parameters'].items())
        return '<robot><gazebo><plugin filename="libastra_physical_ackermann_p02_plugin.so"><dynamicsSeed>'+str(CONTRACT['seed'])+'</dynamicsSeed>'+params+'</plugin></gazebo></robot>'
    def master(self):
        pubs=[['/cmd_vel_navigation',['/actuation_command_source']],['/cmd_vel',['/relay']]]
        if self.mode=='duplicate_publisher':pubs[0][1].append('/intruder')
        return 1,'ok',[pubs,[],[]]
    def publisher(self,topic,kind,**kwargs):
        self.pub.append(topic);self.logs.append(('advertise',topic))
        def publish(message):
            msg=copy.deepcopy(message)
            if topic=='/cmd_vel_navigation':self.commands.append({'ns':self.clock,'v':msg.linear.x,'w':msg.angular.z,'paused':self.paused,'wall':self.wall})
            elif topic=='/astra_fixture/command_event':self.events.append(msg)
            elif topic=='/astra_fixture/command_source_status':
                data={v.key:v.value for v in msg.status[0].values};self.status.append(data)
                if data['state'] in ['complete','fault']:self.state_seen_wall=self.wall
            elif topic=='/astra_fixture/schedule_start_ns':self.start_stamps.append(msg.data)
        return Box(publish=publish)
    def service(self,name,kind,callback):
        self.callbacks[name]=callback;self.logs.append(('advertise_service',name));return Box()
    def sleep(self,seconds):
        self.wall+=int(seconds*1e9)
        if self.start is None and '/astra_fixture/start' in self.callbacks and self.wall>=60_000_000:
            self.clock=20_000_000;self.paused=False
            r=self.callbacks['/astra_fixture/start'](None);self.responses.append(r)
            # A second call must never replace an accepted or rejected identity.
            self.responses.append(self.callbacks['/astra_fixture/start'](None))
            if r.success:self.start=json.loads(r.message)['start_ns']
            else:self.start=-1
        elif self.start is not None and self.start>=0:
            elapsed=self.clock-self.start
            if self.mode=='shutdown_in_loss' and elapsed>=33_000_000_000:self.shutdown=True;return
            if self.mode=='shutdown_moving' and elapsed>=3_000_000_000:self.shutdown=True;return
            if self.mode=='stall_in_loss' and elapsed>=33_000_000_000:pass
            elif self.mode=='regress_in_loss' and elapsed>=33_000_000_000 and not self.action:
                self.clock=self.start+31_000_000_000;self.action=True
            elif self.mode=='skip_slots' and elapsed>=15_000_000_000 and not self.action:
                self.clock+=160_000_000;self.action=True
            else:self.clock+=int(seconds*1e9)
        if self.state_seen_wall is not None and self.wall-self.state_seen_wall>=10_000_000:self.shutdown=True
        if self.wall>60_000_000_000:raise AssertionError('fake runtime exceeded bounded loop')
    def run(self):
        result=m.runtime()
        key=self.mode
        RESULTS[key]={'exit_code':result,'command_count':len(self.commands),'events':len(self.events),'states':[x['state'] for x in self.status],'start':self.start,'responses':[{'success':x.success,'message':x.message} for x in self.responses],
                      'commands_in_loss':[x for x in self.commands if self.start is not None and self.start+32_000_000_000<=x['ns']<self.start+36_000_000_000],
                      'bootstrap_count':sum(x['paused'] for x in self.commands),'status_resident_wall_ns':None if self.state_seen_wall is None else self.wall-self.state_seen_wall}
        return result

@pytest.mark.parametrize('mode',['normal','shutdown_in_loss','shutdown_moving','stall_in_loss','regress_in_loss','skip_slots'])
def test_full_runtime_fake_transport_and_loss_suppression(monkeypatch,mode):
    env=FakeRuntime(monkeypatch,mode)
    assert env.run()==0
    assert env.responses[0].success and not env.responses[1].success
    identity=json.loads(env.responses[0].message)
    assert identity=={'start_ns':env.start,'run_id':CONTRACT['run_id'],'seed':CONTRACT['seed'],'contract_sha256':SHA}
    assert env.start_stamps==[env.start]
    assert not RESULTS[mode]['commands_in_loss']
    assert all(x['v']==x['w']==0 for x in env.commands if x['paused'])
    assert len(env.events)==len(env.commands)
    assert [x[0] for x in env.logs].index('advertise_service')>max(i for i,x in enumerate(env.logs) if x[0]=='advertise')
    if mode in ['normal','skip_slots']:
        assert env.status[-1]['state']=='complete'
        assert RESULTS[mode]['status_resident_wall_ns']>=10_000_000
        assert sum(e.status[0].message=='terminal_zero' for e in env.events)==1
    elif mode in ['stall_in_loss','regress_in_loss']:
        assert env.status[-1]['state']=='fault'
        assert RESULTS[mode]['status_resident_wall_ns']>=10_000_000
        assert not any(e.status[0].message=='fault_stop_zero' for e in env.events)
    else:
        # Early ROS shutdown adds no command; source status remains running.
        # The enclosing required-node/probe must reject this incomplete run.
        assert env.status[-1]['state']=='running'
        assert not any(e.status[0].message in ['terminal_zero','fault_stop_zero'] for e in env.events)

def test_runtime_refuses_unpaused_creation(monkeypatch):
    env=FakeRuntime(monkeypatch,'not_paused')
    with pytest.raises(RuntimeError,match='paused bootstrap'):env.run()
    assert env.pub==[] and not env.commands

def test_runtime_start_failure_consumes_one_shot_and_stays_fault(monkeypatch):
    env=FakeRuntime(monkeypatch,'duplicate_publisher')
    assert env.run()==0
    assert not env.responses[0].success and not env.responses[1].success
    assert env.status[-1]['state']=='fault' and env.start_stamps==[]
    assert env.commands[-1]['v']==env.commands[-1]['w']==0

def test_inclusive_ns_boundaries_randomized_delays_and_no_bursts():
    rng=random.Random(926008)
    for trial in range(100):
        s=m.IntegerSchedule(CONTRACT);start=s.start(rng.randrange(1,10**18));now=start;emitted=[]
        while now<start+37_000_000_000:
            command=s.poll(now)
            if command is not None:
                send_ns=now+rng.randrange(0,100_000_001)
                sent=s.prepare_send(command,send_ns)
                if sent is not None:emitted.append(sent)
                now=send_ns
            now+=rng.randrange(1,80_000_001)
        assert all(not start+32_000_000_000<=x.source_ns<start+36_000_000_000 for x in emitted)
        regular=[x for x in emitted if x.phase!='terminal_zero']
        assert len({x.scheduled_ns for x in regular})==len(regular)
        assert sum(x.phase=='terminal_zero' for x in emitted)<=1

def test_invalid_contract_identity_and_status_scope(tmp_path):
    for key,value in [('run_id','wrong'),('seed',True),('stage','old'),('vehicle_operation_authorized',True),('retry_allowed',True)]:
        c=copy.deepcopy(CONTRACT);c[key]=value;p=tmp_path/'c.yaml';p.write_text(yaml.safe_dump(c));h=hashlib.sha256(p.read_bytes()).hexdigest()
        with pytest.raises(ValueError):m.load_contract(p,h,CONTRACT['seed'],CONTRACT['run_id'])

def test_snapshot_not_changed():
    assert SOURCE_SHA==hashlib.sha256(P.read_bytes()).hexdigest()
    assert SHA==hashlib.sha256(C.read_bytes()).hexdigest()

def test_write_offline_evidence():
    assert RESULTS
    output=S/'audits/m12_p02_command_source_independent_runtime.json'
    output.write_text(json.dumps({'source_path':str(P),'source_sha256':SOURCE_SHA,'contract_sha256':SHA,'test_path':str(Path(__file__).resolve()),'test_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scope':'Synthetic Python runtime fixtures with fake rospy/time/transport. No ROS master or Gazebo used; not simulation evidence.','fixtures':RESULTS,'known_limit':'Early ROS shutdown returns0 with last source status running; enclosing probe must require complete status and retain shutdown failure. Command timestamp is sampled immediately before Python publish, not an atomic source stamp on Twist at driver receipt.'},indent=2)+'\n')
