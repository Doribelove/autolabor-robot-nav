#!/usr/bin/env python3
"""P02 isolated command source; integer simulation schedule, no dense/raw I/O."""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlparse
import xml.etree.ElementTree as ET
import yaml

PERIOD_NS = 50_000_000
START_LEAD_NS = 100_000_000
TOTAL_NS = 36_000_000_000
LOSS_START_NS = 32_000_000_000
# Exact P01 schedule; this successor changes measurement/scheduling only.
FROZEN_PHASES = (
    ('stationary',2,0.,'center',0.), ('accelerate',4,1.2,'center',0.),
    ('turn_positive',3,1.2,'center',.45), ('turn_negative',3,1.2,'center',-.45),
    ('saturation_positive',3,1.2,'center',.65), ('saturation_negative',3,1.2,'center',-.65),
    ('zero_speed_yaw',3,0.,'yaw',.6), ('reaccelerate_brake',4,1.2,'center',0.),
    ('commanded_brake',3,0.,'center',0.), ('reaccelerate_loss',4,1.2,'center',0.),
    ('command_loss',4,None,'loss',None))


def seconds_ns(value):
    if isinstance(value,bool):raise ValueError('boolean duration')
    try:value=Decimal(str(value))*1_000_000_000
    except InvalidOperation as exc:raise ValueError('invalid duration') from exc
    if not value.is_finite() or value!=value.to_integral_value() or value<0:
        raise ValueError('duration must be exact nonnegative integer nanoseconds')
    return int(value)


def require_ns(value):
    if type(value) is not int or value<0 or value>=2**63:raise ValueError('invalid integer source nanoseconds')
    return value


def compile_phases(contract):
    if seconds_ns(contract['publish_period_s'])!=PERIOD_NS or seconds_ns(contract['total_schedule_s'])!=TOTAL_NS:
        raise ValueError('P01 cadence/duration must remain frozen')
    L=float(contract['effective_plugin_parameters']['wheelbase'])
    if L!=.65:raise ValueError('frozen wheelbase changed')
    phases=[];start=0
    if len(contract['phase_schedule'])!=len(FROZEN_PHASES):raise ValueError('phase count changed')
    for row,(name,seconds,speed,mode,request) in zip(contract['phase_schedule'],FROZEN_PHASES):
        expected={'name':name,'duration_s':float(seconds)}
        if mode=='loss':expected['publish']=False
        else:expected.update(speed_mps=speed,**{('yaw_rate_request_radps' if mode=='yaw' else 'center_steering_request_rad'):request})
        # Strict field set prevents a hidden phase modifier; numeric identity is
        # semantic (2 and2.0 represent the same exact duration).
        if row!=expected or any(isinstance(row.get(k),bool) for k in expected if k!='publish'):
            raise ValueError('frozen phase changed '+name)
        end=start+seconds_ns(row['duration_s'])
        command=None if mode=='loss' else (speed,request if mode=='yaw' else speed*math.tan(request)/L)
        phases.append((name,start,end,command));start=end
    if start!=TOTAL_NS:raise ValueError('total interval mismatch')
    return tuple(phases)


@dataclass(frozen=True)
class Command:
    phase: str
    v: float
    w: float
    source_ns: int
    scheduled_ns: int
    missed_slots: int = 0


class IntegerSchedule:
    """Pure state machine. Late polls skip old slots, never replay old phases."""
    def __init__(self,contract):
        self.phases=compile_phases(contract)
        self.state='bootstrap';self.start_ns=None;self.last_ns=None
        self.last_slot=-1;self.loss_seen=False;self.start_called=False;self.failure=None

    def start(self,now_ns):
        if self.start_called:raise RuntimeError('start may only be requested once')
        self.start_called=True
        require_ns(now_ns)
        if now_ns<=0 or self.state!='bootstrap':raise ValueError('positive progressing simulation clock required')
        require_ns(now_ns+START_LEAD_NS+TOTAL_NS)
        self.start_ns=now_ns+START_LEAD_NS;self.last_ns=now_ns;self.state='running'
        return self.start_ns

    def loss_active(self,now_ns):
        if self.start_ns is None:return False
        elapsed=now_ns-self.start_ns
        # Remember entry across a clock regression so a fault cannot inject a
        # zero into a loss interval by pretending it is an earlier phase.
        return LOSS_START_NS<=elapsed<TOTAL_NS or (self.loss_seen and elapsed<TOTAL_NS)

    def fault(self,reason,now_ns):
        if self.state=='fault':return None
        allowed=not self.loss_active(now_ns)
        self.state='fault';self.failure=str(reason)
        return Command('fault_stop_zero',0.,0.,now_ns,now_ns) if allowed else None

    def poll(self,now_ns):
        require_ns(now_ns)
        if self.state!='running':return None
        if now_ns<self.last_ns:raise RuntimeError('simulation clock regressed')
        self.last_ns=now_ns
        elapsed=now_ns-self.start_ns
        if elapsed<0:return None
        if elapsed>=TOTAL_NS:
            self.state='complete'
            return Command('terminal_zero',0.,0.,now_ns,self.start_ns+TOTAL_NS)
        if elapsed>=LOSS_START_NS:self.loss_seen=True
        slot=elapsed//PERIOD_NS
        if slot<=self.last_slot:return None
        missed=max(0,slot-self.last_slot-1);self.last_slot=slot
        phase=next(row for row in self.phases if row[1]<=elapsed<row[2])
        if phase[3] is None:return None
        return Command(phase[0],*phase[3],now_ns,self.start_ns+slot*PERIOD_NS,missed)

    def prepare_send(self,command,now_ns):
        require_ns(now_ns)
        special=command.phase in ('bootstrap_zero','fault_stop_zero','terminal_zero')
        if command.phase not in ('bootstrap_zero','fault_stop_zero') and now_ns<self.last_ns:
            raise RuntimeError('simulation clock regressed before command send')
        if self.loss_active(now_ns):
            self.loss_seen=True
            return None
        if not special:
            elapsed=now_ns-self.start_ns
            phase=next((r for r in self.phases if r[1]<=elapsed<r[2]),None)
            if phase is None or phase[3] is None:return None
            actual_slot=elapsed//PERIOD_NS
            additional_missed=max(0,actual_slot-self.last_slot)
            self.last_slot=max(self.last_slot,actual_slot);self.last_ns=now_ns
            return Command(phase[0],*phase[3],now_ns,self.start_ns+actual_slot*PERIOD_NS,
                           command.missed_slots+additional_missed)
        if command.phase=='terminal_zero' and now_ns<self.start_ns+TOTAL_NS:return None
        return Command(command.phase,command.v,command.w,now_ns,command.scheduled_ns,command.missed_slots)


class SourceClockGuard:
    def __init__(self,stall_ns):
        self.stall_ns=require_ns(stall_ns);self.last_ns=None;self.last_progress_wall_ns=None

    def reset(self,now_ns,wall_ns):
        self.last_ns=require_ns(now_ns);self.last_progress_wall_ns=require_ns(wall_ns)

    def observe(self,now_ns,wall_ns):
        require_ns(now_ns);require_ns(wall_ns)
        if self.last_ns is None:raise RuntimeError('clock guard was not started')
        if now_ns<self.last_ns:raise RuntimeError('simulation clock regressed')
        if now_ns>self.last_ns:self.last_progress_wall_ns=wall_ns
        if wall_ns-self.last_progress_wall_ns>self.stall_ns:raise RuntimeError('simulation clock stalled')
        self.last_ns=now_ns


def load_contract(path,expected_sha,seed,run_id):
    if not isinstance(expected_sha,str) or re.fullmatch('[0-9a-f]{64}',expected_sha) is None:
        raise ValueError('explicit exact contract SHA256 required')
    if type(seed) is not int or seed<=0:raise ValueError('explicit positive seed required')
    if not isinstance(run_id,str) or re.fullmatch('[A-Za-z0-9_.-]+',run_id) is None:
        raise ValueError('invalid explicit run_id')
    payload=Path(path).read_bytes()
    if hashlib.sha256(payload).hexdigest()!=expected_sha:raise ValueError('contract SHA mismatch')
    contract=yaml.safe_load(payload)
    if contract.get('stage')!='astra_m12_physics_p06' or contract.get('run_id')!=run_id or type(contract.get('seed')) is not int or contract['seed']!=seed:
        raise ValueError('exact P02 stage/run_id/seed mismatch')
    for key in ('navigation_performance_evidence','vehicle_operation_authorized','training_authorized','retry_allowed'):
        if contract.get(key) is not False:raise ValueError('forbidden contract scope '+key)
    if contract.get('maximum_episodes')!=1 or contract.get('command_topic')!='/cmd_vel_navigation' or contract.get('relay_output_topic')!='/cmd_vel':
        raise ValueError('fixture capacity/command chain mismatch')
    compile_phases(contract)
    for key in ('readiness_wall_s','wall_cap_s','clock_stall_wall_s'):
        if not math.isfinite(float(contract[key])) or float(contract[key])<=0:raise ValueError('invalid '+key)
    return contract


def validate_runtime_environment(master_uri,sim_time,sim_only,description,contract,seed):
    uri=urlparse(master_uri)
    if uri.scheme!='http' or uri.hostname not in ('127.0.0.1','localhost','::1') or uri.username or uri.password:
        raise RuntimeError('isolated loopback ROS master required')
    if sim_time is not True or sim_only is not True:raise RuntimeError('simulation markers required')
    root=ET.fromstring(description)
    drivers=root.findall("gazebo/plugin[@filename='libastra_physical_ackermann_p05_plugin.so']")
    if len(drivers)!=1:raise ValueError('unique P02 physical plugin required')
    plugin=drivers[0]
    if int(plugin.findtext('dynamicsSeed'))!=seed:raise ValueError('effective driver seed mismatch')
    for key,value in contract['effective_plugin_parameters'].items():
        actual=float(plugin.findtext(key))
        if not math.isfinite(actual) or abs(actual-float(value))>1e-10:raise ValueError('effective plugin parameter mismatch '+key)
    for key,value in contract['effective_boolean_parameters'].items():
        if plugin.findtext(key,'').lower()!=str(value).lower():raise ValueError('effective boolean mismatch '+key)


def runtime():
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int64
    from std_srvs.srv import Trigger,TriggerResponse
    from gazebo_msgs.srv import GetPhysicsProperties
    rospy.init_node('actuation_command_source')
    seed=rospy.get_param('~seed');run_id=rospy.get_param('~run_id');contract_sha=rospy.get_param('~contract_sha256')
    contract=load_contract(rospy.get_param('~contract_path'),contract_sha,seed,run_id)
    validate_runtime_environment(os.environ.get('ROS_MASTER_URI',''),rospy.get_param('/use_sim_time',False),
        rospy.get_param('/m2_gazebo/simulation_only',False),rospy.get_param('/robot_description'),contract,seed)
    rospy.wait_for_service('/gazebo/get_physics_properties',timeout=contract['readiness_wall_s'])
    if not rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)().pause:
        raise RuntimeError('command source must be created during paused bootstrap')
    schedule=IntegerSchedule(contract);lock=threading.RLock();failure_request=[]
    command_pub=rospy.Publisher('/cmd_vel_navigation',Twist,queue_size=1)
    event_pub=rospy.Publisher('/astra_fixture/command_event',DiagnosticArray,queue_size=1000)
    status_pub=rospy.Publisher('/astra_fixture/command_source_status',DiagnosticArray,queue_size=1000,latch=True)
    start_pub=rospy.Publisher('/astra_fixture/schedule_start_ns',Int64,queue_size=1,latch=True)
    created_wall=time.monotonic_ns();next_prime_wall=created_wall
    clock_guard=SourceClockGuard(seconds_ns(contract['clock_stall_wall_s']));last_status=''

    def diagnostics(name,level,message,values,stamp):
        array=DiagnosticArray();array.header.stamp=stamp
        row=DiagnosticStatus();row.name=name;row.hardware_id='simulation-only';row.level=level;row.message=message
        row.values=[KeyValue(key=str(k),value=str(v)) for k,v in values.items()];array.status=[row]
        return array

    def publish_status(reason=''):
        nonlocal last_status
        state=schedule.state
        values={'state':state,'run_id':run_id,'seed':seed,'contract_sha256':contract_sha,
                'reason':reason,'start_ns':schedule.start_ns if schedule.start_ns is not None else ''}
        status_pub.publish(diagnostics('astra_actuation_command_source',DiagnosticStatus.ERROR if state=='fault' else DiagnosticStatus.OK,
            reason or state,values,rospy.Time.now()))
        last_status=state

    def send(command):
        # Stamp at the actual send, not at an intended/scheduled slot.
        source=rospy.Time.now();now_ns=source.to_nsec()
        # Re-evaluate at send time, including a crossed phase/loss boundary.
        command=schedule.prepare_send(command,now_ns)
        if command is None:return
        msg=Twist();msg.linear.x=command.v;msg.angular.z=command.w
        wall=time.monotonic();command_pub.publish(msg)
        event_pub.publish(diagnostics('astra_fixture_command',DiagnosticStatus.OK,command.phase,
            {'phase':command.phase,'v':repr(command.v),'w':repr(command.w),'wall_monotonic_s':repr(wall),
             'scheduled_ns':command.scheduled_ns,'lateness_ns':now_ns-command.scheduled_ns,'missed_slots':command.missed_slots},source))

    def unique_publishers():
        code,message,system=rospy.get_master().getSystemState()
        if code!=1:raise RuntimeError('master publisher query failed')
        pubs=dict(system[0])
        if pubs.get('/cmd_vel_navigation')!=[rospy.get_name()] or len(pubs.get('/cmd_vel',[]))!=1:
            raise RuntimeError('command publisher uniqueness failed')

    def start(_request):
        with lock:
            if schedule.start_called:return TriggerResponse(False,json.dumps({'error':'start already requested'}))
            try:
                unique_publishers()
                now_ns=rospy.Time.now().to_nsec();schedule.start(now_ns)
                clock_guard.reset(now_ns,time.monotonic_ns())
                start_pub.publish(Int64(data=schedule.start_ns));publish_status()
                return TriggerResponse(True,json.dumps({'start_ns':schedule.start_ns,'run_id':run_id,'seed':seed,'contract_sha256':contract_sha},sort_keys=True))
            except Exception as exc:
                schedule.start_called=True
                command=schedule.fault(str(exc),rospy.Time.now().to_nsec())
                if command is not None:send(command)
                publish_status(str(exc));return TriggerResponse(False,json.dumps({'error':str(exc)}))

    start_service=rospy.Service('/astra_fixture/start',Trigger,start)
    publish_status()

    def watch_publishers():
        # Keep XMLRPC and its possible wall delay off the command-sending loop.
        while not rospy.is_shutdown():
            time.sleep(.5)
            with lock:active=schedule.state=='running'
            if not active:continue
            try:unique_publishers()
            except Exception as exc:
                with lock:failure_request.append(str(exc))
    threading.Thread(target=watch_publishers,daemon=True).start()
    while not rospy.is_shutdown():
        with lock:
            # Read after taking the same lock as the start callback, avoiding
            # a stale pre-start sample being diagnosed as clock regression.
            now_ns=rospy.Time.now().to_nsec();wall=time.monotonic_ns()
            try:
                if schedule.state=='bootstrap':
                    if wall-created_wall>seconds_ns(contract['readiness_wall_s']):raise RuntimeError('bootstrap wall deadline')
                    if wall>=next_prime_wall:
                        send(Command('bootstrap_zero',0.,0.,now_ns,now_ns))
                        next_prime_wall+=((wall-next_prime_wall)//PERIOD_NS+1)*PERIOD_NS
                elif schedule.state=='running':
                    if failure_request:raise RuntimeError(failure_request[0])
                    if wall-created_wall>seconds_ns(contract['wall_cap_s']):raise RuntimeError('command source wall cap')
                    clock_guard.observe(now_ns,wall)
                    command=schedule.poll(now_ns)
                    if command is not None:send(command)
                    if last_status!=schedule.state:publish_status()
            except Exception as exc:
                command=schedule.fault(str(exc),now_ns)
                if command is not None:send(command)
                publish_status(str(exc));rospy.logerr('P02 command source fault: %s',exc)
        # Complete/fault intentionally stay alive until the recorder/probe ends.
        time.sleep(.001)
    return 0


if __name__=='__main__':
    raise SystemExit(runtime())
