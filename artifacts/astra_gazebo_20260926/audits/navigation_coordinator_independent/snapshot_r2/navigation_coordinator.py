"""Pure shared navigation activation/command ownership protocol, no ROS.

Gate time is the actual Odometry header of the first eligible crossing, not
Twist receipt or coordinator wall/ROS callback time. Headerless method commands
retain receipt-only semantics. Inactive method wrappers prohibit moving output
before the published gate and must acknowledge its exact identity.
"""
from dataclasses import dataclass
import math
from route_world_adapter import RouteFrame,wrap_angle

SCHEMA='astra_common_navigation_v1'
PERIOD_NS=50_000_000
ACTIVATION_NS=5_000_000_000
TERMINAL_PHASES=('fault','complete','timeout','stopped')
METHODS=('astra_analytic_route_v1','astra_pct_route_v1','frozen_r2_v060_shared_activation','actual_teb_common_route','dwa_common_route')

@dataclass(frozen=True)
class CoordinatorConfig:
    run_id:str
    seed:int
    method:str
    start_pose:tuple=(-4.,0.,0.)
    goal_pose:tuple=(5.,0.,0.)
    spawn_pose:tuple=(-5.4,0.,0.)
    warmup_speed_mps:float=.8
    timeout_ns:int=22_000_000_000
    maximum_source_age_ns:int=200_000_000
    maximum_command_receipt_age_ns:int=150_000_000
    activation_ack_timeout_ns:int=1_000_000_000
    def __post_init__(self):
        if not self.run_id or type(self.seed)is not int or self.seed<=0 or self.method not in METHODS:raise ValueError('explicit shared navigation identity required')
        if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in (self.start_pose,self.goal_pose,self.spawn_pose)):raise ValueError('finite SE2 references required')
        if self.warmup_speed_mps!=.8 or self.timeout_ns!=22_000_000_000:raise ValueError('shared .8m/s warmup and22s timeout frozen')
        for n in (self.timeout_ns,self.maximum_source_age_ns,self.maximum_command_receipt_age_ns,self.activation_ack_timeout_ns):
            if type(n)is not int or n<=0:raise ValueError('integer time configuration required')
        if self.method=='frozen_r2_v060_shared_activation' and (self.start_pose,self.goal_pose,self.spawn_pose)!=((-4.,0.,0.),(5.,0.,0.),(-5.4,0.,0.)):
            raise ValueError('frozen R2 supports only legacy x route; new fullroute input is a different method')


class NavigationCoordinator:
    def __init__(self,config):
        self.config=config;self.route_frame=RouteFrame(*config.start_pose);self.phase='bootstrap';self.fault_reason=None
        self.armed=False;self.ready=False;self.latest_odom=None;self.gate=None;self.last_clock_ns=None;self.activation_source_ns=None
        self.method_ack=None;self.command=None;self.command_sequence=0;self.command_floor=0;self.trace_sequence=0;self.terminal_source_ns=None;self.terminal_reason=None;self.terminal_time_authority=None
        self.late_faults=[];self.late_fault_count=0
    def terminal(self,phase,reason,stamp,authority):
        if self.phase in TERMINAL_PHASES:return
        self.phase=phase;self.command=None;self.terminal_source_ns=stamp;self.terminal_reason=reason;self.terminal_time_authority=authority
    def fault(self,reason):
        if self.phase in TERMINAL_PHASES:
            self.late_fault_count+=1
            row={'reason':str(reason),'actual_odom_source_ns':None if self.latest_odom is None else self.latest_odom['stamp_ns'],
                 'publisher_visible_clock_ns':self.last_clock_ns,'count':1}
            if self.late_faults and self.late_faults[-1]['reason']==row['reason']:
                row['count']=self.late_faults[-1]['count']+1;self.late_faults[-1]=row
            else:self.late_faults.append(row);self.late_faults=self.late_faults[-64:]
            return
        self.fault_reason=str(reason)
        self.terminal('fault',self.fault_reason,0 if self.latest_odom is None else self.latest_odom['stamp_ns'],'latest_actual_odom_header_at_fault_detection_not_fault_generation_stamp')
    def request_stop(self):
        if self.phase not in ('fault','complete','timeout','stopped'):
            self.terminal('stopped','external_stop',0 if self.latest_odom is None else self.latest_odom['stamp_ns'],'latest_actual_odom_header_at_stop_request')
    def clock(self,now_ns):
        if type(now_ns)is not int or now_ns<0:raise ValueError('nonnegative integer publisher clock required')
        if self.last_clock_ns is not None and now_ns<self.last_clock_ns:self.fault('publisher_clock_regression')
        self.last_clock_ns=now_ns
    def method_status(self,row):
        if row.get('run_id')!=self.config.run_id or row.get('seed')!=self.config.seed or row.get('method')!=self.config.method:
            self.fault('method_identity_mismatch');return
        if row.get('state')=='fault':self.fault('method_fault:'+str(row.get('reason')));return
        self.ready=row.get('ready')is True
        if row.get('state')=='active':
            if self.gate is None or row.get('gate_source_ns')!=self.gate['source_ns']:
                self.fault('method_ack_wrong_or_absent_gate');return
            if self.phase=='activating':
                self.method_ack=dict(row);self.command_floor=self.command_sequence;self.command=None;self.phase='active'
    def arm(self,now_ns):
        self.clock(now_ns)
        if self.armed or self.phase!='bootstrap' or now_ns<=0 or now_ns>=ACTIVATION_NS:raise ValueError('one-shot arm requires positive source time before5s activation')
        if not self.ready or self.latest_odom is None or not 0<=now_ns-self.latest_odom['stamp_ns']<=self.config.maximum_source_age_ns:raise ValueError('method/actual odometry readiness absent')
        self.armed=True;return {'run_id':self.config.run_id,'seed':self.config.seed,'activation_ns':ACTIVATION_NS}
    def observe_odom(self,stamp_ns,pose,body_twist):
        if type(stamp_ns)is not int or stamp_ns<=0 or len(pose)!=3 or len(body_twist)!=3 or not all(math.isfinite(v) for v in tuple(pose)+tuple(body_twist)):
            self.fault('invalid_actual_odometry');return None
        if self.latest_odom is not None and stamp_ns<=self.latest_odom['stamp_ns']:
            self.fault('actual_odometry_source_replay_or_regression');return None
        self.latest_odom={'stamp_ns':stamp_ns,'pose_map':tuple(pose),'body_twist':tuple(body_twist)}
        longitudinal,lateral,yaw=self.route_frame.pose_to_route(pose)
        if self.phase=='warmup':
            if stamp_ns-self.activation_source_ns>12_000_000_000:self.fault('warmup_timeout')
            elif longitudinal>=0:
                if longitudinal>.03 or abs(lateral)>.03 or abs(yaw)>.03 or abs(body_twist[0]-.8)>.04:
                    self.fault('shared_gate_state_outside_frozen_tolerances')
                else:
                    self.gate={'schema':SCHEMA,'run_id':self.config.run_id,'seed':self.config.seed,'method':self.config.method,'source_ns':stamp_ns,
                        'source':'actual_odom_header_first_crossing','pose_map':list(pose),'body_twist':list(body_twist),'activation_source_ns':self.activation_source_ns,
                        'goal_pose':list(self.config.goal_pose),'deadline_source_ns':stamp_ns+self.config.timeout_ns,'deadline_ns':stamp_ns+self.config.timeout_ns}
                    self.phase='activating';self.command=None;self.command_floor=self.command_sequence;return dict(self.gate)
        if self.phase in ('active','activating') and self.gate:
            if stamp_ns>=self.gate['deadline_source_ns']:self.terminal('timeout','source_deadline',self.gate['deadline_source_ns'],'declared_gate_actual_odom_source_plus22s')
            elif math.dist(pose[:2],self.config.goal_pose[:2])<=.25 and abs(wrap_angle(pose[2]-self.config.goal_pose[2]))<=.25:self.terminal('complete','actual_base_goal',stamp_ns,'first_actual_odom_header_meeting_base_goal')
        return None
    def method_command(self,v,w,receipt_ros_ns,receipt_wall_ns):
        if type(receipt_ros_ns)is not int or receipt_ros_ns<0 or type(receipt_wall_ns)is not int or receipt_wall_ns<=0 or not all(math.isfinite(x) for x in (v,w)):
            self.fault('nonfinite_or_unstamped_command_receipt');return
        self.command_sequence+=1
        if self.gate is None and (abs(v)>1e-12 or abs(w)>1e-12):self.fault('moving_method_command_before_gate');return
        if self.phase=='active' and self.command_sequence>self.command_floor:
            self.command={'v':float(v),'w':float(w),'receipt_ros_ns':receipt_ros_ns,'receipt_wall_ns':receipt_wall_ns,'sequence':self.command_sequence,'generation_source_ns':None}
    def output(self,now_ns):
        self.clock(now_ns);self.trace_sequence+=1;v=w=0.;reason=self.phase
        if self.phase not in ('fault','complete','timeout','stopped') and now_ns>=ACTIVATION_NS:
            if not self.armed:self.fault('not_armed_before_common_activation')
            elif self.latest_odom is None or not 0<=now_ns-self.latest_odom['stamp_ns']<=self.config.maximum_source_age_ns:self.fault('actual_odometry_stale_or_future')
            elif self.phase=='bootstrap' and self.latest_odom['stamp_ns']>=ACTIVATION_NS:
                self.phase='warmup';self.activation_source_ns=self.latest_odom['stamp_ns'];reason='shared_warmup'
        if self.phase=='warmup':v=.8;reason='shared_warmup'
        elif self.phase=='activating':
            reason='gate_waiting_method_activation_ack'
            if now_ns-self.gate['source_ns']>self.config.activation_ack_timeout_ns:self.fault('method_activation_ack_timeout')
        elif self.phase=='active':
            # Stop output at the declared deadline, but decide timeout only
            # after ordered actual odometry reaches its source watermark.
            # An earlier goal packet may still be in transport.
            if now_ns>=self.gate['deadline_source_ns']:reason='deadline_zero_waiting_ordered_actual_odometry'
            elif self.command is None:reason='waiting_fresh_post_ack_method_command'
            elif not 0<=now_ns-self.command['receipt_ros_ns']<=self.config.maximum_command_receipt_age_ns:reason='stale_or_future_method_receipt_zero'
            else:v,w=self.command['v'],self.command['w'];reason='method_passthrough'
        if self.phase in ('fault','complete','timeout','stopped'):v=w=0.;reason=self.fault_reason if self.phase=='fault' else self.phase
        return {'schema':SCHEMA,'sequence':self.trace_sequence,'run_id':self.config.run_id,'seed':self.config.seed,'method':self.config.method,
            'publisher_visible_clock_ns':now_ns,'actual_odom_source_ns':None if self.latest_odom is None else self.latest_odom['stamp_ns'],
            'gate_source_ns':None if self.gate is None else self.gate['source_ns'],'phase':self.phase,'terminal_source_ns':self.terminal_source_ns,'terminal_reason':self.terminal_reason,'terminal_time_authority':self.terminal_time_authority,'v':v,'w':w,'reason':reason,
            'selected_method_receipt':None if self.command is None else dict(self.command),'method_command_generation_stamp_known':False,
            'late_fault_count':self.late_fault_count,'late_faults':[dict(row) for row in self.late_faults]}
