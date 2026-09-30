"""Pure live input bridge: exact timestamp scan TF + causal real odometry.

Uses the frozen real-scan bucket implementation; no extra TF lookup, no padding,
no odometry interpolation, and no simulator truth. Missing causal odometry may
wait only within the original scan's 200ms lifetime.
"""
from route_sensor_buffer import RouteSensorBuffer

class AwaitingCausalOdometry(ValueError):pass

class TrackSensorBuffer(RouteSensorBuffer):
    def __init__(self,core):
        super().__init__();self.core=core;self.accepted_scan_count=0

    def scan(self,stamp_ns,tf_stamp_ns,ranges,angle_min,angle_increment,range_min,range_max,laser_pose_map,now_ns,laser_quaternion_xyzw=None):
        candidates=[stamp for stamp in self.odometry if stamp<=stamp_ns and stamp_ns-stamp<=self.MAX_AGE_NS]
        if not candidates:raise AwaitingCausalOdometry('exactTF scan waiting for at-or-before actual odometry')
        odom_stamp=max(candidates);pose,twist,roll_pitch=self.odometry[odom_stamp]
        # Tracker receipt canonicalizes pose numerics to float. Normalize the
        # retained real frame too, so integer-valued fixture poses hash identically.
        frame=super().scan(stamp_ns,tf_stamp_ns,ranges,angle_min,angle_increment,range_min,range_max,tuple(float(v) for v in laser_pose_map),now_ns,laser_quaternion_xyzw)
        self.core.accept_scan(frame,tf_stamp_ns,dict(stamp_ns=odom_stamp,pose_map=pose,body_twist=twist),now_ns)
        self.accepted_scan_count+=1
        return frame

    def health(self,now_ns):
        failures=super().health(now_ns)
        failures.extend(self.core.tracker.health(now_ns))
        snapshot=self.core.tracker.snapshot
        if self.fresh is not None and (snapshot is None or snapshot.source_ns!=self.fresh.stamp_ns):failures.append('fresh_scan_tracker_commit_mismatch')
        return failures


class SharedLifecycle:
    """Method-local lifecycle. Only shared status may assert a task outcome."""
    TERMINAL=('complete','timeout','stopped','fault')
    def __init__(self,run_id,seed,method):
        self.identity=(run_id,seed,method)
        self.state=dict(external_gate=None,shared_terminal=None,phase='bootstrap',fault=None,late_faults=[],late_fault_count=0,
            activation_wait_reason=None,map_ready=False,route_ready=False,start_ns=None,started_once=False,
            selected_action=0,next_decision_ns=None,next_control_ns=None,
            bootstrap_zero_stamp_discarded={'scan':0,'odom':0,'joint':0})

    def check_identity(self,row):
        if row.get('schema')!='astra_common_navigation_v1' or (row.get('run_id'),row.get('seed'),row.get('method'))!=self.identity:
            raise ValueError('shared navigation identity mismatch')
        if type(row.get('seed'))is not int:raise ValueError('integer shared seed required')

    def fail(self,reason):
        s=self.state
        if s['shared_terminal'] is not None or s['phase'] in self.TERMINAL:
            s['late_fault_count']+=1;s['late_faults']=(s['late_faults']+[str(reason)])[-64:]
            return
        s['fault']=str(reason);s['phase']='fault';s['selected_action']=0

    def gate(self,row):
        self.check_identity(row)
        if type(row.get('source_ns'))is not int or row['source_ns']<=0:raise ValueError('integer shared gate source required')
        if type(row.get('deadline_ns'))is not int or row['deadline_ns']-row['source_ns']!=22_000_000_000:raise ValueError('fixed22s shared deadline required')
        old=self.state['external_gate']
        if old is not None and old!=row:raise ValueError('shared gate changed')
        self.state['external_gate']=dict(row)

    def status(self,row):
        self.check_identity(row)
        if row.get('phase') not in self.TERMINAL:return False
        stamp=row.get('terminal_source_ns')
        if type(stamp)is not int or stamp<0 or not row.get('terminal_time_authority'):raise ValueError('shared terminal source authority missing')
        terminal={k:row.get(k) for k in ('phase','terminal_source_ns','terminal_reason','terminal_time_authority')}
        if self.state['shared_terminal'] is not None:
            if self.state['shared_terminal']!=terminal:self.fail('changed_shared_terminal_ignored')
            return False
        self.state['shared_terminal']=terminal
        # Local sensing faults are retained separately, never a substituted outcome.
        self.state['phase']=terminal['phase'];self.state['selected_action']=0
        return True

    def stop(self):
        if self.state['shared_terminal'] is None and self.state['phase'] not in self.TERMINAL:self.state['phase']='stopped'
        self.state['selected_action']=0

    def may_activate(self,now_ns):
        s=self.state;gate=s['external_gate']
        return s['phase']=='bootstrap' and not s['started_once'] and gate is not None and gate['source_ns']<=now_ns<gate['deadline_ns']
