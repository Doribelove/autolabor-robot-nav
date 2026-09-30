"""B03 sensor-time/route integration core, no ROS import or live authority."""
from collections import deque
from dataclasses import replace
import math
import numpy as np
from baseline_rear_axle_adapter_b02 import (Pose3,StampedOdometry,adapt_odometry,
    rigid_pose_to_base,planar_pose_to_rear,convert_fixed_route,covariance,vector,
    rear_goal_ball_base_error_bound,POSE_COVARIANCE_CONVENTION,TWIST_COVARIANCE_CONVENTION)
from route_world_adapter import wrap_angle

SCHEMA='astra_baseline_rear_runtime_b03_v1'
METHODS=('actual_teb_rear_reference_b03','dwa_rear_reference_b03')
FRAME='rear_axle_kinematic'
AGE_NS=200_000_000
TILT_LIMIT_RAD=.05
INTERNAL_XY_M=.15
INTERNAL_YAW_RAD=.10
BASE_XY_M=.25
BASE_YAW_RAD=.25
ROW_KEYS={'source_ns','parent_frame','child_frame','position','quaternion','linear','angular','pose_covariance','twist_covariance'}


def quaternion_pose(position,quaternion):
    xyz=vector(position,3);q=vector(quaternion,4);norm2=sum(v*v for v in q)
    if abs(norm2-1.)>1e-6:raise ValueError('raw quaternion must be unit before Pose3 normalization')
    pose=Pose3(xyz,q);r=np.asarray(pose.rotation)
    pitch=math.asin(max(-1.,min(1.,-r[2,0])));roll=math.atan2(r[2,1],r[2,2]);yaw=math.atan2(r[1,0],r[0,0])
    if abs(roll)+abs(pitch)>TILT_LIMIT_RAD:raise ValueError('pose exceeds declared .05rad tilt envelope')
    return pose,(roll,pitch,yaw),norm2


class RearOdometryStream:
    def __init__(self):
        self.pending=deque();self.last_clock=None;self.last_ingress=None;self.latest=None;self.fault_reason=None;self.accepted=0
    def fault(self,reason):
        if self.fault_reason is None:self.fault_reason=str(reason)
        self.pending.clear()
    def _clock(self,now):
        if self.fault_reason:raise ValueError('permanent_fault:'+self.fault_reason)
        if type(now)is not int or now<0:raise ValueError('integer nonnegative ROS clock required')
        if self.last_clock is not None and now<self.last_clock:raise ValueError('ROS clock regression')
        self.last_clock=now
    def ingest(self,row,now):
        try:
            self._clock(now)
            if not isinstance(row,dict) or set(row)!=ROW_KEYS:raise ValueError('exact actual odometry fields required')
            stamp=row['source_ns']
            if type(stamp)is not int or stamp<0:raise ValueError('integer actual source stamp required')
            if row['parent_frame']!='odom' or row['child_frame']!='base_link':raise ValueError('actual odom/base frames required')
            if self.last_ingress is not None and stamp<=self.last_ingress:raise ValueError('duplicate or regressed source stamp')
            if abs(stamp-now)>AGE_NS:raise ValueError('odom source outside200ms past/future transport bound')
            if len(self.pending)>=1000:raise ValueError('pending source-clock queue overflow')
            pose,rpy,norm2=quaternion_pose(row['position'],row['quaternion'])
            twist=vector(tuple(row['linear'])+tuple(row['angular']),6)
            pc=tuple(covariance(row['pose_covariance'],6).ravel());tc=tuple(covariance(row['twist_covariance'],6).ravel())
            self.last_ingress=stamp
            self.pending.append((stamp,now,pose,twist,pc,tc,{'raw_quaternion':list(row['quaternion']),'raw_quaternion_norm_squared':norm2,'roll_pitch_yaw':list(rpy)}))
        except Exception as exc:self.fault(str(exc));raise
    def drain(self,now):
        try:
            self._clock(now);output=[]
            while self.pending and self.pending[0][0]<=now:
                stamp,receipt,pose,twist,pc,tc,audit=self.pending.popleft()
                if now-stamp>AGE_NS:raise ValueError('queued actual odometry became stale')
                sample=StampedOdometry(stamp,receipt,now,'odom','base_link',pose,twist,pc,tc)
                rear=replace(adapt_odometry(sample,now),child_frame=FRAME)
                self.latest=rear;self.accepted+=1;output.append((rear,audit))
            self.health(now)
            return output
        except Exception as exc:self.fault(str(exc));raise
    def health(self,now):
        try:
            self._clock(now)
            if self.latest is not None and not 0<=now-self.latest.source_ns<=AGE_NS:raise ValueError('actual rear odometry stale/future')
            return self.latest is not None and self.latest.source_ns>0
        except Exception as exc:self.fault(str(exc));raise
    def check_publication(self,sample,now):
        try:
            self._clock(now)
            if now<sample.available_ns or not 0<=now-sample.source_ns<=AGE_NS:raise ValueError('rear output expired before publication')
        except Exception as exc:self.fault(str(exc));raise


def goal_tolerance_contract():
    tilt_translation=.325*(1-math.cos(TILT_LIMIT_RAD))
    bound=rear_goal_ball_base_error_bound(INTERNAL_XY_M,INTERNAL_YAW_RAD)+tilt_translation
    assert bound<BASE_XY_M and INTERNAL_YAW_RAD<BASE_YAW_RAD
    return {'internal_xy_m':INTERNAL_XY_M,'internal_yaw_rad':INTERNAL_YAW_RAD,
        'shared_base_xy_m':BASE_XY_M,'shared_base_yaw_rad':BASE_YAW_RAD,
        'maximum_base_xy_at_internal_goal_m':bound,'tilt_projection_pad_m':tilt_translation,
        'dwa_latch_xy_goal_tolerance':False,'actual_base_terminal_authority_unchanged':True,
        'strict_callback_wall_order_guaranteed':False,
        'scope':'internal current-pose goal set is strictly inside shared base goal set; result callback never controls shared terminal'}


class RearFixedRoute:
    """Reference conversion of the exact shared provider orientation convention."""
    def __init__(self,route,data,maximum_start_distance_m=2.):
        if data['frame_id']!='map' or data.get('provider_returns_exact_full_polyline')is not True:raise ValueError('exact common map-frame fixed route required')
        if maximum_start_distance_m!=2.:raise ValueError('original common start-distance guard must remain2m')
        self.route=route;self.data=data;self.maximum_start_distance_m=maximum_start_distance_m
        base=[(*p,route.goal_yaw if i==len(route.points)-1 else route.sample(route.arcs[i])[2]) for i,p in enumerate(route.points)]
        self.converted=convert_fixed_route(base,data['goal_xy_yaw'])
    def request(self,start_frame,start_position,start_quaternion,goal_frame,goal_position,goal_quaternion):
        if start_frame!='map' or goal_frame!='map':raise ValueError('request frame must be map')
        start,_,_=quaternion_pose(start_position,start_quaternion);goal,rpy,_=quaternion_pose(goal_position,goal_quaternion)
        if abs(rpy[0])+abs(rpy[1])>1e-9 or abs(goal.xyz[2])>1e-9:raise ValueError('fixed planar goal required')
        base_start=rigid_pose_to_base(start);base_goal=rigid_pose_to_base(goal);expected=self.data['goal_xy_yaw']
        if self.route.project(base_start.xyz[:2])['distance_m']>self.maximum_start_distance_m:raise ValueError('start_too_far_from_fixed_route')
        if math.dist(base_goal.xyz[:2],expected[:2])>1e-6 or abs(wrap_angle(rpy[2]-expected[2]))>1e-6:raise ValueError('goal_changed_no_implicit_replan')
        # Immutable same full sequence for every request; no start insertion,
        # no online replanning, clipping or method-dependent pruning.
        return self.converted['rear_poses']


class RearAdapterHealth:
    """Coordinator-side adapter status queue, retaining future source stamps."""
    def __init__(self,run_id,seed,method):
        self.identity={'run_id':run_id,'seed':seed,'method':method};self.pending=deque();self.latest=None;self.last_received=None
    def accept(self,row,now):
        if any(type(row.get(k))is not type(v) or row[k]!=v for k,v in self.identity.items()) or row.get('schema')!=SCHEMA:raise ValueError('rear adapter status identity mismatch')
        if row.get('state') not in ('bootstrap','ready','fault') or type(row.get('ready'))is not bool:raise ValueError('rear adapter status state malformed')
        if row['state']=='fault':raise ValueError('rear adapter fault:'+str(row.get('reason')))
        stamp=row.get('status_source_ns');odom=row.get('latest_odom_source_ns')
        if type(stamp)is not int or stamp<0 or stamp-now>AGE_NS or (self.last_received is not None and stamp<self.last_received):raise ValueError('rear adapter status source regression/future')
        if odom is not None and (type(odom)is not int or odom<0 or odom>stamp):raise ValueError('rear adapter status odom source malformed')
        if row['ready'] and (row['state']!='ready' or odom is None or odom<=0):raise ValueError('rear ready without actual source')
        if len(self.pending)>=1000:raise ValueError('rear status queue overflow')
        self.last_received=stamp;self.pending.append(dict(row))
    def ready(self,now):
        while self.pending and self.pending[0]['status_source_ns']<=now:self.latest=self.pending.popleft()
        r=self.latest
        return bool(r and r['ready'] and 0<=now-r['status_source_ns']<=AGE_NS and 0<=now-r['latest_odom_source_ns']<=AGE_NS)
