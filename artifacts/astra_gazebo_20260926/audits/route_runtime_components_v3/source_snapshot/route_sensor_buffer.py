"""Integer-time sensor input for route policies; no ROS or simulator truth.

Every scan with its own timestamp TF updates the fresh shield cloud. Separately,
six 200 ms model buckets select real scans at-or-before each bucket deadline.
There are no invented warmup rows and no float timestamp subtraction.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def positive_ns(value):
    if type(value)is not int or value<=0 or value>=2**63:raise ValueError('positive integer source ns required')
    return value


@dataclass(frozen=True)
class ScanFrame:
    stamp_ns:int
    model_ranges:object
    points_map:object
    laser_pose_map:tuple


class RouteSensorBuffer:
    PERIOD_NS=200_000_000
    MAX_AGE_NS=200_000_000
    def __init__(self):
        self.fresh=None;self.history=deque(maxlen=6);self.frames=deque(maxlen=32)
        self.next_bucket_ns=None;self.last_now_ns=None;self.fault=None
        self.odometry={};self.joints={};self.state=None;self.fresh_count=0;self.history_count=0

    def observe_time(self,now_ns):
        positive_ns(now_ns)
        if self.last_now_ns is not None and now_ns<self.last_now_ns:self.fault='clock_regression_requires_new_episode'
        self.last_now_ns=now_ns
        if self.fault:raise RuntimeError(self.fault)

    def scan(self,stamp_ns,tf_stamp_ns,ranges,angle_min,angle_increment,range_min,range_max,laser_pose_map,now_ns,laser_quaternion_xyzw=None):
        self.observe_time(now_ns);positive_ns(stamp_ns)
        if stamp_ns!=tf_stamp_ns:raise ValueError('scan requires its exact timestamp TF')
        if stamp_ns>now_ns or now_ns-stamp_ns>self.MAX_AGE_NS:raise ValueError('future or stale scan')
        if self.fresh is not None and stamp_ns<=self.fresh.stamp_ns:raise ValueError('scan replay/out-of-order')
        values=np.asarray(ranges,dtype=float)
        if len(laser_pose_map)!=3 or values.ndim!=1 or len(values)<2 or not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min,range_max,*laser_pose_map)) or angle_increment<=0 or not 0<range_min<range_max:
            raise ValueError('invalid scan geometry')
        if np.any(np.isnan(values)) or np.any(np.isneginf(values)):raise ValueError('invalid scan return')
        # +inf is an actual no-return; invalid low readings never become free.
        if np.any(np.isfinite(values)&(values<range_min)):raise ValueError('scan below declared minimum')
        valid=np.isfinite(values)&(values<range_max)
        yaw=laser_pose_map[2];angles=angle_min+np.arange(len(values))*angle_increment
        if laser_quaternion_xyzw is None:
            points=np.column_stack((laser_pose_map[0]+values[valid]*np.cos(angles[valid]+yaw),laser_pose_map[1]+values[valid]*np.sin(angles[valid]+yaw)))
        else:
            from geometry import Pose3
            q=tuple(laser_quaternion_xyzw)
            if len(q)!=4 or not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1.)>1e-5:raise ValueError('normalized exact-time laser quaternion required')
            rotation=np.asarray(Pose3(xyzw=q).rotation)
            local=np.column_stack((values[valid]*np.cos(angles[valid]),values[valid]*np.sin(angles[valid]),np.zeros(np.sum(valid))))
            points=(local@rotation.T)[:,:2]+np.asarray(laser_pose_map[:2])
        beams=np.linspace(-math.pi,math.pi,72,endpoint=False)
        indices=np.rint((beams-angle_min)/angle_increment).astype(int)
        if np.any(indices<0) or np.any(indices>=len(values)):raise ValueError('new schema requires full360deg scan coverage')
        model=np.minimum(values[indices],min(range_max,8.)).astype(np.float32)
        frame=ScanFrame(stamp_ns,model,points,tuple(laser_pose_map));self.frames.append(frame);self.fresh=frame;self.fresh_count+=1
        if self.next_bucket_ns is None:self.next_bucket_ns=stamp_ns
        while self.next_bucket_ns<=stamp_ns:
            candidates=[row for row in self.frames if row.stamp_ns<=self.next_bucket_ns]
            selected=candidates[-1] if candidates else None
            if selected is None or self.next_bucket_ns-selected.stamp_ns>self.MAX_AGE_NS or (self.history and selected.stamp_ns<=self.history[-1][1].stamp_ns):
                self.history.clear()
            else:
                self.history.append((self.next_bucket_ns,selected));self.history_count+=1
            self.next_bucket_ns+=self.PERIOD_NS
        return frame

    def odom(self,stamp_ns,pose_map,body_twist,roll_pitch=(0.,0.)):
        positive_ns(stamp_ns)
        values=tuple(pose_map)+tuple(body_twist)+tuple(roll_pitch)
        if len(values)!=8 or not all(math.isfinite(v) for v in values):raise ValueError('invalid actual odometry')
        if stamp_ns in self.odometry:raise ValueError('duplicate odometry source stamp')
        self.odometry[stamp_ns]=(tuple(pose_map),tuple(body_twist),tuple(roll_pitch));self._join(stamp_ns)

    def joint(self,stamp_ns,names,positions):
        positive_ns(stamp_ns)
        if len(names)!=len(positions) or len(set(names))!=len(names) or not all(math.isfinite(v) for v in positions):raise ValueError('invalid actual joint state')
        row=dict(zip(names,positions))
        if not {'front_left_steer_joint','front_right_steer_joint'}<=set(row):raise ValueError('actual steering joints absent')
        if stamp_ns in self.joints:raise ValueError('duplicate joints source stamp')
        self.joints[stamp_ns]=row;self._join(stamp_ns)

    def _join(self,stamp_ns):
        if stamp_ns in self.odometry and stamp_ns in self.joints:
            if self.state is None or stamp_ns>self.state['stamp_ns']:
                pose,twist,roll_pitch=self.odometry[stamp_ns]
                self.state={'stamp_ns':stamp_ns,'pose_map':pose,'body_twist':twist,'roll_pitch':roll_pitch,'joints':self.joints[stamp_ns]}
        for collection in (self.odometry,self.joints):
            for key in sorted(collection)[:-32]:del collection[key]

    def health(self,now_ns):
        self.observe_time(now_ns);failures=[]
        for key,stamp in (('fresh_scan',None if self.fresh is None else self.fresh.stamp_ns),('actual_state',None if self.state is None else self.state['stamp_ns'])):
            if stamp is None or not 0<=now_ns-stamp<=self.MAX_AGE_NS:failures.append(key+'_missing_future_or_stale')
        if len(self.history)!=6:failures.append('six_real_model_history_rows_not_ready')
        elif now_ns-self.history[-1][0]>self.PERIOD_NS+self.MAX_AGE_NS:failures.append('model_history_stale')
        return failures

    def model_history(self):
        if len(self.history)!=6:raise ValueError('no fabricated warmup history permitted')
        return np.stack([row.model_ranges for _,row in self.history]),[bucket for bucket,_ in self.history],[row.stamp_ns for _,row in self.history]
