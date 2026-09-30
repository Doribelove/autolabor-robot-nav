"""B02 pure reference-point adapter; no ROS, planner, launch or config changes.

M2 rear planning origin is (-0.325,0,0) relative to base_link, with identical
orientation. Retain actual rear lateral slip. SE(2) helpers reuse the existing
axle adapter; full rigid pose/twist functions preserve tilt and 6x6 covariance.
Pose covariance supports ONLY parent-axis small-rotation error coordinates,
not arbitrary Euler-angle derivatives. No covariance/no-slip/safety claim is
inferred from an all-zero covariance in the original physical odometry.
"""
from dataclasses import dataclass,replace
import hashlib
import json
import math
import numpy as np
from axle_reference_adapter import (base_pose_to_rear,rear_pose_to_base,
    base_twist_to_rear,rear_twist_to_base,base_local_point_to_rear)
from geometry import Pose3
from route_world_adapter import wrap_angle

SCHEMA='astra_baseline_rear_reference_b02_v1'
WHEELBASE_M=.65
OFFSET_BODY=(-.325,0.,0.)
REAR_FRAME='rear_axle_plan_b02'
POSE_COVARIANCE_CONVENTION='parent_axes_position_and_fixed_axes_small_rotation'
TWIST_COVARIANCE_CONVENTION='child_axes_linear_and_angular_velocity'


def vector(values,n):
    values=tuple(float(v) for v in values)
    if len(values)!=n or not all(math.isfinite(v) for v in values):raise ValueError('finite correctly sized vector required')
    return values


def skew(v):
    x,y,z=vector(v,3)
    return np.asarray([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def covariance(value,n):
    c=np.asarray(value,dtype=float)
    if c.shape==(n*n,):c=c.reshape(n,n)
    if c.shape!=(n,n) or not np.all(np.isfinite(c)):raise ValueError('finite covariance matrix required')
    scale=max(1.,float(np.max(np.abs(c))))
    if not np.allclose(c,c.T,rtol=0.,atol=1e-12*scale):raise ValueError('covariance must be symmetric')
    if float(np.linalg.eigvalsh(c).min()) < -1e-12*scale:raise ValueError('covariance must be positive semidefinite')
    return c.copy()


def _propagate(c,j):
    out=j@covariance(c,len(j))@j.T
    return tuple(map(float,out.ravel()))


def planar_pose_to_rear(pose):return base_pose_to_rear(pose,WHEELBASE_M)
def planar_pose_to_base(pose):return rear_pose_to_base(pose,WHEELBASE_M)
def planar_twist_to_rear(twist):return base_twist_to_rear(twist,WHEELBASE_M)
def planar_twist_to_base(twist):return rear_twist_to_base(twist,WHEELBASE_M)


def planar_pose_jacobian(yaw):
    yaw=vector((yaw,),1)[0];a=WHEELBASE_M/2
    return np.asarray([[1.,0.,a*math.sin(yaw)],[0.,1.,-a*math.cos(yaw)],[0.,0.,1.]])


def planar_pose_covariance_to_rear(cov,yaw):
    """First-order covariance of [parent x,parent y,yaw]; includes cross terms."""
    return _propagate(cov,planar_pose_jacobian(yaw))


def planar_twist_covariance_to_rear(cov):
    """Exact linear covariance of [body vx,body vy,wz] at the new point."""
    return _propagate(cov,np.asarray([[1.,0.,0.],[0.,1.,-.325],[0.,0.,1.]]))


def rigid_pose_to_rear(pose):
    if not isinstance(pose,Pose3):raise TypeError('typed Pose3 required')
    return Pose3(pose.transform(OFFSET_BODY),pose.xyzw)


def rigid_pose_to_base(pose):
    if not isinstance(pose,Pose3):raise TypeError('typed Pose3 required')
    return Pose3(pose.transform(tuple(-v for v in OFFSET_BODY)),pose.xyzw)


def rigid_twist_jacobian(to_rear=True):
    r=np.asarray(OFFSET_BODY)*(1. if to_rear else -1.)
    j=np.eye(6);j[:3,3:]=-skew(r)
    return j


def rigid_twist_to_rear(twist_linear_angular):
    return tuple(map(float,rigid_twist_jacobian()@vector(twist_linear_angular,6)))


def rigid_twist_to_base(twist_linear_angular):
    return tuple(map(float,rigid_twist_jacobian(False)@vector(twist_linear_angular,6)))


def rigid_twist_covariance_to_rear(cov,convention):
    if convention!=TWIST_COVARIANCE_CONVENTION:raise ValueError('unsupported twist covariance coordinate convention')
    return _propagate(cov,rigid_twist_jacobian())


def rigid_pose_jacobian(pose):
    if not isinstance(pose,Pose3):raise TypeError('typed Pose3 required')
    j=np.eye(6);j[:3,3:]=-skew(pose.rotate(OFFSET_BODY))
    return j


def rigid_pose_covariance_to_rear(cov,pose,convention):
    """First-order push-forward, fixed-axis infinitesimal parent rotation error.

    It is NOT exact propagation of a finite/nonlinear orientation distribution.
    Euler-RPY parameter covariances with another convention must be converted
    explicitly before calling; unsupported conventions are rejected.
    """
    if convention!=POSE_COVARIANCE_CONVENTION:raise ValueError('unsupported pose covariance coordinate convention')
    return _propagate(cov,rigid_pose_jacobian(pose))


def footprint_to_rear(vertices):
    """Translate every existing 2D vertex; no hull, padding, rotation or omission."""
    points=tuple(base_local_point_to_rear(vertex,WHEELBASE_M) for vertex in vertices)
    if len(points)<3:raise ValueError('full footprint needs at least three vertices')
    return points


def local_points3_to_rear(vertices):
    """Represent the exact same 3D component/sensor points in the rear frame."""
    return tuple(tuple(v-r for v,r in zip(vector(p,3),OFFSET_BODY)) for p in vertices)


@dataclass(frozen=True)
class StampedOdometry:
    source_ns:int
    received_ns:int
    available_ns:int
    parent_frame:str
    child_frame:str
    pose:Pose3
    body_twist:tuple
    pose_covariance:tuple
    twist_covariance:tuple
    pose_covariance_convention:str=POSE_COVARIANCE_CONVENTION
    twist_covariance_convention:str=TWIST_COVARIANCE_CONVENTION

    def __post_init__(self):
        for field in ('source_ns','received_ns','available_ns'):
            n=getattr(self,field)
            if type(n)is not int or n<0:raise ValueError('nonnegative integer source/receipt/availability times required')
        # A callback may see source 2ms ahead of local /clock; receipt is retained,
        # but availability must wait until the source is actually observable.
        if self.available_ns<max(self.source_ns,self.received_ns):raise ValueError('available time predates source or receipt')
        if not isinstance(self.pose,Pose3):raise TypeError('typed full rigid pose required')
        for frame in (self.parent_frame,self.child_frame):
            if not isinstance(frame,str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):raise ValueError('canonical frame name required')
        if self.parent_frame==self.child_frame:raise ValueError('distinct parent/child frames required')
        object.__setattr__(self,'body_twist',vector(self.body_twist,6))
        for name in ('pose_covariance','twist_covariance'):
            object.__setattr__(self,name,tuple(map(float,covariance(getattr(self,name),6).ravel())))
        if self.pose_covariance_convention!=POSE_COVARIANCE_CONVENTION or self.twist_covariance_convention!=TWIST_COVARIANCE_CONVENTION:raise ValueError('unsupported covariance coordinates')


def adapt_odometry(sample,now_ns):
    if not isinstance(sample,StampedOdometry) or sample.child_frame!='base_link':raise ValueError('base_link stamped odometry required')
    if type(now_ns)is not int or now_ns<sample.available_ns:raise ValueError('cannot use odometry before actual availability')
    # Pure conversion does not refresh time or certify freshness; caller must
    # apply its unchanged input-health age/TF gate to sample.source_ns.
    return replace(sample,child_frame=REAR_FRAME,pose=rigid_pose_to_rear(sample.pose),
        body_twist=rigid_twist_to_rear(sample.body_twist),
        pose_covariance=rigid_pose_covariance_to_rear(sample.pose_covariance,sample.pose,sample.pose_covariance_convention),
        twist_covariance=rigid_twist_covariance_to_rear(sample.twist_covariance,sample.twist_covariance_convention))


def static_tf_spec():
    return {'parent':'base_link','child':REAR_FRAME,'translation_xyz':list(OFFSET_BODY),
            'rotation_xyzw':[0.,0.,0.,1.],'exclusive_owner_required':True,
            'publish_second_odom_to_rear_transform':False}


def _goal_equal(actual,expected,tolerance=1e-6):
    return math.dist(actual[:2],expected[:2])<=tolerance and abs(wrap_angle(actual[2]-expected[2]))<=tolerance


def convert_fixed_route(base_pose_xyyaw,base_goal_xyyaw):
    """One-to-one reference conversion of supplied oriented guide poses.

    Do NOT reconstruct headings from shifted XY: each orientation is inherited,
    including the provider's explicit final goal yaw. Kinks are preserved and
    can induce rear-position jumps; this is not new planning or smoothing.
    """
    base=tuple(vector(p,3) for p in base_pose_xyyaw);goal=vector(base_goal_xyyaw,3)
    if len(base)<2 or not _goal_equal(base[-1],goal):raise ValueError('fixed base route terminal pose must match original base goal')
    rear=tuple(planar_pose_to_rear(p) for p in base)
    canonical=lambda x:hashlib.sha256(json.dumps(x,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    return {'schema':SCHEMA,'base_poses':base,'rear_poses':rear,'base_goal':goal,
            'rear_goal':planar_pose_to_rear(goal),'base_oriented_path_sha256':canonical(base),
            'rear_oriented_path_sha256':canonical(rear),'same_oriented_physical_configurations':True,
            'replanning_or_smoothing':False,'ackermann_feasibility_qualified':False}


def get_plan_request_to_base(start_rear,goal_rear,expected_base_goal):
    """Feed the existing provider its ORIGINAL base reference request.

    A future ROS shim retains headers/stamps, validates/map-transforms frames,
    invokes the base provider, then converts every returned oriented pose.
    The C++ fixed-route endpoint check must compare rear plan to rear goal.
    """
    start=planar_pose_to_base(start_rear);goal=planar_pose_to_base(goal_rear)
    expected=vector(expected_base_goal,3)
    if not _goal_equal(goal,expected):raise ValueError('goal_changed_no_implicit_replan')
    return start,goal


def base_goal_reached_from_rear(rear_pose,base_goal,xy_tolerance,yaw_tolerance):
    """Component equivalent base predicate; shared coordinator still owns it."""
    pose=planar_pose_to_base(rear_pose);goal=vector(base_goal,3)
    xy,yaw=vector((xy_tolerance,yaw_tolerance),2)
    if xy<0 or not 0<=yaw<=math.pi:raise ValueError('invalid goal tolerances')
    return math.dist(pose[:2],goal[:2])<=xy and abs(wrap_angle(pose[2]-goal[2]))<=yaw


def rear_goal_ball_base_error_bound(rear_xy_error,yaw_error):
    """Triangle upper bound, not a command to change any current tolerance."""
    xy,yaw=vector((rear_xy_error,yaw_error),2)
    if xy<0 or not 0<=yaw<=math.pi:raise ValueError('nonnegative bounded errors required')
    return xy+WHEELBASE_M*math.sin(yaw/2)
