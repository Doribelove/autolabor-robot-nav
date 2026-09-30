"""Source-math independent B02 checks; synthetic, no physics or ROS."""
from pathlib import Path
from dataclasses import replace
import math,sys,json
import numpy as np
import pytest,yaml
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import baseline_rear_axle_adapter_b02 as b

def matrix(axis,angle):
    k=np.zeros((3,3));e=np.eye(3)[axis];k[:,0]=np.cross(e,[1,0,0]);k[:,1]=np.cross(e,[0,1,0]);k[:,2]=np.cross(e,[0,0,1]);return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)
def world2(p,points):
    c,s=math.cos(p[2]),math.sin(p[2]);return np.asarray([(p[0]+c*x-s*y,p[1]+s*x+c*y) for x,y in points])
def sample():return b.StampedOdometry(1_002_000_000,1_000_000_000,1_005_000_000,'odom','base_link',b.Pose3.from_xyz_rpy((1,2,.1),(.03,-.05,.7)),(1.2,.21,.03,.1,-.2,.6),(0.,)*36,(0.,)*36)

@pytest.mark.parametrize('rpy',[(0,0,0),(.1,.05,.7),(-.1,-.05,-1.2),(.03,0,2.3)])
def test_full_rigid_points_and_world_twist_crossproduct(rpy):
    p=b.Pose3.from_xyz_rpy((2.,-3.,.6),rpy);r=b.rigid_pose_to_rear(p);R=np.asarray(p.rotation);off=np.array([-.325,0,0.])
    np.testing.assert_allclose(r.xyz,np.array(p.xyz)+R@off,atol=1e-15,rtol=0)
    pts=((.4,.2,.8),(-.7,-.3,.15),(0,0,1.))
    for source,target in zip(pts,b.local_points3_to_rear(pts)):
        np.testing.assert_allclose(p.transform(source),r.transform(target),atol=1e-14,rtol=0)
    v=np.array([1.2,.2,.03]);w=np.array([.1,-.2,.6]);actual=b.rigid_twist_to_rear((*v,*w))
    np.testing.assert_allclose(R@actual[:3],R@v+np.cross(R@w,R@off),atol=1e-14,rtol=0)
    np.testing.assert_allclose(actual[3:],w,atol=0,rtol=0)
    np.testing.assert_allclose(b.rigid_twist_to_base(actual),(*v,*w),atol=1e-15,rtol=0)

@pytest.mark.parametrize('pitch',[0.,.05,-.05])
def test_actual_16_64_64_polygons_planar_equality_tilt_residual_not_erased(pitch):
    data=yaml.safe_load((W/'src/simulation/m2_gazebo/config/astra_navigation_common_footprint.yaml').read_text());polys=[data['TebLocalPlannerROS']['footprint_model']['vertices'],data['local_costmap']['footprint'],data['global_costmap']['footprint']]
    assert list(map(len,polys))==[16,64,64]
    p=b.Pose3.from_xyz_rpy((2,-3,.1),(.04,pitch,.8));rear=b.rigid_pose_to_rear(p)
    for poly in polys:
        after=world2((*rear.xyz[:2],.8),b.footprint_to_rear(poly));before=world2((2,-3,.8),poly)
        delta=after-before;expected=.325*(1-math.cos(pitch))*np.array([math.cos(.8),math.sin(.8)])
        np.testing.assert_allclose(delta,np.tile(expected,(len(poly),1)),atol=1e-14,rtol=0)
        if pitch:assert np.max(np.linalg.norm(delta,axis=1))>.0004

def test_parent_axis_pose_jacobian_finite_difference_independent_rotation_matrices():
    p=sample().pose;R=np.asarray(p.rotation);off=np.array([-.325,0,0.]);h=1e-6;J=np.eye(6)
    for axis in range(3):J[:3,axis+3]=((matrix(axis,h)@R@off)-(matrix(axis,-h)@R@off))/(2*h)
    np.testing.assert_allclose(b.rigid_pose_jacobian(p),J,atol=1e-10,rtol=0)
    G=np.arange(36,dtype=float).reshape(6,6)/37+np.eye(6);C=G@G.T;expected=(J@G)@(J@G).T
    actual=np.array(b.rigid_pose_covariance_to_rear(C,p,b.POSE_COVARIANCE_CONVENTION)).reshape(6,6)
    np.testing.assert_allclose(actual,expected,atol=1e-9,rtol=0)
    assert abs(actual[0,5])>0 and abs(actual[1,3])>0

def test_correlated_twist_covariance_independent_linear_noise_columns():
    G=np.array([[1,2,0,1,0,2],[0,1,1,0,3,0],[1,0,2,1,0,0],[.2,.3,0,.4,0,0],[0,.2,.1,0,.5,0],[.1,0,.4,0,0,.7]])
    C=G@G.T;off=np.array([-.325,0,0.]);H=G.copy()
    for i in range(6):H[:3,i]=G[:3,i]+np.cross(G[3:,i],off)
    got=np.array(b.rigid_twist_covariance_to_rear(C,b.TWIST_COVARIANCE_CONVENTION)).reshape(6,6)
    np.testing.assert_allclose(got,H@H.T,atol=1e-14,rtol=0)

@pytest.mark.parametrize('kind',['unknown_pose_convention','wrong_twist_axes','negative_eigenvalue','asymmetric','nonfinite'])
def test_covariance_failure_modes(kind):
    C=np.eye(6)
    with pytest.raises(ValueError):
        if kind=='unknown_pose_convention':b.rigid_pose_covariance_to_rear(C,sample().pose,'Euler_RPY')
        elif kind=='wrong_twist_axes':b.rigid_twist_covariance_to_rear(C,'parent_axes')
        elif kind=='negative_eigenvalue':C[0,0]=-.01;b.rigid_twist_covariance_to_rear(C,b.TWIST_COVARIANCE_CONVENTION)
        elif kind=='asymmetric':C[0,1]=.1;b.rigid_twist_covariance_to_rear(C,b.TWIST_COVARIANCE_CONVENTION)
        else:C[0,0]=float('inf');b.rigid_twist_covariance_to_rear(C,b.TWIST_COVARIANCE_CONVENTION)

def test_availability_not_source_retime_and_stale_caller_obligation():
    s=sample()
    with pytest.raises(ValueError):b.adapt_odometry(s,s.available_ns-1)
    r=b.adapt_odometry(s,s.available_ns);assert (r.source_ns,r.received_ns,r.available_ns)==(s.source_ns,s.received_ns,s.available_ns)
    assert r.parent_frame=='odom' and r.child_frame==b.REAR_FRAME and s.child_frame=='base_link'
    assert r.body_twist[1]==pytest.approx(.015) # true residual remains
    assert r.pose_covariance==(0.,)*36 and r.twist_covariance==(0.,)*36
    stale=b.adapt_odometry(s,s.source_ns+10_000_000_000);assert stale.source_ns==s.source_ns # pure conversion deliberately not healthgate
    with pytest.raises(ValueError):b.adapt_odometry(replace(s,child_frame=b.REAR_FRAME),s.available_ns)

def test_oriented_route_roundtrip_terminal_yaw_and_kink_preserved():
    original=((-4.,0.,.6),(0.,1.,.4),(0.,1.,-.4),(5.,0.,.7));goal=original[-1];out=b.convert_fixed_route(original,goal)
    recovered=[b.planar_pose_to_base(p) for p in out['rear_poses']];np.testing.assert_allclose(recovered,original,atol=1e-14,rtol=0)
    assert out['rear_poses'][-1]==out['rear_goal'] and out['rear_poses'][-1][2]==.7
    assert math.dist(out['rear_poses'][1][:2],out['rear_poses'][2][:2])>.25 # orientation kink entails point jump, not smoothed
    assert out['ackermann_feasibility_qualified'] is False
    start,target=b.get_plan_request_to_base(out['rear_poses'][0],out['rear_goal'],goal);np.testing.assert_allclose(target,goal,atol=1e-14,rtol=0)
    with pytest.raises(ValueError):b.get_plan_request_to_base(out['rear_poses'][0],goal,goal)

def test_goal_tolerance_counterexample_and_exact_base_predicate():
    angle=.2;shift=.325*np.array([math.cos(angle)-1,math.sin(angle)]);goal=np.array([5.,0.]);rear_goal=goal-np.array([.325,0]);rear=tuple(rear_goal+.24*shift/np.linalg.norm(shift))+(angle,)
    assert math.dist(rear[:2],rear_goal)<.25 and abs(angle)<=.2
    base=b.planar_pose_to_base(rear);error=math.dist(base[:2],goal)
    assert error==pytest.approx(.3048917208204383) and error>.25
    assert not b.base_goal_reached_from_rear(rear,(5.,0.,0.),.25,.25)
    assert error==pytest.approx(b.rear_goal_ball_base_error_bound(.24,.2))
