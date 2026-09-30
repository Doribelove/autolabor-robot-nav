"""Small analytical/finite-difference tests; no ROS, simulator or benchmark."""
from dataclasses import replace
from pathlib import Path
import math,sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from baseline_rear_axle_adapter_b02 import *


def world2(pose,point):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return (x+c*point[0]-s*point[1],y+s*point[0]+c*point[1])


def sample():
    return StampedOdometry(100,102,104,'odom','base_link',Pose3.from_xyz_rpy((2.,3.,.1),(.01,-.02,.7)),
        (1.2,.325*.6+.04,.02,.03,-.04,.6),tuple(np.eye(6).ravel()),tuple((np.eye(6)*2).ravel()))


class ReferenceTests(unittest.TestCase):
    def test_straight_and_left_right_ideal_no_slip(self):
        for w in [0.,.6,-.6,.7805,-.7805]:
            vx,vy,w2=planar_twist_to_rear((1.2,.325*w,w))
            self.assertEqual(vx,1.2);self.assertAlmostEqual(vy,0.);self.assertEqual(w2,w)
    def test_real_rear_slip_is_not_zeroed(self):
        for slip in [-.05,.02,.1]:self.assertAlmostEqual(planar_twist_to_rear((1.2,.325*.6+slip,.6))[1],slip)
    def test_se2_inverse_rotated_and_translated(self):
        for angle in [0.,.4,math.pi/2,-2.1,2*math.pi]:
            p=(3.7,-1.9,angle);np.testing.assert_allclose(planar_pose_to_base(planar_pose_to_rear(p)),p,rtol=0,atol=1e-15)
    def test_planar_twist_inverse(self):
        x=(1.1,.21,-.78);np.testing.assert_allclose(planar_twist_to_base(planar_twist_to_rear(x)),x,atol=1e-15)
    def test_complete_footprint_world_vertices_identical(self):
        vertices=[(-.7,-.4),(-.7,.4),(.6,.4),(.8,.1),(.8,-.1),(.6,-.4)]
        transformed=footprint_to_rear(vertices);self.assertEqual(len(transformed),len(vertices))
        for yaw in [-2.,0.,.9,math.pi]:
            base=(3.,-4.,yaw);rear=planar_pose_to_rear(base)
            for b,r in zip(vertices,transformed):np.testing.assert_allclose(world2(base,b),world2(rear,r),rtol=0,atol=1e-14)
    def test_real_16_and_64_vertex_config_invariance(self):
        import yaml
        W=Path(__file__).resolve().parents[3]
        data=yaml.safe_load((W/'src/simulation/m2_gazebo/config/astra_navigation_common_footprint.yaml').read_text())
        polygons=[data['local_costmap']['footprint'],data['global_costmap']['footprint'],data['TebLocalPlannerROS']['footprint_model']['vertices']]
        self.assertEqual([len(p) for p in polygons],[64,64,16])
        base=(1.5,-2.7,.77);rear=planar_pose_to_rear(base)
        for p in polygons:
            r=footprint_to_rear(p)
            np.testing.assert_allclose([world2(base,v) for v in p],[world2(rear,v) for v in r],atol=1e-14,rtol=0)
    def test_rigid_pose_retains_tilt_and_orientation(self):
        p=sample().pose;r=rigid_pose_to_rear(p)
        self.assertEqual(r.xyzw,p.xyzw);np.testing.assert_allclose(rigid_pose_to_base(r).xyz,p.xyz,atol=1e-15)
    def test_rigid_sensor_and_component_world_points_invariant(self):
        p=sample().pose;r=rigid_pose_to_rear(p);vertices=[(0,0,.6),(.7,.4,.1),(-.7,-.4,.1)]
        transformed=local_points3_to_rear(vertices)
        np.testing.assert_allclose([p.transform(v) for v in vertices],[r.transform(v) for v in transformed],atol=1e-15)
    def test_full_twist_preserves_all_components_and_cross_product(self):
        v=np.array([1.,.2,.3]);w=np.array([.4,.5,.6]);r=np.asarray(OFFSET_BODY)
        rear=rigid_twist_to_rear((*v,*w));np.testing.assert_allclose(rear[:3],v+np.cross(w,r),atol=1e-15)
        self.assertEqual(tuple(rear[3:]),tuple(w));np.testing.assert_allclose(rigid_twist_to_base(rear),(*v,*w),atol=1e-15)
    def test_tilt_limits_exact_yaw_only_costmap_equivalence(self):
        p=Pose3.from_xyz_rpy(rpy=(0.,.05,.7));r=rigid_pose_to_rear(p)
        vertices=[(-.7,-.4),(.7,-.4),(.7,.4),(-.7,.4)]
        old_points=np.array([world2((0.,0.,.7),v) for v in vertices])
        rear_points=np.array([world2((*r.xyz[:2],.7),v) for v in footprint_to_rear(vertices)])
        residual=np.linalg.norm(rear_points-old_points,axis=1)
        np.testing.assert_allclose(residual,.325*(1-math.cos(.05)),atol=1e-15)
        self.assertGreater(residual[0],0.)  # Not silently claimed exact in tilt.
    def test_invalid_shapes_and_nonfinite_rejected(self):
        for fn,arg in [(planar_pose_to_rear,[1,2]),(planar_twist_to_rear,[1,np.nan,0]),(rigid_twist_to_rear,[0]*5),(footprint_to_rear,[(0,0)]*2)]:
            with self.subTest(fn=fn),self.assertRaises(ValueError):fn(arg)


class CovarianceTests(unittest.TestCase):
    def test_planar_pose_jacobian_independent_finite_difference(self):
        p=np.array([2.,-3.,.83]);h=1e-6
        jac=np.column_stack([(np.array(planar_pose_to_rear(p+np.eye(3)[i]*h))-np.array(planar_pose_to_rear(p-np.eye(3)[i]*h)))/(2*h) for i in range(3)])
        np.testing.assert_allclose(planar_pose_jacobian(p[2]),jac,rtol=0,atol=3e-10)
    def test_planar_pose_full_cross_covariance(self):
        a=np.array([[1,.3,.2],[.2,2,.4],[0,.1,.3]]);c=a@a.T;j=planar_pose_jacobian(.7)
        actual=np.array(planar_pose_covariance_to_rear(c,.7)).reshape(3,3)
        np.testing.assert_allclose(actual,(j@a)@(j@a).T,atol=1e-14);self.assertNotEqual(actual[0,2],c[0,2])
    def test_planar_twist_covariance_known_lateral_omega_term(self):
        c=np.diag([.04,.09,.16]);out=np.array(planar_twist_covariance_to_rear(c)).reshape(3,3)
        self.assertAlmostEqual(out[1,1],.09+.325**2*.16);self.assertAlmostEqual(out[1,2],-.325*.16)
    def test_rigid_pose_fixed_axes_jacobian_independent_finite_difference(self):
        p=sample().pose;h=1e-6;jac=np.eye(6)
        for i in range(3):
            axis=np.zeros(3);axis[i]=h
            plusq=Pose3.from_xyz_rpy(rpy=axis).compose(Pose3(xyzw=p.xyzw)).xyzw
            minusq=Pose3.from_xyz_rpy(rpy=-axis).compose(Pose3(xyzw=p.xyzw)).xyzw
            plus=rigid_pose_to_rear(Pose3(p.xyz,plusq));minus=rigid_pose_to_rear(Pose3(p.xyz,minusq))
            jac[:3,3+i]=(np.array(plus.xyz)-np.array(minus.xyz))/(2*h)
        np.testing.assert_allclose(rigid_pose_jacobian(p),jac,rtol=0,atol=3e-10)
    def test_rigid_twist_covariance_all_correlations(self):
        a=np.arange(36,dtype=float).reshape(6,6)/20+np.eye(6);c=a@a.T
        # Each column represents an independent unit random source, transformed
        # by the actual point velocity function instead of covariance routine.
        b=np.column_stack([rigid_twist_to_rear(a[:,i]) for i in range(6)])
        out=np.array(rigid_twist_covariance_to_rear(c,TWIST_COVARIANCE_CONVENTION)).reshape(6,6)
        np.testing.assert_allclose(out,b@b.T,rtol=0,atol=1e-14)
    def test_planar_slice_matches_full_covariance(self):
        indices=[0,1,5];c=np.zeros((6,6));sub=np.array([[.3,.02,.01],[.02,.4,.03],[.01,.03,.1]]);c[np.ix_(indices,indices)]=sub
        p=Pose3.from_xyz_rpy(rpy=(0,0,.7));full=np.array(rigid_pose_covariance_to_rear(c,p,POSE_COVARIANCE_CONVENTION)).reshape(6,6)
        np.testing.assert_allclose(full[np.ix_(indices,indices)],np.array(planar_pose_covariance_to_rear(sub,.7)).reshape(3,3),atol=1e-15)
    def test_nonzero_covariance_conventions_are_not_guessed(self):
        with self.assertRaises(ValueError):rigid_pose_covariance_to_rear(np.eye(6),sample().pose,'Euler_RPY_arbitrary')
        with self.assertRaises(ValueError):rigid_twist_covariance_to_rear(np.eye(6),'parent_axes')
    def test_invalid_covariance_rejected(self):
        for c in [np.full((6,6),np.nan),-np.eye(6),np.triu(np.ones((6,6))),np.eye(5)]:
            with self.subTest(c=str(c)),self.assertRaises(ValueError):rigid_twist_covariance_to_rear(c,TWIST_COVARIANCE_CONVENTION)
    def test_zero_covariance_not_fabricated_uncertainty(self):
        self.assertEqual(rigid_twist_covariance_to_rear(np.zeros((6,6)),TWIST_COVARIANCE_CONVENTION),(0.,)*36)


class TimingRouteGoalTests(unittest.TestCase):
    def test_source_receipt_availability_preserved(self):
        s=sample();r=adapt_odometry(s,104)
        self.assertEqual((r.source_ns,r.received_ns,r.available_ns),(100,102,104));self.assertEqual(r.parent_frame,s.parent_frame);self.assertEqual(r.child_frame,REAR_FRAME)
        self.assertAlmostEqual(r.body_twist[1],.04);self.assertEqual(s.child_frame,'base_link')
    def test_available_now_boundary(self):
        with self.assertRaises(ValueError):adapt_odometry(sample(),103)
        self.assertEqual(adapt_odometry(sample(),104).source_ns,100)
    def test_future_clock_callback_can_be_retained_not_early_used(self):
        s=replace(sample(),source_ns=104,received_ns=102,available_ns=104)
        with self.assertRaises(ValueError):adapt_odometry(s,102)
        self.assertEqual(adapt_odometry(s,104).received_ns,102)
    def test_bad_time_or_frame_rejected(self):
        for kw in [{'available_ns':99},{'source_ns':1.2},{'parent_frame':'/odom'},{'parent_frame':'base_link'}]:
            with self.subTest(kw=kw),self.assertRaises(ValueError):replace(sample(),**kw)
        with self.assertRaises(ValueError):adapt_odometry(replace(sample(),child_frame=REAR_FRAME),104)
    def test_stale_sample_not_retimed_or_certified(self):
        r=adapt_odometry(sample(),1_000_000_000);self.assertEqual(r.source_ns,100)
    def test_route_every_oriented_pose_and_final_goal_transformed(self):
        p=[(-4.,0.,.3),(-1.,1.,0.),(1.,1.,-.3),(5.,0.,.7)];bundle=convert_fixed_route(p,(5.,0.,.7))
        np.testing.assert_allclose([planar_pose_to_base(r) for r in bundle['rear_poses']],p,atol=1e-15)
        self.assertEqual(bundle['rear_poses'][-1],bundle['rear_goal']);self.assertNotEqual(bundle['rear_oriented_path_sha256'],bundle['base_oriented_path_sha256'])
    def test_route_xy_without_heading_rejected(self):
        with self.assertRaises(ValueError):convert_fixed_route([(-4,0),(5,0)],(5,0,0))
    def test_goal_heading_not_silently_recomputed_from_last_chord(self):
        bundle=convert_fixed_route([(-4.,0.,0.),(5.,0.,.7)],(5.,0.,.7))
        self.assertEqual(bundle['rear_poses'][-1][2],.7);self.assertNotEqual(bundle['rear_poses'][-1][1],0.)
    def test_getplan_roundtrip_and_changed_goal_rejected(self):
        base_start=(-4.,.1,.3);base_goal=(5.,0.,.7)
        start,goal=get_plan_request_to_base(planar_pose_to_rear(base_start),planar_pose_to_rear(base_goal),base_goal)
        np.testing.assert_allclose(start,base_start,atol=1e-15);np.testing.assert_allclose(goal,base_goal,atol=1e-15)
        with self.assertRaises(ValueError):get_plan_request_to_base(planar_pose_to_rear(base_start),(5.,0.,.7),base_goal)
    def test_same_radius_rear_goal_is_not_base_goal_equivalent(self):
        yaw=.2;delta=np.array([.325*(math.cos(yaw)-1),.325*math.sin(yaw)])
        rear_goal=np.array([4.675,0.]);rear_xy=rear_goal+.24*delta/np.linalg.norm(delta);rear=(*rear_xy,yaw)
        self.assertLess(math.dist(rear_xy,rear_goal),.25);self.assertLessEqual(yaw,.20)
        self.assertFalse(base_goal_reached_from_rear(rear,(5.,0.,0.),.25,.25))
        self.assertAlmostEqual(math.dist(planar_pose_to_base(rear)[:2],(5.,0.)),rear_goal_ball_base_error_bound(.24,.2))
    def test_base_goal_predicate_exact_under_reference_shift(self):
        goal=(5.,.2,.3)
        for base in [(5.,.2,.3),(4.8,.2,.3),(4.6,.2,.3),(5.,.2,.8)]:
            expected=math.dist(base[:2],goal[:2])<=.25 and abs(wrap_angle(base[2]-goal[2]))<=.25
            self.assertEqual(base_goal_reached_from_rear(planar_pose_to_rear(base),goal,.25,.25),expected)
    def test_tf_single_parent_no_second_odom_publisher(self):
        tf=static_tf_spec();self.assertEqual((tf['parent'],tf['child']),('base_link',REAR_FRAME));self.assertFalse(tf['publish_second_odom_to_rear_transform'])

if __name__=='__main__':unittest.main()
