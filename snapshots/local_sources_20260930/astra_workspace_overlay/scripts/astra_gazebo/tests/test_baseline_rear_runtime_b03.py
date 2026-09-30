"""Small pure and actual-message transport checks; no ROS master or simulation."""
import ast,copy,hashlib,importlib.util,json,math,sys,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
P=Path(__file__).resolve().parents[1];W=P.parents[1];G=W/'src/simulation/m2_gazebo'
sys.path.insert(0,str(P));sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import genpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String
from baseline_rear_runtime_b03 import *
from common_route import CommonRoute

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
node=load('rear_b03_actual_node',G/'scripts/astra_baseline_rear_adapter_b03.py')

def message(ns=1_000_000_000):
    m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=genpy.Time(ns//10**9,ns%10**9)
    m.pose.pose.position.x=2.;m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=1.2;m.twist.twist.linear.y=.235;m.twist.twist.angular.z=.6
    m.pose.covariance=list(np.eye(6).ravel());m.twist.covariance=list((np.eye(6)*2).ravel());return m

def stamp(ns):return genpy.Time(ns//10**9,ns%10**9)

class StreamTests(unittest.TestCase):
    def test_actual_message_path_retains_source_and_all_fields(self):
        s=RearOdometryStream();out=[];m=message();r=node.consume_message(s,m,lambda:1_000_000_000,out.append,Odometry,stamp)
        self.assertEqual(len(out),1);self.assertEqual(out[0].header.stamp,m.header.stamp);self.assertEqual(out[0].child_frame_id,FRAME)
        self.assertAlmostEqual(out[0].pose.pose.position.x,1.675);self.assertAlmostEqual(out[0].twist.twist.linear.y,.04)
        self.assertEqual(out[0].twist.twist.angular.z,.6);self.assertEqual(len(out[0].pose.covariance),36)
        self.assertNotEqual(out[0].twist.covariance,m.twist.covariance);self.assertEqual(r[0][0].received_ns,1_000_000_000)
    def test_two_ms_future_waits_without_retiming(self):
        s=RearOdometryStream();out=[];m=message(1_002_000_000)
        self.assertEqual(node.consume_message(s,m,lambda:1_000_000_000,out.append,Odometry,stamp),[]);self.assertEqual(out,[])
        r=node.publish_pending(s,lambda:1_002_000_000,out.append,Odometry,stamp)
        self.assertEqual(out[0].header.stamp.to_nsec(),1_002_000_000);self.assertEqual(r[0][0].received_ns,1_000_000_000)
    def test_source_age_boundary(self):
        for delta,passes in [(AGE_NS,True),(AGE_NS+1,False)]:
            s=RearOdometryStream();out=[]
            if passes:node.consume_message(s,message(),lambda:1_000_000_000+delta,out.append,Odometry,stamp);self.assertEqual(len(out),1)
            else:
                with self.assertRaises(ValueError):node.consume_message(s,message(),lambda:1_000_000_000+delta,out.append,Odometry,stamp)
                self.assertFalse(out);self.assertIsNotNone(s.fault_reason)
    def test_future_beyond_bound_faults(self):
        s=RearOdometryStream()
        with self.assertRaises(ValueError):s.ingest(node.message_row(message(1_200_000_001)),1_000_000_000)
    def test_duplicate_regression_and_clock_rollback_are_permanent(self):
        for kind in ['duplicate','regression','clock']:
            s=RearOdometryStream();s.ingest(node.message_row(message()),1_000_000_000);s.drain(1_000_000_000)
            with self.assertRaises(ValueError):
                if kind=='clock':s.drain(999_999_999)
                else:s.ingest(node.message_row(message(1_000_000_000 if kind=='duplicate' else 999_999_999)),1_000_000_000)
            with self.assertRaises(ValueError):s.ingest(node.message_row(message(1_020_000_000)),1_020_000_000)
    def test_publication_deadline_uses_fresh_actual_clock(self):
        s=RearOdometryStream();clock=iter([1_000_000_000,1_000_000_000,1_200_000_001]);out=[]
        with self.assertRaises(ValueError):node.consume_message(s,message(),lambda:next(clock),out.append,Odometry,stamp)
        self.assertFalse(out);self.assertIn('expired',s.fault_reason)
    def test_raw_quaternion_rejected_before_normalization(self):
        for q in [(0.,0.,0.,2.),(0.,0.,0.,0.),(float('nan'),0.,0.,1.)]:
            m=message();m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q;s=RearOdometryStream()
            with self.assertRaises(ValueError):s.ingest(node.message_row(m),1_000_000_000)
    def test_frame_tilt_nonfinite_covariance_rejected(self):
        for kind in ['frame','tilt','position','covariance']:
            m=message()
            if kind=='frame':m.child_frame_id='/base_link'
            elif kind=='tilt':m.pose.pose.orientation.x=math.sin(.051/2);m.pose.pose.orientation.w=math.cos(.051/2)
            elif kind=='position':m.pose.pose.position.x=float('inf')
            else:m.pose.covariance=[float('nan')]*36
            with self.assertRaises(ValueError):RearOdometryStream().ingest(node.message_row(m),1_000_000_000)
    def test_stale_latest_faults_not_republished(self):
        s=RearOdometryStream();s.ingest(node.message_row(message()),1_000_000_000);s.drain(1_000_000_000)
        with self.assertRaises(ValueError):s.drain(1_200_000_001)
    def test_zero_stamp_only_bootstrap_never_ready(self):
        s=RearOdometryStream();s.ingest(node.message_row(message(0)),0);s.drain(0);self.assertFalse(s.health(0))

class RouteGoalConfigTests(unittest.TestCase):
    def fixture(self):
        route=CommonRoute([(-4.,0.),(-1.,1.),(5.,0.)],.7)
        data={'frame_id':'map','provider_returns_exact_full_polyline':True,'goal_xy_yaw':[5.,0.,.7]}
        return RearFixedRoute(route,data)
    def test_actual_service_message_path_exact_endpoint(self):
        service=self.fixture();request=types.SimpleNamespace(start=PoseStamped(),goal=PoseStamped())
        for target,p in [(request.start,planar_pose_to_rear((-4.,0.,.3))),(request.goal,planar_pose_to_rear((5.,0.,.7)))]:
            target.header.frame_id='map';target.pose.position.x,target.pose.position.y=p[:2];target.pose.orientation.z=math.sin(p[2]/2);target.pose.orientation.w=math.cos(p[2]/2)
        result=node.route_response_poses(service,request);self.assertEqual(result,service.converted['rear_poses'])
        self.assertEqual(result[-1],planar_pose_to_rear((5.,0.,.7)));self.assertEqual(len(result),3)
        request.goal.pose.position.x=5.
        with self.assertRaises(ValueError):node.route_response_poses(service,request)
    def test_goal_full_pose_is_not_recomputed_from_chord(self):
        s=self.fixture();self.assertEqual(s.converted['rear_poses'][-1][2],.7)
        self.assertNotEqual(s.converted['rear_poses'][-1][:2],(4.675,0.))
    def test_full_tilted_rear_start_inverse_is_correct(self):
        s=self.fixture();base=Pose3.from_xyz_rpy((-4,0,.02),(.01,.02,.3))
        from baseline_rear_axle_adapter_b02 import rigid_pose_to_rear
        start=rigid_pose_to_rear(base);goal=Pose3.from_xyz_rpy((*planar_pose_to_rear((5,0,.7))[:2],0),(0,0,.7))
        self.assertEqual(s.request('map',start.xyz,start.xyzw,'map',goal.xyz,goal.xyzw),s.converted['rear_poses'])
    def test_internal_goal_bound_strictly_inside_shared(self):
        c=goal_tolerance_contract();self.assertLess(c['maximum_base_xy_at_internal_goal_m'],.25);self.assertEqual(c['internal_xy_m'],.15);self.assertEqual(c['internal_yaw_rad'],.10)
        self.assertFalse(c['dwa_latch_xy_goal_tolerance']);self.assertFalse(c['strict_callback_wall_order_guaranteed'])
    def test_final_yaml_wiring_and_full_footprint_translation(self):
        import yaml
        old=yaml.safe_load((G/'config/astra_navigation_common_footprint.yaml').read_text());new=yaml.safe_load((G/'config/astra_baseline_rear_b03.yaml').read_text())
        for k in ['local_costmap','global_costmap']:
            self.assertEqual(new[k]['robot_base_frame'],FRAME);self.assertEqual(new[k]['footprint_padding'],0.)
            np.testing.assert_allclose(np.asarray(new[k]['footprint'])-np.asarray(old[k]['footprint']),np.tile([.325,0.],(64,1)),atol=1e-15)
        np.testing.assert_allclose(np.asarray(new['TebLocalPlannerROS']['footprint_model']['vertices'])-np.asarray(old['TebLocalPlannerROS']['footprint_model']['vertices']),np.tile([.325,0.],(16,1)),atol=1e-15)
        for k in ['TebLocalPlannerROS','DWAPlannerROS']:
            self.assertEqual(new[k]['odom_topic'],'/astra/rear_odom');self.assertEqual(new[k]['xy_goal_tolerance'],.15);self.assertEqual(new[k]['yaw_goal_tolerance'],.10)
        self.assertFalse(new['DWAPlannerROS']['latch_xy_goal_tolerance'])
    def test_inherited_shared_coordinator_classes_AST_unchanged(self):
        def classes(p):return {n.name:ast.dump(n,include_attributes=False) for n in ast.parse(p.read_text()).body if isinstance(n,ast.ClassDef)}
        self.assertEqual(classes(P/'navigation_coordinator_n02.py'),classes(P/'navigation_coordinator_b03.py'))
    def test_launch_overlay_last_and_base_coordinator_odom_unremapped(self):
        import xml.etree.ElementTree as E
        r=E.parse(G/'launch/astra_baseline_rear_b03.launch').getroot();move=r.find("node[@name='move_base']")
        self.assertTrue(move.findall('rosparam')[-1].get('file').endswith('astra_baseline_rear_b03.yaml'))
        coordinator=r.find("node[@name='astra_navigation_coordinator']");self.assertFalse(coordinator.findall('remap'))
        self.assertEqual(r.find("node[@name='astra_baseline_rear_adapter_b03']").get('required'),'true')
        self.assertEqual(r.find("node[@name='astra_navigation_collector']").get('required'),'true')
        self.assertEqual(len([n for n in r.findall('node') if 'spawn' in n.get('type','')]),0)

class AdapterStatusTests(unittest.TestCase):
    def row(self,stamp=1_000_000_000,odom=1_000_000_000):return {'schema':SCHEMA,'run_id':'fixture','seed':42,'method':METHODS[0],'state':'ready','ready':True,'status_source_ns':stamp,'latest_odom_source_ns':odom}
    def test_future_status_waits_and_exact_age(self):
        h=RearAdapterHealth('fixture',42,METHODS[0]);h.accept(self.row(1_002_000_000,1_002_000_000),1_000_000_000)
        self.assertFalse(h.ready(1_000_000_000));self.assertTrue(h.ready(1_002_000_000));self.assertTrue(h.ready(1_202_000_000));self.assertFalse(h.ready(1_202_000_001))
    def test_future_status_does_not_invalidate_previous_fresh(self):
        h=RearAdapterHealth('fixture',42,METHODS[0]);h.accept(self.row(),1_000_000_000);self.assertTrue(h.ready(1_000_000_000));h.accept(self.row(1_022_000_000,1_022_000_000),1_020_000_000);self.assertTrue(h.ready(1_020_000_000))
    def test_wrong_identity_fault_and_future_rejected(self):
        for changes in [{'seed':True},{'state':'fault','reason':'sample_fault'},{'status_source_ns':1_200_000_001},{'latest_odom_source_ns':1_000_000_001}]:
            h=RearAdapterHealth('fixture',42,METHODS[0]);row=self.row();row.update(changes)
            with self.assertRaises(ValueError):h.accept(row,1_000_000_000)

if __name__=='__main__':unittest.main()
