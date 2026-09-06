import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import rospy
import tf2_ros
from geometry_msgs.msg import Transform,Point,Vector3
from autolabor_bpu_perception.msg import DetectedObjects,DetectedObject

spec=importlib.util.spec_from_file_location('adapter',str(Path(__file__).resolve().parents[1]/'scripts/obstacle_adapter_node.py'))
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.node=adapter.ObstacleAdapter.__new__(adapter.ObstacleAdapter)
        for key,value in dict(enabled=False,target='camera_init',age=.35,confidence=.6,latest=None,receipt=0.,
            previous=[],next_id=1,previous_session='',previous_stamp=0.,lock=threading.Lock(),navigation=None).items():setattr(self.node,key,value)
        self.node.candidate=Mock();self.node.tf_buffer=Mock()
        transform=Transform();transform.rotation.w=1.;transform.translation.x=10
        self.node.tf_buffer.lookup_transform.return_value=NS(transform=transform)
        self.now=patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(200));self.now.start();self.addCleanup(self.now.stop)

    def message(self,stamp=200):
        m=DetectedObjects();m.header.stamp=rospy.Time.from_sec(stamp);m.header.frame_id='base_link'
        m.valid=True;m.calibrated=True;m.session_id='one';m.model_sha256='7a12188054b4f5a05c112dd6d8611436d84cda3e060e11f72d1f1e739040593c'
        m._connection_header={'callerid':'/centerpoint_mid360'}
        item=DetectedObject();item.class_id=8;item.confidence=.9;item.position_valid=True;item.dimensions_valid=True
        item.position=Point(2,0,1);item.dimensions=Vector3(.5,.5,1.8);m.objects=[item]
        return m

    def test_two_distinct_observations_and_no_header_mutation(self):
        m=self.message(199.9);self.node.callback(m);self.node.tick(None)
        self.assertEqual(self.node.candidate.publish.call_args[0][0].obstacles,[])
        self.node.tick(None)
        self.assertEqual(self.node.candidate.publish.call_args[0][0].obstacles,[])
        newer=self.message(200);self.node.callback(newer);self.node.tick(None)
        output=self.node.candidate.publish.call_args[0][0]
        self.assertEqual(len(output.obstacles),1)
        self.assertGreater(min(p.x for p in output.obstacles[0].polygon.points),11)
        self.assertEqual(newer.header.frame_id,'base_link')
        self.assertEqual(output.header.stamp,newer.header.stamp)
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_args[0][2],newer.header.stamp)

    def test_expired_source_never_renews_by_receipt(self):
        self.node.callback(self.message(199));self.node.tick(None)
        self.assertEqual(self.node.candidate.publish.call_args[0][0].obstacles,[])
        self.node.tf_buffer.lookup_transform.assert_not_called()

    def test_tf_failure_drops_obstacles(self):
        self.node.callback(self.message());self.node.tf_buffer.lookup_transform.side_effect=tf2_ros.LookupException('missing')
        with patch.object(rospy,'logwarn_throttle'):self.node.tick(None)
        self.assertEqual(self.node.candidate.publish.call_args[0][0].obstacles,[])

    def test_replay_and_unknown_publisher_cannot_replace_source(self):
        first=self.message();self.node.callback(first);self.node.callback(self.message(199.9))
        self.assertIs(self.node.latest,first)
        foreign=self.message(200.01);foreign._connection_header={'callerid':'/unknown'}
        with patch.object(rospy,'logwarn_throttle'):self.node.callback(foreign)
        self.assertIs(self.node.latest,first)

    def test_nan_geometry_cannot_reach_candidate(self):
        m=self.message();m.objects[0].yaw=float('nan');self.node.callback(m)
        with patch.object(rospy,'logwarn_throttle'):self.node.tick(None)
        self.assertEqual(self.node.candidate.publish.call_args[0][0].obstacles,[])


if __name__=='__main__':unittest.main()
