"""Execute the real ROS publisher method with in-memory transport and time.

Uses real ROS message classes/CvBridge; no ROS master, BPU, camera or robot starts.
"""
from collections import deque
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_from_matrix
from autolabor_fod_vision.litter_filter import LitterFilter, FilterConfig

sys.path.insert(0,str(Path(__file__).resolve().parent))
from litter_scenes import scene, BOTTLE, PLANT

spec = importlib.util.spec_from_file_location('litter_test_node',
    Path(__file__).resolve().parents[1]/'scripts/fcos_rgbd_node.py')
node_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node_module)


class Publisher:
    def __init__(self):self.messages=[]
    def get_num_connections(self):return 1
    def publish(self,message):
        # Serialization catches NaNs in audit separately and ROS type/field errors.
        message.serialize(io.BytesIO())
        self.messages.append(copy.deepcopy(message))


class LitterRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.depth,self.k,self.mount,self.boxes = scene([PLANT,BOTTLE])
        self.node = node_module.FcosRgbd.__new__(node_module.FcosRgbd)
        node = self.node
        node.mount=self.mount;node.lock=threading.Lock();node.bridge=CvBridge()
        node.session='offline-runtime-test';node.maximum_age=.35
        node.litter_filter=LitterFilter(FilterConfig(budget_ms=100))
        node.processed=node.depth_matches=node.depth_valid=node.expired=0
        node.last_result=0.;node.last_stamp=10.
        node.depth=deque();node.info=deque();node.report=Mock()
        for name in ('pub','candidate_pub','audit_pub','image_pub','raw_image_pub'):
            setattr(node,name,Publisher())
        transform=TransformStamped()
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z = self.mount['left_position']
        matrix=np.eye(4);matrix[:3,:3]=self.mount['optical_to_base']
        q=quaternion_from_matrix(matrix)
        transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=q
        def lookup(target,*_):
            if target=='map':raise tf2_ros.LookupException('no map in offline test')
            return transform
        node.tf_buffer=Mock();node.tf_buffer.lookup_transform.side_effect=lookup

    def publish(self,stamp=10.,depth_delta=0.,depth_frame=None,encoding='32FC1',now_age=.05):
        node=self.node
        image=np.full((360,640,3),45,dtype=np.uint8)
        frame=node.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        frame.header.stamp=rospy.Time.from_sec(stamp)
        frame.header.frame_id='zed2_left_camera_optical_frame'
        depth=self.depth if encoding=='32FC1' else (self.depth*1000).astype(np.uint16)
        message=node.bridge.cv2_to_imgmsg(depth,encoding=encoding)
        message.header=copy.deepcopy(frame.header)
        message.header.stamp=rospy.Time.from_sec(stamp+depth_delta)
        if depth_frame:message.header.frame_id=depth_frame
        info=CameraInfo();info.header=copy.deepcopy(frame.header);info.width=640;info.height=360;info.K=self.k
        node.depth.clear();node.info.clear();node.depth.append(message);node.info.append(info)
        result=dict(timing=dict(vendor_infer_ms=4.),client_timing={})
        with patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(stamp+now_age)):
            node.publish(frame,image,self.boxes,result)

    def test_real_publisher_keeps_raw_and_filters_only_environment(self):
        for i in range(3):self.publish(10+i*.07)
        raw=self.node.pub.messages[-1];candidate=self.node.candidate_pub.messages[-1]
        self.assertEqual(len(raw.objects),2)
        self.assertEqual(len(candidate.objects),1)
        self.assertEqual(candidate.objects[0].class_name,'suitcase')
        self.assertFalse(raw.motion_eligible);self.assertFalse(candidate.motion_eligible)
        self.assertTrue(raw.calibrated)
        self.assertEqual(raw.header.stamp,candidate.header.stamp)
        audit=json.loads(self.node.audit_pub.messages[-1].data)
        self.assertEqual(audit['counts'],dict(environment=1,candidate=1,uncertain=0))
        self.assertEqual(self.node.raw_image_pub.messages[-1].header.frame_id,'bpu_preview_raw_only')
        self.assertEqual(self.node.image_pub.messages[-1].header.frame_id,'bpu_preview_demo_only')
        # Raw plant rectangle exists; filtered image at the same box is untouched.
        x,y=map(int,self.boxes[0]['bbox'][:2])
        raw_image=self.node.bridge.imgmsg_to_cv2(self.node.raw_image_pub.messages[-1])
        filtered=self.node.bridge.imgmsg_to_cv2(self.node.image_pub.messages[-1])
        self.assertFalse(np.array_equal(raw_image[y,x],filtered[y,x]))
        self.assertTrue(np.array_equal(filtered[y,x],[45,45,45]))

    def test_wrong_depth_timestamp_or_optical_frame_keeps_all(self):
        for options in ({'depth_delta':.1},{'depth_frame':'wrong_optical_frame'}):
            self.setUp()
            for i in range(3):self.publish(10+i*.07,**options)
            self.assertEqual(len(self.node.candidate_pub.messages[-1].objects),2)
            audit=json.loads(self.node.audit_pub.messages[-1].data)
            self.assertEqual(audit['counts']['environment'],0)

    def test_millimeter_depth_matches_meter_depth(self):
        for i in range(3):self.publish(10+i*.07,encoding='16UC1')
        self.assertEqual(len(self.node.candidate_pub.messages[-1].objects),1)

    def test_stale_fusion_publishes_no_boxes_images_or_audit(self):
        self.publish(now_age=.4)
        self.assertEqual(self.node.expired,1)
        for name in ('pub','candidate_pub','audit_pub','image_pub','raw_image_pub'):
            self.assertEqual(len(getattr(self.node,name).messages),0)

    def test_heartbeat_clears_raw_filtered_and_audit(self):
        for i in range(3):self.publish(10+i*.07)
        self.node.last_result=0.
        self.node.heartbeat(None)
        for name in ('pub','candidate_pub'):
            message=getattr(self.node,name).messages[-1]
            self.assertFalse(message.valid);self.assertEqual(message.objects,[])
        self.assertFalse(json.loads(self.node.audit_pub.messages[-1].data)['valid'])

    def test_missing_tf_keeps_boxes_without_claiming_calibrated_position(self):
        self.node.tf_buffer.lookup_transform.side_effect=tf2_ros.LookupException('unavailable')
        for i in range(3):self.publish(10+i*.07)
        message=self.node.candidate_pub.messages[-1]
        self.assertFalse(message.calibrated)
        self.assertEqual(len(message.objects),2)
        self.assertFalse(any(o.position_valid for o in message.objects))


if __name__=='__main__':unittest.main()
