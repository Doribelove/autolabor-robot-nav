"""Exercise the real two-stage node publisher with synthetic detections/RGB-D/TF.

Only model inference and ROS transport are replaced. No ROS master or robot I/O.
"""
import copy
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix

spec=importlib.util.spec_from_file_location('map_test_node',Path(__file__).resolve().parents[1]/'scripts/fod_detect_and_classify_node.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Publisher:
    def __init__(self):self.messages=[]
    def get_num_connections(self):return 0
    def publish(self,message):
        message.serialize(io.BytesIO())
        self.messages.append(copy.deepcopy(message))


class FakeRuntime:
    def __init__(self,**kwargs):
        self.model_name='synthetic detector + classifier'
        self.detector_model_sha256=self.classifier_model_sha256='0'*64
        self.runtime_path=self.runtime_version='test'
        self.ultralytics_path=self.ultralytics_version=self.detector_path=self.classifier_path='test'
        self.gam_layer_count=1
        self.boxes=[SimpleNamespace(bbox=(48,30,115,90),track_id=1,confidence=.9)]
    def set_detector_confidence(self,value):return value
    def detect(self,*_):return self.boxes,.1
    def classify(self,crops):return [np.array([.01,.96,.01,.01,.01]) for _ in crops],.1
    def close(self):pass


class MapRuntimeTest(unittest.TestCase):
    def setUp(self):
        package=Path(__file__).resolve().parents[1]
        params=yaml.safe_load((package/'config/detect_and_classify.yaml').read_text())['detect_and_classify']
        params['motion']['enabled']=False
        params['map_localization']['mount']=yaml.safe_load((package.parent/'autolabor_bpu_perception/config/zed_mount.yaml').read_text())
        def param(name,default=None):
            if name=='~required_class_names':return 'metal,plastic,paper,glass,kitchen_waste'
            if name=='~two_stage_tracker_config':return 'test.yaml'
            if name.startswith('~detect_and_classify/'):
                value=params
                for part in name.split('/')[1:]:
                    if not isinstance(value,dict) or part not in value:return default
                    value=value[part]
                return value
            return default
        self.stamp=100.0
        self.publishers={}
        def publisher(topic,*_,**__):
            self.publishers[topic]=Publisher();return self.publishers[topic]
        patches=[patch.object(module,'TwoStageUltralyticsRuntime',FakeRuntime),
                 patch.object(module.rospy,'get_param',side_effect=param),
                 patch.object(module.rospy,'Publisher',side_effect=publisher),
                 patch.object(module.rospy,'Subscriber',return_value=Mock()),
                 patch.object(module.rospy,'Service',return_value=Mock()),
                 patch.object(module.rospy,'set_param'),patch.object(module.rospy,'on_shutdown'),
                 patch.object(module.rospy,'resolve_name',side_effect=lambda x:x),
                 patch.object(module.tf2_ros,'TransformListener'),
                 patch.object(module.tf2_ros,'Buffer',return_value=Mock()),
                 patch.object(module.threading.Thread,'start'),
                 patch.object(module.DetectAndClassifyNode,'_publish_diagnostic'),
                 patch.object(module.rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(self.stamp))]
        for p in patches:p.start();self.addCleanup(p.stop)
        self.node=module.DetectAndClassifyNode()
        self.node.tf_buffer.lookup_transform=Mock(side_effect=self.lookup)
        self.map_available=True;self.base_available=True;self.map_x=10.;self.old_mount=False
        self.frame_id='test_left_camera_optical_frame'
        self.bridge=CvBridge()
        self.depth=np.full((120,160),2.5,dtype=np.float32);self.depth[42:78,58:103]=1.2
        self.rgb=np.random.default_rng(7).integers(0,256,(120,160,3),dtype=np.uint8)

    def lookup(self,target,source,stamp,timeout):
        self.assertAlmostEqual(stamp.to_sec(),self.stamp)
        if target=='base_link' and not self.base_available:
            raise tf2_ros.LookupException('camera extrinsics unavailable')
        matrix=self.node.map_localizer.expected.copy() if target=='base_link' else np.eye(4)
        if target=='map':
            if not self.map_available:raise tf2_ros.ExtrapolationException('source time unavailable')
            matrix[0,3]=self.map_x
        if self.old_mount and target=='base_link':matrix[0,3]+=.05
        message=TransformStamped();message.header.stamp=stamp;message.header.frame_id=target;message.child_frame_id=source
        message.transform.translation.x,message.transform.translation.y,message.transform.translation.z=matrix[:3,3]
        q=quaternion_from_matrix(matrix)
        (message.transform.rotation.x,message.transform.rotation.y,message.transform.rotation.z,message.transform.rotation.w)=q
        return message

    def run_frame(self,depth_delta=0.,state='LOCALIZED',depth_frame=None):
        self.stamp+=.125
        rgb=self.bridge.cv2_to_imgmsg(self.rgb,'bgr8');rgb.header.stamp=rospy.Time.from_sec(self.stamp);rgb.header.frame_id=self.frame_id
        depth=self.bridge.cv2_to_imgmsg(self.depth,'32FC1');depth.header=copy.deepcopy(rgb.header)
        depth.header.stamp=rospy.Time.from_sec(self.stamp+depth_delta)
        if depth_frame is not None:depth.header.frame_id=depth_frame
        info=CameraInfo();info.header=copy.deepcopy(rgb.header);info.width=160;info.height=120
        info.K=[100,0,80,0,100,60,0,0,1];info.P=[100,0,80,0,0,100,60,0,0,0,1,0]
        self.node._depth_callback(depth);self.node._camera_info_callback(info)
        self.node._localization_callback(String('state='+state+';'))
        self.node._process_frame(module.FrameEnvelope(rgb,self.stamp,self.frame_id))
        return self.publishers['/fod/vision/map_positions'].messages[-1]

    def test_source_stamped_map_publication_with_valid_rgbd(self):
        result=self.run_frame()
        self.assertEqual(result.header.frame_id,'map');self.assertEqual(result.header.stamp,result.source_header.stamp)
        self.assertTrue(result.positions[0].position_valid)
        self.assertGreater(result.positions[0].position.x,10)
        point=result.positions[0]
        self.assertTrue(point.local_position_valid)
        self.assertEqual((point.local_frame,point.local_status),('mid360','OK'))
        np.testing.assert_allclose(
            [point.position.x-self.map_x-.2,point.position.y],
            [point.local_position.x,point.local_position.y])
        self.assertTrue(np.isnan(point.position.z))
        self.assertTrue(np.isnan(point.local_position.z))
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_count,3)
        self.assertTrue(point.odom_position_valid)
        self.assertFalse(result.map_stable)
        self.assertEqual(result.odom_frame, 'camera_init')

    def prepare_flat_paper(self):
        self.depth[:] = 0.90
        self.node.motion_enabled = True  # Transport is mocked; no ROS/robot I/O.
        self.node.runtime.classify = lambda crops: (
            [np.array([.01, .01, .96, .01, .01]) for _ in crops], .1)
        for _ in range(16):
            result = self.run_frame()
        return result

    def test_flat_paper_keeps_current_depth_and_map_and_motion_outputs(self):
        result = self.prepare_flat_paper()
        point = result.positions[0]
        self.assertEqual(point.material_class, 'paper')
        self.assertTrue(point.position_valid, point.status)
        self.assertTrue(point.local_position_valid, point.local_status)
        self.assertAlmostEqual(point.depth_m, 0.90, places=5)
        targets = list(self.node.object_map.objects.values())
        self.assertTrue(any(t.depth_locked and t.world_locked for t in targets))
        self.assertTrue(all(not t.depth_separated_from_background for t in targets))
        legacy = self.publishers['/fod/detections'].messages[-1]
        self.assertTrue(legacy.depth_synchronized)
        self.assertEqual(len(legacy.detections), 1)
        self.assertEqual(legacy.detections[0].class_name, 'paper')
        self.assertTrue(legacy.detections[0].depth_valid)
        self.assertAlmostEqual(legacy.detections[0].depth_m, 0.90, places=5)

    def test_flat_paper_invalid_depth_does_not_reuse_locked_measurement(self):
        self.prepare_flat_paper()
        self.depth[:] = np.nan
        point = self.run_frame().positions[0]
        self.assertFalse(point.position_valid)
        self.assertFalse(point.local_position_valid)
        self.assertTrue(point.status.startswith('DEPTH_UNAVAILABLE'))
        legacy = self.publishers['/fod/detections'].messages[-1]
        self.assertTrue(all(not d.depth_valid for d in legacy.detections))

    def test_flat_paper_wrong_timestamp_still_rejected(self):
        self.prepare_flat_paper()
        self.node._depth_messages.clear()
        point = self.run_frame(depth_delta=.08).positions[0]
        self.assertFalse(point.position_valid)
        self.assertFalse(point.local_position_valid)
        legacy = self.publishers['/fod/detections'].messages[-1]
        self.assertFalse(legacy.depth_synchronized)
        self.assertTrue(all(not d.depth_valid for d in legacy.detections))

    def test_locked_object_recomputed_after_robot_translation(self):
        for _ in range(7):result=self.run_frame()
        self.assertTrue(any(t.world_locked for t in self.node.object_map.objects.values()))
        before=result.positions[0].position.x
        local_before=result.positions[0].local_position
        self.map_x+=.2
        after=self.run_frame().positions[0]
        self.assertTrue(after.position_valid)
        self.assertAlmostEqual(after.position.x-before,.2,places=6)
        np.testing.assert_allclose([after.local_position.x,after.local_position.y],
                                   [local_before.x,local_before.y])

    def test_landmark_context_filters_only_observation_map_and_preserves_control(self):
        for _ in range(50): result = self.run_frame()
        self.assertTrue(result.map_stable)
        matrix = np.asarray(result.stable_map_from_odom).reshape(4,4)
        point = result.positions[0]
        reconstructed = matrix @ np.array([point.odom_position.x, point.odom_position.y,
                                            point.odom_position.z, 1])
        np.testing.assert_allclose(reconstructed[:2], [point.position.x, point.position.y])
        session = result.odom_session_id
        self.map_x += 1.8
        result = self.run_frame()
        self.assertFalse(result.map_stable)
        self.assertEqual(session, result.odom_session_id)
        self.assertTrue(result.positions[0].position_valid)  # raw measurement unchanged
        self.assertTrue(result.positions[0].odom_position_valid)
        self.assertEqual(result.stability_status, 'MAP_CORRECTION_JUMP')

    def test_no_detections_does_not_reset_stable_localization_window(self):
        self.node.runtime.boxes = []
        for _ in range(50): result = self.run_frame()
        self.assertTrue(result.map_stable)
        self.assertEqual([], result.positions)

    def test_map_loss_clears_previously_locked_coordinates(self):
        for _ in range(7):self.run_frame()
        self.map_available=False
        result=self.run_frame().positions[0]
        self.assertFalse(result.position_valid);self.assertEqual(result.status,'MAP_TF_UNAVAILABLE')
        self.assertTrue(np.isnan(result.position.x))
        self.assertTrue(result.local_position_valid);self.assertEqual(result.local_status,'OK')

    def test_localization_lost_clears_coordinates_and_recovers(self):
        self.assertTrue(self.run_frame().positions[0].position_valid)
        self.node.tf_buffer.lookup_transform.reset_mock()
        result=self.run_frame(state='LOST').positions[0]
        self.assertFalse(result.position_valid);self.assertEqual(result.status,'LOCALIZATION_UNAVAILABLE')
        self.assertTrue(result.local_position_valid)
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_count,2)
        self.assertTrue(self.run_frame().positions[0].position_valid)

    def test_missing_camera_transform_clears_both_positions(self):
        self.run_frame()
        self.base_available=False
        result=self.run_frame().positions[0]
        self.assertFalse(result.position_valid);self.assertFalse(result.local_position_valid)
        self.assertEqual(result.local_status,'BASE_TF_UNAVAILABLE')
        self.assertTrue(np.isnan(result.local_position.x))

    def test_no_depth_cannot_republish_old_point(self):
        for _ in range(7):self.run_frame()
        self.depth[:]=float('nan')
        result=self.run_frame().positions[0]
        self.assertFalse(result.position_valid);self.assertTrue(result.status.startswith('DEPTH_UNAVAILABLE'))
        self.assertFalse(result.local_position_valid);self.assertTrue(np.isnan(result.local_position.x))

    def test_wrong_depth_timestamp_rejected(self):
        result=self.run_frame(depth_delta=.08).positions[0]
        self.assertFalse(result.position_valid);self.assertFalse(result.local_position_valid)

    def test_wrong_depth_frame_rejected(self):
        result=self.run_frame(depth_frame='right_optical').positions[0]
        self.assertFalse(result.position_valid);self.assertFalse(result.local_position_valid)

    def test_bad_extrinsics_rejected(self):
        self.old_mount=True
        result=self.run_frame().positions[0]
        self.assertFalse(result.position_valid);self.assertEqual(result.status,'EXTRINSIC_MISMATCH')
        self.assertFalse(result.local_position_valid);self.assertEqual(result.local_status,'EXTRINSIC_MISMATCH')

    def test_empty_frame_clears_previous_targets(self):
        self.run_frame();self.node.runtime.boxes=[]
        result=self.run_frame()
        self.assertEqual(result.positions,[]);self.assertEqual(result.status,'NO_DETECTIONS')

    def test_two_objects_share_exact_frame_transform(self):
        self.node.runtime.boxes.append(SimpleNamespace(bbox=(35,20,125,100),track_id=2,confidence=.8))
        result=self.run_frame()
        self.assertEqual(len(result.positions),2)
        self.assertEqual(self.node.tf_buffer.lookup_transform.call_count,3)

    def test_expired_processing_does_not_publish_new_coordinates(self):
        self.run_frame()
        publisher=self.publishers['/fod/vision/map_positions']
        before=len(publisher.messages)
        with patch.object(self.node,'_source_age',return_value=.6):
            self.run_frame()
        self.assertEqual(len(publisher.messages),before)

    def test_localization_loss_during_classification_invalidates_output(self):
        original=self.node.runtime.classify
        def classify(crops):
            self.node._localization_callback(String('state=LOST;'))
            return original(crops)
        self.node.runtime.classify=classify
        result=self.run_frame().positions[0]
        self.assertFalse(result.position_valid)
        self.assertEqual(result.status,'LOCALIZATION_UNAVAILABLE')
        self.assertTrue(result.local_position_valid)


if __name__=='__main__':unittest.main()
