#!/usr/bin/env python3
"""Ensure visual persistence never becomes a fabricated control observation."""
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from std_msgs.msg import Header

from autolabor_fod_vision.detector import Detection, InferenceResult
from autolabor_fod_vision.preview_tracking import PreviewStabilizer
from autolabor_fod_vision.detection_roi import DetectionRoi
from autolabor_fod_vision.depth import RegisteredDepthFusion, DetectionTracker
from autolabor_fod_msgs.msg import FodDetectionArray

spec = importlib.util.spec_from_file_location(
    "fod_detector_node", str(Path(__file__).resolve().parents[1] / "scripts/fod_detector_node.py")
)
module = importlib.util.module_from_spec(spec)
# Image conversion is unchanged and exercised by the live camera check. Isolate
# it here so the control-message contract test does not need a native CV bridge.
with patch.dict('sys.modules', {'cv_bridge': SimpleNamespace(CvBridge=None)}):
    spec.loader.exec_module(module)


class DetectorPreviewContractTest(unittest.TestCase):
    def setUp(self):
        clock = patch.object(module.rospy.Time, "now", return_value=rospy.Time.from_sec(20))
        clock.start()
        self.addCleanup(clock.stop)

    def test_three_current_boxes_carry_serializable_stereo_range_and_independent_ids(self):
        boxes = [Detection(0,"Metal",c,x,30,x+20,50,x+10,50)
                 for x,c in [(5,.99),(45,.4),(85,.8)]]
        node=module.DetectorNode.__new__(module.DetectorNode)
        node.detector=SimpleNamespace(predict=lambda image: InferenceResult(boxes,10.0),
                                     model_name='best.pt',model_sha256='a'*64,task='detect')
        node.detector_profile='airport_recall'
        node.roi=DetectionRoi()
        node.preview_stabilizer=PreviewStabilizer()
        node.depth_fusion=RegisteredDepthFusion()
        node.detection_tracker=DetectionTracker()
        node.selected_track_id=0
        node.recovery_overlay=None
        node.depth_fusion.set_camera(10,'left',128,96,[100,0,64,0,0,100,48,0,0,0,1,0])
        depth=np.full((96,128),np.nan,np.float32)
        for x,z in [(5,3),(45,1),(85,2)]:depth[30:50,x:x+20]=z
        node.depth_fusion.add_depth(10,'left',depth,'32FC1')
        node.bridge=SimpleNamespace(imgmsg_to_cv2=lambda msg,desired_encoding:msg.image,
                                    cv2_to_imgmsg=lambda array,encoding:SimpleNamespace(image=array))
        node.processed_frames=0
        node.debug_every_n=1
        node.debug_max_fps=30.0
        node._next_debug_time=0.0
        node.smoke_test_only=False
        node._publish_diagnostic=lambda *args:None
        messages=[]
        node.detection_pub=SimpleNamespace(publish=messages.append)
        node.debug_pub=SimpleNamespace(publish=lambda msg:None,get_num_connections=lambda:1)
        node._process(SimpleNamespace(image=np.zeros((96,128,3),np.uint8),
                                     header=Header(stamp=rospy.Time.from_sec(10),frame_id='left')))
        wire=io.BytesIO()
        messages[0].serialize(wire)
        received=FodDetectionArray().deserialize(wire.getvalue())
        self.assertEqual(received.header.stamp,rospy.Time.from_sec(10))
        self.assertEqual(len({d.track_id for d in received.detections}),3)
        self.assertTrue(all(d.depth_valid for d in received.detections))
        self.assertEqual([d.depth_m for d in received.detections],[3,1,2])
        nearest=min(received.detections,key=lambda d:d.range_m)
        self.assertEqual(nearest.bbox.x_offset,45)
        self.assertAlmostEqual(nearest.confidence,.4)

    def test_missing_current_frame_publishes_empty_even_while_preview_holds(self):
        observed = Detection(0, "Metal", 0.11, 10, 10, 30, 30, 20, 30)
        results = iter([InferenceResult([observed], 20.0), InferenceResult([], 20.0)])
        node = module.DetectorNode.__new__(module.DetectorNode)
        node.detector = SimpleNamespace(
            predict=lambda image: next(results), model_name="best.pt",
            model_sha256="a" * 64, task="detect",
        )
        node.detector_profile = "airport_recall"
        node.depth_fusion = None
        node.selected_track_id = 0
        node.recovery_overlay = None
        node.roi = DetectionRoi()
        node.preview_stabilizer = PreviewStabilizer()
        node.bridge = SimpleNamespace(
            imgmsg_to_cv2=lambda msg, desired_encoding: msg.image,
            cv2_to_imgmsg=lambda array, encoding: SimpleNamespace(image=array),
        )
        node.processed_frames = 0
        node.debug_every_n = 1
        node.debug_max_fps = 30.0
        node._next_debug_time = 0.0
        node.smoke_test_only = False
        node._publish_diagnostic = lambda *args: None
        control_messages, preview_images = [], []
        node.detection_pub = SimpleNamespace(publish=control_messages.append)
        node.debug_pub = SimpleNamespace(publish=preview_images.append,get_num_connections=lambda:1)
        for stamp in (1.0, 1.1):
            image = SimpleNamespace(
                image=np.zeros((60, 80, 3), dtype=np.uint8),
                header=Header(stamp=rospy.Time.from_sec(stamp), frame_id='camera'),
            )
            with patch.object(module.time,"monotonic",return_value=stamp):
                node._process(image)
        self.assertEqual(len(control_messages[0].detections), 1)
        self.assertAlmostEqual(control_messages[0].detections[0].confidence, 0.11)
        self.assertEqual(control_messages[0].detections[0].bbox.x_offset, 10)
        self.assertEqual(control_messages[1].detections, [])
        self.assertEqual(control_messages[1].header.stamp, rospy.Time.from_sec(1.1))
        self.assertEqual(node.preview_box_count, 1)
        self.assertEqual(node.preview_held_count, 1)
        self.assertEqual(len(preview_images), 2)


if __name__ == '__main__':
    unittest.main()
