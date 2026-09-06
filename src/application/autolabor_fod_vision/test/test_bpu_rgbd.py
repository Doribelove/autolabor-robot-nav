import unittest
import numpy as np
from autolabor_fod_vision.bpu_rgbd import Preprocessor,source_boxes,estimate_boxes,FCOS_SHA
from autolabor_bpu_perception.protocol import validate_request


class RgbdTest(unittest.TestCase):
    def test_transport_shape_and_backprojection(self):
        payload,rows,scales=Preprocessor().prepare(np.zeros((720,1280,3),np.uint8))
        validate_request(dict(model='fcos',sequence=1,stamp_ns=10,width=1280,height=720,rows=rows),payload)
        result=dict(model_sha256=FCOS_SHA,motion_eligible=False,detections=[
            dict(class_id=0,confidence=.8,bbox_model_px=[70,70,140,140])])
        boxes=source_boxes(result,1280,720,scales,{0:'person'})
        np.testing.assert_allclose(boxes[0]['bbox'],[100,100,200,200])

    def test_wrong_model_or_nan_box_rejected(self):
        result=dict(model_sha256='wrong',motion_eligible=False,detections=[])
        with self.assertRaises(ValueError):source_boxes(result,1280,720,(.7,.7),{0:'person'})
        result.update(model_sha256=FCOS_SHA,detections=[dict(class_id=0,confidence=.8,bbox_model_px=[0,0,np.nan,10])])
        with self.assertRaises(ValueError):source_boxes(result,1280,720,(.7,.7),{0:'person'})

    def test_foreground_depth_in_background_box(self):
        depth=np.full((100,100),5,np.float32);depth[30:70,30:70]=2
        estimate=estimate_boxes([dict(bbox=[20,20,80,80])],depth,[100,0,50,0,100,50,0,0,1])[0]
        self.assertTrue(estimate.valid);self.assertAlmostEqual(estimate.depth_m,2)
        self.assertGreaterEqual(estimate.sample_count,24)
        self.assertAlmostEqual(estimate.camera_point[2],2)

    def test_invalid_depth_has_no_metric_position(self):
        estimate=estimate_boxes([dict(bbox=[0,0,100,100])],np.full((100,100),np.nan,np.float32),
            [100,0,50,0,100,50,0,0,1])[0]
        self.assertFalse(estimate.valid)

    def test_large_roi_sampling_preserves_camera_coordinates(self):
        depth=np.full((720,1280),5,np.float32);depth[260:540,660:940]=3
        result=estimate_boxes([dict(bbox=[600,200,1000,600])],depth,[1000,0,640,0,1000,360,0,0,1])[0]
        self.assertTrue(result.valid)
        # Grid quantization is bounded by the four-source-pixel stride.
        self.assertAlmostEqual(result.camera_point[0],.48,delta=.012)
        self.assertAlmostEqual(result.camera_point[1],.12,delta=.012)
        self.assertAlmostEqual(result.camera_point[2],3)


if __name__=='__main__':unittest.main()
