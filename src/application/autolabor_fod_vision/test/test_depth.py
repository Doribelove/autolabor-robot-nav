#!/usr/bin/env python3
from dataclasses import replace
import math
import unittest
import numpy as np

from autolabor_fod_vision.depth import RegisteredDepthFusion, DetectionTracker
from autolabor_fod_vision.detector import Detection


def box(x=40, distance=2.0, confidence=0.6):
    return Detection(0, "Metal", confidence, x, 30, x+20, 50, x+10, 50,
                     depth_valid=True, depth_m=distance, range_m=distance)


class RegisteredDepthTest(unittest.TestCase):
    def fusion(self, data, stamp=10.0, frame="left", encoding="32FC1"):
        fusion = RegisteredDepthFusion()
        fusion.set_camera(10.0, "left", 100, 80, [100,0,50,0,0,100,40,0,0,0,1,0])
        fusion.add_depth(stamp, frame, data, encoding)
        return fusion

    def test_median_rejects_nan_inf_zero_and_outlier(self):
        depth = np.full((80,100), 2.0, np.float32)
        depth[35:45,45] = [np.nan,np.inf,0,0,100,100,0,0,np.nan,0]
        result = self.fusion(depth).enrich([box()],10,"left",100,80)[0]
        self.assertTrue(result.depth_valid)
        self.assertAlmostEqual(result.depth_m,2)
        self.assertAlmostEqual(result.range_m,2)

    def test_nearest_depth_frame_and_mm_units_and_resized_registered_depth(self):
        fusion = self.fusion(np.full((40,50),2000,np.uint16),encoding="16UC1")
        fusion.add_depth(9.96,"left",np.full((40,50),9.0,np.float32),"32FC1")
        result=fusion.enrich([box(60)],10,"left",100,80)[0]
        self.assertAlmostEqual(result.depth_m,2)
        self.assertAlmostEqual(result.range_m,2*math.sqrt(1.04))

    def test_missing_invalid_unsynchronized_or_wrong_camera_never_means_near(self):
        for array,stamp,frame in [(np.zeros((80,100)),10,"left"),
                                  (np.full((80,100),np.nan),10,"left"),
                                  (np.ones((80,100)),9.8,"left"),
                                  (np.ones((80,100)),10,"right"),
                                  (np.ones((80,80)),10,"left")]:
            with self.subTest(stamp=stamp,frame=frame,shape=array.shape):
                d=self.fusion(array,stamp,frame).enrich([box()],10,"left",100,80)[0]
                self.assertFalse(d.depth_valid)
                self.assertEqual(d.range_m,0)
        d=RegisteredDepthFusion().enrich([box()],10,"left",100,80)[0]
        self.assertFalse(d.depth_valid)

    def test_box_sampling_does_not_use_excluded_image_rows_or_other_objects(self):
        depth=np.full((80,100),20.0,np.float32)
        depth[30:50,5:25]=3
        depth[30:50,40:60]=1
        depth[30:50,75:95]=2
        result=self.fusion(depth).enrich([box(5),box(40),box(75)],10,"left",100,80)
        self.assertEqual([d.depth_m for d in result],[3,1,2])


class DetectionIdentityTest(unittest.TestCase):
    def test_three_objects_keep_ids_despite_confidence_order_and_small_motion(self):
        tracker=DetectionTracker()
        first=tracker.update([box(5,3),box(40,1),box(75,2)],10,(100,80))
        second=tracker.update([box(77,2.02,.99),box(7,3.01,.4),box(42,1.01,.3)],10.03,(100,80))
        self.assertEqual([d.track_id for d in second],
                         [first[2].track_id,first[0].track_id,first[1].track_id])

    def test_depth_separates_overlapping_objects(self):
        tracker=DetectionTracker()
        first=tracker.update([box(40,1),box(40,3)],10,(100,80))
        second=tracker.update([box(40,3),box(40,1)],10.03,(100,80))
        self.assertEqual([d.track_id for d in second],[first[1].track_id,first[0].track_id])

    def test_missing_selected_object_does_not_transfer_its_id_to_bystander(self):
        tracker=DetectionTracker()
        first=tracker.update([box(40,1),box(60,2)],10,(100,80))
        second=tracker.update([box(58,2)],10.03,(100,80))
        self.assertEqual(second[0].track_id,first[1].track_id)
        self.assertEqual(tracker.update([],10.06,(100,80)),[])
        back=tracker.update([box(42,1)],10.2,(100,80))
        self.assertEqual(back[0].track_id,first[0].track_id)

    def test_depth_hole_keeps_identity_without_inventing_a_current_measurement(self):
        tracker=DetectionTracker()
        first=tracker.update([box()],10,(100,80))[0]
        second=tracker.update([replace(box(),depth_valid=False,depth_m=0,range_m=0)],10.03,(100,80))[0]
        self.assertEqual(first.track_id,second.track_id)
        self.assertFalse(second.depth_valid)
        self.assertEqual(second.range_m,0)
        later=tracker.update([box()],12.1,(100,80))[0]
        self.assertNotEqual(first.track_id,later.track_id)


if __name__ == '__main__':
    unittest.main()
