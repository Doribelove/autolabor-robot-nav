import copy
import math
import unittest
import numpy as np
from autolabor_bpu_perception.core import fresh, rotation_rpy, zed_mount, ImageTracker, obstacle_polygon


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.config=dict(confirmed=True,reference='left_camera',mid360_in_base_m=[.2,0,1],
            forward_m=.42,right_m=.14,up_m=0,pitch_down_deg=41,yaw_left_deg=0,roll_deg=0)

    def test_measured_lens_pose_survives_actual_urdf_chain(self):
        mount=zed_mount(self.config)
        rb=rotation_rpy(*mount['base_rpy']);rc=rb@rotation_rpy(0,.05,0)
        lens=mount['base_position']+rb@np.array([0,0,.015])+rc@np.array([-.01,.06,0])
        np.testing.assert_allclose(lens,[.62,-.14,1],atol=1e-12)
        optical=rc@rotation_rpy(-math.pi/2,0,-math.pi/2)
        np.testing.assert_allclose(optical[:,2],[math.cos(math.radians(41)),0,-math.sin(math.radians(41))],atol=1e-12)
        np.testing.assert_allclose(optical,mount['optical_to_base'],atol=1e-12)
        self.assertAlmostEqual(mount['base_rpy'][1],math.radians(41)-.05)

    def test_incomplete_mount_rejected(self):
        for key,value in [('confirmed',False),('up_m',None),('pitch_down_deg',float('nan')),('reference','mounting_base')]:
            with self.subTest(key=key), self.assertRaises(ValueError):zed_mount(dict(self.config,**{key:value}))

    def test_time_boundaries(self):
        for stamp in [0,9,10.1,float('nan')]:self.assertFalse(fresh(stamp,10))
        self.assertTrue(fresh(9.8,10));self.assertTrue(fresh(10.01,10))

    def test_tracking_identity_and_no_stale_output(self):
        tracker=ImageTracker();one=dict(class_id=1,bbox=[0,0,20,20])
        first=tracker.update([one],10)[0]['object_id']
        second=tracker.update([one,dict(one,bbox=[1,0,21,20])],10.1)
        self.assertEqual(second[0]['object_id'],first)
        self.assertEqual(len(set(d['object_id'] for d in second)),2)
        self.assertEqual(tracker.update([],10.2),[])
        self.assertNotEqual(tracker.update([one],11)[0]['object_id'],first)
        with self.assertRaises(ValueError):tracker.update([one],10.9)

    def test_rotated_polygon_and_invalid_geometry(self):
        polygon=obstacle_polygon([1,2,0],[2,1,1],math.pi/2,padding=0)
        np.testing.assert_allclose(polygon.min(axis=0),[.5,1],atol=1e-12)
        np.testing.assert_allclose(polygon.max(axis=0),[1.5,3],atol=1e-12)
        with self.assertRaises(ValueError):obstacle_polygon([1,2,0],[2,-1,1],0)


if __name__=='__main__':unittest.main()
