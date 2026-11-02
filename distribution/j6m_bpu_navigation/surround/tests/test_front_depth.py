import json
from pathlib import Path
import unittest

import numpy as np

from surround.ground_bev import GroundBev
from surround.front_depth import FrontDepthRenderer, encode_rgbd, decode_rgbd


class FrontDepthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config=json.loads((Path(__file__).resolve().parents[4]/'config/four_camera_bev.json').read_text())
        cls.bev=GroundBev(config)
        cls.renderer=FrontDepthRenderer(cls.bev)

    def sample(self):
        return np.zeros((720,1280,3),np.uint8),np.zeros((720,1280),np.uint16),np.full((600,600,3),17,np.uint8)

    def test_vertical_object_stays_at_stereo_range_instead_of_ground_ray_distance(self):
        image,depth,base=self.sample()
        image[310:470,625:645]=[20,90,200];depth[310:470,625:645]=1500
        result,front,stats=self.renderer.compose(image,depth,base,{})
        rows,cols=np.nonzero(front.any(axis=2))
        self.assertGreater(len(rows),0)
        # Every height on this vertical surface has X = camera_x + 1.5m.
        expected=self.bev.pixel(.96+1.5,0)[1]
        self.assertLessEqual(abs(float(np.median(rows))-expected),1.)
        self.assertLessEqual(rows.max()-rows.min(),2)
        np.testing.assert_array_equal(result[~self.renderer.region],base[~self.renderer.region])
        self.assertFalse(stats['hidden_ground_reconstructed'])

    def test_top_down_z_buffer_keeps_highest_observed_surface(self):
        image,depth,base=self.sample()
        image[300,635]=[0,0,255];image[450,635]=[255,0,0]
        depth[300,635]=depth[450,635]=1500
        _,front,_=self.renderer.compose(image,depth,base,{})
        self.assertTrue((front[:,:,2]>0).any())
        self.assertFalse((front[:,:,0]>0).any())

    def test_invalid_depth_leaves_unknown_front_without_hiding_other_cameras(self):
        image,depth,base=self.sample()
        result,front,stats=self.renderer.compose(image,depth,base,{})
        self.assertFalse(front.any());self.assertFalse(result[self.renderer.region].any())
        np.testing.assert_array_equal(result[~self.renderer.region],base[~self.renderer.region])
        self.assertEqual(stats['rendered_cells'],0)

    def test_native_raster_matches_reference_on_depth_edges_and_occlusion(self):
        if self.renderer.native is None:self.skipTest('Native renderer not built')
        image,depth,base=self.sample()
        image[300:500,570:700]=[30,80,160];depth[300:500,570:700]=1500
        image[350:390,600:650]=[180,90,20];depth[350:390,600:650]=1200
        reference=FrontDepthRenderer(self.bev,use_native=False)
        expected,_,_=reference.compose(image,depth,base,{})
        actual,_,_=self.renderer.compose(image,depth,base,{})
        np.testing.assert_array_equal(actual,expected)

    def test_rgbd_transport_preserves_mm_capture_identity_and_rejects_corruption(self):
        image,depth,_=self.sample();depth[350:450,600:700]=1234
        source=dict(session='zed-test',sequence=7,source_stamp_ns=1000_000_000_000,
                    K_rectified=self.bev.intrinsics['front'].tolist())
        packet=encode_rgbd(image,depth,source)
        rgb,decoded,meta=decode_rgbd(packet,now=1000.1)
        np.testing.assert_array_equal(decoded,depth);self.assertEqual(meta,source)
        self.assertEqual(rgb.shape,image.shape)
        damaged=bytearray(packet);damaged[-5]^=1
        with self.assertRaisesRegex(ValueError,'digest'):decode_rgbd(damaged,now=1000.1)
        with self.assertRaises(ValueError):decode_rgbd(packet[:-1],now=1000.1)
        with self.assertRaisesRegex(ValueError,'Stale'):decode_rgbd(packet,now=1002.)


if __name__=='__main__':unittest.main()
