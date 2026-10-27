#!/usr/bin/env python3
import base64
import math
import unittest
import zlib

from autolabor_fod_control.bev_cleaning import (
    VisibleTargets, forward_crossing, ground_objects, sweep_candidates, transform_point,
)
from autolabor_fod_control.cleaning_geometry import footprint_is_free, map_target_to_wheel_odom
from types import SimpleNamespace as NS


class BevGeometryTest(unittest.TestCase):
    def metadata(self, supported=True):
        mask = bytes([int(supported)]*400)
        return dict(dimensions=[20, 20], metres_per_pixel=.1,
                    x_range_m=[-1, 1], y_range_m=[-1, 1],
                    ground_support_zlib_b64=base64.b64encode(zlib.compress(mask)).decode())

    def box(self, bbox=None):
        return dict(bbox=bbox or [4, 2, 6, 4], confidence=.8, class_name='paper')

    def test_metric_axes_and_half_pixel_match_ground_renderer(self):
        point = ground_objects(self.metadata(), [self.box()])[0]
        self.assertAlmostEqual(.65, point['x'])
        self.assertAlmostEqual(.45, point['y'])
        self.assertEqual('paper', point['label'])

    def test_black_unobserved_region_does_not_create_a_navigation_target(self):
        self.assertEqual([], ground_objects(self.metadata(False), [self.box()]))

    def test_bad_detection_cannot_escape_extent(self):
        for bbox in ([-1, 2, 6, 4], [4, 2, 6, 21], [4, 2, 4, 4], [math.nan, 2, 6, 4]):
            self.assertEqual([], ground_objects(self.metadata(), [self.box(bbox)]))

    def test_corrupt_or_oversized_mask_is_rejected(self):
        for data in (b'', bytes(401), bytes(400)+b'trailer'):
            meta = self.metadata()
            meta['ground_support_zlib_b64'] = base64.b64encode(zlib.compress(data)).decode()
            with self.assertRaises(ValueError):
                ground_objects(meta, [self.box()])

    def test_base_extrinsic_and_map_rotation_are_composed_in_order(self):
        base = (10, 20, math.pi/2)
        origin = transform_point(base, (.4, -.1))
        point = transform_point((*origin, base[2]+math.pi/2), (2, 0))
        self.assertAlmostEqual(8.1, point[0])
        self.assertAlmostEqual(20.4, point[1])

    def test_every_sweep_passes_target_forward_with_offset(self):
        for robot in ((0,0,0), (3,4,math.pi)):
            for start, end in sweep_candidates(robot, (2,1)):
                n=30
                poses=[(start[0]+(end[0]-start[0])*i/n,
                        start[1]+(end[1]-start[1])*i/n,start[2]) for i in range(n+1)]
                self.assertTrue(any(forward_crossing(a,b,(2,1)) for a,b in zip(poses,poses[1:])))

    def test_goal_success_at_wrong_position_cannot_count_as_swept(self):
        self.assertFalse(forward_crossing((0,0,0), (.1,0,0), (1,0)))
        self.assertFalse(forward_crossing((.4,.3,0), (.5,.3,0), (1,0)))

    def test_reverse_spin_and_localization_jump_cannot_complete(self):
        for previous,current in (((.5,0,0),(.4,0,0)),
                                 ((.48,0,-.2),(.48,0,.2)), ((0,0,0),(1,0,0))):
            self.assertFalse(forward_crossing(previous,current,(1,0)))

    def test_short_true_forward_reference_crossing_completes(self):
        self.assertTrue(forward_crossing((.4,0,0),(.55,0,0),(1,0)))

    def test_current_visible_points_merge_and_expire_without_map_archive(self):
        registry = VisibleTargets()
        obs = lambda x: dict(x=x,y=0,label='plastic')
        registry.update([obs(1)], 10)
        registry.update([obs(1.03)], 10.1)
        self.assertEqual([1], [t.identity for t in registry.visible(10.2)])
        registry.update([], 13)
        self.assertEqual([], registry.visible(13))

    def test_confirmed_pass_is_not_queued_again_during_same_round(self):
        registry = VisibleTargets()
        obs = [dict(x=1,y=0,label='paper')]
        registry.update(obs, 10)
        registry.complete(registry.visible(10)[0])
        registry.update(obs, 10.2)
        self.assertEqual([], registry.visible(10.2))

    def test_overlapping_camera_boxes_form_one_cleaning_location(self):
        registry = VisibleTargets()
        registry.update([dict(x=1,y=0,label='paper'),dict(x=1.04,y=0,label='plastic')],10)
        self.assertEqual(1,len(registry.visible(10)))

    def test_full_footprint_rejects_obstacle_inside_body(self):
        info = NS(resolution=.1, width=40, height=40,
                  origin=NS(position=NS(x=0,y=0), orientation=NS(x=0,y=0,z=0,w=1)))
        grid = NS(info=info, data=[0]*1600)
        shape=[(-.5,-.3),(.5,-.3),(.5,.3),(-.5,.3)]
        self.assertTrue(footprint_is_free(grid,(2,2,0),shape))
        grid.data[20*40+24]=100
        self.assertFalse(footprint_is_free(grid,(2,2,0),shape))

    def test_existing_visual_controller_wheel_frame_conversion_is_preserved(self):
        self.assertEqual((3.,4.), map_target_to_wheel_odom(0,0,0,1,2,2,2,0))


if __name__ == '__main__':
    unittest.main()
