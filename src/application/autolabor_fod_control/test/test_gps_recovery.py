#!/usr/bin/env python3
import math
from dataclasses import replace
import unittest

from autolabor_fod_control.gps_recovery import FodGpsMemory, depth_point_in_world, local_to_gps, inside_vehicle_footprint, depth_steering_curvature
from autolabor_fod_control.visual_servo import PixelDetection
from gps_module.long_range import gps_to_xy, rotate_xy


def detection(identity=1, distance=4.0):
    return PixelDetection(0, "Metal", .9, 610, 350, 40, 20, 630, 370,
                          track_id=identity, depth_valid=True, depth_m=distance, range_m=distance)


class GpsMemoryTest(unittest.TestCase):
    def test_identity_and_material_change_reconnect_without_moving_gps_endpoint(self):
        m = FodGpsMemory(frames=3)
        for i in range(3):
            m.observe(1+i*.05, [(detection(), (4,0,0)),
                               (detection(2,4.1), (4.1,.5,0))], (0,0))
        old = m.snapshot(30,104)
        # A known neighbour cannot inherit the selected target's identity.
        self.assertFalse(m.match_visible(1.15, [detection(2,4.1)]))
        self.assertTrue(m.recently_visible(1.2,.30))
        self.assertFalse(m.recently_visible(1.5,.30))
        alias = replace(detection(77,3.9), class_id=2, class_name='Plastic',
                        x=615,anchor_u=635)
        self.assertTrue(m.match_visible(1.3,[alias,detection(2,4.1)]))
        new = m.snapshot(30,104)
        self.assertEqual(new['track_id'],77)
        for key in ['fod_x','fod_y','goal_x','goal_y','sample_stamp']:
            self.assertEqual(new[key],old[key])

    def test_closer_stable_depth_corrects_initial_point_but_cannot_drift_away(self):
        m = FodGpsMemory(frames=3)
        for i in range(3):
            m.observe(1+i*.05, [(detection(distance=4.5),(4.1,.2,0))],(0,0))
        for i in range(3):
            m.observe(2+i*.05, [(detection(distance=2.0),(4.0,.02,0))],(2,0))
        self.assertEqual(m.point,(4.0,.02,0))
        self.assertAlmostEqual(m.snapshot(30,104)['sample_stamp'],2.1)
        # Reused ID on the chassis and an unbounded series of depth drifts.
        for i in range(30):
            m.observe(3+i*.05, [(detection(distance=1.5-i*.01),(5+i*.02,2,0))],(3,0))
        self.assertEqual(m.point,(4.0,.02,0))

    def test_depth_steering_crosses_offset_fod_with_short_detector_gaps(self):
        projection=[525.,0,635.,0,0,525.,366.,0,0,0,1,0]
        optical=(-.5,.5,-.5,.5)
        for offset in (-1.,1.):
            x,y,yaw=0.,0.,0.
            closest=99.;command=0.
            for step in range(700):
                dx,dy=5.-x,offset-y
                forward=math.cos(yaw)*dx+math.sin(yaw)*dy
                lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
                closest=min(closest,math.hypot(dx,dy))
                if forward<.25:break
                u=635.-525.*lateral/forward
                d=replace(detection(distance=forward),x=u-20,anchor_u=u)
                # 0.4-second missed detections while crossing a ground seam.
                if not 120<=step<140:
                    command,_=depth_steering_curvature(d,projection,(0,0,0),optical,.7)
                yaw += .6*command*.02
                x += .6*math.cos(yaw)*.02
                y += .6*math.sin(yaw)*.02
            self.assertLess(abs(lateral),.08)
            self.assertLess(closest,.27)

    def test_depth_steering_accounts_for_camera_offset_from_chassis_centre(self):
        projection=[525.,0,635.,0,0,525.,366.,0,0,0,1,0]
        for depth in (1.,2.,5.):
            # The left lens sits 2.5 cm RIGHT of centre; the centred ground
            # point projects left of the principal point by a depth-dependent amount.
            u=635.-525.*.025/depth
            d=replace(detection(distance=depth),x=u-20,anchor_u=u)
            curvature,lateral=depth_steering_curvature(d,projection,(0,-.025,0),(-.5,.5,-.5,.5),.7)
            self.assertAlmostEqual(curvature,0.,places=6)
            self.assertAlmostEqual(lateral,0.,places=6)

    def test_sweeper_and_chassis_are_excluded_but_ground_ahead_is_eligible(self):
        footprint = [[.8,.4],[.8,-.4],[-.8,-.4],[-.8,.4]]
        self.assertTrue(inside_vehicle_footprint((.31,.04,0),footprint))
        self.assertTrue(inside_vehicle_footprint((.8,-.4,0),footprint))
        self.assertFalse(inside_vehicle_footprint((.81,.0,0),footprint))
        self.assertFalse(inside_vehicle_footprint((.4,.41,0),footprint))

    def test_slow_drift_cannot_move_original_fod_or_crossing_endpoint(self):
        memory = FodGpsMemory()
        for i in range(6):
            memory.observe(1+i*.05,[(detection(),(4,0,0))],(0,0))
        original = memory.snapshot(30,104)
        for i in range(1,300):
            memory.observe(2+i*.05,[(detection(),(4+i*.025,0,0))],(i*.025,0))
        self.assertEqual(memory.snapshot(30,104),original)

    def test_projection_uses_depth_z_and_box_centre_not_bottom_anchor_or_range(self):
        p = [500, 0, 630, 0, 0, 500, 360, 0, 0, 0, 1, 0]
        d = replace(detection(), range_m=5.)
        # Optical axes -> x-forward, y-left, z-up, and a translated camera.
        world = depth_point_in_world(d, p, (10,20,.5), (-.5,.5,-.5,.5))
        self.assertAlmostEqual(world[0], 14.)
        self.assertAlmostEqual(world[1], 20.)
        self.assertAlmostEqual(world[2], .5)

    def test_inverse_matches_actual_gps_manager_at_multiple_headings(self):
        for yaw in (0, math.pi/2, -.8, math.pi):
            lat, lon = local_to_gps(50, -21, 30.3007, 104.3898, yaw)
            xy = rotate_xy(*gps_to_xy(lat, lon, 30.3007, 104.3898), yaw)
            self.assertAlmostEqual(xy[0], 50., places=6)
            self.assertAlmostEqual(xy[1], -21., places=6)

    def test_six_distinct_stable_frames_nearest_only_and_crossing_from_fod(self):
        m = FodGpsMemory()
        for i in range(6):
            m.observe(1+i*.1, [(detection(3, 5), (5,1,0)),
                              (detection(1, 4), (4,0,0)),
                              (detection(2, 4.5), (4.5,-1,0))], (0,0))
            if i < 5:
                self.assertIsNone(m.point)
        self.assertEqual(m.track_id, 1)
        target = m.snapshot(30,104)
        self.assertAlmostEqual(target['goal_x'], 4.8)
        self.assertAlmostEqual(target['goal_y'], 0.)
        for i in range(10):
            m.observe(2+i*.1, [(detection(2,.2), (.2,-1,0))], (3.5,0))
        self.assertEqual(m.track_id, 1)
        self.assertEqual(m.snapshot(30,104)['fod_latitude'], target['fod_latitude'])
        self.assertAlmostEqual(m.snapshot(30,104)['goal_x'], 4.8)

    def test_no_lock_for_bad_depth_far_target_or_unstable_world_point(self):
        for name in ('far', 'invalid', 'unstable', 'duplicate_stamp'):
            with self.subTest(name=name):
                m = FodGpsMemory()
                for i in range(12):
                    d = detection(distance=6.01 if name == 'far' else 4.)
                    if name == 'invalid':
                        d = replace(d, depth_valid=False)
                    m.observe(1 if name == 'duplicate_stamp' else 1+i*.1,
                              [(d, (4, i%2 if name == 'unstable' else 0, 0))], (0,0))
                self.assertIsNone(m.point)

    def test_motion_compensation_same_world_point_and_no_switch_after_loss(self):
        m = FodGpsMemory()
        for i in range(6):
            m.observe(1+i*.1, [(detection(distance=5-i*.1), (5,2,0))], (i*.1,2))
        first = m.snapshot(30,104)
        for i in range(6):
            m.observe(2+i*.1, [(detection(1, .5), (12,8,0))], (4.5,2))
        self.assertEqual(m.point, (5,2,0))
        m.frozen = True
        m.observe(4, [(detection(), (5.1,2,0))], (5,2))
        self.assertEqual(m.snapshot(30,104)['goal_x'], first['goal_x'])

    def test_gap_before_stability_requires_new_consecutive_observations(self):
        m = FodGpsMemory()
        for i in range(5):
            m.observe(1+i*.1, [(detection(), (4,0,0))], (0,0))
        m.observe(2, [], (0,0))
        m.observe(3, [(detection(), (4,0,0))], (0,0))
        self.assertIsNone(m.point)
        self.assertEqual(len(m.samples), 1)


if __name__ == '__main__':
    unittest.main()
