import copy
import math
from pathlib import Path
import unittest

import numpy as np
import yaml

from autolabor_bpu_perception.core import rotation_rpy, zed_mount
from autolabor_fod_vision.map_localization import (
    MapLocalizer, MapTransform, current_map_point, current_local_point, localization_ready,
    rectified_intrinsics, rigid_matrix,
)
from autolabor_fod_vision.two_stage import DepthClusterEstimate, estimate_clustered_depth
from autolabor_fod_vision.landmark_stability import LandmarkStability


class LandmarkStabilityTest(unittest.TestCase):
    def test_five_correlated_frames_cannot_confirm_map(self):
        gate = LandmarkStability()
        for i in range(5): gate.observe(1+i*.125, np.eye(4))
        self.assertIsNone(gate.matrix)
        for i in range(5, 49): gate.observe(1+i*.125, np.eye(4))
        self.assertEqual(gate.status, 'STABLE')

    def test_normal_correction_jitter_is_filtered_but_jump_restarts_window(self):
        gate = LandmarkStability()
        for i in range(65):
            matrix = np.eye(4)
            matrix[0, 3] = .03 if (i//16) % 2 else -.03
            gate.observe(1+i*.125, matrix)
        self.assertEqual(gate.status, 'STABLE')
        matrix[1, 3] = 1.8
        gate.observe(9.125, matrix)
        self.assertIsNone(gate.matrix)
        self.assertEqual(gate.status, 'MAP_CORRECTION_JUMP')
        self.assertGreater(gate.generation, 0)
        for i in range(1, 49): gate.observe(9.125+i*.125, matrix)
        np.testing.assert_allclose(gate.matrix, matrix)

    def test_rotation_gap_loss_and_clock_reset_suspend_confirmation(self):
        for reason in ('rotation', 'gap', 'loss', 'clock'):
            gate = LandmarkStability()
            for i in range(49): gate.observe(1+i*.125, np.eye(4))
            if reason == 'rotation':
                matrix = np.eye(4)
                matrix[:3, :3] = rotation_rpy(0, 0, math.radians(5))
                gate.observe(7.125, matrix)
            elif reason == 'gap': gate.observe(8, np.eye(4))
            elif reason == 'clock': gate.observe(2, np.eye(4))
            else: gate.invalidate('LOST')
            self.assertIsNone(gate.matrix, reason)

    def test_nonrigid_correction_rejected(self):
        matrix = np.eye(4); matrix[0, 0] = 2
        with self.assertRaises(ValueError): LandmarkStability().observe(1, matrix)


class MapLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((Path(__file__).resolve().parents[2] /
            'autolabor_bpu_perception/config/zed_mount.yaml').read_text())
        self.localizer = MapLocalizer(self.config)
        self.map_base = np.eye(4)

    def lookup(self, target, source):
        if (target, source) == ('base_link', 'zed2_left_camera_optical_frame'):
            return self.localizer.expected.copy()
        self.assertEqual((target, source), ('map', 'base_link'))
        return self.map_base.copy()

    def test_user_extrinsics_axis_signs_and_no_double_stereo_baseline(self):
        transform = self.localizer.transform('zed2_left_camera_optical_frame', self.lookup)
        point = current_map_point(DepthClusterEstimate(True, camera_point=(0, 0, 1)), transform)
        a = math.radians(42)
        np.testing.assert_allclose(point, [.62 + math.cos(a), -.13, .97-math.sin(a)], atol=1e-12)
        self.assertEqual(self.config['pitch_down_deg'], 42)

    def test_map_translation_yaw_and_full_height(self):
        self.map_base[:3, :3] = rotation_rpy(0, 0, math.pi/2)
        self.map_base[:3, 3] = [10, 20, .3]
        transform = self.localizer.transform('zed2_left_camera_optical_frame', self.lookup)
        point = current_map_point(DepthClusterEstimate(True, camera_point=(0, 0, 1)), transform)
        np.testing.assert_allclose(point, [10.13,20.62+math.cos(math.radians(42)),1.27-math.sin(math.radians(42))])

    def test_local_coordinates_need_only_calibrated_camera_to_vehicle(self):
        calls = []
        def lookup(target, source):
            calls.append((target, source))
            if target == 'map':
                raise LookupError('map unavailable')
            return self.localizer.expected.copy()
        transform = self.localizer.radar_transform(
            self.localizer.base_transform('zed2_left_camera_optical_frame', lookup))
        point = current_local_point(DepthClusterEstimate(True, camera_point=(0, 0, 1)), transform)
        angle = math.radians(42)
        np.testing.assert_allclose(point, [.42 + math.cos(angle), -.13])
        self.assertEqual(transform.target_frame, 'mid360')
        self.assertEqual(point.shape, (2,))
        self.assertEqual(calls, [('base_link', 'zed2_left_camera_optical_frame')])
        self.assertIsNone(current_map_point(DepthClusterEstimate(True, camera_point=(0, 0, 1)), transform))

    def test_map_and_local_coordinates_represent_the_same_foreground_point(self):
        estimate = DepthClusterEstimate(True, camera_point=(.2, .1, 1.4))
        base_transform = self.localizer.base_transform('zed2_left_camera_optical_frame', self.lookup)
        radar_transform = self.localizer.radar_transform(base_transform)
        local = current_local_point(estimate, radar_transform)
        for yaw in [-1.2, 0, 2.3]:
            self.map_base[:3, :3] = rotation_rpy(.03, -.05, yaw)
            self.map_base[:3, 3] = [3, -2, .1]
            world = current_map_point(estimate, self.localizer.map_transform(base_transform, self.lookup))
            base_point = np.linalg.inv(self.map_base) @ np.r_[world, 1]
            np.testing.assert_allclose(local, base_point[:2] - self.localizer.mid360_in_base[:2])
            np.testing.assert_allclose(current_local_point(estimate, radar_transform), local)

    def test_radar_coordinates_do_not_depend_on_navigation_base_origin(self):
        estimate = DepthClusterEstimate(True, camera_point=(.2, .1, 1.4))
        points = []
        for offset in ([.2, 0, 1], [9, -3, .8]):
            config = copy.deepcopy(self.config)
            config['mid360_in_base_m'] = offset
            localizer = MapLocalizer(config)
            transform = localizer.radar_transform(MapTransform(localizer.expected, 'base_link'))
            points.append(current_local_point(estimate, transform))
        np.testing.assert_allclose(points[0], points[1])

    def test_new_camera_translation_affects_global_xy_under_vehicle_tilt(self):
        old = copy.deepcopy(self.config)
        old.update(right_m=.14, up_m=0.)
        previous = MapLocalizer(old)
        self.map_base[:3, :3] = rotation_rpy(.1, .2, .3)
        estimate = DepthClusterEstimate(True, camera_point=(.2, .1, 1.4))
        current = current_map_point(estimate, self.localizer.map_transform(
            MapTransform(self.localizer.expected, 'base_link'), self.lookup))
        before = current_map_point(estimate, previous.map_transform(
            MapTransform(previous.expected, 'base_link'), self.lookup))
        np.testing.assert_allclose(current - before, self.map_base[:3, :3] @ [0, .01, -.03])

    def test_pixel_depth_to_map_roundtrip_at_several_robot_poses(self):
        k = [400,0,160,0,410,120,0,0,1]
        for yaw in [-1.2, 0, 2.3]:
            self.map_base[:3,:3] = rotation_rpy(.03, -.05, yaw)
            self.map_base[:3,3] = [3,-2,.1]
            transform = self.localizer.transform('zed2_left_camera_optical_frame', self.lookup)
            camera = np.array([.2, .1, 1.4])
            u, v = k[0]*camera[0]/camera[2]+k[2], k[4]*camera[1]/camera[2]+k[5]
            reconstructed = ((u-k[2])*camera[2]/k[0], (v-k[5])*camera[2]/k[4], camera[2])
            world = current_map_point(DepthClusterEstimate(True,camera_point=reconstructed), transform)
            np.testing.assert_allclose(np.linalg.inv(transform.matrix) @ np.r_[world,1], np.r_[camera,1])

    def test_actual_foreground_cluster_uses_selected_pixels_and_depth(self):
        depth = np.full((120,160),2.5,dtype=np.float32)
        depth[42:78,58:103] = 1.2
        estimate = estimate_clustered_depth(depth,(48,30,115,90),[100,0,80,0,100,60,0,0,1])
        self.assertTrue(estimate.valid,estimate.reason)
        self.assertAlmostEqual(estimate.depth_m,1.2,places=5)
        point = current_map_point(estimate, self.localizer.transform('zed2_left_camera_optical_frame',self.lookup))
        recovered = np.linalg.inv(self.localizer.expected) @ np.r_[point,1]
        np.testing.assert_allclose(recovered[:3],estimate.camera_point,atol=1e-12)

    def test_old_41_degree_mount_is_rejected(self):
        old = copy.deepcopy(self.config);old['pitch_down_deg']=41
        old_mount = MapLocalizer(old)
        with self.assertRaisesRegex(ValueError,'EXTRINSIC_MISMATCH'):
            self.localizer.transform('zed2_left_camera_optical_frame',lambda *_:old_mount.expected)

    def test_shifted_default_mount_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'EXTRINSIC_MISMATCH'):
            self.localizer.transform('zed2_left_camera_optical_frame',lambda *_:np.eye(4))

    def test_non_left_frame_is_rejected(self):
        for frame in ['', 'map', 'zed2_right_camera_optical_frame']:
            with self.assertRaisesRegex(ValueError,'UNSUPPORTED_CAMERA_FRAME'):
                self.localizer.transform(frame,self.lookup)

    def test_missing_map_does_not_use_odom(self):
        calls=[]
        def lookup(target, source):
            calls.append(target)
            if target=='map':raise LookupError('map unavailable')
            return self.localizer.expected
        with self.assertRaises(LookupError):
            self.localizer.transform('zed2_left_camera_optical_frame',lookup)
        self.assertEqual(calls,['base_link','map'])

    def test_nonfinite_and_nonrigid_tf_rejected(self):
        for value in [float('nan'),2,-1]:
            matrix=np.eye(4);matrix[0,0]=value
            with self.assertRaises(ValueError):rigid_matrix(matrix)

    def test_invalid_depth_and_wrong_frame_have_no_position(self):
        self.assertIsNone(current_map_point(DepthClusterEstimate(False),MapTransform(np.eye(4))))
        for point in [(0,0,float('nan')),(0,0,0),(0,0,-1)]:
            self.assertIsNone(current_map_point(DepthClusterEstimate(True,camera_point=point),MapTransform(np.eye(4))))
        estimate=DepthClusterEstimate(True,camera_point=(1,2,3))
        self.assertIsNone(current_map_point(estimate,MapTransform(np.eye(4),'odom')))
        self.assertIsNone(current_map_point(estimate,None))
        self.assertIsNone(current_local_point(estimate, MapTransform(np.eye(4), 'map')))
        self.assertIsNone(current_local_point(estimate, MapTransform(np.eye(4), 'base_link')))
        self.assertIsNone(current_local_point(DepthClusterEstimate(False), MapTransform(np.eye(4), 'mid360')))
        self.assertIsNone(current_local_point(DepthClusterEstimate(True, camera_point=(0, 0, float('nan'))),
                                             MapTransform(np.eye(4), 'mid360')))

    def test_fresh_localization_required(self):
        self.assertTrue(localization_ready('state=LOCALIZED;overlap=0.99',10,10.1))
        for status,now in [('state=LOST;',10.1),('state=LOCALIZED;',10.51),('state=LOCALIZED;',9),('',10.1)]:
            self.assertFalse(localization_ready(status,10,now))

    def test_rectified_left_projection_matrix(self):
        p=[400,0,320,0,0,401,240,0,0,0,1,0]
        self.assertEqual(rectified_intrinsics(p),[400,0,320,0,401,240,0,0,1])
        for index,value in [(0,0),(3,-48.4),(5,float('nan')),(10,0)]:
            bad=p.copy();bad[index]=value
            with self.assertRaises(ValueError):rectified_intrinsics(bad)

    def test_calibration_digest_changes_with_angle(self):
        config=copy.deepcopy(self.config);config['pitch_down_deg']=41
        self.assertNotEqual(self.localizer.calibration_sha256,MapLocalizer(config).calibration_sha256)

    def test_wrapper_urdf_mount_compensation_matches_left_optical_origin(self):
        mount=zed_mount(self.config)
        rb=rotation_rpy(*mount['base_rpy']);rc=rb@rotation_rpy(0,.05,0)
        left=mount['base_position']+rb@np.array([0,0,.015])+rc@np.array([-.01,.06,0])
        np.testing.assert_allclose(left,[.62,-.13,.97],atol=1e-12)


if __name__=='__main__':unittest.main()
