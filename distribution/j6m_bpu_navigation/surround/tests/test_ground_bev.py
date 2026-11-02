"""Analytic geometry and visibility checks for the measured three-camera rig."""
import json
import copy
import math
from pathlib import Path
import unittest

import numpy as np
import cv2

from surround.ground_bev import GroundBev


class GroundGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[4]
        cls.config = json.loads((root/'config/three_camera_bev.json').read_text())
        cls.bev = GroundBev(cls.config)

    def test_optical_axes_intersect_ground_in_correct_directions(self):
        distance_side = .80/math.tan(math.radians(22))
        targets = {'left': [0, .215+distance_side, 0],
                   'right': [0, -.215-distance_side, 0]}
        for name, target in targets.items():
            uv, depth = self.bev.project(name, target)
            self.assertGreater(depth, 0)
            np.testing.assert_allclose(uv, self.bev.intrinsics[name][:2, 2], atol=1e-8)

    def test_image_right_matches_physical_camera_axes(self):
        # Upright front image right is vehicle right; left camera image right
        # points toward vehicle front; right camera image right points rearward.
        for name, center, point in [('front', [1, -.015, 0], [1, -.2, 0]),
                                     ('left', [0, 2, 0], [.2, 2, 0]),
                                     ('right', [0, -2, 0], [-.2, -2, 0])]:
            uv, _ = self.bev.project(name, point)
            origin, _ = self.bev.project(name, center)
            self.assertGreater(uv[0], origin[0], name)

    def test_upside_down_input_is_corrected_once(self):
        image = np.zeros((540, 960, 3), np.uint8)
        image[0, 0] = [9, 27, 81]
        fixed = self.bev.upright('left', image)
        np.testing.assert_array_equal(fixed[-1, -1], [9, 27, 81])
        np.testing.assert_array_equal(fixed[0, 0], [0, 0, 0])
        zed = np.zeros((720, 1280, 3), np.uint8)
        self.assertIs(self.bev.upright('front', zed), zed)

    def test_blending_preserves_unseen_ground_and_source_colors(self):
        frames = {name: np.full((c['perspective_size'][1], c['perspective_size'][0], 3),
                               [31, 97, 149], np.uint8) for name, c in self.config['cameras'].items()}
        image, counts, _ = self.bev.compose(frames)
        self.assertTrue((image[counts == 0] == 0).all())
        self.assertGreater((counts > 1).sum(), 0)
        np.testing.assert_allclose(image[counts > 0], np.tile([31, 97, 149], ((counts > 0).sum(), 1)), atol=1)
        col, row = [round(v) for v in self.bev.pixel(-1, 0)]
        self.assertEqual(counts[row, col], 0, 'Unobserved rear center must stay empty')

    def test_incomplete_rig_cannot_appear_as_complete_fusion(self):
        with self.assertRaisesRegex(ValueError, 'All configured'):
            self.bev.compose({})

    def test_cached_projection_matches_full_grid_reference_with_masks(self):
        config = self.four_camera_fixture()
        config['cameras']['left']['exclude_polygons_upright_px'] = [
            [[0, 0], [959, 0], [959, 539], [0, 539]]]
        bev = GroundBev(config)
        rng = np.random.RandomState(731)
        frames = {n: rng.randint(0, 256, (c['perspective_size'][1],
                  c['perspective_size'][0], 3), dtype=np.uint8) for n, c in config['cameras'].items()}
        total = np.zeros((bev.height, bev.width, 3), np.float32)
        weights = np.zeros((bev.height, bev.width), np.float32)
        for n, (mx, my, weight) in bev.maps.items():
            projected = cv2.remap(frames[n], mx, my, cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT)
            total += projected.astype(np.float32)*weight[..., None]
            weights += weight
        reference = np.clip(total/np.maximum(weights[..., None], 1e-10), 0, 255).astype(np.uint8)
        image, counts, layers = bev.compose(dict(reversed(list(frames.items()))))
        np.testing.assert_allclose(image, reference, atol=1, rtol=0)
        self.assertFalse(layers['left'].any())
        self.assertFalse(image[counts == 0].any())
        counts[:] = 0  # A caller must not corrupt cached visibility of later frames.
        self.assertTrue(bev.compose(frames)[1].any())

    def four_camera_fixture(self):
        config = copy.deepcopy(self.config)
        config['cameras']['rear'] = dict(source='j6m_hardware_gdc', rx1_port=2,
            perspective_size=[960, 540], K_perspective=config['cameras']['left']['K_perspective'],
            position_ground_rig_m=[-.4, 0, .71], yaw_left_deg=180,
            pitch_down_deg=35, rotate_180=False)
        return config

    def test_rear_adds_support_behind_rig_without_fabricating_missing_input(self):
        config = self.four_camera_fixture()
        bev = GroundBev(config)
        frames = {n: np.full((c['perspective_size'][1], c['perspective_size'][0], 3),
                            [13, 71, 211], np.uint8) for n, c in config['cameras'].items()}
        image, counts, layers = bev.compose(frames)
        col, row = [round(v) for v in bev.pixel(-1, 0)]
        self.assertGreater(counts[row, col], 0)
        np.testing.assert_allclose(image[row, col], [13, 71, 211], atol=1)
        self.assertGreater(layers['rear'][row, col].sum(), 0)
        del frames['rear']
        with self.assertRaisesRegex(ValueError, 'All configured'):
            bev.compose(frames)

    def test_rear_camera_axes_point_backward_and_image_right_points_left(self):
        bev = GroundBev(self.four_camera_fixture())
        center, depth = bev.project('rear', [-1.4, 0, 0])
        right, _ = bev.project('rear', [-1.4, .2, 0])
        self.assertGreater(depth, 0)
        self.assertGreater(right[0], center[0])

    def test_missing_rear_installation_is_reported_without_guessing_geometry(self):
        config = self.four_camera_fixture()
        config['cameras']['rear']['position_ground_rig_m'] = None
        with self.assertRaisesRegex(ValueError, 'Missing installation parameters for rear'):
            GroundBev(config)

    def test_level_front_projects_ground_below_horizon_with_measured_height(self):
        # A horizontal optical axis has no finite ground intersection. Ground
        # pixels converge to the horizon as forward distance increases.
        k = self.bev.intrinsics['front']
        for distance in (0.5, 1.0, 2.0):
            uv, depth = self.bev.project('front', [.96+distance, 0, 0])
            self.assertAlmostEqual(depth, distance)
            np.testing.assert_allclose(uv,
                [k[0, 2], k[1, 2]+k[1, 1]*.32/distance], atol=1e-8)
        _, depth = self.bev.project('front', [.50, 0, 0])
        self.assertLess(depth, 0, 'Ground behind the front camera is not visible')

    def test_single_source_removes_double_exposure_without_moving_ground_or_losing_support(self):
        config=self.four_camera_fixture()
        original=GroundBev(config)
        config['blend_mode']='single_source'
        selected=GroundBev(config)
        frames={n:np.full((c['perspective_size'][1],c['perspective_size'][0],3),color,np.uint8)
                for (n,c),color in zip(config['cameras'].items(),[30,90,150,210])}
        image,counts,layers=selected.compose(frames)
        np.testing.assert_array_equal(counts,original.support_counts)
        self.assertFalse(image[counts==0].any())
        self.assertGreater((counts>1).sum(),0)
        for index,name in enumerate(selected.source_names):
            region=selected.source_owner==index
            self.assertTrue((selected.maps[name][2][region]>0).all())
            np.testing.assert_array_equal(image[region],layers[name][region])
            np.testing.assert_array_equal(selected.maps[name][0],original.maps[name][0])
            np.testing.assert_array_equal(selected.maps[name][1],original.maps[name][1])
        np.testing.assert_array_equal(image,selected.compose(dict(reversed(list(frames.items()))))[0])

    def test_single_source_excluded_camera_cannot_own_overlap(self):
        config=self.four_camera_fixture();config['blend_mode']='single_source'
        config['cameras']['left']['exclude_polygons_upright_px']=[[[0,0],[959,0],[959,539],[0,539]]]
        bev=GroundBev(config)
        self.assertFalse((bev.source_owner==bev.source_names.index('left')).any())


if __name__ == '__main__':
    unittest.main()
