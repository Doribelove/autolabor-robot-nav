"""Analytic geometry and visibility checks for the measured three-camera rig."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from surround.ground_bev import GroundBev


class GroundGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[4]
        cls.config = json.loads((root/'config/three_camera_bev.json').read_text())
        cls.bev = GroundBev(cls.config)

    def test_optical_axes_intersect_ground_in_correct_directions(self):
        distance_side = .71/math.tan(math.radians(15))
        distance_front = .71/math.tan(math.radians(self.config['cameras']['front']['pitch_down_deg']))
        targets = {'left': [0, .185+distance_side, 0],
                   'right': [0, -.185-distance_side, 0],
                   'front': [.08+distance_front, -.015, 0]}
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
        with self.assertRaisesRegex(ValueError, 'Three actual'):
            self.bev.compose({})

    def test_current_zed_static_gravity_points_up_in_ground_frame(self):
        gravity = self.config['cameras']['front']['attitude_estimate']['gravity_up_direction_left_optical']
        np.testing.assert_allclose(self.bev.transforms['front'][:3, :3] @ gravity,
                                   [0, 0, 1], atol=1e-6)


if __name__ == '__main__':
    unittest.main()
