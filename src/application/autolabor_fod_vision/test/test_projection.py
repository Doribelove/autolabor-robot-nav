#!/usr/bin/env python3

import unittest
from math import pi

import numpy as np

from autolabor_fod_vision.projection import (
    ProjectionError,
    optical_rotation_base,
    pixel_to_camera_ray,
    project_ground_point_to_pixel,
    project_pixel_to_ground,
)


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.k = np.array(
            [[700.0, 0.0, 480.0], [0.0, 700.0, 270.0], [0.0, 0.0, 1.0]]
        )
        self.rotation = optical_rotation_base(25.0 * pi / 180.0)
        self.origin = np.array([0.3, 0.0, 1.0])

    def test_known_ground_point_round_trip(self):
        expected = np.array([2.0, 0.25, 0.0])
        u, v = project_ground_point_to_pixel(
            expected, self.k, self.rotation, self.origin
        )
        actual = project_pixel_to_ground(
            u,
            v,
            self.k,
            [],
            self.rotation,
            self.origin,
            ground_z=0.0,
        )
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_camera_center_ray_hits_ground(self):
        actual = project_pixel_to_ground(
            480.0,
            270.0,
            self.k,
            [0.0] * 5,
            self.rotation,
            self.origin,
        )
        self.assertGreater(actual[0], self.origin[0])
        self.assertAlmostEqual(actual[1], 0.0, places=9)
        self.assertAlmostEqual(actual[2], 0.0, places=9)

    def test_uncalibrated_camera_rejected(self):
        with self.assertRaises(ProjectionError):
            pixel_to_camera_ray(10.0, 10.0, [0.0] * 9, [])

    def test_invalid_pinhole_distortion_rejected(self):
        with self.assertRaises(ProjectionError):
            pixel_to_camera_ray(10.0, 10.0, self.k, [0.0] * 3)

    def test_invalid_camera_matrix_shape_rejected(self):
        with self.assertRaises(ProjectionError):
            pixel_to_camera_ray(10.0, 10.0, [1.0] * 8, [])

    def test_backward_intersection_rejected(self):
        with self.assertRaises(ProjectionError):
            project_pixel_to_ground(
                480.0,
                270.0,
                self.k,
                [],
                np.eye(3),
                [0.0, 0.0, 1.0],
            )


if __name__ == "__main__":
    unittest.main()
