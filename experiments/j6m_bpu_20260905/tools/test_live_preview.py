from types import SimpleNamespace
import unittest
import numpy as np
from live_preview import image_bgr


class ImageTest(unittest.TestCase):
    def message(self, encoding, data, step):
        return SimpleNamespace(width=2, height=1, encoding=encoding, data=bytes(data), step=step)

    def test_bgr_padding(self):
        frame = image_bgr(self.message("bgr8", [1, 2, 3, 4, 5, 6, 99, 99], 8))
        np.testing.assert_array_equal(frame, [[[1, 2, 3], [4, 5, 6]]])

    def test_rgb(self):
        frame = image_bgr(self.message("rgb8", [1, 2, 3, 4, 5, 6], 6))
        np.testing.assert_array_equal(frame, [[[3, 2, 1], [6, 5, 4]]])

    def test_alpha(self):
        frame = image_bgr(self.message("bgra8", [1, 2, 3, 255, 4, 5, 6, 255], 8))
        np.testing.assert_array_equal(frame, [[[1, 2, 3], [4, 5, 6]]])

    def test_bad_layout(self):
        for message in (self.message("mono8", [1, 2], 2), self.message("rgb8", [1, 2], 6)):
            with self.assertRaises(ValueError):
                image_bgr(message)
