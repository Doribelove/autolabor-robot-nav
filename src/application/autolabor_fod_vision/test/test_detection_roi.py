#!/usr/bin/env python3
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from autolabor_fod_vision.detector import Detection, InferenceResult
from autolabor_fod_vision.detection_roi import DetectionRoi, mark_recovery_region


class GroundRoiTest(unittest.TestCase):
    def test_production_lower_half_excludes_top_and_restores_depth_coordinates(self):
        root = Path(__file__).resolve().parents[4]
        config = yaml.safe_load((root / 'config/fod_zed_ground_roi.yaml').read_text())
        roi = DetectionRoi(config['roi_top_fraction'], config['roi_bottom_fraction'])
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        image[:360] = 81
        image[360:] = 197
        boxes = [Detection(0, 'Metal', .8, 10, y, 40, y+20, 25, y+20)
                 for y in (0, 160, 330)]
        seen = []
        model = SimpleNamespace(predict=lambda frame: (
            seen.append(frame.copy()) or InferenceResult(boxes, 10.)))
        result = roi.predict(model, image)
        np.testing.assert_array_equal(seen[0], image[360:])
        self.assertEqual([d.ymin for d in result.detections], [360, 520, 690])
        self.assertEqual([d.anchor_v for d in result.detections], [380, 540, 710])
        shown = roi.mark_preview(image.copy())
        np.testing.assert_array_equal(shown[600, 100], image[600, 100])
        self.assertTrue(np.all(shown[100, 100] < image[100, 100]))

    def test_excluded_rows_never_reach_model_and_coordinates_return_to_full_image(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        image[:180] = 255
        image[450:] = 255
        seen = []
        box = Detection(2, "Plastic", .8, 10, 20, 40, 60, 25, 60, [(10, 20), (40, 60)])
        def predict(crop):
            seen.append(crop)
            return InferenceResult([box], 12.0)
        result = DetectionRoi(.25, .625).predict(SimpleNamespace(predict=predict), image)
        self.assertEqual(seen[0].shape, (270, 1280, 3))
        self.assertFalse(seen[0].any())
        actual = result.detections[0]
        self.assertEqual((actual.xmin, actual.ymin, actual.ymax, actual.anchor_v),
                         (10, 200, 240, 240))
        self.assertEqual(actual.mask, [(10, 200), (40, 240)])
        self.assertEqual(actual.confidence, .8)

    def test_preview_keeps_full_image_and_dims_only_excluded_rows(self):
        image = np.full((720, 1280, 3), 200, dtype=np.uint8)
        shown = DetectionRoi(.25, .625).mark_preview(image.copy())
        self.assertEqual(shown.shape, image.shape)
        self.assertTrue((shown[50, 600] == 70).all())
        self.assertTrue((shown[300, 600] == 200).all())
        self.assertTrue((shown[600, 600] == 70).all())

    def test_default_is_full_frame(self):
        self.assertEqual(DetectionRoi().rows(720), (0, 720))
        for top, bottom in [(.7, .5), (-.1, .5), (0, 1.1), (float('nan'), 1)]:
            with self.assertRaises(ValueError):
                DetectionRoi(top, bottom)

    def test_recovery_overlay_uses_controller_geometry_and_latched_ready_state(self):
        image = np.full((720, 1280, 3), 100, dtype=np.uint8)
        state = {"recovery_region_px": [236, 314.56, 1004, 449.38],
                 "recovery_region_entered": False, "blind_target_m": .8}
        shown = mark_recovery_region(image.copy(), state)
        self.assertTrue((shown[380, 236] == [0, 200, 255]).all())
        self.assertTrue((shown[500, 640] == image[500, 640]).all())
        self.assertTrue((shown[250, 640] == image[250, 640]).all())
        state["recovery_region_entered"] = True
        shown = mark_recovery_region(image.copy(), state)
        self.assertTrue((shown[380, 236] == [60, 230, 80]).all())

    def test_invalid_or_missing_region_does_not_break_the_preview(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        for state in (None, {}, {"recovery_region_px": [0, float('nan'), 20, 50]}):
            self.assertIs(mark_recovery_region(image, state), image)

    def test_box_at_crop_edge_stays_inside_retained_rows(self):
        box = Detection(0, "Metal", .8, 10, 240, 40, 270, 25, 270)
        model = SimpleNamespace(predict=lambda image: InferenceResult([box], 10))
        result = DetectionRoi(.25, .625).predict(model, np.zeros((720, 1280, 3)))
        self.assertEqual(result.detections[0].ymax, 449)
        self.assertEqual(result.detections[0].anchor_v, 449)


if __name__ == '__main__':
    unittest.main()
