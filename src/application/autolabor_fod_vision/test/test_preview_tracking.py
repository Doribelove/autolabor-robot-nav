#!/usr/bin/env python3
import unittest
from dataclasses import replace

from autolabor_fod_vision.detector import Detection
from autolabor_fod_vision.preview_tracking import PreviewStabilizer


def detection(x=100.0, class_id=0, confidence=0.10):
    return Detection(class_id, ["Metal", "Plastic"][class_id], confidence,
                     x, 100.0, x + 40.0, 140.0, x + 20.0, 140.0)


class PreviewTrackingTest(unittest.TestCase):
    def setUp(self):
        self.tracker = PreviewStabilizer(alpha=0.5, hold_sec=0.3)

    def update(self, detections, stamp, size=(1280, 720)):
        return self.tracker.update(detections, stamp, size)

    def test_weak_first_detection_is_visible_immediately(self):
        result = self.update([detection(confidence=0.10)], 1.0)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].held_for_sec, 0.0)
        self.assertEqual(result[0].detection.confidence, 0.10)

    def test_short_miss_is_marked_and_hold_expires_by_time(self):
        self.update([detection()], 1.0)
        result = self.update([], 1.2)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0].held_for_sec, 0.2)
        self.assertEqual(self.update([], 1.31), [])

    def test_zero_hold_still_smooths_new_observations(self):
        self.tracker = PreviewStabilizer(alpha=0.5, hold_sec=0.0)
        self.update([detection()], 1.0)
        result = self.update([detection(x=104)], 1.05)
        self.assertAlmostEqual(result[0].detection.xmin, 102.0)
        self.assertEqual(self.update([], 1.1), [])

    def test_jitter_smoothed_without_mutating_current_input(self):
        self.update([detection()], 1.0)
        raw = detection(x=104, confidence=0.12)
        result = self.update([raw], 1.05)[0].detection
        self.assertAlmostEqual(result.xmin, 102.0)
        self.assertAlmostEqual(result.anchor_u, 122.0)
        self.assertEqual(raw.xmin, 104.0)
        self.assertEqual(raw.confidence, 0.12)
        self.assertIsNot(raw, result)

    def test_fast_movement_has_no_smoothing_drag(self):
        track_id = self.update([detection()], 1.0)[0].track_id
        result = self.update([detection(x=126)], 1.05)[0]
        self.assertEqual(result.track_id, track_id)
        self.assertEqual(result.detection.xmin, 126.0)

    def test_single_class_flip_does_not_flicker_label(self):
        self.update([detection()], 1.0)
        self.update([detection()], 1.05)
        self.update([detection()], 1.10)
        raw = detection(class_id=1)
        result = self.update([raw], 1.15)[0]
        self.assertEqual(result.detection.class_name, "Metal")
        self.assertEqual(raw.class_name, "Plastic")
        for index in range(10):
            result = self.update([detection(class_id=1)], 1.20 + index * 0.05)[0]
        self.assertEqual(result.detection.class_name, "Plastic")

    def test_two_nearby_objects_remain_two_with_reversed_input(self):
        before = self.update([detection(x=100), detection(x=160)], 1.0)
        after = self.update([detection(x=163), detection(x=102)], 1.05)
        self.assertEqual(len(after), 2)
        self.assertEqual([b.track_id for b in before], [b.track_id for b in after])
        self.assertLess(after[0].detection.xmin, after[1].detection.xmin)

    def test_old_track_cannot_relabel_new_object_after_long_gap(self):
        before = self.update([detection()], 1.0)[0]
        after = self.update([detection(class_id=1)], 2.0)[0]
        self.assertNotEqual(before.track_id, after.track_id)
        self.assertEqual(after.detection.class_name, "Plastic")

    def test_image_resize_and_time_rewind_clear_history(self):
        self.update([detection()], 2.0)
        self.assertEqual(self.update([], 2.05, (640, 360)), [])
        self.update([detection()], 3.0)
        self.assertEqual(self.update([], 1.0), [])

    def test_large_replacement_does_not_leave_duplicate_held_box(self):
        self.update([detection()], 1.0)
        larger = replace(detection(), xmin=91, xmax=149, ymin=91, ymax=149)
        self.assertEqual(len(self.update([larger], 1.05)), 1)


if __name__ == "__main__":
    unittest.main()
