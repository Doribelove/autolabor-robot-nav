#!/usr/bin/env python3

import unittest

from autolabor_fod_vision.tracking import GroundObservation, MultiTargetTracker


def observation(x=2.0, y=0.25, class_id=0, confidence=0.9):
    return GroundObservation(
        class_id=class_id,
        class_name="fod",
        confidence=confidence,
        x=x,
        y=y,
    )


class TrackingTest(unittest.TestCase):
    def test_three_hits_confirm_target(self):
        tracker = MultiTargetTracker(min_hits=3, max_age=1.0)
        tracker.update([observation()], 1.0)
        tracker.update([observation(2.02, 0.24)], 1.1)
        tracker.update([observation(1.99, 0.26)], 1.2)
        target, status, count = tracker.select_target(1.2)
        self.assertIsNotNone(target)
        self.assertEqual(status, "TRACKING")
        self.assertEqual(count, 1)
        self.assertEqual(target.track_id, 1)

    def test_stale_track_removed(self):
        tracker = MultiTargetTracker(min_hits=1, max_age=0.5)
        tracker.update([observation()], 2.0)
        target, _, _ = tracker.select_target(2.6)
        self.assertIsNone(target)
        self.assertEqual(tracker.tracks, [])

    def test_ambiguous_targets_are_rejected_by_default(self):
        tracker = MultiTargetTracker(
            min_hits=1, max_age=1.0, association_distance=0.3
        )
        tracker.update([observation(2.0, -0.5), observation(2.0, 0.5)], 3.0)
        target, status, count = tracker.select_target(3.0)
        self.assertIsNone(target)
        self.assertEqual(status, "AMBIGUOUS")
        self.assertEqual(count, 2)

    def test_out_of_order_update_does_not_overwrite_track(self):
        tracker = MultiTargetTracker(min_hits=1, max_age=5.0)
        tracker.update([observation(2.0, 0.0)], 5.0)
        tracker.update([observation(9.0, 0.0)], 4.0)
        self.assertAlmostEqual(tracker.tracks[0].x, 2.0)
        self.assertEqual(tracker.tracks[0].last_observed, 5.0)


if __name__ == "__main__":
    unittest.main()
