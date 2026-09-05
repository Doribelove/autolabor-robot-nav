#!/usr/bin/env python3

import math
import unittest

import numpy as np

from autolabor_fod_vision.two_stage import (
    _depth_cluster_score,
    DepthClusterEstimate,
    LatestFrameSlot,
    MATERIAL_CLASSES,
    ObjectObservation,
    WorldObjectMap,
    context_crop,
    crop_sharpness,
    estimate_clustered_depth,
)


class LatestFrameSlotTest(unittest.TestCase):
    def test_new_frame_overwrites_unread_frame_without_queue_growth(self):
        slot = LatestFrameSlot()
        self.assertFalse(slot.put("frame-1"))
        self.assertTrue(slot.put("frame-2"))
        self.assertTrue(slot.put("frame-3"))
        self.assertEqual(slot.pending, 1)
        self.assertEqual(slot.received, 3)
        self.assertEqual(slot.overwritten, 2)
        self.assertEqual(slot.take(timeout=0.0), "frame-3")
        self.assertEqual(slot.pending, 0)

    def test_stop_discards_pending_frame_and_unblocks_take(self):
        slot = LatestFrameSlot()
        slot.put("old")
        slot.stop()
        self.assertIsNone(slot.take(timeout=0.0))
        self.assertEqual(slot.pending, 0)


class CropAndDepthTest(unittest.TestCase):
    def test_depth_cluster_score_rewards_more_supported_compact_centered_components(self):
        baseline = _depth_cluster_score(0.23, 0.75, 0.12, 0.45, 0.008, 1.0)
        more_support = _depth_cluster_score(0.63, 0.75, 0.12, 0.45, 0.008, 1.0)
        more_compact = _depth_cluster_score(0.23, 0.95, 0.12, 0.45, 0.008, 1.0)
        more_centered = _depth_cluster_score(0.23, 0.75, 0.02, 0.85, 0.008, 1.0)

        self.assertGreater(more_support, baseline)
        self.assertGreater(more_compact, baseline)
        self.assertGreater(more_centered, baseline)

    def test_depth_cluster_area_score_saturates_without_penalizing_large_components(self):
        at_saturation = _depth_cluster_score(0.70, 0.80, 0.05, 0.80, 0.005, 1.0)
        above_saturation = _depth_cluster_score(0.95, 0.80, 0.05, 0.80, 0.005, 1.0)

        self.assertAlmostEqual(at_saturation, above_saturation)

    def test_context_crop_expands_twenty_percent_in_memory(self):
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        crop, bounds = context_crop(image, (40, 30, 80, 70), 0.20)
        self.assertEqual(bounds, (32, 22, 88, 78))
        self.assertEqual(crop.shape, (56, 56, 3))
        self.assertTrue(np.shares_memory(image, crop))

    def test_sharpness_rejects_flat_blur_and_accepts_edges(self):
        flat = np.full((64, 64, 3), 127, dtype=np.uint8)
        edged = flat.copy()
        edged[:, 32:] = 255
        self.assertEqual(crop_sharpness(flat), 0.0)
        self.assertGreater(crop_sharpness(edged), 18.0)

    def test_depth_cluster_uses_geometry_not_nearest_layer(self):
        depth = np.full((100, 100), 3.0, dtype=np.float32)
        depth[35:72, 32:69] = 2.0  # centered target
        depth[12:30, 12:30] = 1.0  # nearer but off-center distractor
        estimate = estimate_clustered_depth(
            depth,
            (5, 5, 95, 95),
            [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0],
            inset_fraction=0.0,
            minimum_samples=20,
        )
        self.assertTrue(estimate.valid, estimate.reason)
        self.assertAlmostEqual(estimate.depth_m, 2.0, places=2)
        self.assertNotAlmostEqual(estimate.depth_m, 1.0, places=1)

    def test_depth_cluster_median_rejects_extreme_flying_points(self):
        depth = np.full((100, 120), 5.0, dtype=np.float32)
        depth[28:78, 38:88] = 1.75
        depth[36, 45] = 0.31
        depth[44, 63] = 14.90
        depth[52, 70] = np.nan
        depth[60, 80] = np.inf
        estimate = estimate_clustered_depth(
            depth,
            (20, 15, 105, 90),
            [120.0, 0.0, 60.0, 0.0, 120.0, 50.0, 0.0, 0.0, 1.0],
            inset_fraction=0.0,
            minimum_samples=20,
            aggregation="median",
        )

        self.assertTrue(estimate.valid, estimate.reason)
        self.assertAlmostEqual(estimate.depth_m, 1.75, places=2)
        self.assertLess(estimate.mad_m, 0.01)

    def test_flat_surface_without_separable_object_is_invalid(self):
        depth = np.full((80, 100), 2.5, dtype=np.float32)
        estimate = estimate_clustered_depth(
            depth,
            (10, 10, 90, 70),
            [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0],
            inset_fraction=0.0,
            minimum_samples=20,
        )
        self.assertFalse(estimate.valid)
        self.assertIn("flat surface", estimate.reason)
        self.assertTrue(math.isnan(estimate.depth_m))

    def test_mean_aggregation_returns_three_dimensional_camera_point(self):
        depth = np.full((120, 160), 3.0, dtype=np.float32)
        depth[42:82, 60:100] = 1.5
        estimate = estimate_clustered_depth(
            depth,
            (45.0, 25.0, 115.0, 100.0),
            [100.0, 0.0, 80.0, 0.0, 100.0, 60.0, 0.0, 0.0, 1.0],
            minimum_samples=20,
            aggregation="mean",
        )
        self.assertTrue(estimate.valid, estimate.reason)
        self.assertEqual(len(estimate.camera_point), 3)

    def test_sparse_depth_is_invalid(self):
        depth = np.full((80, 100), np.nan, dtype=np.float32)
        depth[40:42, 50:52] = 1.2
        estimate = estimate_clustered_depth(
            depth,
            (10, 10, 90, 70),
            [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0],
            minimum_samples=20,
        )
        self.assertFalse(estimate.valid)


class WorldObjectMapTest(unittest.TestCase):
    @staticmethod
    def observation(track_id, x, bbox=(10.0, 10.0, 30.0, 30.0)):
        return ObjectObservation(
            track_id=track_id,
            bbox=bbox,
            detect_confidence=0.9,
            depth_valid=True,
            depth_m=1.5,
            world_position=np.asarray([x, 0.0, 0.0], dtype=np.float64),
            world_frame="map",
        )

    @staticmethod
    def depth_estimate(depth_m, separated=True):
        return DepthClusterEstimate(
            valid=True,
            depth_m=float(depth_m),
            camera_point=(0.0, 0.0, float(depth_m)),
            separated_from_background=bool(separated),
        )

    def test_track_id_change_reuses_object_and_classification_history(self):
        object_map = WorldObjectMap(
            max_world_distance_m=0.30,
            vote_window=5,
            minimum_stable_votes=3,
            reclassify_interval_frames=5,
        )
        first = object_map.associate([self.observation(11, 1.00)], 10.0)[0]
        plastic = np.asarray([0.02, 0.90, 0.03, 0.03, 0.02])
        appearance = np.ones(64, dtype=np.float32) / 8.0
        object_map.add_classification(first, plastic, appearance, 1)
        object_map.add_classification(first, plastic, appearance, 6)
        object_map.add_classification(first, plastic, appearance, 11)
        self.assertEqual(first.stable_material, "plastic")
        second = object_map.associate([self.observation(99, 1.08)], 11.0)[0]
        self.assertEqual(second.object_id, first.object_id)
        self.assertEqual(second.current_track_id, 99)
        self.assertEqual(second.state, "REIDENTIFIED")
        self.assertEqual(len(second.votes), 3)
        self.assertEqual(second.stable_material, "plastic")
        self.assertFalse(object_map.should_classify(second, appearance, 12))

    def test_short_visual_track_break_reuses_object_without_current_world_point(self):
        object_map = WorldObjectMap(reclassify_interval_frames=5)
        appearance = np.zeros(64, dtype=np.float32)
        appearance[7] = 1.0
        first = object_map.associate(
            [
                ObjectObservation(
                    10,
                    (20.0, 20.0, 60.0, 60.0),
                    0.9,
                    appearance=appearance,
                )
            ],
            5.0,
        )[0]
        object_map.add_classification(
            first, [0.05, 0.80, 0.05, 0.05, 0.05], appearance, 1
        )
        second = object_map.associate(
            [
                ObjectObservation(
                    88,
                    (22.0, 20.0, 62.0, 60.0),
                    0.88,
                    appearance=appearance.copy(),
                )
            ],
            5.4,
        )[0]
        self.assertEqual(second.object_id, first.object_id)
        self.assertEqual(second.state, "REIDENTIFIED")
        self.assertEqual(len(second.votes), 1)
        self.assertFalse(second.depth_valid)

    def test_assignment_is_one_to_one(self):
        object_map = WorldObjectMap(max_world_distance_m=0.30)
        original = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        matched = object_map.associate(
            [self.observation(2, 0.04), self.observation(3, 0.08)], 2.0
        )
        identifiers = [target.object_id for target in matched]
        self.assertIn(original.object_id, identifiers)
        self.assertEqual(len(set(identifiers)), 2)

    def test_weighted_vote_window_is_bounded(self):
        object_map = WorldObjectMap(vote_window=5, minimum_stable_votes=3)
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        appearance = np.ones(64, dtype=np.float32) / 8.0
        for frame in range(7):
            probabilities = np.full(len(MATERIAL_CLASSES), 0.01, dtype=np.float32)
            probabilities[0] = 0.96
            object_map.add_classification(target, probabilities, appearance, frame)
        self.assertEqual(len(target.votes), 5)
        self.assertEqual(target.stable_material, "metal")
        self.assertGreater(target.classify_confidence, 0.90)
        self.assertEqual(target.state, "CONFIRMED")

    def test_confirmed_state_requires_minimum_stable_votes(self):
        object_map = WorldObjectMap(
            vote_window=5, minimum_stable_votes=3, stable_confidence=0.55
        )
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        appearance = np.ones(64, dtype=np.float32) / 8.0
        probabilities = np.asarray([0.02, 0.92, 0.02, 0.02, 0.02])
        object_map.add_classification(target, probabilities, appearance, 1)
        self.assertEqual(target.state, "ACTIVE")
        object_map.associate([self.observation(1, 0.0)], 1.1)
        self.assertEqual(target.state, "ACTIVE")
        object_map.add_classification(target, probabilities, appearance, 6)
        self.assertEqual(target.state, "ACTIVE")
        object_map.add_classification(target, probabilities, appearance, 11)
        self.assertEqual(target.state, "CONFIRMED")

    def test_five_depth_samples_lock_to_robust_inlier_mean(self):
        object_map = WorldObjectMap(
            depth_lock_samples=5,
            depth_lock_min_inliers=3,
            depth_outlier_min_m=0.08,
        )
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        values = (1.00, 1.02, 4.00, 0.99, 1.01)
        for frame_index, depth_m in enumerate(values, start=1):
            object_map.record_depth_observation(
                target,
                self.depth_estimate(depth_m),
                1.0 + frame_index * 0.1,
                frame_index,
            )
            if frame_index < 5:
                self.assertFalse(target.depth_locked)
        self.assertTrue(target.depth_locked)
        self.assertAlmostEqual(target.stable_depth_m, 1.005, places=3)
        self.assertEqual(len(target.depth_samples), 5)

    def test_locked_depth_skips_until_validation_and_survives_track_change(self):
        object_map = WorldObjectMap(
            depth_lock_samples=5,
            depth_validation_interval_frames=12,
        )
        target = object_map.associate([self.observation(7, 1.0)], 1.0)[0]
        for frame_index in range(1, 6):
            object_map.record_depth_observation(
                target,
                self.depth_estimate(1.0 + 0.005 * frame_index),
                1.0 + frame_index * 0.1,
                frame_index,
            )
        self.assertFalse(
            object_map.should_sample_depth(
                target, target.bbox, None, frame_index=16
            )
        )
        self.assertTrue(
            object_map.should_sample_depth(
                target, target.bbox, None, frame_index=17
            )
        )
        reidentified = object_map.associate(
            [self.observation(88, 1.04)], 2.0
        )[0]
        self.assertEqual(reidentified.object_id, target.object_id)
        self.assertTrue(reidentified.depth_locked)
        self.assertAlmostEqual(
            reidentified.stable_depth_m, target.stable_depth_m, places=6
        )

    def test_large_validation_change_forces_depth_and_world_reacquisition(self):
        object_map = WorldObjectMap(
            depth_lock_samples=5,
            world_lock_samples=3,
            depth_validation_max_abs_change_m=0.15,
        )
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        for frame_index in range(1, 6):
            object_map.record_depth_observation(
                target,
                self.depth_estimate(1.0),
                1.0 + frame_index * 0.1,
                frame_index,
                np.asarray([0.01 * frame_index, 0.0, 0.0]),
                "map",
            )
        self.assertTrue(target.depth_locked)
        self.assertTrue(target.world_locked)
        event = object_map.record_depth_observation(
            target,
            self.depth_estimate(1.40),
            3.0,
            17,
        )
        self.assertEqual(event, "REACQUIRING")
        self.assertFalse(target.depth_locked)
        self.assertFalse(target.world_locked)
        self.assertEqual(len(target.depth_samples), 1)
        self.assertIsNone(target.world_position)

    def test_three_tf_samples_lock_world_point_and_ignore_later_updates(self):
        object_map = WorldObjectMap(world_lock_samples=3)
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        positions = (0.00, 0.02, 3.00)
        for frame_index, x in enumerate(positions, start=1):
            object_map.record_depth_observation(
                target,
                self.depth_estimate(1.0),
                1.0 + frame_index * 0.1,
                frame_index,
                np.asarray([x, 0.0, 0.0]),
                "map",
            )
        self.assertTrue(target.world_locked)
        self.assertAlmostEqual(float(target.world_position[0]), 0.01, places=3)
        locked_position = target.world_position.copy()
        object_map.record_depth_observation(
            target,
            self.depth_estimate(1.0),
            2.0,
            4,
            np.asarray([0.20, 0.0, 0.0]),
            "map",
        )
        np.testing.assert_allclose(target.world_position, locked_position)
        self.assertEqual(len(target.world_samples), 3)

    def test_two_failed_validation_frames_clear_locked_depth(self):
        object_map = WorldObjectMap(
            depth_lock_samples=5,
            depth_validation_failures_before_reacquire=2,
        )
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        for frame_index in range(1, 6):
            object_map.record_depth_observation(
                target,
                self.depth_estimate(1.0),
                1.0 + frame_index * 0.1,
                frame_index,
            )
        self.assertFalse(object_map.note_depth_failure(target, 17))
        self.assertTrue(target.depth_locked)
        self.assertTrue(object_map.note_depth_failure(target, 18))
        self.assertFalse(target.depth_locked)
        self.assertEqual(len(target.depth_samples), 0)

    def test_lost_and_expired_states_are_explicit(self):
        object_map = WorldObjectMap(memory_timeout_sec=3.0)
        target = object_map.associate([self.observation(1, 0.0)], 1.0)[0]
        object_map.associate([], 2.0)
        self.assertEqual(target.state, "LOST")
        object_map.associate([], 5.1)
        self.assertEqual(target.state, "EXPIRED")
        self.assertTrue(object_map.mark_cleaned(target.object_id))
        self.assertEqual(target.state, "CLEANED")


if __name__ == "__main__":
    unittest.main()
