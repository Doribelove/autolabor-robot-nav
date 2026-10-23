import copy
import ctypes as C
import json
import os
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from distribution.j6m_bpu_navigation.surround import CAMERAS
from distribution.j6m_bpu_navigation.surround.contracts import Calibration, ClockMapping, Frame, transform
from distribution.j6m_bpu_navigation.surround.geometry import Pose, PoseBuffer, GroundProjector, Rectifier
from distribution.j6m_bpu_navigation.surround.scheduler import Scheduler
from distribution.j6m_bpu_navigation.surround.tracking import fuse, TargetStore
from distribution.j6m_bpu_navigation.surround.pipeline import Pipeline
from distribution.j6m_bpu_navigation.surround.inference import FixtureEngine
from distribution.j6m_bpu_navigation.surround.capture import decode_yuv, capture_issues, Meta
from distribution.j6m_bpu_navigation.surround.__main__ import load_config, DEFAULT_CONFIG, ROOT

T0 = 1_700_000_000_000_000_000


def calibration():
    r = np.array([[0., -2**-.5, 2**-.5], [-1., 0, 0], [0, -2**-.5, -2**-.5]])
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, [0, 0, 1]
    c = dict(intrinsics_valid=True, extrinsics_valid=True,
             raw_size=[640, 480], rectified_size=[640, 480], lens_model="fisheye",
             K_raw=[[300., 0, 320.], [0, 300., 240.], [0, 0, 1]],
             K_rectified=[[300., 0, 320.], [0, 300., 240.], [0, 0, 1]], D=[0., 0, 0, 0],
             rectification_id="test-perspective", T_base_camera_optical=t.tolist(),
             valid_ground_polygon_px=[[3, 3], [636, 3], [636, 476], [3, 476]],
             calibration_evidence="TEST ONLY", pixel_sigma=1.,
             extrinsic_angle_sigma_rad=.001, position_sigma_m=.01)
    return Calibration(dict(schema="j6m.surround.calibration.v1", id="test", synthetic=True,
                            ground_valid=True, ground_plane_base=[0, 0, 1., 0],
                            cameras={name: copy.deepcopy(c) for name in CAMERAS}))


def frame(camera="front", seq=0, stamp=T0):
    return Frame(camera, "capture", seq, stamp, stamp, stamp + 10_000_000, "test-clock",
                 "test-clock-map", "test", np.zeros((480, 640, 3), np.uint8),
                 True, "test-perspective", True)


def pose(stamp=T0, x=0):
    t = np.eye(4)
    t[0, 3] = x
    return Pose.from_dict(dict(stamp_ns=stamp, map_id="map-v1", localization_session="loc-v1",
                              T_map_base=t.tolist(), covariance_xy=[[.001, 0], [0, .001]], valid=True))


def observation(camera="front", x=1., stamp=T0, seq=0, index=0):
    f = frame(camera, seq, stamp)
    return dict(id=f.key + "/%d" % index, frame_key=f.key, camera=camera, class_id=0,
                map_id="map-v1", localization_session="loc-v1", calibration_id="test",
                stamp_ns=stamp, raw_stamp=stamp, clock_mapping_id="test-clock-map",
                clock_id="test-clock", rectification_id="test-perspective",
                last_observed_ns=stamp, bbox=[300., 200., 340., 240.], base_xyz=[x, 0, 0],
                map_xyz=[x, 0, 0], covariance_xy=[[.01, 0], [0, .01]],
                detect_confidence=.8, classify_confidence=.8, material="paper",
                probabilities=[.05, .05, .8, .05, .05])


class GeometryTests(unittest.TestCase):
    def test_ground_contact_and_exposure_pose(self):
        p = GroundProjector(calibration())
        base, world, cov, pixel = p.project("front", [300, 200, 340, 240], pose(x=4))
        np.testing.assert_allclose(base, [1, 0, 0], atol=1e-10)
        np.testing.assert_allclose(world, [5, 0, 0], atol=1e-10)
        self.assertEqual(pixel, (320., 240.))
        self.assertGreater(np.linalg.eigvalsh(cov).min(), 0)

    def test_missing_calibration_never_identity_fallback(self):
        c = calibration()
        c.cameras["front"]["extrinsics_valid"] = False
        with self.assertRaisesRegex(ValueError, "incomplete"):
            GroundProjector(c).project("front", [300, 200, 340, 240], pose())

    def test_invalid_rotation_and_nonfinite(self):
        t = np.eye(4)
        t[0, 0] = -1
        with self.assertRaises(ValueError):
            transform(t)
        t[0, 0] = np.nan
        with self.assertRaises(ValueError):
            transform(t)

    def test_border_horizon_and_uncertainty_rejected(self):
        p = GroundProjector(calibration())
        for box in ([300, 200, 340, 480], [300, -65, 340, -60]):
            with self.assertRaises(ValueError):
                p.project("front", box, pose())
        with self.assertRaises(ValueError):
            p.ray("front", [320, -60])
        p.calibration.cameras["front"]["extrinsic_angle_sigma_rad"] = .3
        with self.assertRaises(ValueError):
            p.project("front", [300, 160, 340, 180], pose())

    def test_ground_plane_is_not_implicitly_base_z_zero(self):
        c = calibration()
        c.data["ground_plane_base"] = [0, 0, 1., .5]
        p = GroundProjector(c).ray("front", [320, 240])
        np.testing.assert_allclose(p, [1.5, 0, -.5], atol=1e-10)

    def test_rectified_identity_and_dimensions(self):
        r = Rectifier(calibration())
        f = frame()
        self.assertIs(r.apply(f), f.image)
        f.rectification_id = "BEV-bin-is-not-perspective"
        with self.assertRaises(ValueError):
            r.apply(f)
        f.rectified = False
        f.image = np.zeros((100, 100, 3), np.uint8)
        with self.assertRaises(ValueError):
            r.apply(f)

    def test_cpu_rectifier_uses_calibrated_output(self):
        f = frame()
        f.rectified = False
        output = Rectifier(calibration(), offline_reference=True).apply(f)
        self.assertEqual(output.shape, (480, 640, 3))

    def test_pose_interpolation_and_no_extrapolation(self):
        b = PoseBuffer()
        b.put(pose())
        b.put(pose(T0 + 80_000_000, 2))
        np.testing.assert_allclose(b.at(T0 + 40_000_000).T_map_base[:3, 3], [1, 0, 0])
        with self.assertRaises(ValueError):
            b.at(T0 + 90_000_000)

    def test_pose_localization_loss_reset_and_gap(self):
        b = PoseBuffer()
        b.put(pose())
        p = pose(T0 + 80_000_000)
        p.valid = False
        b.put(p)
        with self.assertRaises(ValueError):
            b.at(T0 + 40_000_000)
        p = pose(T0 + 100_000_000)
        p.localization_session = "new"
        b.put(p)
        with self.assertRaises(ValueError):
            b.at(T0)
        b.put(pose(T0 + 300_000_000))
        with self.assertRaises(ValueError):
            b.at(T0 + 200_000_000)


class TimingSchedulerTests(unittest.TestCase):
    def mapping(self):
        return dict(id="measured", clock_id="phc0", verified=True, evidence="fixture",
                    source_semantics="frame_start", raw_origin=100, epoch_origin_ns=T0,
                    scale_num=1000, scale_den=1, exposure_offset_ns=-1_000_000,
                    valid_from_ns=T0 - 10_000_000, valid_until_ns=T0 + 1_000_000_000,
                    uncertainty_ns=1000)

    def test_clock_preserves_raw_and_exposure_offset(self):
        c = ClockMapping(self.mapping())
        self.assertEqual(c.convert(1100, T0 + 20_000_000), T0)
        with self.assertRaises(ValueError):
            c.convert(1100, T0 + 2_000_000_000)
        with self.assertRaises(ValueError):
            c.convert(0, T0)

    def test_unverified_clock_never_uses_arrival_time(self):
        d = self.mapping()
        d["verified"] = False
        with self.assertRaises(ValueError):
            ClockMapping(d)
        with self.assertRaises(ValueError):
            frame().validate(T0 - 1)
        with self.assertRaises(ValueError):
            frame().validate(T0 + 500_000_001)
        f = frame()
        f.stamp_ns = 0
        with self.assertRaises(ValueError):
            Scheduler().put(f, T0)

    def test_latest_frame_bounded_and_reordered_rejected(self):
        s = Scheduler()
        for i in range(100):
            s.put(frame(seq=i, stamp=T0 + i * 1_000_000), T0 + i * 1_000_000)
        self.assertEqual(s.snapshot()["pending"], 1)
        self.assertEqual(s.take(T0 + 99_000_000).sequence, 99)
        self.assertEqual(s.snapshot()["overwritten_front"], 99)
        with self.assertRaises(ValueError):
            s.put(frame(seq=50, stamp=T0 + 100_000_000), T0 + 100_000_000)

    def test_reverse_priority_and_forward_rear_drop(self):
        for direction, expected in (("reverse", "rear"), ("forward", "front")):
            s = Scheduler()
            s.set_motion(direction, False, None, T0 + 1_000_000_000)
            for name in CAMERAS:
                s.put(frame(name), T0)
            self.assertEqual(s.take(T0).camera, expected)
            if direction == "forward":
                self.assertEqual(s.snapshot()["direction_skip_rear"], 1)

    def test_side_fairness_after_primary_budget(self):
        s = Scheduler()
        for camera in CAMERAS:
            s.put(frame(camera), T0)
        self.assertEqual(s.take(T0).camera, "front")
        s.put(frame(seq=1, stamp=T0 + 1_000_000), T0 + 1_000_000)
        self.assertIn(s.take(T0 + 1_000_000).camera, ("left", "right"))

    def test_stale_queue_dropped(self):
        s = Scheduler()
        s.put(frame(), T0)
        self.assertIsNone(s.take(T0 + 600_000_000))
        self.assertEqual(s.snapshot()["stale_front"], 1)

    def test_expired_motion_does_not_disable_rear_forever(self):
        s = Scheduler()
        s.set_motion("forward", False, None, T0 - 1)
        s.put(frame("rear"), T0)
        self.assertEqual(s.take(T0).camera, "rear")


class FusionTrackTests(unittest.TestCase):
    def test_overlap_pairs_are_required(self):
        a, b = observation(), observation("left", 1.05)
        self.assertEqual(len(fuse([a, b], [])), 2)
        r = fuse([a, b], [["front", "left"]])
        self.assertEqual(len(r), 1)
        self.assertEqual(len(r[0]["sources"]), 2)

    def test_two_close_objects_are_not_collapsed_by_radius(self):
        obs = [observation(index=0, x=1.), observation(index=1, x=1.2),
               observation("left", x=1.04, index=0), observation("left", x=1.24, index=1)]
        fused = fuse(obs, [["front", "left"]])
        self.assertEqual(len(fused), 2)
        self.assertTrue(all(len(o["sources"]) == 2 for o in fused))

    def test_ambiguous_pair_stays_separate(self):
        obs = [observation(index=0, x=1.), observation(index=1, x=1.2),
               observation("left", x=1.1)]
        self.assertEqual(len(fuse(obs, [["front", "left"]])), 3)

    def test_different_exposure_map_and_duplicate_ids(self):
        a, b = observation(), observation("left", stamp=T0 + 100_000_001)
        self.assertEqual(len(fuse([a, b], [["front", "left"]])), 2)
        b["stamp_ns"] = T0
        b["map_id"] = "other"
        self.assertEqual(len(fuse([a, b], [["front", "left"]])), 2)
        with self.assertRaises(ValueError):
            fuse([a, a], [])

    def test_material_disagreement_does_not_duplicate_physical_trash(self):
        a, b = observation(), observation("left", 1.02)
        b["material"] = "metal"
        self.assertEqual(len(fuse([a, b], [["front", "left"]])), 1)

    def confirmed_store(self):
        s = TargetStore("test")
        for i in range(3):
            obs = fuse([observation(stamp=T0 + i * 150_000_000, seq=i)], [])
            s.update(obs, T0 + i * 150_000_000)
        return s

    def test_first_observation_tentative_and_temporal_confirmation(self):
        s = TargetStore("test")
        s.update(fuse([observation(), observation("left", 1.02)], [["front", "left"]]), T0)
        t = s.snapshot(T0)[0]
        self.assertEqual((t["status"], t["hits"]), ("tentative", 1))
        s = self.confirmed_store()
        self.assertEqual(s.snapshot(T0 + 300_000_000)[0]["status"], "confirmed")

    def test_unscheduled_camera_does_not_count_misses_and_stale_persists(self):
        s = self.confirmed_store()
        for i in range(10):
            s.update([], T0 + 400_000_000 + i, visibility=lambda _: 0)
        self.assertEqual(s.snapshot(T0)[0]["visible_misses"], 0)
        s.update([], T0 + 8_000_000_000)
        self.assertEqual(s.snapshot(T0 + 8_000_000_000)[0]["status"], "stale")

    def test_visible_misses_count_each_actual_frame_once(self):
        s = self.confirmed_store()
        for i in range(5):
            stamp = T0 + (i + 4) * 150_000_000
            for _ in range(3):
                s.update([], stamp, visibility=lambda t: stamp)
        self.assertEqual(s.snapshot(T0)[0]["visible_misses"], 5)
        self.assertEqual(s.snapshot(T0)[0]["status"], "stale")

    def test_completed_tombstone_blocks_reclean_and_epoch_resets(self):
        s = self.confirmed_store()
        key = next(iter(s.targets))
        s.set_status(key, "completed")
        s.update(fuse([observation(stamp=T0 + 600_000_000, seq=4)], []), T0 + 600_000_000)
        self.assertEqual(len(s.targets), 1)
        self.assertEqual(s.targets[key]["status"], "completed")
        s.reset_if_changed("new-map", "new-session", "test")
        self.assertEqual(s.targets, {})


class PipelineTests(unittest.TestCase):
    def test_missing_pose_still_has_2d_and_no_fake_depth(self):
        e = FixtureEngine()
        e.objects = [dict(bbox=[300, 200, 340, 240], detect_confidence=.8)]
        p = Pipeline(calibration(), e, PoseBuffer(), {})
        result = p.process(frame(), T0 + 10_000_000)
        self.assertEqual(len(result["detections"]), 1)
        self.assertEqual(result["observations"], [])
        self.assertEqual(p.store.targets, {})

    def test_missing_intrinsics_does_not_run_detector_on_distorted_image(self):
        c = calibration()
        c.cameras["front"]["intrinsics_valid"] = False
        class NeverEngine:
            def infer(self, *_):
                raise AssertionError("Detector called without calibrated perspective")
        p = Pipeline(c, NeverEngine(), PoseBuffer(), {})
        result = p.process(frame(), T0)
        self.assertEqual(result["state"], "awaiting_intrinsics")

    def test_fast_same_camera_frames_do_not_become_multiple_targets(self):
        e, b = FixtureEngine(), PoseBuffer()
        e.objects = [dict(bbox=[300, 200, 340, 240], detect_confidence=.8)]
        p = Pipeline(calibration(), e, b, {})
        for i in range(10):
            stamp = T0 + i * 40_000_000
            b.put(pose(stamp))
            p.process(frame(seq=i, stamp=stamp), stamp + 5_000_000)
        while p.pending:
            p.tick(stamp + 5_000_000, flush=True)
        self.assertEqual(len(p.store.targets), 1)
        self.assertEqual(next(iter(p.store.targets.values()))["status"], "confirmed")

    def test_ground_is_explicit_no_stereo_compatibility_flag(self):
        e, b = FixtureEngine(), PoseBuffer()
        e.objects = [dict(bbox=[300, 200, 340, 240], detect_confidence=.8)]
        b.put(pose())
        p = Pipeline(calibration(), e, b, {})
        r = p.process(frame(), T0)
        self.assertFalse(r["observations"][0]["depth_valid"])
        self.assertEqual(r["observations"][0]["position_source"], "calibrated_ground_plane")
        self.assertFalse(r["motion_eligible"])

    def test_map_switch_clears_targets_before_next_camera_frame(self):
        e, b = FixtureEngine(), PoseBuffer()
        e.objects = [dict(bbox=[300, 200, 340, 240], detect_confidence=.8)]
        b.put(pose())
        p = Pipeline(calibration(), e, b, {})
        p.process(frame(), T0)
        p.tick(T0, flush=True)
        self.assertEqual(len(p.store.targets), 1)
        new = pose(T0 + 50_000_000)
        new.map_id = "different_map"
        b.put(new)
        state = p.tick(T0 + 50_000_000)
        self.assertEqual(state["targets"], [])
        self.assertEqual(state["map_identity"][0], "different_map")

    def test_localization_loss_is_explicit_even_without_new_camera(self):
        b = PoseBuffer()
        b.put(pose())
        p = Pipeline(calibration(), FixtureEngine(), b, {})
        p.process(frame(), T0)
        invalid = pose(T0 + 50_000_000)
        invalid.valid = False
        b.put(invalid)
        state = p.tick(T0 + 50_000_000)
        self.assertFalse(state["localization_valid"])
        self.assertIsNone(state["display_pose"])

    def test_source_geometry_preserved_letterbox_inverse(self):
        import sys
        sys.path.insert(0, str(ROOT / "distribution/j6m_bpu_navigation/tools"))
        from board_preprocess import detector_input
        from production_io import detector_output
        image = np.zeros((1536, 1920, 3), np.uint8)
        _, g = detector_input(image)
        ratio, left, top, _, _ = g
        expected = np.array([500, 900, 700, 1100], np.float32)
        raw = np.zeros((1, 5, 12096), np.float32)
        raw[0, :4, 0] = [600 * ratio + left, 1000 * ratio + top, 200 * ratio, 200 * ratio]
        raw[0, 4, 0] = .9
        actual = detector_output(raw, g)[0, :4]
        np.testing.assert_allclose(actual, expected, atol=.001)

    def test_yuv_layout_and_nv12_payload(self):
        yuyv = np.array([16, 128, 235, 128] * 2, np.uint8)
        bgr = decode_yuv(yuyv, 2, 2, "YUYV")
        self.assertLess(int(bgr[0, 0].max()), 2)
        self.assertGreater(int(bgr[0, 1].min()), 250)
        bgr = decode_yuv(np.array([16, 16, 16, 16, 128, 128], np.uint8), 2, 2, "NV12")
        self.assertLess(int(bgr.max()), 2)
        bgr = decode_yuv(np.array([16] * 4 + [128] * 4, np.uint8), 2, 2, "NV16")
        self.assertLess(int(bgr.max()), 2)
        with self.assertRaises(ValueError):
            decode_yuv(yuyv, 2, 2, "guess")

    def test_default_hardware_config_is_blocked_and_models_preserved(self):
        c = load_config(DEFAULT_CONFIG, ROOT)
        self.assertTrue(capture_issues(c["capture"]))
        self.assertEqual(c["inference"]["detector_zed"]["model_name"], "detector_zed")
        self.assertIsNone(c["capture"]["cameras"]["front"]["pipeline"])

    def test_result_creation_time_includes_inference(self):
        from unittest.mock import patch
        e, b = FixtureEngine(), PoseBuffer()
        e.objects = []
        b.put(pose())
        p = Pipeline(calibration(), e, b, {})
        with patch("distribution.j6m_bpu_navigation.surround.pipeline.time.time_ns",
                   side_effect=[T0 + 10_000_000, T0 + 30_000_000, T0 + 35_000_000]):
            r = p.process(frame())
        self.assertEqual(r["produced_ns"], T0 + 35_000_000)
        self.assertEqual(r["source_age_ms"], 35.)


class BpuBudgetTests(unittest.TestCase):
    def engine(self):
        from unittest.mock import Mock
        from distribution.j6m_bpu_navigation.surround.inference import BpuEngine
        import sys
        # Device construction is replaced for this test; retain its ordinary
        # local helper search paths without loading any SDK or opening a lock.
        sys.path.insert(0, str(ROOT / "distribution/j6m_bpu_navigation/tools"))
        sys.path.insert(0, str(ROOT / "src/application/autolabor_fod_vision/src"))
        e = BpuEngine.__new__(BpuEngine)
        e.config = dict(confidence=.2, max_crops=2, classify_budget_ms=1000)
        e.identity = {"test": "only_fake_device_outputs"}
        raw = np.zeros((1, 5, 12096), np.float32)
        for i in range(6):
            raw[0, :, i] = [170 + i * 110, 250, 60, 80, .9]
        detector = Mock()
        detector.infer.return_value = ([raw], {})
        classifier = Mock()
        classifier.infer.return_value = ([np.array([[.8, .05, .05, .05, .05]], np.float32)], {})
        e.engines = dict(detector_zed=detector, classifier=classifier)
        return e

    def test_crop_limit_keeps_unclassified_detections(self):
        from unittest.mock import patch
        e = self.engine()
        with patch("distribution.j6m_bpu_navigation.surround.inference.time.time_ns", return_value=T0):
            objects, timing = e.infer(frame().image, T0 + 100_000_000)
        self.assertEqual(len(objects), 6)
        self.assertEqual(e.engines["classifier"].infer.call_count, 2)
        self.assertEqual(timing["unclassified"], 4)
        self.assertTrue(all(o["material"] == "unknown" for o in objects[2:]))

    def test_expired_source_never_starts_detector(self):
        from unittest.mock import patch
        e = self.engine()
        with patch("distribution.j6m_bpu_navigation.surround.inference.time.time_ns", return_value=T0):
            with self.assertRaises(ValueError):
                e.infer(frame().image, T0)
        e.engines["detector_zed"].infer.assert_not_called()

    def test_classification_time_budget(self):
        from unittest.mock import patch
        e = self.engine()
        e.config["classify_budget_ms"] = 0
        with patch("distribution.j6m_bpu_navigation.surround.inference.time.time_ns", return_value=T0):
            objects, timing = e.infer(frame().image, T0 + 100_000_000)
        self.assertEqual(timing["unclassified"], 6)
        e.engines["classifier"].infer.assert_not_called()


if __name__ == "__main__":
    unittest.main()
