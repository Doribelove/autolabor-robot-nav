import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

import cv2
import numpy as np

from distribution.j6m_bpu_navigation.surround import CAMERAS
from distribution.j6m_bpu_navigation.surround.bev import BevComposer
from distribution.j6m_bpu_navigation.surround.bev_cli import Inbox, Output, read_frames, run_stream
from distribution.j6m_bpu_navigation.surround.bev_demo import make_scene
from distribution.j6m_bpu_navigation.surround.contracts import Calibration, read_json
from distribution.j6m_bpu_navigation.surround.geometry import Rectifier


class BevTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.manifest, cls.cal_file = make_scene(Path(cls.temp.name) / "input")
        cls.cal = Calibration(read_json(cls.cal_file))
        cls.config = read_json(Path(__file__).resolve().parents[1] / "config/bev.json")
        cls.composer = BevComposer(cls.cal, cls.config)
        cls.frames, cls.now = read_frames(cls.manifest)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_raw_fisheye_recovers_four_metric_landmarks(self):
        bird, counts, meta = self.composer.compose(self.frames, self.now)
        for x, y, colour in ((2, 0, [30, 80, 240]), (0, -2, [40, 220, 70]),
                             (-2, 0, [230, 100, 30]), (0, 2, [40, 220, 230])):
            u, v = map(round, self.composer.pixel(x, y))
            self.assertGreater(counts[v, u], 0)
            np.testing.assert_allclose(bird[v, u], colour, atol=2)
        self.assertGreater(meta["coverage_fraction"], .9)
        self.assertTrue(meta["synthetic"])
        self.assertEqual(meta["sources"]["front"]["raw_stamp"], self.frames["front"].raw_stamp)

    def test_unobserved_centre_remains_black_with_zero_coverage(self):
        bird, counts, _ = self.composer.compose(self.frames, self.now)
        u, v = map(round, self.composer.pixel(0, 0))
        self.assertEqual(int(counts[v, u]), 0)
        self.assertFalse(bird[v, u].any())

    def test_raw_and_rectified_paths_agree_on_landmarks(self):
        rectifier = Rectifier(self.cal, offline_reference=True)
        frames = copy.deepcopy(self.frames)
        for name, f in frames.items():
            f.image = rectifier.apply(f)
            f.rectified = True
            f.rectification_id = self.cal.cameras[name]["rectification_id"]
        raw, _, _ = self.composer.compose(self.frames, self.now)
        rectified, _, _ = self.composer.compose(frames, self.now)
        for x, y in ((2, 0), (0, -2), (-2, 0), (0, 2)):
            u, v = map(round, self.composer.pixel(x, y))
            np.testing.assert_allclose(raw[v-3:v+4, u-3:u+4], rectified[v-3:v+4, u-3:u+4], atol=2)

    def test_live_bev_only_maps_perspective_images_to_ground(self):
        frames = copy.deepcopy(self.frames)
        reference = Rectifier(self.cal, offline_reference=True)
        for name, f in frames.items():
            f.image = reference.apply(f)
            f.rectified = True
            f.rectification_id = self.cal.cameras[name]["rectification_id"]
        with mock.patch("cv2.fisheye.projectPoints", side_effect=AssertionError("CPU fisheye map")):
            live = BevComposer(self.cal, self.composer.config, perspective_only=True)
            actual, counts, _ = live.compose(frames, self.now)
        expected, expected_counts, _ = self.composer.compose(frames, self.now)
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(counts, expected_counts)
        with self.assertRaisesRegex(ValueError, "GDC perspective"):
            live.compose(self.frames, self.now)

    def test_overlap_normalization_does_not_brighten_constant_input(self):
        frames = copy.deepcopy(self.frames)
        for f in frames.values():
            f.image[:] = [75, 130, 200]
        bird, counts, _ = self.composer.compose(frames, self.now)
        self.assertTrue((counts > 1).any())
        np.testing.assert_allclose(bird[counts > 0], np.broadcast_to([75, 130, 200], bird[counts > 0].shape), atol=1)

    def test_missing_camera_is_not_filled_with_an_old_image(self):
        with self.assertRaisesRegex(ValueError, "all four"):
            self.composer.compose({"front": self.frames["front"]}, self.now)

    def test_source_skew_and_stale_frames_rejected(self):
        frames = copy.deepcopy(self.frames)
        frames["front"].stamp_ns -= 60_000_000
        with self.assertRaisesRegex(ValueError, "skew"):
            self.composer.compose(frames, self.now)
        with self.assertRaisesRegex(ValueError, "stale"):
            self.composer.compose(self.frames, self.now + 600_000_000)

    def test_calibration_identity_and_synthetic_mixing_rejected(self):
        frames = copy.deepcopy(self.frames)
        frames["front"].calibration_id = "different-calibration"
        with self.assertRaisesRegex(ValueError, "identity"):
            self.composer.compose(frames, self.now)
        frames["front"].calibration_id = self.cal.id
        frames["front"].synthetic = False
        with self.assertRaisesRegex(ValueError, "mixed"):
            self.composer.compose(frames, self.now)

    def test_wrong_rectification_or_image_shape_rejected(self):
        frames = copy.deepcopy(self.frames)
        frames["front"].rectified = True
        with self.assertRaisesRegex(ValueError, "Rectified"):
            self.composer.compose(frames, self.now)
        frames["front"].rectified = False
        frames["front"].image = frames["front"].image[:100]
        with self.assertRaisesRegex(ValueError, "dimensions"):
            self.composer.compose(frames, self.now)

    def test_pending_calibration_has_no_geometry_fallback(self):
        cal = Calibration(read_json(Path(__file__).resolve().parents[1] / "config/calibration.template.json"))
        with self.assertRaisesRegex(ValueError, "Intrinsics unavailable"):
            BevComposer(cal, self.config)

    def test_oversized_grid_rejected_before_map_allocation(self):
        config = copy.deepcopy(self.config)
        config["x_range_m"] = [-1000, 1000]
        with self.assertRaisesRegex(ValueError, "dimensions"):
            BevComposer(self.cal, config)

    def test_manifest_cannot_reference_images_outside_dataset(self):
        data = read_json(self.manifest)
        data["frames"]["front"]["image"] = "../../outside.png"
        path = self.manifest.parent / "escape.json"
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "escapes"):
            read_frames(path)

    def test_inbox_rejects_restart_reorder_and_synthetic(self):
        inbox = Inbox()
        f = copy.deepcopy(self.frames["front"])
        with self.assertRaisesRegex(ValueError, "Synthetic"):
            inbox.put(f, self.now)
        f.synthetic = False
        inbox.put(f, self.now)
        with self.assertRaisesRegex(ValueError, "non-increasing"):
            inbox.put(f, self.now)
        other = copy.deepcopy(f)
        other.session = "new-session"
        other.sequence += 1
        other.stamp_ns += 1
        with self.assertRaisesRegex(ValueError, "session"):
            inbox.put(other, self.now)
        self.assertEqual(len(inbox.snapshot()), 1)

    def test_output_is_traceable_and_cannot_overwrite_a_session(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result"
            out = Output(path, self.cal, True)
            bird, coverage, meta = self.composer.compose(self.frames, self.now)
            out.save(bird, coverage, meta, self.frames)
            self.assertTrue(read_json(path / "bev.json")["offline"])
            self.assertFalse(read_json(path / "state.json")["localization_valid"])
            self.assertEqual(cv2.imread(str(path / "coverage.png"), 0).max(), coverage.max())
            with self.assertRaises(FileExistsError):
                Output(path, self.cal, True)

    def test_live_deadline_includes_computation_before_publish(self):
        bird, coverage, meta = self.composer.compose(self.frames, self.now)
        meta["max_source_age_ms"] = 50
        with tempfile.TemporaryDirectory() as directory:
            out = Output(Path(directory) / "result", self.cal, False)
            with mock.patch("distribution.j6m_bpu_navigation.surround.bev_cli.time.time_ns",
                            return_value=meta["source_oldest_ns"] + 51_000_000):
                with self.assertRaisesRegex(ValueError, "expired"):
                    out.save(bird, coverage, meta, self.frames)
            self.assertFalse((out.path / "bev.png").exists())

    def test_frozen_stream_not_republished_and_stops_invalid(self):
        # Temporary unit fixture only: never used by a camera or saved as a real calibration.
        data = copy.deepcopy(self.cal.data)
        data["synthetic"] = False
        config = dict(self.config, max_source_age_ms=300, output_hz=30)
        composer = BevComposer(Calibration(data), config)
        frames = copy.deepcopy(self.frames)
        now = time.time_ns()
        for f in frames.values():
            f.synthetic = False
            f.stamp_ns, f.received_ns = now, now
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stream"
            code = run_stream(composer, path, .55, lambda: frames)
            state = read_json(path / "state.json")
            self.assertEqual(code, 0)
            self.assertEqual(state["frames_produced"], 1)
            self.assertTrue(state["service_stopped"])
            self.assertFalse(state["bev_valid"])
            self.assertIn("stale", state["error"])
            self.assertEqual(read_json(path / "bev.json")["source_oldest_ns"], now)


if __name__ == "__main__":
    unittest.main()
