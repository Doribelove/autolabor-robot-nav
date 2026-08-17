#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "gps_geofence_recorder.py"
SPEC = importlib.util.spec_from_file_location("gps_geofence_recorder", str(SCRIPT))
RECORDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECORDER)


class GpsGeofenceRecorderTest(unittest.TestCase):
    def test_convex_hull_keeps_maximum_outer_polygon(self):
        samples = [
            {"latitude": 30.0, "longitude": 104.0},
            {"latitude": 30.0, "longitude": 104.001},
            {"latitude": 30.001, "longitude": 104.001},
            {"latitude": 30.001, "longitude": 104.0},
            {"latitude": 30.0005, "longitude": 104.0005},
        ]
        hull = RECORDER.convex_hull(samples)
        self.assertEqual(len(hull), 4)
        self.assertNotIn(samples[-1], hull)

    def test_multiple_regions_are_persisted_independently(self):
        document = RECORDER.empty_document()
        first = RECORDER.begin_region(document, "apron_a")
        second = RECORDER.begin_region(document, "runway_edge")
        self.assertIsNot(first, second)
        for latitude, longitude in (
            (30.0, 104.0),
            (30.0, 104.001),
            (30.001, 104.0),
        ):
            RECORDER.append_sample(document, "apron_a", latitude, longitude)
        vertices = RECORDER.close_region(document, "apron_a")
        self.assertEqual(len(vertices), 3)
        self.assertTrue(RECORDER.find_region(document, "apron_a")["enabled"])
        self.assertFalse(RECORDER.find_region(document, "runway_edge")["enabled"])

    def test_atomic_yaml_round_trip(self):
        document = RECORDER.empty_document()
        RECORDER.begin_region(document, "zone_1")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fences.yaml"
            RECORDER.save_document(path, document)
            loaded = RECORDER.load_document(path)
        self.assertEqual(loaded, document)

    def test_collinear_samples_are_rejected(self):
        samples = [
            {"latitude": 30.0, "longitude": 104.0},
            {"latitude": 30.001, "longitude": 104.001},
            {"latitude": 30.002, "longitude": 104.002},
        ]
        with self.assertRaises(ValueError):
            RECORDER.convex_hull(samples)

    def test_parser_accepts_gui_fix_and_live_reload(self):
        arguments = RECORDER.make_parser().parse_args(
            [
                "--file",
                "/tmp/fences.yaml",
                "--apply-live",
                "add",
                "zone_1",
                "--latitude",
                "30.123456789",
                "--longitude",
                "104.987654321",
            ]
        )
        self.assertTrue(arguments.apply_live)
        self.assertEqual(arguments.command, "add")
        self.assertAlmostEqual(arguments.latitude, 30.123456789)
        self.assertAlmostEqual(arguments.longitude, 104.987654321)


if __name__ == "__main__":
    unittest.main()
