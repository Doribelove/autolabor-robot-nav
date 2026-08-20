#!/usr/bin/env python3

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/fast_lio_map_localizer.cpp").read_text(encoding="utf-8")
LAUNCH = (ROOT / "launch/known_map_localization.launch").read_text(encoding="utf-8")
README = (ROOT / "README.md").read_text(encoding="utf-8")


class KnownMapLocalizerContractTest(unittest.TestCase):
    def test_uses_multiscale_icp_and_map_to_odom(self):
        self.assertGreaterEqual(SOURCE.count("IterativeClosestPoint"), 2)
        self.assertIn("coarse_max_correspondence_", SOURCE)
        self.assertIn("fine_max_correspondence_", SOURCE)
        self.assertIn("map_to_odom_", SOURCE)
        self.assertIn('transform.header.frame_id = map_frame_', SOURCE)
        self.assertIn('transform.child_frame_id = odom_frame_', SOURCE)

    def test_requires_prior_map_and_initial_pose(self):
        self.assertIn('private_nh_.param<std::string>("map_file"', SOURCE)
        self.assertIn("initialPoseCallback", SOURCE)
        self.assertIn("WAITING_INITIAL_POSE", SOURCE)
        self.assertIn('required="true"', LAUNCH)

    def test_has_upstream_attribution_and_no_open3d_runtime(self):
        self.assertIn("HViktorTsoi/FAST_LIO_LOCALIZATION", README)
        self.assertNotIn("open3d", SOURCE.lower())


if __name__ == "__main__":
    unittest.main()
