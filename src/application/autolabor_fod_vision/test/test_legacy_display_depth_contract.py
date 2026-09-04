#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "detector_smoke.yaml"
LAUNCH = PACKAGE / "launch" / "perception.launch"
ZED_LAUNCH = PACKAGE / "launch" / "zed_fod_detection.launch"
ADAPTER = PACKAGE / "scripts" / "fod_vision_result_adapter_node.py"


class LegacyDisplayDepthContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.launch_source = LAUNCH.read_text(encoding="utf-8")
        cls.zed_launch_source = ZED_LAUNCH.read_text(encoding="utf-8")
        cls.adapter_source = ADAPTER.read_text(encoding="utf-8")

    def test_bounded_source_history_and_median_cluster_are_configured(self):
        depth = self.config["display_depth"]
        self.assertTrue(depth["enabled"])
        self.assertEqual(depth["buffer_size"], 120)
        self.assertEqual(depth["aggregation"], "median")
        self.assertEqual(depth["camera_info_topic"], "/fod_camera/camera_info")
        self.assertLessEqual(depth["sync_tolerance_sec"], 0.06)

    def test_adapter_matches_depth_and_intrinsics_to_detection_source(self):
        for text in (
            "source.header.stamp.to_sec()",
            "source.header.frame_id",
            "nearest_synchronized_message(",
            "estimate_clustered_depth(",
            "camera_info.K",
            "aggregation=self.depth_aggregation",
        ):
            self.assertIn(text, self.adapter_source)
        self.assertNotIn("rospy.Time.now().to_sec()", self.adapter_source)

    def test_depth_is_qt_only_and_locateanything_remains_motion_isolated(self):
        self.assertIn(
            'and self.backend_id == "locateanything"', self.adapter_source
        )
        self.assertIn(
            '"/fod/vision/results", FodVisionDetectionArray', self.adapter_source
        )
        self.assertNotIn(
            '"/fod/detections", FodVisionDetectionArray', self.adapter_source
        )
        self.assertIn(
            'rospy.set_param("~display_depth_motion_isolated", True)',
            self.adapter_source,
        )

    def test_sensor_callbacks_only_append_to_bounded_buffers(self):
        callback_source = self.adapter_source[
            self.adapter_source.index("def _depth_callback") :
            self.adapter_source.index("def _matching_sensor_bundle")
        ]
        self.assertIn("self._depth_messages.append(message)", callback_source)
        self.assertIn("self._camera_info_messages.append(message)", callback_source)
        self.assertIn("previous.header.stamp == message.header.stamp", callback_source)
        self.assertIn("self.camera_info_duplicates_dropped += 1", callback_source)
        self.assertNotIn("imgmsg_to_cv2", callback_source)
        self.assertNotIn("estimate_clustered_depth", callback_source)

    def test_launch_passes_registered_depth_and_camera_info_to_adapter(self):
        self.assertIn('arg name="camera_info_topic"', self.launch_source)
        self.assertIn('name="display_depth/enabled"', self.launch_source)
        self.assertIn('name="display_depth/depth_topic"', self.launch_source)
        self.assertIn('name="display_depth/camera_info_topic"', self.launch_source)

    def test_locateanything_uses_only_the_long_display_depth_history(self):
        self.assertIn(
            'arg name="enable_detector_depth_fusion"', self.launch_source
        )
        self.assertIn('arg name="enable_display_depth"', self.launch_source)
        self.assertIn(
            'value="$(arg enable_detector_depth_fusion)"', self.launch_source
        )
        self.assertIn(
            'value="$(arg enable_display_depth)"', self.launch_source
        )
        self.assertIn(
            "default=\"$(eval arg('backend') != 'locateanything')\"",
            self.zed_launch_source,
        )
        self.assertIn(
            '<arg name="enable_display_depth" default="true"/>',
            self.zed_launch_source,
        )
        self.assertEqual(self.config["display_depth"]["buffer_size"], 120)


if __name__ == "__main__":
    unittest.main()
