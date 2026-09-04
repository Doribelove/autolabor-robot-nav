#!/usr/bin/env python3

from hashlib import sha256
from pathlib import Path
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "detect_and_classify.yaml"
NODE = PACKAGE / "scripts" / "fod_detect_and_classify_node.py"
RUNTIME = PACKAGE / "src" / "autolabor_fod_vision" / "two_stage_runtime.py"


def digest(path):
    value = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


class TwoStageRuntimeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
            "detect_and_classify"
        ]
        cls.node_source = NODE.read_text(encoding="utf-8")
        cls.runtime_source = RUNTIME.read_text(encoding="utf-8")

    def test_yaml_has_latest_frame_and_disabled_ground_roi_contract(self):
        runtime = self.config["runtime"]
        self.assertEqual(runtime["frame_queue_size"], 1)
        self.assertTrue(runtime["drop_old_frames"])
        self.assertEqual(runtime["max_frame_age_ms"], 150)
        self.assertEqual(runtime["max_result_age_ms"], 350)
        self.assertIs(self.config["ground_roi"]["enabled"], False)
        self.assertEqual(self.config["detector"]["conf"], 0.20)

    def test_pinned_weights_and_class_order_match_disk(self):
        detector = self.config["detector"]
        classifier = self.config["classifier"]
        detector_path = Path(detector["model"])
        classifier_path = Path(classifier["model"])
        self.assertEqual(digest(detector_path), detector["expected_sha256"])
        self.assertEqual(digest(classifier_path), classifier["expected_sha256"])
        self.assertEqual(
            classifier["class_names"],
            ["metal", "plastic", "paper", "glass", "kitchen_waste"],
        )

    def test_models_are_loaded_in_constructor_and_not_in_frame_callback(self):
        self.assertIn("TwoStageUltralyticsRuntime(", self.node_source)
        callback = self.node_source[
            self.node_source.index("def _image_callback") :
            self.node_source.index("def _depth_callback")
        ]
        self.assertNotIn("YOLO(", callback)
        self.assertNotIn("predict(", callback)
        self.assertNotIn("imgmsg_to_cv2", callback)
        self.assertIn('self.load_counts = {"detector": 0, "classifier": 0}', self.runtime_source)

    def test_detector_confidence_is_runtime_adjustable_through_global_service(self):
        for text in (
            "GLOBAL_CONFIDENCE_PARAM",
            "CONFIDENCE_SERVICE",
            "DetectionConfidenceController(",
            "self.runtime.set_detector_confidence",
            '"detector_confidence_supported"',
            '"detector_confidence"',
        ):
            self.assertIn(text, self.node_source)
        self.assertIn("def set_detector_confidence", self.runtime_source)
        self.assertIn("detector_confidence = self.detector_confidence", self.runtime_source)

    def test_live_crop_path_has_no_image_file_write(self):
        combined = self.node_source + self.runtime_source
        self.assertNotIn("cv2.imwrite", combined)
        self.assertNotIn("Image.save", combined)
        self.assertIn("context_crop(", self.node_source)
        self.assertIn("source=list(crops_bgr)", self.runtime_source)

    def test_source_timestamp_depth_tf_and_backend_gate_are_explicit(self):
        for text in (
            "frame.stamp_sec",
            "lookup_transform(",
            "rospy.Time.from_sec(stamp_sec)",
            "depth_tolerance_sec",
            'BACKEND_ID = "detect_and_classify"',
            'legacy.model_task = BACKEND_ID',
            'legacy.depth_synchronized = False',
        ):
            self.assertIn(text, self.node_source)

    def test_object_depth_and_tf_cache_contract(self):
        depth = self.config["depth"]
        transform = self.config["transform"]
        self.assertEqual(depth["lock_valid_samples"], 5)
        self.assertEqual(depth["lock_min_inliers"], 3)
        self.assertEqual(depth["validation_interval_frames"], 12)
        self.assertEqual(depth["validation_failures_before_reacquire"], 2)
        self.assertTrue(transform["per_source_frame_lookup_cache"])
        self.assertEqual(transform["world_lock_samples"], 3)
        self.assertEqual(transform["max_consecutive_failures"], 10)
        self.assertGreater(transform["failure_backoff_sec"], 0.0)

        process_source = self.node_source[
            self.node_source.index("def _process_frame") :
            self.node_source.index("def _worker_loop")
        ]
        self.assertLess(
            process_source.index("self.runtime.detect("),
            process_source.index("self._matching_sensor_bundle(frame)"),
        )
        self.assertEqual(
            process_source.count("self._lookup_source_transform("), 1
        )
        self.assertIn("known_target.world_locked", process_source)
        self.assertIn("target.depth_locked", process_source)
        self.assertIn("Qt receives N/A", process_source)

        lookup_source = self.node_source[
            self.node_source.index("def _lookup_source_transform") :
            self.node_source.index("def _roi_for_box")
        ]
        self.assertIn("rospy.Time.from_sec(stamp_sec)", lookup_source)
        self.assertIn("self.tf_max_consecutive_failures", lookup_source)
        self.assertIn("self.tf_backoff_until_monotonic", lookup_source)
        self.assertEqual(lookup_source.count("lookup_transform("), 1)
        self.assertNotIn("def _transform_camera_point", self.node_source)


if __name__ == "__main__":
    unittest.main()
