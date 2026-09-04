#!/usr/bin/env python3

from pathlib import Path
import tempfile
import unittest

import numpy as np

from autolabor_fod_vision.clip_filter import (
    ClipDetectionFilter,
    resolve_clip_layout,
    validate_clip_provenance,
)
from autolabor_fod_vision.detector import Detection


def detection(confidence, x1=2.0, y1=2.0, x2=8.0, y2=8.0):
    return Detection(
        class_id=0,
        class_name="metal",
        confidence=confidence,
        xmin=x1,
        ymin=y1,
        xmax=x2,
        ymax=y2,
        anchor_u=(x1 + x2) / 2.0,
        anchor_v=y2,
    )


class FakeRuntime:
    def __init__(self, probabilities):
        self.probabilities = list(probabilities)
        self.calls = []

    def score_positive(self, crops):
        self.calls.append(list(crops))
        return self.probabilities[: len(crops)], 12.5 if crops else 0.0


class ClipDetectionFilterTest(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((12, 12, 3), dtype=np.uint8)

    def test_high_confidence_skips_clip_and_is_kept(self):
        runtime = FakeRuntime([])
        result = ClipDetectionFilter(runtime).filter(
            self.image, [detection(0.6001)]
        )
        self.assertEqual(len(result.detections), 1)
        self.assertEqual(result.stats.high_confidence_kept, 1)
        self.assertEqual(runtime.calls, [])

    def test_low_confidence_skips_clip_and_is_dropped(self):
        runtime = FakeRuntime([])
        result = ClipDetectionFilter(runtime).filter(
            self.image, [detection(0.1999)]
        )
        self.assertEqual(result.detections, [])
        self.assertEqual(result.stats.low_confidence_dropped, 1)
        self.assertEqual(runtime.calls, [])

    def test_exact_boundaries_are_batched_for_clip(self):
        runtime = FakeRuntime([0.51, 0.49])
        candidates = [detection(0.20), detection(0.60)]
        result = ClipDetectionFilter(runtime).filter(self.image, candidates)
        self.assertEqual(result.detections, [candidates[0]])
        self.assertEqual(result.stats.clip_candidates, 2)
        self.assertEqual(result.stats.clip_kept, 1)
        self.assertEqual(result.stats.clip_dropped, 1)
        self.assertEqual(len(runtime.calls), 1)
        self.assertEqual(len(runtime.calls[0]), 2)

    def test_output_order_is_preserved_across_gate_paths(self):
        runtime = FakeRuntime([0.8, 0.2])
        candidates = [
            detection(0.80),
            detection(0.40),
            detection(0.10),
            detection(0.60),
        ]
        result = ClipDetectionFilter(runtime).filter(self.image, candidates)
        self.assertEqual(result.detections, candidates[:2])
        self.assertEqual(result.stats.output_count, 2)

    def test_invalid_medium_crop_is_dropped_without_clip(self):
        runtime = FakeRuntime([])
        result = ClipDetectionFilter(runtime).filter(
            self.image, [detection(0.40, x1=5.0, x2=5.0)]
        )
        self.assertEqual(result.detections, [])
        self.assertEqual(result.stats.invalid_crop_dropped, 1)
        self.assertEqual(runtime.calls, [])

    def test_uncalibrated_detector_sends_every_valid_box_to_clip(self):
        runtime = FakeRuntime([0.90, 0.10])
        candidates = [detection(0.0), detection(0.95)]
        result = ClipDetectionFilter(
            runtime,
            use_confidence_gate=False,
        ).filter(self.image, candidates)

        self.assertEqual(result.detections, [candidates[0]])
        self.assertEqual(result.stats.clip_candidates, 2)
        self.assertEqual(result.stats.high_confidence_kept, 0)
        self.assertEqual(result.stats.low_confidence_dropped, 0)
        self.assertEqual(len(runtime.calls), 1)

    def test_rejects_invalid_threshold_order(self):
        with self.assertRaises(ValueError):
            ClipDetectionFilter(FakeRuntime([]), 0.7, 0.6)


class ClipRuntimeLayoutTest(unittest.TestCase):
    def test_accepts_isolated_python_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            package = root / "clip"
            package.mkdir()
            (package / "__init__.py").write_text("", encoding="utf-8")
            import_root, package_root = resolve_clip_layout(str(root))
            self.assertEqual(import_root, root)
            self.assertEqual(package_root, package)

    def test_rejects_missing_package(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileNotFoundError):
                resolve_clip_layout(temporary)

    def test_accepts_official_pinned_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            package = root / "clip"
            metadata = root / "clip-1.0.dist-info"
            package.mkdir()
            metadata.mkdir()
            (package / "__init__.py").write_text("", encoding="utf-8")
            commit = "d05afc436d78f1c48dc0dbf8e5980a9d471f35f6"
            (metadata / "direct_url.json").write_text(
                '{"url":"https://github.com/openai/CLIP.git",'
                '"vcs_info":{"commit_id":"' + commit + '"}}',
                encoding="utf-8",
            )
            self.assertEqual(
                validate_clip_provenance(str(root), commit),
                "https://github.com/openai/CLIP.git",
            )

    def test_rejects_non_openai_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            package = root / "clip"
            metadata = root / "clip-1.0.dist-info"
            package.mkdir()
            metadata.mkdir()
            (package / "__init__.py").write_text("", encoding="utf-8")
            commit = "d05afc436d78f1c48dc0dbf8e5980a9d471f35f6"
            (metadata / "direct_url.json").write_text(
                '{"url":"https://github.com/example/CLIP.git",'
                '"vcs_info":{"commit_id":"' + commit + '"}}',
                encoding="utf-8",
            )
            with self.assertRaises(RuntimeError):
                validate_clip_provenance(str(root), commit)


if __name__ == "__main__":
    unittest.main()
