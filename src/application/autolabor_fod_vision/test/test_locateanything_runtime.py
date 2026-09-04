#!/usr/bin/env python3

from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest

from autolabor_fod_vision.locateanything_runtime import (
    LocateAnythingCategory,
    LocateAnythingDetector,
    parse_categories,
    parse_locateanything_answer,
    validate_max_image_side,
    verify_model_manifest,
)
from autolabor_fod_vision.locateanything_worker import (
    _resize_source_image,
    _semantic_prompt_parts,
)


class LocateAnythingManifestTest(unittest.TestCase):
    REQUIRED_FILES = (
        "config.json",
        "model.safetensors.index.json",
        "model-00001-of-00002.safetensors",
        "model-00002-of-00002.safetensors",
        "tokenizer_config.json",
        "preprocessor_config.json",
        "processor_config.json",
        "modeling_locateanything.py",
        "processing_locateanything.py",
    )

    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name).resolve()
        entries = []
        for index, relative in enumerate(self.REQUIRED_FILES):
            payload = "test-file-{}\n".format(index).encode("utf-8")
            path = self.root / relative
            path.write_bytes(payload)
            entries.append(
                {
                    "path": relative,
                    "size": len(payload),
                    "sha256": sha256(payload).hexdigest(),
                }
            )
        manifest_data = {
            "schema_version": 1,
            "repo_id": "nvidia/LocateAnything-3B",
            "revision": "c32291ca5e996f5a7a485845b4f57a233936bba0",
            "files": entries,
        }
        self.manifest = self.root / "deployment_manifest.json"
        self.manifest.write_text(
            json.dumps(manifest_data, sort_keys=True), encoding="utf-8"
        )
        self.digest = sha256(self.manifest.read_bytes()).hexdigest()

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_verifies_manifest_identity_and_all_declared_files(self):
        verified = verify_model_manifest(
            str(self.root), str(self.manifest), self.digest, verify_files=True
        )

        self.assertEqual(verified.digest, self.digest)
        self.assertEqual(verified.repo_id, "nvidia/LocateAnything-3B")
        self.assertEqual(len(verified.files), len(self.REQUIRED_FILES))

    def test_rejects_same_size_file_tampering(self):
        target = self.root / self.REQUIRED_FILES[0]
        target.write_bytes(b"x" * target.stat().st_size)

        with self.assertRaisesRegex(RuntimeError, "SHA256 mismatch"):
            verify_model_manifest(
                str(self.root), str(self.manifest), self.digest, verify_files=True
            )


class LocateAnythingAnswerParserTest(unittest.TestCase):
    def setUp(self):
        self.categories = parse_categories(
            [
                {
                    "class_id": 0,
                    "class_name": "metal",
                    "prompt": "metal trash on the ground",
                    "aliases": ["metal can"],
                },
                {
                    "class_id": 1,
                    "class_name": "plastic",
                    "prompt": "plastic trash on the ground",
                    "aliases": ["plastic bag"],
                },
                {
                    "class_id": 4,
                    "class_name": "kitchen_waste",
                    "prompt": "food scraps on the ground",
                    "aliases": ["kitchen waste"],
                },
            ]
        )

    def test_maps_aliases_and_rejects_duplicate_unknown_and_full_frame_boxes(self):
        answer = (
            "<ref>plastic bag</ref><box><100><200><300><600></box>"
            "<box><100><200><300><600></box>"
            "<ref>unmapped debris</ref><box><500><500><600><600></box>"
            "<ref>kitchen waste</ref><box><0><0><1000><1000></box>"
        )

        detections, ignored = parse_locateanything_answer(
            answer, self.categories, image_width=1000, image_height=500
        )

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["class_id"], 1)
        self.assertEqual(detections[0]["class_name"], "plastic")
        self.assertEqual(detections[0]["xmin"], 100.0)
        self.assertEqual(detections[0]["ymax"], 300.0)
        self.assertEqual(ignored, 3)

    def test_single_category_accepts_an_unlabelled_box(self):
        category = (LocateAnythingCategory(2, "paper", "paper litter"),)

        detections, ignored = parse_locateanything_answer(
            "<box><100><100><250><300></box>",
            category,
            image_width=640,
            image_height=480,
        )

        self.assertEqual(ignored, 0)
        self.assertEqual([item["class_name"] for item in detections], ["paper"])

    def test_single_category_maps_specific_vlm_labels_to_one_output_class(self):
        category = (
            LocateAnythingCategory(
                0,
                "trash",
                "trash on the ground that needs cleaning",
                ("trash", "garbage", "litter"),
            ),
        )

        detections, ignored = parse_locateanything_answer(
            "<ref>plastic food wrapper</ref><box><100><100><250><300></box>",
            category,
            image_width=640,
            image_height=480,
        )

        self.assertEqual(ignored, 0)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["class_id"], 0)
        self.assertEqual(detections[0]["class_name"], "trash")

    def test_rejects_invalid_category_contracts(self):
        with self.assertRaisesRegex(ValueError, "unique"):
            parse_categories(
                [
                    {"class_id": 0, "class_name": "metal", "prompt": "metal"},
                    {"class_id": 0, "class_name": "glass", "prompt": "glass"},
                ]
            )

    def test_preserves_multiple_short_queries_for_one_output_class(self):
        categories = parse_categories(
            [
                {
                    "class_id": 0,
                    "class_name": "trash",
                    "query_prompts": [
                        "a paper ball lying on the floor",
                        "a discarded bottle lying on the floor",
                    ],
                }
            ]
        )

        self.assertEqual(categories[0].prompt, categories[0].query_prompts[0])
        self.assertEqual(len(categories[0].grounding_prompts), 2)


class LocateAnythingSemanticPreloadTest(unittest.TestCase):
    def _detector_without_worker(self):
        detector = object.__new__(LocateAnythingDetector)
        detector.categories = (
            LocateAnythingCategory(
                0,
                "trash",
                "discarded trash or litter lying on the floor",
                ("trash", "litter"),
            ),
        )
        detector.generation_mode = "hybrid"
        detector.max_new_tokens = 128
        detector.temperature = 0.0
        detector.max_image_side = 448
        detector.max_detections = 100
        detector.min_box_area_fraction = 0.00005
        detector.max_box_area_fraction = 0.75
        return detector

    def test_configuration_contains_semantics_exactly_once(self):
        request = self._detector_without_worker()._configuration_request()

        self.assertEqual(request["op"], "configure")
        self.assertEqual(len(request["categories"]), 1)
        self.assertIn("discarded trash", request["categories"][0]["prompt"])
        self.assertEqual(request["max_image_side"], 448)

    def test_realtime_request_contains_only_frame_data(self):
        request = LocateAnythingDetector._prediction_request(7, b"jpeg")

        self.assertEqual(
            set(request), {"op", "id", "image_jpeg_b64"}
        )
        self.assertEqual(request["op"], "predict")
        self.assertEqual(request["id"], 7)
        self.assertNotIn("categories", request)
        self.assertNotIn("generation_mode", request)

    def test_semantic_template_is_split_around_only_the_image_span(self):
        class FakeProcessor:
            image_placeholder = "image"
            image_start_token = "<img>"
            image_end_token = "</img>"

            @staticmethod
            def py_apply_chat_template(
                messages, tokenize=False, add_generation_prompt=False
            ):
                assert tokenize is False
                assert add_generation_prompt is True
                question = messages[0]["content"][1]["text"]
                return "PREFIX<image-1>{}SUFFIX".format(question)

        categories = self._detector_without_worker().categories
        prefix, suffix, template, digest = _semantic_prompt_parts(
            FakeProcessor(), categories
        )

        self.assertEqual(prefix, "PREFIX<image 1><img>")
        self.assertTrue(suffix.startswith("</img>Locate all the instances"))
        self.assertNotIn("<image-1>", prefix + suffix)
        self.assertIn(categories[0].prompt, template)
        self.assertEqual(len(digest), 64)

    def test_semantic_template_separates_each_short_query(self):
        class FakeProcessor:
            image_placeholder = "image"
            image_start_token = "<img>"
            image_end_token = "</img>"

            @staticmethod
            def py_apply_chat_template(
                messages, tokenize=False, add_generation_prompt=False
            ):
                return "PREFIX<image-1>{}SUFFIX".format(
                    messages[0]["content"][1]["text"]
                )

        category = LocateAnythingCategory(
            0,
            "trash",
            "a paper ball lying on the floor",
            (),
            (
                "a paper ball lying on the floor",
                "a discarded bottle lying on the floor",
            ),
        )

        _, _, template, _ = _semantic_prompt_parts(
            FakeProcessor(), (category,)
        )

        self.assertIn(
            "a paper ball lying on the floor</c>"
            "a discarded bottle lying on the floor",
            template,
        )


class LocateAnythingInputSizingTest(unittest.TestCase):
    class FakeImage:
        def __init__(self, size):
            self.size = size
            self.resize_calls = []

        def resize(self, size):
            self.resize_calls.append(size)
            return LocateAnythingInputSizingTest.FakeImage(size)

    def test_zero_size_limit_passes_native_frame_without_resize(self):
        image = self.FakeImage((640, 360))

        output = _resize_source_image(image, validate_max_image_side(0))

        self.assertIs(output, image)
        self.assertEqual(image.resize_calls, [])

    def test_positive_size_limit_retains_legacy_downscale(self):
        image = self.FakeImage((640, 360))

        output = _resize_source_image(image, validate_max_image_side(448))

        self.assertEqual(image.resize_calls, [(448, 252)])
        self.assertEqual(output.size, (448, 252))

    def test_rejects_ambiguous_nonzero_small_limit(self):
        with self.assertRaisesRegex(ValueError, "native input"):
            validate_max_image_side(100)


class LocateAnythingWorkerEnvironmentTest(unittest.TestCase):
    def test_every_managed_cache_log_and_temporary_path_stays_in_model_root(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory).resolve()
            detector = object.__new__(LocateAnythingDetector)
            detector.model_root = str(root)

            environment = detector._worker_environment()

            managed_path_keys = (
                "HF_HOME",
                "HUGGINGFACE_HUB_CACHE",
                "TRANSFORMERS_CACHE",
                "HF_ASSETS_CACHE",
                "HF_MODULES_CACHE",
                "TORCH_HOME",
                "XDG_CACHE_HOME",
                "CUDA_CACHE_PATH",
                "TRITON_CACHE_DIR",
                "NUMBA_CACHE_DIR",
                "TMPDIR",
                "PYTHONPYCACHEPREFIX",
            )
            for key in managed_path_keys:
                path = Path(environment[key]).resolve()
                path.relative_to(root)
                self.assertTrue(path.is_dir(), key)
            self.assertEqual(environment["TRANSFORMERS_OFFLINE"], "1")
            self.assertEqual(environment["HF_HUB_OFFLINE"], "1")


if __name__ == "__main__":
    unittest.main()
