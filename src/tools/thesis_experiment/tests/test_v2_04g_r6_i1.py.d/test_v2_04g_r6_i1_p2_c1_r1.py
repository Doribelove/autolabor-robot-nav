"""Offline regression tests for the P2-C1-R1 integrity repair."""

import copy
import hashlib
import importlib.util
from pathlib import Path
import sys
import unittest

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SCRIPTS = WORKSPACE / "src/tools/thesis_experiment/scripts"
SOURCE = WORKSPACE / "src/tools/thesis_experiment/src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = load(
    "_test_p2_c1_r1_builder",
    SCRIPTS / "v2_04g_r6_i1_p2_c1_r1_release_builder.py",
)
REVIEWER = load(
    "_test_p2_c1_r1_reviewer",
    SCRIPTS / "v2_04g_r6_i1_p2_c1_r1_offline_reviewer.py",
)
from thesis_experiment.v2_04g_r6_i1_p2_c1_r1_release import validate_release


class P2C1R1OfflineTest(unittest.TestCase):
    def setUp(self):
        self.schedule = yaml.safe_load(
            (WORKSPACE / BUILDER.SCHEDULE_RELATIVE).read_text(encoding="utf-8")
        )
        self.old = yaml.safe_load(
            (WORKSPACE / BUILDER.OLD_SCHEDULE_RELATIVE).read_text(encoding="utf-8")
        )

    def test_exact_timeout_repairs_only(self):
        observed = {}
        for old_row, row in zip(self.old["rows"], self.schedule["rows"]):
            reconstructed = copy.deepcopy(row)
            reconstructed["stage"] = "V2-04G-P2-C1"
            if row["sequence"] in BUILDER.EXPECTED_TIMEOUT_REPAIRS:
                observed[row["sequence"]] = (
                    float(old_row["timeout_s"]),
                    float(row["timeout_s"]),
                )
                reconstructed["timeout_s"] = old_row["timeout_s"]
            self.assertEqual(reconstructed, old_row)
        self.assertEqual(observed, BUILDER.EXPECTED_TIMEOUT_REPAIRS)

    def test_all_scene_receipts_and_timeouts(self):
        for row in self.schedule["rows"]:
            payload = (WORKSPACE / row["instance"]["path"]).read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(), row["instance"]["sha256"])
            self.assertEqual(len(payload), row["instance"]["size_bytes"])
            scene = yaml.safe_load(payload)["scene"]
            self.assertEqual(scene["scene_id"], row["scene_id"])
            self.assertEqual(scene["seed"], row["seed"])
            self.assertEqual(float(scene["timeout_s"]), float(row["timeout_s"]))

    def test_builder_is_deterministic_while_release_absent(self):
        if (WORKSPACE / BUILDER.RELEASE_RELATIVE).exists():
            self.skipTest("unique release already frozen")
        expected = BUILDER.build_bundle(WORKSPACE)
        for relative, payload in expected.items():
            self.assertEqual((WORKSPACE / relative).read_bytes(), payload)

    def test_review_or_release_validator(self):
        release = WORKSPACE / BUILDER.RELEASE_RELATIVE
        if not release.exists():
            first = REVIEWER.build_review(WORKSPACE)
            second = REVIEWER.build_review(WORKSPACE)
            self.assertEqual(first, second)
            self.assertEqual(first["review_result"], "pass")
        else:
            digest = hashlib.sha256(release.read_bytes()).hexdigest()
            result = validate_release(
                digest, WORKSPACE, require_fresh_state=True
            )
            self.assertEqual(
                result["timeout_repairs_verified"],
                list(BUILDER.EXPECTED_TIMEOUT_REPAIRS),
            )


if __name__ == "__main__":
    unittest.main()
