"""Offline regressions for P1-E1 exact execution and raw replay surfaces."""

import ast
import csv
import hashlib
import io
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))

from thesis_experiment import v2_04g_r6_i1_p1_statistics as statistics
from thesis_experiment import v2_04g_r6_i1_p1_e1_replay as replay


SCHEDULE = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
RUNNER = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_exact_runner.py"
)
LAUNCH = WORKSPACE / (
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p1_e1_execution.launch"
)


class P1E1OfflineTests(unittest.TestCase):
    def test_exact_frozen_schedule(self):
        schedule = yaml.safe_load(SCHEDULE.read_text(encoding="utf-8"))
        rows = statistics.validate_schedule(schedule)
        self.assertEqual(len(rows), 270)
        self.assertEqual({row["seed"] for row in rows}, set(range(5201, 5291)))
        self.assertTrue(all(row["attempt"] == 1 for row in rows))

    def test_overlay_replay_exact_tiling_and_method_mapping(self):
        planned = {
            "method_id": statistics.CONTROL,
            "scene_id": "v2-04g-p1-s5201",
            "seed": 5201,
            "attempt": 1,
        }
        document = {
            "stage": replay.STAGE,
            **planned,
            "behavior_read_only": True,
            "runtime_policy_received_trace": False,
            "goal_dispatch_stamp_s": 10.0,
            "terminal_event_stamp_s": 20.0,
            "observation_duration_s": 10.0,
            "events": [
                {"stamp_s": 10.0, "offset_s": 0.0, "raw_state": "NONE", "mapped_state": "NONE"},
                {"stamp_s": 12.5, "offset_s": 2.5, "raw_state": "CROSSING", "mapped_state": "LEGACY_DYNAMIC"},
                {"stamp_s": 18.0, "offset_s": 8.0, "raw_state": "NONE", "mapped_state": "NONE"},
            ],
        }
        duration, intervals = replay._overlay_intervals(document, planned)
        self.assertEqual(duration, 10.0)
        self.assertEqual(intervals[0], {"start_s": 0.0, "end_s": 2.5, "state": "NONE"})
        self.assertEqual(intervals[-1]["end_s"], 10.0)
        self.assertAlmostEqual(sum(row["end_s"] - row["start_s"] for row in intervals), 10.0)

    def test_overlay_wrong_method_mapping_fails(self):
        planned = {
            "method_id": statistics.TREATMENT,
            "scene_id": "v2-04g-p1-s5261",
            "seed": 5261,
            "attempt": 1,
        }
        document = {
            "stage": replay.STAGE,
            **planned,
            "behavior_read_only": True,
            "runtime_policy_received_trace": False,
            "goal_dispatch_stamp_s": 1.0,
            "terminal_event_stamp_s": 2.0,
            "observation_duration_s": 1.0,
            "events": [{
                "stamp_s": 1.0, "offset_s": 0.0,
                "raw_state": "CROSSING", "mapped_state": "LEGACY_DYNAMIC",
            }],
        }
        with self.assertRaises(replay.P1ReplayError):
            replay._overlay_intervals(document, planned)

    def test_trace_metrics_are_replayed_from_csv(self):
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=replay.TRACE_COLUMNS)
        writer.writeheader()
        base = {
            "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0,
            "linear_velocity_mps": 0.0, "angular_velocity_radps": 0.0,
            "commanded_speed_mps": 0.0, "clearance_m": 0.4,
            "goal_distance_m": 1.0, "collision": "false", "goal_reached": "false",
            "contact_count": 0, "topology_id": "fixed_teb",
            "global_replan_count": 0, "recovery_count": 0,
            "gear": "NEUTRAL", "predicted_ttc_s": "",
        }
        writer.writerow({"stamp_s": 5.0, **base})
        writer.writerow({"stamp_s": 7.0, **base, "goal_reached": "true"})
        rows = replay._trace(stream.getvalue().encode("utf-8"))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[-1]["stamp_s"] - rows[0]["stamp_s"], 2.0)
        self.assertTrue(rows[-1]["goal_reached"])

    def test_runner_has_no_row_selection_or_resume_cli(self):
        tree = ast.parse(RUNNER.read_text(encoding="utf-8"))
        option_literals = {
            node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and node.value.startswith("--")
        }
        self.assertNotIn("--start-sequence", option_literals)
        self.assertNotIn("--end-sequence", option_literals)
        self.assertNotIn("--resume", option_literals)
        self.assertNotIn("--retry", option_literals)
        self.assertIn("--release-sha256", option_literals)
        self.assertIn("--execute", option_literals)

    def test_launch_has_structural_three_method_switches(self):
        root = ET.parse(str(LAUNCH)).getroot()
        args = {node.attrib["name"] for node in root.findall("arg")}
        self.assertTrue({
            "seed", "start_rule_supervisor", "start_typed_transaction",
            "load_balanced_anchor", "publish_teb_obstacles",
        }.issubset(args))
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("extra", (WORKSPACE / "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch").read_text(encoding="utf-8"))
        self.assertIn("dynamics_seed:=$(arg seed)", (WORKSPACE / "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch").read_text(encoding="utf-8"))
        self.assertIn('<group if="$(arg start_rule_supervisor)">', text)

    def test_p1_source_instances_remain_test_id(self):
        schedule = yaml.safe_load(SCHEDULE.read_text(encoding="utf-8"))
        for planned in schedule["episodes"][::3]:
            path = WORKSPACE / planned["instance"]["path"]
            payload = path.read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(), planned["instance"]["sha256"])
            instance = yaml.safe_load(payload)
            self.assertEqual(instance["scene"]["split"], "test_id")


if __name__ == "__main__":
    unittest.main()
