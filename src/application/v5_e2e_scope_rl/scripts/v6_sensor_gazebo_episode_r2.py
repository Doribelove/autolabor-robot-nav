#!/usr/bin/env python3
"""V6-03-R2 geometry-aligned sensor episode wrapper."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v5_e2e_scope_rl/scripts/"
    "v6_sensor_gazebo_episode.py"
)
SPEC = importlib.util.spec_from_file_location("v6_03_frozen_episode", str(BASE_PATH))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
MODE_CONTRACT = Path(
    "/home/robot/robot_ws_base_rl/artifacts/v3/planner/v3_08_a0/"
    "v3_08_a0_effective_planner_contract.yaml"
)


class V6SensorGazeboEpisodeR2(BASE.V6SensorGazeboEpisode):
    def __init__(self):
        super().__init__()
        # Exact geometric alignment with the unchanged 1.15 m evaluator:
        # 1.15 combined - 0.36 actor - 0.46 robot = 0.33 m.
        self.runtime_adapter.shield_inflation_m = 0.33

    def run(self):
        report = super().run()
        with MODE_CONTRACT.open("r", encoding="utf-8") as stream:
            mode_contract = yaml.safe_load(stream)
        gate = report["warmup"]
        try:
            inferred = BASE.BASE.spatial_core_r1.infer_mode(
                gate["gate_actor_track"]["coordinate_speed_mps"], mode_contract
            )
            mode_error = ""
        except Exception as error:
            inferred = ""
            mode_error = "{}:{}".format(type(error).__name__, error)
        gate["inferred_actor_mode"] = inferred
        gate["actor_mode_inference_error"] = mode_error
        gate["actor_mode_matches_evaluator_policy"] = inferred == self.actor_policy
        report["stage"] = "V6-03-R2"
        report["hard_shield_geometry_alignment"] = {
            "evaluator_combined_radius_m": 1.15,
            "actor_radius_m": 0.36,
            "robot_radius_m": 0.46,
            "scan_surface_inflation_m": 0.33,
            "derivation": "1.15-0.36-0.46",
        }
        report["execution_integrity_pass"] = bool(
            report["execution_integrity_pass"]
            and gate["actor_mode_matches_evaluator_policy"]
        )
        report["episode_pass"] = report["execution_integrity_pass"]
        temporary = self.report_path + ".r2.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v6_sensor_gazebo_episode_r2")
    try:
        V6SensorGazeboEpisodeR2().run()
        return 0
    except Exception as error:
        rospy.logerr("V6-03-R2 episode failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
