#!/usr/bin/env python3
"""One V3-19 simulation-only four-method thesis benchmark episode."""
import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import spatial_core_r1
from v3_scope_planner import thesis_benchmark_methods as methods


BASE_PATH = Path("/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/scripts/v3_scope_actual_teb_fair_episode.py")
SPEC = importlib.util.spec_from_file_location("v3_19_frozen_v3_11_episode", str(BASE_PATH))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class ThesisBenchmarkEpisode(BASE.FairEpisode):
    def __init__(self):
        requested = rospy.get_param("~method")
        if requested not in methods.METHODS:
            raise RuntimeError("unknown thesis benchmark method: {}".format(requested))
        self.requested_method = requested
        internal = "actual_teb" if requested == "actual_teb" else "v3_spatial_safe"
        rospy.set_param("~method", internal)
        try:
            super().__init__()
        finally:
            rospy.set_param("~method", requested)
        self.ablation_model_path = rospy.get_param("~ablation_model_path")
        self.baseline_planner_path = rospy.get_param("~baseline_planner_path")
        self.baseline_model_path = rospy.get_param("~baseline_model_path")

    def snapshot(self, gate):
        actor = gate["gate_actor_track"]
        return {
            "scene_snapshot_id": "v3_19_{}_{}_{}_{}".format(
                self.scene_id, self.seed, self.actor_policy, self.requested_method
            ),
            "world_model_seq": 1,
            "snapshot_age_s": actor["age_s"],
            "predictor_latency_ms": 0.0,
            "verifier_latency_ms": 0.0,
            "ego": {"x_m": -2.6, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.8},
            "actor": {
                "coordinate_m": actor["coordinate_m"],
                "speed_mps": actor["coordinate_speed_mps"],
                "path_angle_deg": self.actor_angle_deg,
                "monotone_coordinate_certified": True,
            },
            "health": {
                "localization_ok": True,
                "transform_ok": True,
                "actor_track_ok": actor["age_s"] <= 0.2,
                "command_chain_ok": True,
            },
        }

    @staticmethod
    def load(path):
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def plan_v3(self, gate):
        snapshot = self.snapshot(gate)
        if self.requested_method == "fixed_v120_shielded":
            return methods.plan_fixed_v120(
                snapshot, self.load(self.baseline_model_path), self.load(self.baseline_planner_path)
            )
        model_path = self.ablation_model_path if self.requested_method == "v3_no_candidate_conditioning" else self.predictor_model_path
        return methods.plan_spatial(
            snapshot, self.load(model_path), self.load(self.planner_contract_path), self.requested_method
        )

    def run(self):
        report = super().run()
        plan = report.get("v3_plan")
        result = report["result"]
        report["stage"] = "V3-19"
        report["method"] = self.requested_method
        report["method_plan"] = plan
        report["method_dependencies"] = {
            "ablation_model": {"path": self.ablation_model_path, "sha256": BASE.sha256_file(self.ablation_model_path)},
            "baseline_planner": {"path": self.baseline_planner_path, "sha256": BASE.sha256_file(self.baseline_planner_path)},
            "baseline_model": {"path": self.baseline_model_path, "sha256": BASE.sha256_file(self.baseline_model_path)},
        }
        report["task_success"] = bool(result["goal_reached"])
        report["execution_integrity_pass"] = (
            report["gate_physical_valid"]
            and report["warmup"]["actor_mode_matches_evaluator_policy"]
            and result["contact_state_count"] == 0
            and result["minimum_conservative_clearance_m"] >= 0.0
            and result["command_count"] > 0
            and (self.requested_method != "actual_teb" or result["local_plan_count"] > 0)
            and (self.requested_method == "actual_teb" or (plan is not None and plan["selected_evaluation"]["accepted"] is True))
        )
        report["episode_pass"] = bool(report["execution_integrity_pass"])
        report["scientific_inclusion"] = True
        temporary = self.report_path + ".v3_19.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-19 thesis benchmark complete method=%s integrity=%s goal=%s time=%.3f",
            self.requested_method, report["execution_integrity_pass"], report["task_success"],
            result["restricted_elapsed_s"],
        )
        return report


def main():
    rospy.init_node("v3_scope_thesis_benchmark_episode")
    try:
        ThesisBenchmarkEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr("V3-19 thesis benchmark failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())

