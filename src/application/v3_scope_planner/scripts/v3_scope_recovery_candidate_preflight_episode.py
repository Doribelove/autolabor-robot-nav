#!/usr/bin/env python3
"""One fresh V3-13 recovery-candidate preflight episode."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import multi_actor_core
from v3_scope_planner import recovery_candidate_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_multiactor_sequential_clone_episode_v2.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r5_synchronized_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class RecoveryCandidatePreflight(BASE.SynchronizedSequentialCloneEpisode):
    def __init__(self):
        super().__init__()
        self.geometry_contract_path = rospy.get_param(
            "~geometry_contract_path"
        )
        with open(
            self.geometry_contract_path, "r", encoding="utf-8"
        ) as stream:
            self.geometry_contract = yaml.safe_load(stream)
        expected = "recovery_evasive_right_v120_r1"
        if self.forced_candidate_id != expected:
            raise RuntimeError("unreleased recovery candidate")
        if abs(self.branch_target_sim_s - 12.5) > 1.0e-12:
            raise RuntimeError("unreleased recovery branch time")
        if abs(self.combined_radius_m - 1.15) > 1.0e-12:
            raise RuntimeError("unreleased clone clearance radius")

    def branch_candidate(self, decision):
        pose = decision["decision_pose"]
        snapshot = {
            "scene_snapshot_id": (
                "v3_13_r5_{}_{}_{}_125".format(
                    self.scene_id, self.seed, self.block_index
                )
            ),
            "world_model_seq": 1,
            "ego": {
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "yaw_rad": pose["yaw_rad"],
                "speed_mps": decision["decision_speed_mps"],
            },
        }
        candidate_set = recovery_candidate_core.generate_candidate_set(
            snapshot,
            self.contract,
            self.geometry_contract,
            recovery_context=True,
        )
        candidate = next(
            item
            for item in candidate_set["candidates"]
            if item["candidate_id"] == self.forced_candidate_id
        )
        generated, local = (
            recovery_candidate_core.generate_recovery_candidate(
                snapshot,
                self.contract,
                self.geometry_contract,
                recovery_context=True,
            )
        )
        if generated != candidate:
            raise RuntimeError("recovery candidate generation drift")
        failures = multi_actor_core.validate_candidate(candidate, local)
        if failures:
            raise RuntimeError(
                "recovery candidate invalid: {}".format(failures)
            )
        return snapshot, candidate_set, candidate

    def run(self):
        report = super().run()
        result = report["result"]
        report.update(
            {
                "stage": "V3-13-Q1-R5",
                "method": (
                    "fresh_non_evidence_runtime_radius_aligned_"
                    "recovery_candidate_preflight"
                ),
                "geometry_contract": {
                    "path": self.geometry_contract_path,
                    "sha256": BASE.S1.BASE.sha256_file(
                        self.geometry_contract_path
                    ),
                },
                "clone_combined_radius_m": self.combined_radius_m,
                "preflight_scientific_gate_pass": (
                    report["episode_collection_pass"]
                    and result["completed_horizon"]
                    and result["contact_state_count"] == 0
                    and result[
                        "minimum_conservative_clearance_m"
                    ]
                    >= 0.0
                    and result["tracking_error_max_m"] is not None
                    and result["tracking_error_max_m"] <= 0.80
                    and result["maximum_command_gap_s"] <= 0.20
                ),
                "non_evidence_preflight": True,
                "candidate_specific_predictor_qualified": False,
                "actual_TEB_execution_authorized": False,
                "perception_execution_authorized": False,
                "training_authorized": False,
            }
        )
        temporary = self.report_path + ".r5.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-Q1-R5 preflight pass=%s clearance=%.3f",
            report["preflight_scientific_gate_pass"],
            result["minimum_conservative_clearance_m"],
        )
        return report


def main():
    rospy.init_node("v3_scope_recovery_candidate_preflight_episode")
    try:
        RecoveryCandidatePreflight().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R5 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
