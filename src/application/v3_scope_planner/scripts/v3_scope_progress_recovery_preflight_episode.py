#!/usr/bin/env python3
"""One fresh progress-relative recovery-candidate preflight."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import multi_actor_core
from v3_scope_planner import progress_recovery_candidate_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_multiactor_sequential_clone_episode_v2.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r11_synchronized_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class ProgressRecoveryPreflight(
    BASE.SynchronizedSequentialCloneEpisode
):
    def __init__(self):
        super().__init__()
        self.geometry_contract_path = rospy.get_param(
            "~geometry_contract_path"
        )
        with open(
            self.geometry_contract_path, "r", encoding="utf-8"
        ) as stream:
            self.geometry_contract = yaml.safe_load(stream)
        if (
            self.forced_candidate_id
            != "recovery_evasive_right_v120_r2"
        ):
            raise RuntimeError("unreleased progress recovery candidate")
        if abs(self.branch_target_sim_s - 12.5) > 1.0e-12:
            raise RuntimeError("unreleased recovery branch time")
        if abs(self.combined_radius_m - 1.15) > 1.0e-12:
            raise RuntimeError("unreleased clone clearance radius")

    def branch_candidate(self, decision):
        pose = decision["decision_pose"]
        snapshot = {
            "scene_snapshot_id": (
                "v3_13_r11_{}_{}_{}_125".format(
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
        candidate_set = (
            progress_recovery_candidate_core.generate_candidate_set(
                snapshot,
                self.contract,
                self.geometry_contract,
                recovery_context=True,
            )
        )
        candidate = next(
            item
            for item in candidate_set["candidates"]
            if item["candidate_id"] == self.forced_candidate_id
        )
        generated, local = (
            progress_recovery_candidate_core.generate_recovery_candidate(
                snapshot,
                self.contract,
                self.geometry_contract,
                recovery_context=True,
            )
        )
        if generated != candidate:
            raise RuntimeError("progress candidate generation drift")
        failures = multi_actor_core.validate_candidate(candidate, local)
        if failures:
            raise RuntimeError(
                "progress recovery candidate invalid: {}".format(
                    failures
                )
            )
        return snapshot, candidate_set, candidate

    def minimum_trace_static_clearance(self, trace):
        ego_half_width = float(
            self.contract["vehicle"]["ego_half_width_m"]
        )
        ego_radius = float(
            self.contract["vehicle"][
                "ego_circumscribed_radius_m"
            ]
        )
        minimum = float("inf")
        half_width = self.scene.get("corridor_half_width_m")
        for sample in trace:
            x_m = float(sample["ego"]["x_m"])
            y_m = float(sample["ego"]["y_m"])
            if half_width is not None:
                minimum = min(
                    minimum,
                    float(half_width) - abs(y_m) - ego_half_width,
                )
            for rectangle in self.scene.get(
                "static_rectangles", []
            ):
                minimum = min(
                    minimum,
                    multi_actor_core._rectangle_distance(
                        x_m, y_m, rectangle
                    )
                    - ego_radius,
                )
        return 1000.0 if minimum == float("inf") else minimum

    def run(self):
        report = super().run()
        result = report["result"]
        static_clearance = self.minimum_trace_static_clearance(
            result["trace"]
        )
        maximum_lateral = max(
            abs(float(sample["ego"]["y_m"]))
            for sample in result["trace"]
        )
        report.update(
            {
                "stage": "V3-13-Q1-R11",
                "method": (
                    "fresh_non_evidence_translation_invariant_"
                    "progress_recovery_preflight"
                ),
                "geometry_contract": {
                    "path": self.geometry_contract_path,
                    "sha256": BASE.S1.BASE.sha256_file(
                        self.geometry_contract_path
                    ),
                },
                "clone_combined_radius_m": self.combined_radius_m,
                "minimum_trace_static_clearance_m": static_clearance,
                "maximum_absolute_lateral_offset_m": maximum_lateral,
                "preflight_scientific_gate_pass": (
                    report["episode_collection_pass"]
                    and result["completed_horizon"]
                    and result["contact_state_count"] == 0
                    and result[
                        "minimum_conservative_clearance_m"
                    ]
                    >= 0.0
                    and static_clearance >= 0.05
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
        temporary = self.report_path + ".r11.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-Q1-R11 pass=%s dynamic=%.3f static=%.3f",
            report["preflight_scientific_gate_pass"],
            result["minimum_conservative_clearance_m"],
            static_clearance,
        )
        return report


def main():
    rospy.init_node("v3_scope_progress_recovery_preflight_episode")
    try:
        ProgressRecoveryPreflight().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R11 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
