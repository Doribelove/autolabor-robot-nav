#!/usr/bin/env python3
"""Fresh R18-R1 preflight with the non-recursive guard adapter."""

import importlib.util
import math
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import goal_aligned_execution_adapter_r1


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_progress_recovery_preflight_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r18_r1_frozen_preflight", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
STAGE = "V3-13-Q1-R18-R1"
CANDIDATE_ID = "recovery_evasive_right_v120_r3"


class RepairedGoalAlignedRecoveryPreflight(
    BASE.ProgressRecoveryPreflight
):
    def __init__(self):
        BASE.BASE.SynchronizedSequentialCloneEpisode.__init__(self)
        self.geometry_contract_path = rospy.get_param(
            "~geometry_contract_path"
        )
        with open(
            self.geometry_contract_path, "r", encoding="utf-8"
        ) as stream:
            self.geometry_contract = yaml.safe_load(stream)
        if self.forced_candidate_id != CANDIDATE_ID:
            raise RuntimeError("unreleased goal-aligned candidate")
        if abs(self.branch_target_sim_s - 12.5) > 1.0e-12:
            raise RuntimeError("unreleased recovery branch time")
        if abs(self.combined_radius_m - 1.15) > 1.0e-12:
            raise RuntimeError("unreleased clone clearance radius")
        if abs(self.horizon_s - 7.55) > 1.0e-12:
            raise RuntimeError("unreleased candidate horizon")
        self.terminal_guard_activation_count = 0

        def observed_guard(*args, **kwargs):
            command = (
                goal_aligned_execution_adapter_r1.pure_pursuit_command(
                    *args, **kwargs
                )
            )
            if command["terminal_reached_fail_closed"]:
                self.terminal_guard_activation_count += 1
            return command

        BASE.BASE.S1.BASE.execution_core.pure_pursuit_command = (
            observed_guard
        )

    def run(self):
        report = BASE.BASE.SynchronizedSequentialCloneEpisode.run(self)
        result = report["result"]
        static_clearance = self.minimum_trace_static_clearance(
            result["trace"]
        )
        maximum_lateral = max(
            abs(float(sample["ego"]["y_m"]))
            for sample in result["trace"]
        )
        final_pose = result["final_pose"]
        final_goal_distance = math.hypot(
            float(final_pose["x_m"]) - 5.2,
            float(final_pose["y_m"]),
        )
        report.update(
            {
                "stage": STAGE,
                "method": (
                    "fresh_non_evidence_goal_aligned_recovery_"
                    "nonrecursive_adapter_preflight"
                ),
                "geometry_contract": {
                    "path": self.geometry_contract_path,
                    "sha256": BASE.BASE.S1.BASE.sha256_file(
                        self.geometry_contract_path
                    ),
                },
                "clone_combined_radius_m": self.combined_radius_m,
                "minimum_trace_static_clearance_m": static_clearance,
                "maximum_absolute_lateral_offset_m": maximum_lateral,
                "final_common_goal_distance_m": final_goal_distance,
                "terminal_guard_activation_count": (
                    self.terminal_guard_activation_count
                ),
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
                    and final_goal_distance <= 0.25
                    and self.terminal_guard_activation_count >= 1
                ),
                "non_evidence_preflight": True,
                "candidate_specific_predictor_qualified": False,
                "actual_TEB_execution_authorized": False,
                "perception_execution_authorized": False,
                "training_authorized": False,
            }
        )
        temporary = self.report_path + ".r18_r1.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-Q1-R18-R1 pass=%s dynamic=%.3f static=%.3f "
            "goal=%.3f guard=%d",
            report["preflight_scientific_gate_pass"],
            result["minimum_conservative_clearance_m"],
            static_clearance,
            final_goal_distance,
            self.terminal_guard_activation_count,
        )
        return report


def main():
    rospy.init_node(
        "v3_scope_goal_aligned_recovery_preflight_episode_r1"
    )
    try:
        RepairedGoalAlignedRecoveryPreflight().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R18-R1 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
