#!/usr/bin/env python3
"""R19-R2 matched bank with boundary-atomic r3 execution."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml


BANK_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_goal_aligned_recovery_bank_clone_episode.py"
)
ATOMIC_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_goal_aligned_boundary_atomic_preflight_episode.py"
)
BANK_SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_r2_bank_base", BANK_PATH
)
BANK = importlib.util.module_from_spec(BANK_SPEC)
BANK_SPEC.loader.exec_module(BANK)
ATOMIC_SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_r2_atomic_base", ATOMIC_PATH
)
ATOMIC = importlib.util.module_from_spec(ATOMIC_SPEC)
ATOMIC_SPEC.loader.exec_module(ATOMIC)
R3 = "recovery_evasive_right_v120_r3"


class BoundaryAtomicRecoveryBankClone(
    BANK.GoalAlignedRecoveryBankClone
):
    def __init__(self):
        super().__init__()
        self.boundary_terminal_evaluation_count = 0
        self.boundary_terminal_trigger_count = 0

    def execute(self, candidate):
        if self.forced_candidate_id == R3:
            return ATOMIC.BoundaryAtomicPreflight.execute(
                self, candidate
            )
        return super().execute(candidate)

    def run(self):
        report = super().run()
        is_r3 = self.forced_candidate_id == R3
        boundary_pass = (
            self.boundary_terminal_evaluation_count == 1
            and self.boundary_terminal_trigger_count == 1
            if is_r3
            else self.boundary_terminal_evaluation_count == 0
            and self.boundary_terminal_trigger_count == 0
        )
        report.update(
            {
                "stage": "V3-13-Q1-R19-R2",
                "method": (
                    "matched_five_action_positive_propensity_"
                    "boundary_atomic_recovery_bank_clone"
                ),
                "boundary_terminal_evaluation_count": (
                    self.boundary_terminal_evaluation_count
                ),
                "boundary_terminal_trigger_count": (
                    self.boundary_terminal_trigger_count
                ),
                "boundary_atomic_terminal_execution_pass": boundary_pass,
                "terminal_guard_execution_pass": (
                    report["terminal_guard_execution_pass"]
                    and boundary_pass
                    if is_r3
                    else boundary_pass
                ),
            }
        )
        temporary = self.report_path + ".r19_r2.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v3_scope_boundary_atomic_recovery_bank_clone")
    try:
        BoundaryAtomicRecoveryBankClone().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R19-R2 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
