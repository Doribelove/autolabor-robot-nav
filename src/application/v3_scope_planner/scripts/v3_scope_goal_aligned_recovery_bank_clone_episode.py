#!/usr/bin/env python3
"""V3-13 R19 matched bank with goal-aligned r3 execution."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import goal_aligned_execution_adapter_r1


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_progress_recovery_bank_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_progress_bank_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
ACTION_IDS = (
    "spatial_left_v120",
    "spatial_right_v120",
    "straight_v120",
    "certified_stop_v000",
    "recovery_evasive_right_v120_r3",
)
# The inherited methods resolve ACTION_IDS from their isolated module.
BASE.ACTION_IDS = ACTION_IDS


class GoalAlignedRecoveryBankClone(BASE.ProgressRecoveryBankClone):
    def __init__(self):
        super().__init__()
        self.terminal_guard_activation_count = 0
        if self.forced_candidate_id == ACTION_IDS[-1]:
            if abs(self.horizon_s - 7.55) > 1.0e-12:
                raise RuntimeError("unreleased r3 horizon")

            def observed_guard(*args, **kwargs):
                command = (
                    goal_aligned_execution_adapter_r1
                    .pure_pursuit_command(*args, **kwargs)
                )
                if command["terminal_reached_fail_closed"]:
                    self.terminal_guard_activation_count += 1
                return command

            BASE.BASE.S1.BASE.execution_core.pure_pursuit_command = (
                observed_guard
            )
        elif abs(self.horizon_s - 7.0) > 1.0e-12:
            raise RuntimeError("legacy action horizon changed")

    def run(self):
        report = super().run()
        is_r3 = self.forced_candidate_id == ACTION_IDS[-1]
        report.update(
            {
                "stage": "V3-13-Q1-R19",
                "method": (
                    "matched_five_action_positive_propensity_"
                    "goal_aligned_recovery_bank_clone"
                ),
                "recovery_bank_action_ids": list(ACTION_IDS),
                "terminal_guard_applies_to_forced_action": is_r3,
                "terminal_guard_activation_count": (
                    self.terminal_guard_activation_count
                ),
                "terminal_guard_execution_pass": (
                    self.terminal_guard_activation_count >= 1
                    if is_r3
                    else self.terminal_guard_activation_count == 0
                ),
            }
        )
        temporary = self.report_path + ".r19.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-Q1-R19 block=%d action=%s pass=%s guard=%d",
            self.block_index,
            self.forced_candidate_id,
            report["episode_collection_pass"],
            self.terminal_guard_activation_count,
        )
        return report


def main():
    rospy.init_node("v3_scope_goal_aligned_recovery_bank_clone_episode")
    try:
        GoalAlignedRecoveryBankClone().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R19 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
