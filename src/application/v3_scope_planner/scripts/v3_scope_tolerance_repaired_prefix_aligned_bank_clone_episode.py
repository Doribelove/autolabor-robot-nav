#!/usr/bin/env python3
"""R19-R5 prefix-aligned bank with the r3 0.15 m terminal tolerance."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import execution_core
from v3_scope_planner import goal_aligned_execution_adapter_r2


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_prefix_aligned_boundary_atomic_bank_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_r5_prefix_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
R3 = "recovery_evasive_right_v120_r3"


class ToleranceRepairedPrefixAlignedBankClone(
    BASE.PrefixAlignedBoundaryAtomicBankClone
):
    def __init__(self):
        super().__init__()
        if self.forced_candidate_id == R3:
            self.terminal_guard_activation_count = 0

            def observed_guard(*args, **kwargs):
                command = (
                    goal_aligned_execution_adapter_r2
                    .pure_pursuit_command(*args, **kwargs)
                )
                if command["terminal_reached_fail_closed"]:
                    self.terminal_guard_activation_count += 1
                return command

            execution_core.pure_pursuit_command = observed_guard

    def run(self):
        report = super().run()
        report.update(
            {
                "stage": "V3-13-Q1-R19-R5",
                "method": (
                    "fresh_r3_terminal_tolerance_repaired_"
                    "prefix_aligned_boundary_atomic_preflight"
                ),
                "r3_terminal_tolerance_m": 0.15,
                "single_factor": {
                    "name": "r3_terminal_tolerance_m",
                    "control": 0.12,
                    "treatment": 0.15,
                },
                "candidate_geometry_changed": False,
                "legacy_action_execution_changed": False,
            }
        )
        temporary = self.report_path + ".r19_r5.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node(
        "v3_scope_tolerance_repaired_prefix_aligned_bank_clone"
    )
    try:
        ToleranceRepairedPrefixAlignedBankClone().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R19-R5 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
