#!/usr/bin/env python3
"""V3-16 single-factor postbranch-response clone episode."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_tolerance_repaired_prefix_aligned_bank_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_16_response_gated_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class ResponseGatedToleranceRepairedClone(
    BASE.ToleranceRepairedPrefixAlignedBankClone
):
    def run(self):
        report = super().run()
        report.update({
            "stage": "V3-16-S1",
            "method": "postbranch_response_gated_five_action_clone",
            "single_factor": {
                "name": "reactive_actor_response_enable_sim_s",
                "control": 0.0,
                "treatment": 12.55,
                "branch_target_sim_s": 12.5,
            },
            "prebranch_actor_response_disabled": True,
            "candidate_geometry_changed": False,
            "legacy_action_execution_changed": False,
            "policy_or_ranker_training_steps": 0,
            "hardware_accessed": False,
            "real_vehicle_used": False,
        })
        temporary = self.report_path + ".v3_16.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v3_scope_response_gated_tolerance_repaired_clone")
    try:
        ResponseGatedToleranceRepairedClone().run()
        return 0
    except Exception as error:
        rospy.logerr("V3-16-S1 failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__": sys.exit(main())

