#!/usr/bin/env python3
"""Fresh R8 clone episode bound to the expanded recovery predictor."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_recovery_bank_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r8_recovery_bank_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class RecoveryPredictorQualification(BASE.RecoveryBankClone):
    def __init__(self):
        super().__init__()
        self.predictor_model_path = rospy.get_param(
            "~predictor_model_path"
        )
        with open(
            self.predictor_model_path, "r", encoding="utf-8"
        ) as stream:
            model = yaml.safe_load(stream)
        expected = "recovery_evasive_right_v120_r1"
        if expected not in model["heads"]["assertive"]:
            raise RuntimeError("expanded predictor head absent")
        if model["fresh_recovery_qualification_pass"]:
            raise RuntimeError("predictor already claims qualification")

    def run(self):
        report = super().run()
        report.update(
            {
                "stage": "V3-13-Q1-R8",
                "method": (
                    "fresh_five_action_expanded_recovery_"
                    "predictor_qualification_clone"
                ),
                "predictor_model": {
                    "path": self.predictor_model_path,
                    "sha256": BASE.BASE.S1.BASE.sha256_file(
                        self.predictor_model_path
                    ),
                },
                "predictor_used_for_action_execution": False,
                "predictor_evaluated_offline_against_raw_clone_truth": True,
                "actual_TEB_execution_authorized": False,
                "perception_execution_authorized": False,
                "training_authorized": False,
            }
        )
        temporary = self.report_path + ".r8.tmp"
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
        "v3_scope_recovery_predictor_qualification_episode"
    )
    try:
        RecoveryPredictorQualification().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R8 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
