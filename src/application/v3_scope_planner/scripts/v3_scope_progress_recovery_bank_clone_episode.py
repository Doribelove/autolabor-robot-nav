#!/usr/bin/env python3
"""V3-13 five-action matched progress-recovery clone episode."""

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
    "v3_13_r12_synchronized_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
ACTION_IDS = (
    "spatial_left_v120",
    "spatial_right_v120",
    "straight_v120",
    "certified_stop_v000",
    "recovery_evasive_right_v120_r2",
)


class ProgressRecoveryBankClone(
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
        if self.forced_candidate_id not in ACTION_IDS:
            raise RuntimeError("unreleased progress-recovery action")
        if abs(self.branch_target_sim_s - 12.5) > 1.0e-12:
            raise RuntimeError("unreleased recovery branch time")
        if abs(self.behavior_propensity - 0.20) > 1.0e-12:
            raise RuntimeError("unreleased behavior propensity")

    def branch_candidate(self, decision):
        pose = decision["decision_pose"]
        snapshot = {
            "scene_snapshot_id": (
                "v3_13_r12_{}_{}_{}_125".format(
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
            item for item in candidate_set["candidates"]
            if item["candidate_id"] == self.forced_candidate_id
        )
        if self.forced_candidate_id == ACTION_IDS[-1]:
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
        else:
            local = self.contract
        failures = multi_actor_core.validate_candidate(candidate, local)
        if failures:
            raise RuntimeError(
                "progress recovery-bank candidate invalid: {}".format(
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
        report.update(
            {
                "stage": "V3-13-Q1-R12",
                "method": (
                    "matched_five_action_positive_propensity_"
                    "progress_recovery_bank_clone"
                ),
                "geometry_contract": {
                    "path": self.geometry_contract_path,
                    "sha256": BASE.S1.BASE.sha256_file(
                        self.geometry_contract_path
                    ),
                },
                "recovery_bank_action_ids": list(ACTION_IDS),
                "recovery_context": True,
                "minimum_trace_static_clearance_m": (
                    self.minimum_trace_static_clearance(
                        report["result"]["trace"]
                    )
                ),
                "complete_positive_propensity_block_required": True,
                "candidate_specific_predictor_qualified": False,
                "actual_TEB_execution_authorized": False,
                "perception_execution_authorized": False,
                "training_authorized": False,
            }
        )
        temporary = self.report_path + ".r12.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-Q1-R12 block=%d action=%s pass=%s",
            self.block_index,
            self.forced_candidate_id,
            report["episode_collection_pass"],
        )
        return report


def main():
    rospy.init_node("v3_scope_progress_recovery_bank_clone_episode")
    try:
        ProgressRecoveryBankClone().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R12 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
