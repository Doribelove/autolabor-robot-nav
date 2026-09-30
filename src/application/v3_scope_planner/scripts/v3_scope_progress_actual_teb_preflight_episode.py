#!/usr/bin/env python3
"""R16 fair actual-TEB versus progress-recovery V3 preflight episode."""

import importlib.util
import os
from pathlib import Path
import sys
import time

import rospy
import yaml

from v3_scope_planner import multi_actor_core_v1
from v3_scope_planner import progress_expanded_recovery_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/"
    "v3_scope_planner/scripts/"
    "v3_scope_multiactor_actual_teb_fair_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_frozen_fair_episode", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
STAGE = "V3-13-Q1-R16"


class ProgressActualTebPreflight(BASE.MultiActorFairEpisode):
    def __init__(self):
        super().__init__()
        self.geometry_contract_path = rospy.get_param(
            "~geometry_contract_path"
        )
        with open(
            self.geometry_contract_path, "r", encoding="utf-8"
        ) as stream:
            self.geometry_contract = yaml.safe_load(stream)

    def plan_v3(self, label, recovery):
        if not recovery:
            return super().plan_v3(label, recovery)
        pose = self.robot_world_pose()
        snapshot = {
            "scene_snapshot_id": "v3_13_r16_{}_{}_{}_{}".format(
                self.scene_id, self.seed, self.block_index, label
            ),
            "world_model_seq": len(label) + 1,
            "ego": {
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "yaw_rad": pose["yaw_rad"],
                "speed_mps": float(self.odom.twist.twist.linear.x),
            },
            "actors": self.windowed_actor_tracks(),
            "scene": self.scene,
            "health": {
                field: True
                for field in self.contract["hard_verifier"][
                    "required_health_fields"
                ]
            },
        }
        started_ns = time.perf_counter_ns()
        result = progress_expanded_recovery_core.plan(
            snapshot,
            self.predictor_model,
            self.contract,
            self.geometry_contract,
            recovery_context=True,
        )
        latency_ms = (time.perf_counter_ns() - started_ns) / 1.0e6
        selected_id = result["decision"]["selected_candidate_id"]
        selected_candidate = next(
            item
            for item in result["candidate_set"]["candidates"]
            if item["candidate_id"] == selected_id
        )
        selected_evaluation = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"] == selected_id
        )
        return {
            "label": label,
            "sim_time_s": rospy.Time.now().to_sec(),
            "recovery_observer_used": True,
            "snapshot": snapshot,
            "latency_ms": latency_ms,
            "plan_result_digest": multi_actor_core_v1.digest(result),
            "selected_candidate_id": selected_id,
            "selected_candidate": selected_candidate,
            "selected_evaluation": selected_evaluation,
            "safety_fallback_used": result["decision"][
                "safety_fallback_used"
            ],
            "progress_recovery_extension_used": True,
        }

    def run(self):
        report = super().run()
        report["stage"] = STAGE
        report["preflight_only"] = True
        report["included_in_qualification_or_performance_evidence"] = False
        report["progress_recovery_geometry"] = {
            "path": self.geometry_contract_path,
            "sha256": BASE.BASE.sha256_file(
                self.geometry_contract_path
            ),
        }
        report["actual_TEB_performance_outcome_required_for_integration_pass"] = (
            False
        )
        report["training_authorized"] = False
        temporary = self.report_path + ".r16.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v3_scope_progress_actual_teb_preflight_episode")
    try:
        ProgressActualTebPreflight().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R16 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
