#!/usr/bin/env python3
"""Read-only diagnostic wrapper around the frozen V3-13-Q1 runtime."""

import importlib.util
import os
from pathlib import Path
import sys
import time

import rospy
import yaml

from v3_scope_planner import multi_actor_core_v1


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/"
    "v3_scope_planner/scripts/"
    "v3_scope_multiactor_actual_teb_fair_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_frozen_runtime", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class DiagnosticEpisode(BASE.MultiActorFairEpisode):
    def __init__(self):
        super().__init__()
        if self.method != "v3_13_analytic":
            raise RuntimeError("diagnostic method must be v3_13_analytic")
        self.planning_diagnostics = []

    def plan_v3(self, label, recovery):
        pose = self.robot_world_pose()
        tracks = (
            self.windowed_actor_tracks()
            if recovery
            else self.actor_tracks()
        )
        snapshot = {
            "scene_snapshot_id": "v3_13_q1_r2_{}_{}_{}_{}".format(
                self.scene_id, self.seed, self.block_index, label
            ),
            "world_model_seq": len(label) + 1,
            "ego": {
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "yaw_rad": pose["yaw_rad"],
                "speed_mps": float(self.odom.twist.twist.linear.x),
            },
            "actors": tracks,
            "scene": self.scene,
            "health": {
                field: True
                for field in self.contract["hard_verifier"][
                    "required_health_fields"
                ]
            },
        }
        candidate_set = multi_actor_core_v1.generate_candidate_set(
            snapshot, self.contract
        )
        evaluations = [
            multi_actor_core_v1.evaluate_candidate(
                snapshot,
                candidate,
                self.predictor_model,
                self.contract,
            )
            for candidate in candidate_set["candidates"]
        ]
        diagnostic = {
            "label": label,
            "sim_time_s": rospy.Time.now().to_sec(),
            "recovery_observer_used": bool(recovery),
            "snapshot": snapshot,
            "candidate_set_digest": candidate_set[
                "candidate_set_digest"
            ],
            "candidate_evaluations": [
                {
                    "candidate_id": evaluation["candidate_id"],
                    "accepted": evaluation["accepted"],
                    "rejection_codes": evaluation["rejection_codes"],
                    "minimum_dynamic_clearance_m": evaluation[
                        "minimum_dynamic_clearance_m"
                    ],
                    "minimum_static_clearance_m": evaluation[
                        "minimum_static_clearance_m"
                    ],
                    "minimum_clearance_m": evaluation[
                        "minimum_clearance_m"
                    ],
                    "observable_modes": [
                        tube["observable_mode"]
                        for tube in evaluation["actor_tubes"]
                    ],
                }
                for evaluation in evaluations
            ],
            "selected_candidate_id": None,
            "planner_error": None,
        }
        started_ns = time.perf_counter_ns()
        try:
            result = multi_actor_core_v1.plan(
                snapshot, self.predictor_model, self.contract
            )
        except Exception as error:
            diagnostic["latency_ms"] = (
                time.perf_counter_ns() - started_ns
            ) / 1.0e6
            diagnostic["planner_error"] = "{}:{}".format(
                type(error).__name__, error
            )
            self.planning_diagnostics.append(diagnostic)
            raise
        diagnostic["latency_ms"] = (
            time.perf_counter_ns() - started_ns
        ) / 1.0e6
        selected_id = result["decision"]["selected_candidate_id"]
        diagnostic["selected_candidate_id"] = selected_id
        self.planning_diagnostics.append(diagnostic)
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
            "recovery_observer_used": bool(recovery),
            "snapshot": snapshot,
            "latency_ms": diagnostic["latency_ms"],
            "plan_result_digest": multi_actor_core_v1.digest(result),
            "selected_candidate_id": selected_id,
            "selected_candidate": selected_candidate,
            "selected_evaluation": selected_evaluation,
            "safety_fallback_used": result["decision"][
                "safety_fallback_used"
            ],
        }

    def atomic_report(self, value):
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                value, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)

    def run(self):
        try:
            report = super().run()
        except Exception as error:
            report = {
                "schema_version": "1.0",
                "architecture_generation": "v3",
                "stage": "V3-13-Q1-R2-D1",
                "status": "diagnostic_planner_failure_observed",
                "simulation_only": True,
                "method": self.method,
                "scene_id": self.scene_id,
                "block_index": self.block_index,
                "seed": self.seed,
                "actor_policies_evaluator_only": [
                    spec["evaluator_policy"]
                    for spec in self.actor_specs
                ],
                "hidden_policy_runtime_input": False,
                "failure": "{}:{}".format(type(error).__name__, error),
                "planning_diagnostics": self.planning_diagnostics,
                "diagnostic_complete": bool(
                    self.planning_diagnostics
                ),
                "action_override_or_safety_bypass_used": False,
                "episode_pass": False,
                "policy_or_ranker_training_steps": 0,
                "hardware_accessed": False,
                "real_vehicle_used": False,
            }
            self.atomic_report(report)
            raise
        report["stage"] = "V3-13-Q1-R2-D1"
        report["status"] = "diagnostic_episode_completed_without_failure"
        report["planning_diagnostics"] = self.planning_diagnostics
        report["diagnostic_complete"] = True
        report["action_override_or_safety_bypass_used"] = False
        self.atomic_report(report)
        return report


def main():
    rospy.init_node(
        "v3_scope_multiactor_actual_teb_diagnostic_episode"
    )
    try:
        DiagnosticEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R2 diagnostic observed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
