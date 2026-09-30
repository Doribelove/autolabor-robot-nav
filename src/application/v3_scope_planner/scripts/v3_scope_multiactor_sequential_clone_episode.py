#!/usr/bin/env python3
"""One V3-13 matched stop-then-branch sequential clone episode."""

import hashlib
import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import multi_actor_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/"
    "v3_scope_planner/scripts/v3_scope_multiactor_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_sequential_clone_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class SequentialCloneEpisode(BASE.CloneEpisode):
    def __init__(self):
        super().__init__()
        self.pre_action_wait_s = float(
            rospy.get_param("~pre_action_wait_s")
        )
        if self.pre_action_wait_s not in (1.0, 5.0):
            raise RuntimeError("unreleased wait depth")

    def wait_common_stop(self):
        started_s = rospy.Time.now().to_sec()
        trace = []
        rate = rospy.Rate(20)
        self.collecting = True
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            elapsed_s = now_s - started_s
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            trace.append(
                {
                    "elapsed_s": elapsed_s,
                    "ego": {
                        "x_m": pose["x_m"],
                        "y_m": pose["y_m"],
                        "yaw_rad": pose["yaw_rad"],
                        "speed_mps": float(
                            self.odom.twist.twist.linear.x
                        ),
                    },
                    "actors": tracks,
                }
            )
            if elapsed_s >= self.pre_action_wait_s:
                break
            self.publish_command(0.0, 0.0)
            rate.sleep()
        self.collecting = False
        for _ in range(4):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        return {
            "wait_depth_s": self.pre_action_wait_s,
            "duration_s": trace[-1]["elapsed_s"],
            "sample_count": len(trace),
            "trace": trace,
            "trace_digest": multi_actor_core.digest(trace),
            "decision_pose": self.robot_world_pose(),
            "decision_speed_mps": float(
                self.odom.twist.twist.linear.x
            ),
            "decision_actor_tracks": self.actor_tracks(),
        }

    def branch_candidate(self, decision):
        pose = decision["decision_pose"]
        tracks = decision["decision_actor_tracks"]
        snapshot = {
            "scene_snapshot_id": (
                "v3_13_s1_{}_{}_{}_{}".format(
                    self.scene_id,
                    self.seed,
                    self.block_index,
                    int(round(10.0 * self.pre_action_wait_s)),
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
        candidate_set = multi_actor_core.generate_candidate_set(
            snapshot, self.contract
        )
        candidate = next(
            item
            for item in candidate_set["candidates"]
            if item["candidate_id"] == self.forced_candidate_id
        )
        failures = multi_actor_core.validate_candidate(
            candidate, self.contract
        )
        if failures:
            raise RuntimeError(
                "branch candidate invalid: {}".format(failures)
            )
        return snapshot, candidate_set, candidate

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        decision = self.wait_common_stop()
        snapshot, candidate_set, candidate = self.branch_candidate(
            decision
        )
        result = self.execute(candidate)
        gate_pose = gate["gate_pose"]
        gate_valid = (
            abs(gate_pose["x_m"] - self.gate_world_x) <= 0.03
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(gate["gate_speed_mps"] - 0.8) <= 0.04
        )
        decision_pose = decision["decision_pose"]
        decision_valid = (
            -2.70 <= decision_pose["x_m"] <= -2.10
            and abs(decision_pose["y_m"]) <= 0.05
            and abs(decision_pose["yaw_rad"]) <= 0.05
            and abs(decision["decision_speed_mps"]) <= 0.08
            and all(
                track["age_s"] <= 0.20
                for track in decision["decision_actor_tracks"]
            )
        )
        modes = {}
        for track in decision["decision_actor_tracks"]:
            try:
                modes[track["track_id"]] = (
                    multi_actor_core.infer_observable_mode(
                        track["coordinate_speed_mps"], self.contract
                    )
                )
            except Exception as error:
                modes[track["track_id"]] = "ERROR:{}".format(error)
        report = {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": "V3-13-S1",
            "simulation_only": True,
            "method": "matched_stop_then_cloned_branch",
            "forced_candidate_id": self.forced_candidate_id,
            "behavior_propensity": self.behavior_propensity,
            "pre_action_wait_s": self.pre_action_wait_s,
            "block_index": self.block_index,
            "seed": self.seed,
            "scene_id": self.scene_id,
            "actor_count": len(self.actor_specs),
            "actor_policies_evaluator_only": [
                spec["evaluator_policy"] for spec in self.actor_specs
            ],
            "hidden_policy_runtime_input": False,
            "shared_vehicle_model": "autolabor_m2_ackermann",
            "shared_command_boundary": "/cmd_vel_navigation",
            "planner_contract": {
                "path": self.planner_contract_path,
                "sha256": BASE.sha256_file(
                    self.planner_contract_path
                ),
            },
            "warmup": gate,
            "gate_physical_valid": gate_valid,
            "common_stop_prefix": decision,
            "decision_physical_valid": decision_valid,
            "observable_modes_at_branch": modes,
            "branch_snapshot": snapshot,
            "candidate_set_digest": candidate_set[
                "candidate_set_digest"
            ],
            "candidate_digest": candidate["candidate_digest"],
            "result": result,
            "episode_collection_pass": (
                gate_valid
                and decision_valid
                and list(modes.values()).count("assertive")
                == len(self.actor_specs)
                and result["completed_horizon"]
                and result["trace_sample_count"] >= 130
            ),
            "unsafe_outcome_retained": True,
            "policy_or_ranker_training_steps": 0,
            "hardware_accessed": False,
            "real_vehicle_used": False,
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V3-13-S1 sequential clone complete block=%d wait=%.1f action=%s pass=%s",
            self.block_index,
            self.pre_action_wait_s,
            self.forced_candidate_id,
            report["episode_collection_pass"],
        )
        return report


def main():
    rospy.init_node("v3_scope_multiactor_sequential_clone_episode")
    try:
        SequentialCloneEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-S1 sequential clone failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())

