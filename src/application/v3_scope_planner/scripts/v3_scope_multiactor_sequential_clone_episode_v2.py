#!/usr/bin/env python3
"""Synchronized V3-13 stop-then-branch cloned intervention episode."""

import importlib.util
import math
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import multi_actor_core


S1_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/"
    "v3_scope_planner/scripts/"
    "v3_scope_multiactor_sequential_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_s2_sequential_base", S1_PATH
)
S1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S1)


class SynchronizedSequentialCloneEpisode(S1.SequentialCloneEpisode):
    def __init__(self):
        super().__init__()
        self.branch_target_sim_s = float(
            rospy.get_param("~branch_target_sim_s")
        )
        self.speed_estimator_window_s = float(
            rospy.get_param("~speed_estimator_window_s", 0.50)
        )
        if self.branch_target_sim_s not in (8.5, 12.5):
            raise RuntimeError("unreleased absolute branch time")
        if abs(self.speed_estimator_window_s - 0.50) > 1.0e-12:
            raise RuntimeError("unreleased speed estimator window")

    def windowed_actor_track(self, spec):
        track = self.actor_track(spec)
        history = list(self.histories[int(spec["track_id"])])
        if len(history) < 2:
            raise RuntimeError("actor history unavailable")
        end_s = float(history[-1][0])
        start_s = end_s - self.speed_estimator_window_s
        if start_s < float(history[0][0]):
            raise RuntimeError("actor speed window unavailable")
        start_x = self.interpolate(history, start_s, 1)
        start_y = self.interpolate(history, start_s, 2)
        end_x = float(history[-1][1])
        end_y = float(history[-1][2])
        angle_rad = math.radians(float(spec["path_angle_deg"]))
        direction_x = math.cos(angle_rad)
        direction_y = math.sin(angle_rad)
        start_coordinate = (
            (start_x - float(spec["path_origin_x_m"])) * direction_x
            + (start_y - float(spec["path_origin_y_m"])) * direction_y
        )
        end_coordinate = (
            (end_x - float(spec["path_origin_x_m"])) * direction_x
            + (end_y - float(spec["path_origin_y_m"])) * direction_y
        )
        track["coordinate_speed_mps"] = max(
            0.0,
            (end_coordinate - start_coordinate)
            / self.speed_estimator_window_s,
        )
        track["velocity_x_mps"] = (
            (end_x - start_x) / self.speed_estimator_window_s
        )
        track["velocity_y_mps"] = (
            (end_y - start_y) / self.speed_estimator_window_s
        )
        track["velocity_estimator"] = (
            "causal_coordinate_slope_window_0_50_s"
        )
        track["velocity_window_start_s"] = start_s
        track["velocity_window_end_s"] = end_s
        return track

    def windowed_actor_tracks(self):
        return [
            self.windowed_actor_track(spec)
            for spec in sorted(
                self.actor_specs,
                key=lambda item: int(item["track_id"]),
            )
        ]

    def reset_branch_measurements(self):
        self.contact_count = 0
        self.minimum_scan_m = float("inf")
        self.command_count = 0
        self.maximum_command_gap_s = 0.0
        self.last_command_time_s = None

    def wait_common_stop(self):
        started_s = rospy.Time.now().to_sec()
        if started_s >= self.branch_target_sim_s:
            raise RuntimeError("absolute branch barrier already missed")
        trace = []
        next_trace_s = started_s
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            if now_s + 1.0e-12 >= next_trace_s:
                pose = self.robot_world_pose()
                trace.append(
                    {
                        "elapsed_s": now_s - started_s,
                        "sim_time_s": now_s,
                        "ego": {
                            "x_m": pose["x_m"],
                            "y_m": pose["y_m"],
                            "yaw_rad": pose["yaw_rad"],
                            "speed_mps": float(
                                self.odom.twist.twist.linear.x
                            ),
                        },
                        "actors": self.actor_tracks(),
                    }
                )
                next_trace_s += 0.05
            self.publish_command(0.0, 0.0)
            if now_s >= self.branch_target_sim_s:
                break
            rate.sleep()
        observed_s = rospy.Time.now().to_sec()
        decision_tracks = self.windowed_actor_tracks()
        decision_pose = self.robot_world_pose()
        decision_speed = float(self.odom.twist.twist.linear.x)
        return {
            "nominal_wait_depth_s": self.pre_action_wait_s,
            "actual_wait_duration_s": observed_s - started_s,
            "branch_target_sim_s": self.branch_target_sim_s,
            "branch_observed_sim_s": observed_s,
            "branch_clock_absolute_error_s": abs(
                observed_s - self.branch_target_sim_s
            ),
            "sample_count": len(trace),
            "trace": trace,
            "trace_digest": multi_actor_core.digest(trace),
            "decision_pose": decision_pose,
            "decision_speed_mps": decision_speed,
            "decision_actor_tracks": decision_tracks,
        }

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        decision = self.wait_common_stop()
        snapshot, candidate_set, candidate = self.branch_candidate(
            decision
        )
        self.reset_branch_measurements()
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
            and decision["branch_clock_absolute_error_s"] <= 0.012
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
            "stage": "V3-13-S2",
            "simulation_only": True,
            "method": (
                "absolute_clock_matched_stop_then_cloned_branch"
            ),
            "forced_candidate_id": self.forced_candidate_id,
            "behavior_propensity": self.behavior_propensity,
            "pre_action_wait_s": self.pre_action_wait_s,
            "branch_target_sim_s": self.branch_target_sim_s,
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
                "sha256": S1.BASE.sha256_file(
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
            "V3-13-S2 synchronized clone complete block=%d "
            "target=%.1f action=%s pass=%s",
            self.block_index,
            self.branch_target_sim_s,
            self.forced_candidate_id,
            report["episode_collection_pass"],
        )
        return report


def main():
    rospy.init_node(
        "v3_scope_multiactor_sequential_clone_episode_v2"
    )
    try:
        SynchronizedSequentialCloneEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-S2 synchronized clone failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
