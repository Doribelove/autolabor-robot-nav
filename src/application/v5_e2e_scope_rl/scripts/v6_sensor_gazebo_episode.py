#!/usr/bin/env python3
"""One sensor-only V6/analytic/Actual-TEB Gazebo comparison episode."""

import collections
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import time

import rospy
import torch
import yaml

from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.runtime_adapter import SensorRuntimeAdapter
from v3_scope_planner import execution_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/scripts/"
    "v3_scope_actual_teb_fair_episode.py"
)
SPEC = importlib.util.spec_from_file_location("v6_frozen_v3_fair_episode", str(BASE_PATH))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class V6SensorGazeboEpisode(BASE.FairEpisode):
    METHODS = ("pct_scope_rl_v6", "sensor_analytic_prior", "actual_teb")

    def __init__(self):
        self.requested_method = rospy.get_param("~method")
        if self.requested_method not in self.METHODS:
            raise RuntimeError("unknown V6 Gazebo method: " + self.requested_method)
        self.runtime_adapter = None
        self.last_runtime_scan_stamp_s = None
        internal = "actual_teb" if self.requested_method == "actual_teb" else "v3_spatial_safe"
        rospy.set_param("~method", internal)
        try:
            super().__init__()
        finally:
            rospy.set_param("~method", self.requested_method)
        self.actor_checkpoint_path = rospy.get_param("~actor_checkpoint_path", "")
        actor = None
        if self.requested_method == "pct_scope_rl_v6":
            checkpoint = torch.load(
                self.actor_checkpoint_path, map_location="cpu", weights_only=False
            )
            actor = SensorActor.from_actor_checkpoint(checkpoint)
            actor.eval()
        torch.set_num_threads(2)
        self.runtime_adapter = SensorRuntimeAdapter(actor=actor, advantage_gate_margin=0.10)
        self.decision_trace = []

    def _scan(self, message):
        BASE.FairEpisode._scan(self, message)
        if self.runtime_adapter is None:
            return
        stamp_s = message.header.stamp.to_sec()
        if stamp_s <= 0.0:
            stamp_s = rospy.Time.now().to_sec()
        if (
            self.last_runtime_scan_stamp_s is None
            or stamp_s - self.last_runtime_scan_stamp_s >= 0.20
        ):
            scan = self.runtime_adapter.resample_scan(
                message.ranges,
                message.angle_min,
                message.angle_increment,
                message.range_max,
            )
            self.runtime_adapter.append_scan(scan)
            self.last_runtime_scan_stamp_s = stamp_s

    def policy_pose_from_odometry(self):
        pose = self.odom.pose.pose
        return (
            float(pose.position.x) + self.spawn_world_x,
            float(pose.position.y),
            execution_core.quaternion_yaw(pose.orientation),
        )

    def execute_sensor(self):
        started_s = rospy.Time.now().to_sec()
        previous_world_pose = self.robot_world_pose()
        path_length_m = 0.0
        minimum_clearance_m = float("inf")
        tracking_errors = []
        active_plan = None
        next_decision_s = started_s
        goal_reached = False
        timeout = False
        rate = rospy.Rate(20)
        self.collecting = True
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            world_pose = self.robot_world_pose()
            policy_pose = self.policy_pose_from_odometry()
            evaluator_track = self.actor_track()
            step = math.hypot(
                world_pose["x_m"] - previous_world_pose["x_m"],
                world_pose["y_m"] - previous_world_pose["y_m"],
            )
            if step <= 0.25:
                path_length_m += step
            previous_world_pose = world_pose
            minimum_clearance_m = min(
                minimum_clearance_m,
                math.hypot(
                    world_pose["x_m"] - evaluator_track["world_x_m"],
                    world_pose["y_m"] - evaluator_track["world_y_m"],
                )
                - self.combined_radius_m,
            )
            if now_s + 1.0e-9 >= next_decision_s or active_plan is None:
                active_plan = self.runtime_adapter.select(
                    policy_pose,
                    float(self.odom.twist.twist.linear.x),
                    float(self.odom.twist.twist.angular.z),
                    now_s - started_s,
                )
                self.decision_trace.append(
                    {
                        "time_s": now_s,
                        "action_id": active_plan["action"],
                        "action_name": active_plan["action_name"],
                        "analytic_action": active_plan["analytic_action"],
                        "learned_action": active_plan["learned_action"],
                        "intervened": active_plan["intervened"],
                        "residual_margin": active_plan["residual_margin"],
                        "latency_ms": active_plan["latency_ms"],
                        "safe_action_count": int(
                            active_plan["observation"]["action_mask"].sum()
                        ),
                        "minimum_scan_m": float(
                            active_plan["observation"]["scans"][-1].min() * 8.0
                        ),
                    }
                )
                next_decision_s += 0.25
                if next_decision_s <= now_s:
                    next_decision_s = now_s + 0.25
            command = execution_core.pure_pursuit_command(
                active_plan["points"],
                policy_pose[0],
                policy_pose[1],
                policy_pose[2],
                minimum_index=0,
                lookahead_m=self.lookahead_m,
            )
            tracking_errors.append(command["tracking_error_m"])
            position_error = math.hypot(
                world_pose["x_m"] - self.goal_world_x, world_pose["y_m"]
            )
            yaw_error = abs(execution_core.angle_delta(0.0, world_pose["yaw_rad"]))
            goal_reached = position_error <= self.goal_tolerance_m and yaw_error <= 0.25
            if goal_reached:
                break
            if now_s - started_s >= self.timeout_s:
                timeout = True
                break
            self.publish_command(command["linear_x_mps"], command["angular_z_radps"])
            rate.sleep()
        elapsed_s = rospy.Time.now().to_sec() - started_s
        self.collecting = False
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        final_pose = self.robot_world_pose()
        return {
            "goal_reached": bool(goal_reached),
            "timeout": bool(timeout),
            "elapsed_s": elapsed_s,
            "restricted_elapsed_s": min(elapsed_s, self.timeout_s),
            "path_length_m": path_length_m,
            "minimum_conservative_clearance_m": minimum_clearance_m,
            "contact_state_count": self.contact_count,
            "minimum_scan_range_m": self.minimum_scan_m
            if math.isfinite(self.minimum_scan_m)
            else None,
            "command_count": self.cmd_count,
            "maximum_command_gap_s": self.max_cmd_gap_s,
            "local_plan_count": self.local_plan_count,
            "action_state": None,
            "action_state_name": "NOT_USED",
            "final_pose": final_pose,
            "final_position_error_m": math.hypot(
                final_pose["x_m"] - self.goal_world_x, final_pose["y_m"]
            ),
            "final_yaw_error_rad": abs(
                execution_core.angle_delta(0.0, final_pose["yaw_rad"])
            ),
            "tracking_error_max_m": max(tracking_errors) if tracking_errors else None,
            "tracking_error_mean_m": (
                sum(tracking_errors) / len(tracking_errors) if tracking_errors else None
            ),
            "decision_count": len(self.decision_trace),
            "intervention_count": sum(row["intervened"] for row in self.decision_trace),
            "mean_policy_latency_ms": (
                sum(row["latency_ms"] for row in self.decision_trace)
                / len(self.decision_trace)
                if self.decision_trace
                else None
            ),
        }

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        if self.requested_method == "actual_teb":
            result = BASE.FairEpisode.execute(self, gate, None)
        else:
            result = self.execute_sensor()
        gate_pose = gate["gate_pose"]
        gate_physical_valid = (
            abs(gate_pose["x_m"] - self.gate_world_x) <= 0.03
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(gate["gate_speed_mps"] - 0.8) <= 0.04
        )
        actor_hash = (
            sha256(self.actor_checkpoint_path)
            if self.requested_method == "pct_scope_rl_v6"
            else None
        )
        policy_input_audit = {
            "laser_scan_used": self.requested_method != "actual_teb",
            "odometry_used": self.requested_method != "actual_teb",
            "local_goal_used": self.requested_method != "actual_teb",
            "simulator_actor_world_track_used_by_policy": False,
            "gazebo_model_states_used_by_policy": False,
            "privileged_rank_target_used_by_policy": False,
            "actor_checkpoint_contains_privileged_modules": False
            if actor_hash
            else None,
            "evaluator_uses_world_state_for_metrics_only": True,
        }
        execution_integrity_pass = (
            gate_physical_valid
            and result["contact_state_count"] == 0
            and result["command_count"] > 0
            and result["minimum_conservative_clearance_m"] >= 0.0
            and (
                self.requested_method != "actual_teb" or result["local_plan_count"] > 0
            )
            and (
                self.requested_method == "actual_teb" or result["decision_count"] > 0
            )
        )
        report = {
            "schema_version": "1.0",
            "architecture_generation": "v6",
            "stage": "V6-03",
            "simulation_only": True,
            "method": self.requested_method,
            "scene_id": self.scene_id,
            "seed": self.seed,
            "actor_policy_evaluator_only": self.actor_policy,
            "actor_angle_deg": self.actor_angle_deg,
            "shared_vehicle_model": "autolabor_m2_ackermann",
            "shared_command_boundary": "/cmd_vel_navigation",
            "policy_input_audit": policy_input_audit,
            "actor_checkpoint": {
                "path": self.actor_checkpoint_path if actor_hash else None,
                "sha256": actor_hash,
            },
            "warmup": gate,
            "gate_physical_valid": gate_physical_valid,
            "decision_trace": self.decision_trace,
            "result": result,
            "execution_integrity_pass": bool(execution_integrity_pass),
            "episode_pass": bool(execution_integrity_pass),
            "scientific_inclusion": True,
            "hardware_accessed": False,
            "real_vehicle_used": False,
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        rospy.loginfo(
            "V6 sensor Gazebo episode method=%s integrity=%s goal=%s time=%.3f",
            self.requested_method,
            execution_integrity_pass,
            result["goal_reached"],
            result["restricted_elapsed_s"],
        )
        return report


def main():
    rospy.init_node("v6_sensor_gazebo_episode")
    try:
        V6SensorGazeboEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr("V6 Gazebo episode failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())

