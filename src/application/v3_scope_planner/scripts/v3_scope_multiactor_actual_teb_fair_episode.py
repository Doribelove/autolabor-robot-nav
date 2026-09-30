#!/usr/bin/env python3
"""One fair V3-13 analytic versus actual-TebLocalPlannerROS episode."""

import importlib.util
import math
import os
from pathlib import Path
import sys
import time

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path as NavigationPath

from v3_scope_planner import execution_core
from v3_scope_planner import multi_actor_core_v1


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/"
    "v3_scope_planner/scripts/v3_scope_multiactor_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_multiactor_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)

STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class MultiActorFairEpisode(BASE.CloneEpisode):
    def __init__(self):
        super().__init__()
        self.method = rospy.get_param("~method")
        if self.method not in ("v3_13_analytic", "actual_teb"):
            raise RuntimeError("unknown method: {}".format(self.method))
        self.predictor_model_path = rospy.get_param(
            "~predictor_model_path"
        )
        self.goal_world_x = float(
            rospy.get_param("~goal_world_x", 5.2)
        )
        self.goal_tolerance_m = float(
            rospy.get_param("~goal_tolerance_m", 0.25)
        )
        self.timeout_s = float(rospy.get_param("~timeout_s", 22.0))
        self.speed_estimator_window_s = float(
            rospy.get_param("~speed_estimator_window_s", 0.50)
        )
        self.early_recovery_sim_s = float(
            rospy.get_param("~early_recovery_sim_s", 8.5)
        )
        self.late_recovery_sim_s = float(
            rospy.get_param("~late_recovery_sim_s", 12.5)
        )
        if abs(self.speed_estimator_window_s - 0.50) > 1.0e-12:
            raise RuntimeError("unreleased recovery speed estimator")
        if (
            abs(self.early_recovery_sim_s - 8.5) > 1.0e-12
            or abs(self.late_recovery_sim_s - 12.5) > 1.0e-12
        ):
            raise RuntimeError("unreleased recovery clock barrier")
        with open(
            self.predictor_model_path, "r", encoding="utf-8"
        ) as stream:
            self.predictor_model = yaml.safe_load(stream)

        self.obstacle_pub = rospy.Publisher(
            "/move_base/TebLocalPlannerROS/obstacles",
            ObstacleArrayMsg,
            queue_size=1,
        )
        self.local_plan_count = 0
        rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/local_plan",
            NavigationPath,
            self._local_plan,
            queue_size=5,
        )
        self.client = None
        if self.method == "actual_teb":
            self.client = actionlib.SimpleActionClient(
                "/move_base", MoveBaseAction
            )

    def _local_plan(self, message):
        if self.collecting and message.poses:
            self.local_plan_count += 1

    def publish_tracks_for_teb(self, tracks):
        message = ObstacleArrayMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        obstacles = []
        specs = {
            int(spec["track_id"]): spec for spec in self.actor_specs
        }
        for track in sorted(tracks, key=lambda item: int(item["track_id"])):
            spec = specs[int(track["track_id"])]
            obstacle = ObstacleMsg()
            obstacle.id = int(track["track_id"])
            point = Point32()
            point.x = float(track["world_x_m"]) - self.spawn_world_x
            point.y = float(track["world_y_m"])
            point.z = 0.0
            obstacle.polygon.points = [point]
            obstacle.radius = float(spec["actor_radius_m"])
            obstacle.velocities.twist.linear.x = float(
                track["velocity_x_mps"]
            )
            obstacle.velocities.twist.linear.y = float(
                track["velocity_y_mps"]
            )
            yaw_rad = math.radians(float(spec["path_angle_deg"]))
            obstacle.orientation.z = math.sin(0.5 * yaw_rad)
            obstacle.orientation.w = math.cos(0.5 * yaw_rad)
            obstacles.append(obstacle)
        message.obstacles = obstacles
        self.obstacle_pub.publish(message)

    def wait_ready(self):
        super().wait_ready()
        if self.client is not None and not self.client.wait_for_server(
            rospy.Duration(25.0)
        ):
            raise RuntimeError("move_base action server unavailable")

    def warmup(self):
        rate = rospy.Rate(100)
        while (
            not rospy.is_shutdown()
            and rospy.Time.now().to_sec() < self.activation_time_s
        ):
            self.publish_tracks_for_teb(self.actor_tracks())
            self.publish_command(0.0, 0.0)
            rate.sleep()
        activation_lateness_s = max(
            0.0, rospy.Time.now().to_sec() - self.activation_time_s
        )
        if activation_lateness_s > 0.075:
            raise RuntimeError("shared activation barrier missed")
        started_s = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        trace = []
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            self.publish_tracks_for_teb(tracks)
            speed_mps = float(self.odom.twist.twist.linear.x)
            trace.append(
                {
                    "time_s": now_s,
                    "ego_world_x_m": pose["x_m"],
                    "ego_speed_mps": speed_mps,
                    "actors": [
                        {
                            "track_id": track["track_id"],
                            "coordinate_m": track["coordinate_m"],
                            "coordinate_speed_mps": track[
                                "coordinate_speed_mps"
                            ],
                        }
                        for track in tracks
                    ],
                }
            )
            if (
                pose["x_m"] >= self.gate_world_x - 0.018
                and speed_mps >= self.warmup_speed_mps - 0.035
            ):
                break
            if now_s - started_s > self.warmup_timeout_s:
                raise RuntimeError("shared warmup timed out")
            if pose["x_m"] > self.gate_world_x + 0.08:
                raise RuntimeError("shared warmup overshot gate")
            self.publish_command(self.warmup_speed_mps, 0.0)
            rate.sleep()
        return {
            "duration_s": rospy.Time.now().to_sec() - started_s,
            "activation_target_s": self.activation_time_s,
            "activation_lateness_s": activation_lateness_s,
            "sample_count": len(trace),
            "gate_pose": self.robot_world_pose(),
            "gate_speed_mps": float(self.odom.twist.twist.linear.x),
            "gate_actor_tracks": self.actor_tracks(),
            "trace_digest": multi_actor_core_v1.digest(trace),
            "trace_tail": trace[-12:],
        }

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
        origin_x = float(spec["path_origin_x_m"])
        origin_y = float(spec["path_origin_y_m"])
        start_coordinate = (
            (start_x - origin_x) * direction_x
            + (start_y - origin_y) * direction_y
        )
        end_coordinate = (
            (end_x - origin_x) * direction_x
            + (end_y - origin_y) * direction_y
        )
        track["coordinate_speed_mps"] = max(
            0.0,
            (end_coordinate - start_coordinate)
            / self.speed_estimator_window_s,
        )
        track["velocity_x_mps"] = (
            end_x - start_x
        ) / self.speed_estimator_window_s
        track["velocity_y_mps"] = (
            end_y - start_y
        ) / self.speed_estimator_window_s
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

    def plan_v3(self, label, recovery):
        pose = self.robot_world_pose()
        tracks = (
            self.windowed_actor_tracks()
            if recovery
            else self.actor_tracks()
        )
        snapshot = {
            "scene_snapshot_id": "v3_13_q1_{}_{}_{}_{}".format(
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
        started_ns = time.perf_counter_ns()
        result = multi_actor_core_v1.plan(
            snapshot, self.predictor_model, self.contract
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
            "recovery_observer_used": bool(recovery),
            "snapshot": snapshot,
            "latency_ms": latency_ms,
            "plan_result_digest": multi_actor_core_v1.digest(result),
            "selected_candidate_id": selected_id,
            "selected_candidate": selected_candidate,
            "selected_evaluation": selected_evaluation,
            "safety_fallback_used": result["decision"][
                "safety_fallback_used"
            ],
        }

    def send_teb_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = (
            self.goal_world_x - self.spawn_world_x
        )
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

    def goal_state(self, pose):
        position_error = math.hypot(
            pose["x_m"] - self.goal_world_x, pose["y_m"]
        )
        yaw_error = abs(
            execution_core.angle_delta(0.0, pose["yaw_rad"])
        )
        if self.method == "actual_teb":
            reached = (
                self.client.get_state() == GoalStatus.SUCCEEDED
                and position_error <= self.goal_tolerance_m
            )
        else:
            reached = (
                position_error <= self.goal_tolerance_m
                and yaw_error <= 0.22
            )
        return reached, position_error, yaw_error

    def execute(self, initial_plan):
        if self.method == "actual_teb":
            self.send_teb_goal()
        decisions = [initial_plan] if initial_plan is not None else []
        active_candidate = None
        path_index = 0
        if (
            initial_plan is not None
            and initial_plan["selected_candidate_id"]
            != "certified_stop_v000"
        ):
            active_candidate = initial_plan["selected_candidate"]
        next_recovery = (
            self.early_recovery_sim_s
            if initial_plan is not None and active_candidate is None
            else None
        )
        started_s = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_length_m = 0.0
        minimum_clearance_m = float("inf")
        tracking_errors = []
        trace = []
        goal_reached = False
        timeout = False
        self.collecting = True
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            elapsed_s = now_s - started_s
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            self.publish_tracks_for_teb(tracks)
            step_m = math.hypot(
                pose["x_m"] - previous_pose["x_m"],
                pose["y_m"] - previous_pose["y_m"],
            )
            if step_m <= 0.25:
                path_length_m += step_m
            previous_pose = pose
            clearances = [
                math.hypot(
                    pose["x_m"] - track["world_x_m"],
                    pose["y_m"] - track["world_y_m"],
                )
                - self.combined_radius_m
                for track in tracks
            ]
            minimum_clearance_m = min(
                minimum_clearance_m, min(clearances)
            )
            goal_reached, position_error, yaw_error = self.goal_state(
                pose
            )
            trace.append(
                {
                    "elapsed_s": elapsed_s,
                    "sim_time_s": now_s,
                    "ego": {
                        "x_m": pose["x_m"],
                        "y_m": pose["y_m"],
                        "yaw_rad": pose["yaw_rad"],
                        "speed_mps": float(
                            self.odom.twist.twist.linear.x
                        ),
                    },
                    "actors": tracks,
                    "actor_clearances_m": clearances,
                    "active_candidate_id": (
                        active_candidate["candidate_id"]
                        if active_candidate is not None
                        else None
                    ),
                }
            )
            if goal_reached:
                break
            if elapsed_s >= self.timeout_s:
                timeout = True
                break

            if self.method == "v3_13_analytic":
                if (
                    active_candidate is None
                    and next_recovery is not None
                    and now_s >= next_recovery
                ):
                    label = (
                        "early_recovery"
                        if len(decisions) == 1
                        else "late_recovery"
                    )
                    plan = self.plan_v3(label, recovery=True)
                    decisions.append(plan)
                    if (
                        plan["selected_candidate_id"]
                        != "certified_stop_v000"
                    ):
                        active_candidate = plan["selected_candidate"]
                        path_index = 0
                        next_recovery = None
                    elif label == "early_recovery":
                        next_recovery = self.late_recovery_sim_s
                    else:
                        next_recovery = None
                if active_candidate is None:
                    self.publish_command(0.0, 0.0)
                else:
                    command = execution_core.pure_pursuit_command(
                        active_candidate["points"],
                        pose["x_m"],
                        pose["y_m"],
                        pose["yaw_rad"],
                        minimum_index=path_index,
                        lookahead_m=self.lookahead_m,
                    )
                    path_index = max(
                        path_index, command["nearest_index"]
                    )
                    tracking_errors.append(
                        command["tracking_error_m"]
                    )
                    self.publish_command(
                        command["linear_x_mps"],
                        command["angular_z_radps"],
                    )
            rate.sleep()

        elapsed_s = rospy.Time.now().to_sec() - started_s
        self.collecting = False
        if self.client is not None:
            action_state = self.client.get_state()
            if not goal_reached:
                self.client.cancel_goal()
        else:
            action_state = None
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        final_pose = self.robot_world_pose()
        _, final_position_error, final_yaw_error = self.goal_state(
            final_pose
        )
        return {
            "goal_reached": bool(goal_reached),
            "timeout": bool(timeout),
            "elapsed_s": elapsed_s,
            "restricted_elapsed_s": min(elapsed_s, self.timeout_s),
            "path_length_m": path_length_m,
            "minimum_conservative_clearance_m": minimum_clearance_m,
            "contact_state_count": self.contact_count,
            "minimum_scan_range_m": (
                self.minimum_scan_m
                if math.isfinite(self.minimum_scan_m)
                else None
            ),
            "command_count": self.command_count,
            "maximum_command_gap_s": self.maximum_command_gap_s,
            "local_plan_count": self.local_plan_count,
            "action_state": action_state,
            "action_state_name": STATUS_NAMES.get(
                action_state, "NOT_USED"
            ),
            "final_pose": final_pose,
            "final_position_error_m": final_position_error,
            "final_yaw_error_rad": final_yaw_error,
            "tracking_error_max_m": (
                max(tracking_errors) if tracking_errors else None
            ),
            "tracking_error_mean_m": (
                sum(tracking_errors) / len(tracking_errors)
                if tracking_errors
                else None
            ),
            "decisions": decisions,
            "initial_stop": bool(
                decisions
                and decisions[0]["selected_candidate_id"]
                == "certified_stop_v000"
            ),
            "eventual_progress": bool(
                any(
                    decision["selected_candidate_id"]
                    != "certified_stop_v000"
                    for decision in decisions
                )
            ),
            "trace_sample_count": len(trace),
            "trace_digest": multi_actor_core_v1.digest(trace),
            "trace": trace,
        }

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        gate_modes = {}
        mode_matches = {}
        specs = {
            int(spec["track_id"]): spec for spec in self.actor_specs
        }
        for track in gate["gate_actor_tracks"]:
            track_id = int(track["track_id"])
            try:
                mode = multi_actor_core_v1.infer_observable_mode(
                    track["coordinate_speed_mps"], self.contract
                )
            except Exception as error:
                mode = "ERROR:{}:{}".format(
                    type(error).__name__, error
                )
            gate_modes[track_id] = mode
            mode_matches[track_id] = (
                mode == specs[track_id]["evaluator_policy"]
            )
        initial_plan = (
            self.plan_v3("initial_gate", recovery=False)
            if self.method == "v3_13_analytic"
            else None
        )
        result = self.execute(initial_plan)
        gate_pose = gate["gate_pose"]
        gate_valid = (
            abs(gate_pose["x_m"] - self.gate_world_x) <= 0.03
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(gate["gate_speed_mps"] - 0.8) <= 0.04
            and all(
                track["age_s"] <= 0.20
                for track in gate["gate_actor_tracks"]
            )
        )
        v3_decisions_accepted = (
            self.method != "v3_13_analytic"
            or all(
                decision["selected_evaluation"]["accepted"] is True
                for decision in result["decisions"]
            )
        )
        v3_tracking_valid = (
            self.method != "v3_13_analytic"
            or (
                result["tracking_error_max_m"] is not None
                and result["tracking_error_max_m"] <= 0.45
            )
        )
        report = {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": "V3-13-Q1",
            "simulation_only": True,
            "method": self.method,
            "scene_id": self.scene_id,
            "block_index": self.block_index,
            "seed": self.seed,
            "actor_count": len(self.actor_specs),
            "actor_policies_evaluator_only": [
                spec["evaluator_policy"] for spec in self.actor_specs
            ],
            "hidden_policy_runtime_input": False,
            "shared_vehicle_model": "autolabor_m2_ackermann",
            "shared_command_boundary": "/cmd_vel_navigation",
            "shared_oracle_actor_truth_source": True,
            "perception_claim_authorized": False,
            "planner_contract": {
                "path": self.planner_contract_path,
                "sha256": BASE.sha256_file(
                    self.planner_contract_path
                ),
            },
            "predictor_model": {
                "path": self.predictor_model_path,
                "sha256": BASE.sha256_file(
                    self.predictor_model_path
                ),
            },
            "scene_spec": {
                "path": self.scene_spec_path,
                "sha256": BASE.sha256_file(self.scene_spec_path),
            },
            "actor_specs": {
                "path": self.actor_specs_path,
                "sha256": BASE.sha256_file(self.actor_specs_path),
            },
            "warmup": gate,
            "gate_observable_modes": gate_modes,
            "gate_actor_mode_matches_evaluator_policy": mode_matches,
            "gate_physical_valid": gate_valid,
            "result": result,
            "episode_pass": (
                gate_valid
                and all(mode_matches.values())
                and result["goal_reached"]
                and result["contact_state_count"] == 0
                and result["minimum_conservative_clearance_m"] >= 0.0
                and result["command_count"] > 0
                and v3_decisions_accepted
                and v3_tracking_valid
                and (
                    self.method != "actual_teb"
                    or result["local_plan_count"] > 0
                )
                and (
                    self.method != "v3_13_analytic"
                    or not result["initial_stop"]
                    or result["eventual_progress"]
                )
            ),
            "attempt": 1,
            "retry_resume_or_replacement_used": False,
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
            "V3-13-Q1 fair episode complete block=%d method=%s "
            "pass=%s time=%.3f",
            self.block_index,
            self.method,
            report["episode_pass"],
            result["restricted_elapsed_s"],
        )
        return report


def main():
    rospy.init_node("v3_scope_multiactor_actual_teb_fair_episode")
    try:
        MultiActorFairEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1 fair episode failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
