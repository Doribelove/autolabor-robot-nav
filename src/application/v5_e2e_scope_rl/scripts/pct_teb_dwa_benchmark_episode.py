#!/usr/bin/env python3
"""One sensor-fair PCT/analytic/Actual-TEB/DWA Gazebo episode."""

from __future__ import annotations

import hashlib
import math
import os
import sys
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
import torch
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty as EmptyService

from v3_scope_planner import execution_core
from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.benchmark_runtime import (
    DirectionalBenchmarkRuntimeAdapter,
    RuntimeInputHealth,
)


METHODS = ("pct_scope_rl", "pct_shadow", "analytic_prior", "actual_teb", "dwa")
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


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_or_none(value):
    return float(value) if math.isfinite(float(value)) else None


class BenchmarkEpisode:
    def __init__(self):
        self.stage = str(rospy.get_param("~stage", "PCT-BENCH-DEVELOPMENT"))
        self.split = str(rospy.get_param("~split", "development"))
        if self.split not in ("development", "pilot", "formal_round_1", "formal_round_2"):
            raise RuntimeError("unknown benchmark split: " + self.split)
        self.method = rospy.get_param("~method")
        if self.method not in METHODS:
            raise RuntimeError("unknown benchmark method: " + str(self.method))
        self.scene_id = rospy.get_param("~scene_id")
        self.seed = int(rospy.get_param("~seed"))
        self.report_path = rospy.get_param("~report_path")
        self.scene_manifest_path = rospy.get_param("~scene_manifest")
        self.actor_checkpoint_path = rospy.get_param("~actor_checkpoint_path", "")
        self.spawn_world_x = float(rospy.get_param("~spawn_world_x", -5.4))
        self.policy_start_world_x = float(rospy.get_param("~policy_start_world_x", -4.0))
        self.goal_world_x = float(rospy.get_param("~goal_world_x", 5.0))
        self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 0.25))
        self.timeout_s = float(rospy.get_param("~timeout_s", 22.0))
        self.activation_time_s = float(rospy.get_param("~activation_time_s", 5.0))
        self.warmup_speed_mps = float(rospy.get_param("~warmup_speed_mps", 0.8))
        self.warmup_gate_tolerance_m = float(rospy.get_param("~warmup_gate_tolerance_m", 0.030))
        self.lookahead_m = float(rospy.get_param("~lookahead_m", 0.70))
        self.combined_radius_m = float(rospy.get_param("~combined_radius_m", 1.15))
        with open(self.scene_manifest_path, "r", encoding="utf-8") as stream:
            self.scene = yaml.safe_load(stream)
        if self.scene.get("scene_id") != self.scene_id or int(self.scene.get("seed")) != self.seed:
            raise RuntimeError("scene manifest identity mismatch")
        self.actor_models = [str(item["model_name"]) for item in self.scene.get("actors", [])]
        self.ego_model = "autolabor_m2"

        actor = None
        self.actor_checkpoint_sha256 = None
        if self.method in ("pct_scope_rl", "pct_shadow"):
            if not self.actor_checkpoint_path:
                raise RuntimeError("PCT actor checkpoint absent")
            checkpoint = torch.load(
                self.actor_checkpoint_path, map_location="cpu", weights_only=False
            )
            actor = SensorActor.from_actor_checkpoint(checkpoint)
            actor.eval()
            self.actor_checkpoint_sha256 = sha256(self.actor_checkpoint_path)
        torch.set_num_threads(2)
        wall_half_width = self.scene.get("corridor_half_width_m")
        if wall_half_width is None:
            wall_half_width = 20.0
        self.runtime = DirectionalBenchmarkRuntimeAdapter(
            actor=actor,
            start_x_m=self.policy_start_world_x,
            goal_x_m=self.goal_world_x,
            goal_tolerance_m=self.goal_tolerance_m,
            wall_half_width_m=float(wall_half_width),
            static_circles=self.scene.get("static_circles", []),
            static_boxes=self.scene.get("static_boxes", []),
            shield_inflation_m=0.33,
            inference_deadline_ms=20.0,
            maximum_input_age_s=0.20,
            goal_speed_envelope=True,
            safety_response_delay_s=0.35,
            certified_stop_dwell_s=1.50,
        )

        self.odom = None
        self.odom_received_s = None
        self.scan = None
        self.scan_received_s = None
        self.last_runtime_scan_stamp_s = None
        self.model_states = None
        self.model_index = {}
        self.contact_state_count = 0
        self.command_count = 0
        self.command_stamps = []
        self.scan_stamps = []
        self.odom_stamps = []
        self.local_plan_count = 0
        self.minimum_scan_m = float("inf")
        self.collecting = False
        self.tf_success_count = 0
        self.tf_failure_count = 0
        self.maximum_scan_age_s = 0.0
        self.maximum_odom_age_s = 0.0
        self.maximum_tf_age_s = 0.0
        self.decision_trace = []
        self.control_trace = []
        self.scan_transform_success_count = 0
        self.scan_transform_failure_count = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.command_pub = rospy.Publisher("/cmd_vel_navigation", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=50)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models, queue_size=20)
        self.contact_subscriber = rospy.Subscriber(
            "/m2_gazebo/contacts", ContactsState, self._contacts, queue_size=1000
        )
        rospy.Subscriber("/cmd_vel_navigation", Twist, self._command, queue_size=100)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self._local_plan, queue_size=10)
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self._local_plan, queue_size=10)
        self.client = None
        if self.method in ("actual_teb", "dwa"):
            self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def _odom(self, message):
        self.odom = message
        stamp = message.header.stamp.to_sec() or rospy.Time.now().to_sec()
        self.odom_received_s = stamp
        self.odom_stamps.append(stamp)

    def _scan(self, message):
        self.scan = message
        stamp = message.header.stamp.to_sec() or rospy.Time.now().to_sec()
        self.scan_received_s = stamp
        self.scan_stamps.append(stamp)
        finite = [float(value) for value in message.ranges if math.isfinite(float(value))]
        if self.collecting and finite:
            self.minimum_scan_m = min(self.minimum_scan_m, min(finite))
        if self.last_runtime_scan_stamp_s is None or stamp - self.last_runtime_scan_stamp_s >= 0.20:
            try:
                scan = self.runtime.resample_scan(
                    message.ranges,
                    message.angle_min,
                    message.angle_increment,
                    message.range_max,
                )
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    str(message.header.frame_id).lstrip("/"),
                    message.header.stamp,
                    rospy.Duration(0.03),
                )
                scan_pose = (
                    float(transform.transform.translation.x) + self.spawn_world_x,
                    float(transform.transform.translation.y),
                    execution_core.quaternion_yaw(transform.transform.rotation),
                )
                self.runtime.append_scan_with_pose(scan, scan_pose, stamp)
                self.scan_transform_success_count += 1
                self.last_runtime_scan_stamp_s = stamp
            except Exception as error:
                self.scan_transform_failure_count += 1
                rospy.logwarn_throttle(1.0, "benchmark rejected scan: %s", error)

    def _models(self, message):
        self.model_states = message
        self.model_index = {name: index for index, name in enumerate(message.name)}

    def _contacts(self, message):
        if self.collecting:
            self.contact_state_count += len(message.states)

    def _command(self, _message):
        if self.collecting:
            self.command_count += 1
            self.command_stamps.append(rospy.Time.now().to_sec())

    def _local_plan(self, message):
        if self.collecting and message.poses:
            self.local_plan_count += 1

    def robot_world_pose(self):
        if self.model_states is None or self.ego_model not in self.model_index:
            raise RuntimeError("ego world pose unavailable")
        pose = self.model_states.pose[self.model_index[self.ego_model]]
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": execution_core.quaternion_yaw(pose.orientation),
        }

    def policy_pose(self):
        if self.odom is None:
            raise RuntimeError("odometry unavailable")
        pose = self.odom.pose.pose
        return (
            float(pose.position.x) + self.spawn_world_x,
            float(pose.position.y),
            execution_core.quaternion_yaw(pose.orientation),
        )

    def actor_world_poses(self):
        result = []
        for item in self.scene.get("actors", []):
            name = str(item["model_name"])
            if name not in self.model_index:
                continue
            pose = self.model_states.pose[self.model_index[name]]
            result.append(
                {
                    "model_name": name,
                    "x_m": float(pose.position.x),
                    "y_m": float(pose.position.y),
                    "radius_m": float(item.get("radius_m", 0.36)),
                }
            )
        return result

    def conservative_clearance(self, pose):
        robot_eval_radius = self.combined_radius_m - 0.36
        values = []
        half_width = self.scene.get("corridor_half_width_m")
        if half_width is not None:
            values.append(float(half_width) - abs(pose["y_m"]) - robot_eval_radius)
        for actor in self.actor_world_poses():
            values.append(
                math.hypot(pose["x_m"] - actor["x_m"], pose["y_m"] - actor["y_m"])
                - robot_eval_radius
                - actor["radius_m"]
            )
        for circle in self.scene.get("static_circles", []):
            values.append(
                math.hypot(pose["x_m"] - float(circle["x"]), pose["y_m"] - float(circle["y"]))
                - robot_eval_radius
                - float(circle["radius"])
            )
        for box in self.scene.get("static_boxes", []):
            dx = max(abs(pose["x_m"] - float(box["x"])) - 0.5 * float(box["length"]), 0.0)
            dy = max(abs(pose["y_m"] - float(box["y"])) - 0.5 * float(box["width"]), 0.0)
            values.append(math.hypot(dx, dy) - robot_eval_radius)
        return min(values) if values else float("inf")

    def publish_command(self, speed=0.0, yaw_rate=0.0):
        message = Twist()
        message.linear.x = float(speed)
        message.angular.z = float(yaw_rate)
        self.command_pub.publish(message)

    def wait_ready(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=25.0)
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=25.0)
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptyService)
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                if get_model_state(self.ego_model, "world").success:
                    break
            except rospy.ServiceException:
                pass
            rospy.rostime.wallsleep(0.02)
        else:
            raise RuntimeError("M2 model did not spawn")
        unpause()
        deadline = time.monotonic() + 35.0
        first_clock = None
        ready = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now > 0.0 and first_clock is None:
                first_clock = now
            clock_ok = first_clock is not None and now > first_clock + 0.02
            actors_ok = all(name in self.model_index for name in self.actor_models)
            ready = (
                clock_ok
                and self.odom is not None
                and self.scan is not None
                and self.model_states is not None
                and self.ego_model in self.model_index
                and actors_ok
                and len(self.runtime.scan_history) == 6
            )
            if ready:
                break
            rospy.rostime.wallsleep(0.02)
        if not ready:
            raise RuntimeError("simulation topics did not become ready")
        if self.client is not None and not self.client.wait_for_server(rospy.Duration(25.0)):
            raise RuntimeError("move_base action server unavailable")

    def warmup(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < self.activation_time_s:
            self.publish_command(0.0, 0.0)
            rate.sleep()
        started = rospy.Time.now().to_sec()
        trace = []
        while not rospy.is_shutdown():
            pose = self.robot_world_pose()
            speed = float(self.odom.twist.twist.linear.x)
            trace.append({"time_s": rospy.Time.now().to_sec(), "x_m": pose["x_m"], "speed_mps": speed})
            if (
                pose["x_m"]
                >= self.policy_start_world_x - self.warmup_gate_tolerance_m
                and speed >= self.warmup_speed_mps - 0.035
            ):
                break
            if rospy.Time.now().to_sec() - started > 12.0:
                raise RuntimeError("common warmup timed out")
            if pose["x_m"] > self.policy_start_world_x + 0.08:
                raise RuntimeError("common warmup overshot policy start")
            self.publish_command(self.warmup_speed_mps, 0.0)
            rate.sleep()
        return {
            "duration_s": rospy.Time.now().to_sec() - started,
            "gate_pose": self.robot_world_pose(),
            "gate_speed_mps": float(self.odom.twist.twist.linear.x),
            "trace_tail": trace[-12:],
        }

    def input_health(self, now_s):
        scan_age = float("inf") if self.scan_received_s is None else max(0.0, now_s - self.scan_received_s)
        odom_age = float("inf") if self.odom_received_s is None else max(0.0, now_s - self.odom_received_s)
        tf_age = float("inf")
        tf_ok = False
        try:
            transform = self.tf_buffer.lookup_transform("odom", "laser_link", rospy.Time(0), rospy.Duration(0.01))
            stamp = transform.header.stamp.to_sec()
            tf_age = max(0.0, now_s - stamp) if stamp > 0.0 else 0.0
            tf_ok = True
            self.tf_success_count += 1
        except Exception:
            self.tf_failure_count += 1
        if math.isfinite(scan_age):
            self.maximum_scan_age_s = max(self.maximum_scan_age_s, scan_age)
        if math.isfinite(odom_age):
            self.maximum_odom_age_s = max(self.maximum_odom_age_s, odom_age)
        if math.isfinite(tf_age):
            self.maximum_tf_age_s = max(self.maximum_tf_age_s, tf_age)
        scan_frame = str(self.scan.header.frame_id).lstrip("/") if self.scan is not None else ""
        odom_frame = str(self.odom.header.frame_id).lstrip("/") if self.odom is not None else ""
        return RuntimeInputHealth(
            scan_age,
            odom_age,
            tf_age,
            scan_frame_ok=scan_frame == "laser_link",
            odometry_frame_ok=odom_frame == "odom",
            transform_ok=tf_ok,
        )

    def send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_world_x - self.spawn_world_x
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

    def common_metrics_step(self, previous_pose, minimum_clearance, path_length):
        pose = self.robot_world_pose()
        step = math.hypot(pose["x_m"] - previous_pose["x_m"], pose["y_m"] - previous_pose["y_m"])
        if step <= 0.25:
            path_length += step
        minimum_clearance = min(minimum_clearance, self.conservative_clearance(pose))
        return pose, minimum_clearance, path_length

    def goal_state(self, pose):
        position_error = math.hypot(pose["x_m"] - self.goal_world_x, pose["y_m"])
        yaw_error = abs(execution_core.angle_delta(0.0, pose["yaw_rad"]))
        return position_error <= self.goal_tolerance_m and yaw_error <= 0.25, position_error, yaw_error

    def execute_sensor(self):
        started = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_length = 0.0
        minimum_clearance = float("inf")
        active_plan = None
        next_decision = started
        tracking_errors = []
        switches = stops = recoveries = 0
        previous_executed = self.runtime.previous_action
        previous_speed = float(self.odom.twist.twist.linear.x)
        previous_acceleration = 0.0
        max_acceleration = max_jerk = max_yaw_rate = 0.0
        goal_reached = timeout = False
        rate = rospy.Rate(20)
        self.collecting = True
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            pose, minimum_clearance, path_length = self.common_metrics_step(
                previous_pose, minimum_clearance, path_length
            )
            previous_pose = pose
            goal_reached, _, _ = self.goal_state(pose)
            if goal_reached:
                break
            if now - started >= self.timeout_s:
                timeout = True
                break
            policy_pose = self.policy_pose()
            speed = float(self.odom.twist.twist.linear.x)
            yaw_rate = float(self.odom.twist.twist.angular.z)
            health = self.input_health(now)
            if active_plan is None or now + 1.0e-9 >= next_decision:
                suggestion = self.runtime.suggest(
                    policy_pose, speed, yaw_rate, now - started, health
                )
                activation_suggestion = suggestion
                if self.method == "pct_shadow":
                    activation_suggestion = dict(suggestion)
                    activation_suggestion["suggested_action"] = suggestion["analytic_action"]
                validation = self.runtime.revalidate(
                    activation_suggestion, policy_pose, speed, yaw_rate, now - started, self.input_health(rospy.Time.now().to_sec())
                )
                action = int(validation["validated_action"])
                previews = validation["previews"] or suggestion["previews"]
                if previews is None:
                    previews = self.runtime.build(policy_pose, speed, yaw_rate, now - started)[1]
                    action = 0
                active_plan = {
                    "action": action,
                    "points": [
                        {"x_m": float(state[0]), "y_m": float(state[1]), "longitudinal_velocity_mps": float(state[3])}
                        for state in previews[action]
                    ],
                }
                commit = self.runtime.commit_executed(action)
                if action != previous_executed:
                    switches += 1
                if action == 0:
                    stops += 1
                if previous_executed == 0 and action != 0:
                    recoveries += 1
                previous_executed = action
                observation = validation["observation"] or suggestion["observation"]
                safety_diagnostics = self.runtime.candidate_safety_diagnostics(
                    policy_pose, speed, yaw_rate
                )
                self.decision_trace.append(
                    {
                        "time_s": now,
                        "suggested_action": suggestion["suggested_action"],
                        "shadow_control_action": suggestion["analytic_action"] if self.method == "pct_shadow" else None,
                        "validated_action": action,
                        "executed_action": commit["executed_action"],
                        "analytic_action": suggestion["analytic_action"],
                        "learned_action": suggestion["learned_action"],
                        "learned_intervened": suggestion["intervened"],
                        "residual_margin": suggestion["residual_margin"],
                        "inference_latency_ms": suggestion["latency_ms"],
                        "suggestion_fallback_reason": suggestion["fallback_reason"],
                        "activation_fallback_reason": validation["activation_fallback_reason"],
                        "input_health_failures": sorted(set(suggestion["input_health_failures"] + validation["input_health_failures"])),
                        "safe_action_count": int(np.asarray(observation["action_mask"]).sum()) if observation is not None else 1,
                        "action_mask": np.asarray(observation["action_mask"], dtype=np.int8).tolist() if observation is not None else [1, 0, 0, 0, 0, 0, 0, 0],
                        "analytic_prior_scores": np.asarray(observation["analytic_prior"], dtype=np.float64).tolist() if observation is not None else None,
                        "ego_pose": pose,
                        "actor_poses": self.actor_world_poses(),
                        "conservative_clearance_m": self.conservative_clearance(pose),
                        "candidate_safety_diagnostics": safety_diagnostics,
                        "directional_tracker_audit": self.runtime.tracker_audit(),
                        "execution_audit": self.runtime.execution_audit(),
                        "selected_preview_after_one_period": active_plan["points"][1],
                        "selected_preview_terminal": active_plan["points"][-1],
                    }
                )
                next_decision += 0.25
                if next_decision <= now:
                    next_decision = now + 0.25
            command = execution_core.pure_pursuit_command(
                active_plan["points"], policy_pose[0], policy_pose[1], policy_pose[2], lookahead_m=self.lookahead_m
            )
            proposed_speed = command["linear_x_mps"]
            curvature = (
                command["angular_z_radps"] / proposed_speed
                if abs(proposed_speed) > 1.0e-9
                else 0.0
            )
            command["linear_x_mps"], command["angular_z_radps"] = self.runtime.terminal_command(
                active_plan["action"], policy_pose, proposed_speed,
                proposed_speed * curvature,
            )
            self.publish_command(command["linear_x_mps"], command["angular_z_radps"])
            tracking_errors.append(command["tracking_error_m"])
            acceleration = (speed - previous_speed) / 0.05
            jerk = (acceleration - previous_acceleration) / 0.05
            max_acceleration = max(max_acceleration, abs(acceleration))
            max_jerk = max(max_jerk, abs(jerk))
            max_yaw_rate = max(max_yaw_rate, abs(yaw_rate))
            previous_speed = speed
            previous_acceleration = acceleration
            self.control_trace.append(
                {
                    "time_s": now,
                    "ego_pose": pose,
                    "actor_poses": self.actor_world_poses(),
                    "conservative_clearance_m": self.conservative_clearance(pose),
                    "speed_mps": speed,
                    "yaw_rate_radps": yaw_rate,
                }
            )
            rate.sleep()
        elapsed = rospy.Time.now().to_sec() - started
        self.collecting = False
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        return {
            "goal_reached": bool(goal_reached),
            "timeout": bool(timeout),
            "elapsed_s": elapsed,
            "restricted_elapsed_s": min(elapsed, self.timeout_s),
            "path_length_m": path_length,
            "minimum_conservative_clearance_m": finite_or_none(minimum_clearance),
            "contact_state_count": self.contact_state_count,
            "decision_count": len(self.decision_trace),
            "candidate_switch_count": switches,
            "certified_stop_count": stops,
            "stop_recovery_count": recoveries,
            "tracking_error_mean_m": float(np.mean(tracking_errors)) if tracking_errors else None,
            "tracking_error_max_m": max(tracking_errors) if tracking_errors else None,
            "maximum_absolute_acceleration_mps2": max_acceleration,
            "maximum_absolute_jerk_mps3": max_jerk,
            "maximum_absolute_yaw_rate_radps": max_yaw_rate,
            "mask_violation_count": sum(int(row["validated_action"] != row["executed_action"]) for row in self.decision_trace),
        }

    def execute_baseline(self):
        self.send_goal()
        started = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_length = 0.0
        minimum_clearance = float("inf")
        previous_speed = float(self.odom.twist.twist.linear.x)
        previous_acceleration = 0.0
        max_acceleration = max_jerk = max_yaw_rate = 0.0
        goal_reached = timeout = False
        rate = rospy.Rate(20)
        self.collecting = True
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            pose, minimum_clearance, path_length = self.common_metrics_step(
                previous_pose, minimum_clearance, path_length
            )
            previous_pose = pose
            geometric_goal, _, _ = self.goal_state(pose)
            goal_reached = self.client.get_state() == GoalStatus.SUCCEEDED and geometric_goal
            if goal_reached:
                break
            if now - started >= self.timeout_s:
                timeout = True
                break
            self.input_health(now)
            speed = float(self.odom.twist.twist.linear.x)
            yaw_rate = float(self.odom.twist.twist.angular.z)
            acceleration = (speed - previous_speed) / 0.05
            jerk = (acceleration - previous_acceleration) / 0.05
            max_acceleration = max(max_acceleration, abs(acceleration))
            max_jerk = max(max_jerk, abs(jerk))
            max_yaw_rate = max(max_yaw_rate, abs(yaw_rate))
            previous_speed = speed
            previous_acceleration = acceleration
            self.control_trace.append(
                {
                    "time_s": now,
                    "ego_pose": pose,
                    "actor_poses": self.actor_world_poses(),
                    "conservative_clearance_m": self.conservative_clearance(pose),
                    "speed_mps": speed,
                    "yaw_rate_radps": yaw_rate,
                }
            )
            rate.sleep()
        elapsed = rospy.Time.now().to_sec() - started
        action_state = self.client.get_state()
        if not goal_reached:
            self.client.cancel_goal()
        self.collecting = False
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        return {
            "goal_reached": bool(goal_reached),
            "timeout": bool(timeout),
            "elapsed_s": elapsed,
            "restricted_elapsed_s": min(elapsed, self.timeout_s),
            "path_length_m": path_length,
            "minimum_conservative_clearance_m": finite_or_none(minimum_clearance),
            "contact_state_count": self.contact_state_count,
            "move_base_state": action_state,
            "move_base_state_name": STATUS_NAMES.get(action_state, "UNKNOWN"),
            "local_plan_count": self.local_plan_count,
            "maximum_absolute_acceleration_mps2": max_acceleration,
            "maximum_absolute_jerk_mps3": max_jerk,
            "maximum_absolute_yaw_rate_radps": max_yaw_rate,
            "mask_violation_count": 0,
        }

    @staticmethod
    def frequency(stamps):
        intervals = [right - left for left, right in zip(stamps[:-1], stamps[1:]) if right > left]
        return 1.0 / float(np.mean(intervals)) if intervals else None

    def run(self):
        self.wait_ready()
        warmup = self.warmup()
        self.scan_transform_success_count = 0
        self.scan_transform_failure_count = 0
        result = self.execute_baseline() if self.method in ("actual_teb", "dwa") else self.execute_sensor()
        final_pose = self.robot_world_pose()
        _, position_error, yaw_error = self.goal_state(final_pose)
        latencies = [float(row["inference_latency_ms"]) for row in self.decision_trace]
        tf_total = self.tf_success_count + self.tf_failure_count
        result.update(
            {
                "final_pose": final_pose,
                "final_position_error_m": position_error,
                "final_yaw_error_rad": yaw_error,
                "minimum_scan_range_m": finite_or_none(self.minimum_scan_m),
                "command_count": self.command_count,
                "scan_frequency_hz": self.frequency(self.scan_stamps),
                "odometry_frequency_hz": self.frequency(self.odom_stamps),
                "command_frequency_hz": self.frequency(self.command_stamps),
                "maximum_scan_age_s": self.maximum_scan_age_s,
                "maximum_odometry_age_s": self.maximum_odom_age_s,
                "maximum_tf_age_s": self.maximum_tf_age_s,
                "tf_success_rate": self.tf_success_count / tf_total if tf_total else 0.0,
                "mean_inference_latency_ms": float(np.mean(latencies)) if latencies else None,
                "p95_inference_latency_ms": float(np.quantile(latencies, 0.95)) if latencies else None,
                "p99_inference_latency_ms": float(np.quantile(latencies, 0.99)) if latencies else None,
                "inference_deadline_miss_count": sum(value > 20.0 for value in latencies),
                "scan_timestamp_transform_success_rate": self.scan_transform_success_count / max(1, self.scan_transform_success_count + self.scan_transform_failure_count),
                "scan_timestamp_transform_failure_count": self.scan_transform_failure_count,
            }
        )
        gate_pose = warmup["gate_pose"]
        gate_valid = (
            abs(gate_pose["x_m"] - self.policy_start_world_x) <= self.warmup_gate_tolerance_m
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(warmup["gate_speed_mps"] - self.warmup_speed_mps) <= 0.04
        )
        message_contract_pass = (
            result["maximum_scan_age_s"] <= 0.20
            and result["maximum_odometry_age_s"] <= 0.20
            and result["maximum_tf_age_s"] <= 0.20
            and result["tf_success_rate"] >= 0.999
            and result["scan_timestamp_transform_success_rate"] >= 0.999
        )
        execution_integrity_pass = (
            gate_valid
            and message_contract_pass
            and result["contact_state_count"] == 0
            and result["mask_violation_count"] == 0
            and result["command_count"] > 0
            and (result["minimum_conservative_clearance_m"] is None or result["minimum_conservative_clearance_m"] >= 0.0)
            and (self.method not in ("actual_teb", "dwa") or result["local_plan_count"] > 0)
            and (self.method in ("actual_teb", "dwa") or result["decision_count"] > 0)
        )
        report = {
            "schema_version": "1.0",
            "stage": self.stage,
            "simulation_only": True,
            "split": self.split,
            "method": self.method,
            "scene_id": self.scene_id,
            "seed": self.seed,
            "scene_manifest_path": self.scene_manifest_path,
            "scene_manifest_sha256": sha256(self.scene_manifest_path),
            "actor_checkpoint_path": self.actor_checkpoint_path or None,
            "actor_checkpoint_sha256": self.actor_checkpoint_sha256,
            "shared_sensor_policy": "LaserScan+Odometry+TF; no Gazebo actor state for planning",
            "traditional_planner_oracle_actor_track": False,
            "warmup": warmup,
            "gate_valid": bool(gate_valid),
            "message_contract_pass": bool(message_contract_pass),
            "decision_trace": self.decision_trace,
            "control_trace": self.control_trace,
            "directional_tracker_audit": self.runtime.tracker_audit(),
            "execution_audit": self.runtime.execution_audit(),
            "result": result,
            "execution_integrity_pass": bool(execution_integrity_pass),
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
            "benchmark method=%s goal=%s integrity=%s time=%.3f clearance=%s",
            self.method,
            result["goal_reached"],
            execution_integrity_pass,
            result["restricted_elapsed_s"],
            result["minimum_conservative_clearance_m"],
        )
        return report


def main():
    rospy.init_node("pct_teb_dwa_benchmark_episode")
    try:
        BenchmarkEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr("benchmark episode failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
