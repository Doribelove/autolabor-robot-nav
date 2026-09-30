#!/usr/bin/env python3
"""One fair, simulation-only V3 versus actual-TebLocalPlannerROS episode."""

import collections
import hashlib
import json
import math
import os
import sys
import time

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point32, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty as EmptyService

from v3_scope_planner import execution_core
from v3_scope_planner import spatial_core_r1


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


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


class FairEpisode:
    def __init__(self):
        self.method = rospy.get_param("~method")
        if self.method not in ("v3_spatial_safe", "actual_teb"):
            raise RuntimeError("unknown method: {}".format(self.method))
        self.report_path = rospy.get_param("~report_path")
        self.scene_id = rospy.get_param("~scene_id")
        self.seed = int(rospy.get_param("~seed"))
        self.actor_policy = rospy.get_param("~actor_policy")
        self.actor_angle_deg = float(rospy.get_param("~actor_angle_deg"))
        self.actor_radius_m = float(rospy.get_param("~actor_radius_m", 0.36))
        self.actor_model = rospy.get_param(
            "~actor_model_name", "v3_11_reactive_actor"
        )
        self.ego_model = rospy.get_param(
            "~ego_model_name", "autolabor_m2"
        )
        self.spawn_world_x = float(rospy.get_param("~spawn_world_x", -4.2))
        self.gate_world_x = float(rospy.get_param("~gate_world_x", -2.6))
        self.goal_world_x = float(rospy.get_param("~goal_world_x", 5.2))
        self.goal_tolerance_m = float(
            rospy.get_param("~goal_tolerance_m", 0.25)
        )
        self.timeout_s = float(rospy.get_param("~timeout_s", 22.0))
        self.warmup_timeout_s = float(
            rospy.get_param("~warmup_timeout_s", 12.0)
        )
        self.warmup_speed_mps = float(
            rospy.get_param("~warmup_speed_mps", 0.8)
        )
        self.lookahead_m = float(rospy.get_param("~lookahead_m", 0.70))
        self.activation_time_s = float(
            rospy.get_param("~activation_time_s", 5.0)
        )
        self.combined_radius_m = float(
            rospy.get_param("~combined_radius_m", 1.15)
        )
        self.planner_contract_path = rospy.get_param(
            "~planner_contract_path"
        )
        self.predictor_model_path = rospy.get_param("~predictor_model_path")

        self.odom = None
        self.scan = None
        self.model_states = None
        self.model_index = {}
        self.actor_history = collections.deque(maxlen=300)
        self.contact_count = 0
        self.cmd_count = 0
        self.last_cmd_time = None
        self.max_cmd_gap_s = 0.0
        self.local_plan_count = 0
        self.minimum_scan_m = float("inf")
        self.collecting = False

        self.command_pub = rospy.Publisher(
            "/cmd_vel_navigation", Twist, queue_size=1
        )
        self.obstacle_pub = rospy.Publisher(
            "/move_base/TebLocalPlannerROS/obstacles",
            ObstacleArrayMsg,
            queue_size=1,
        )
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=50)
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._models, queue_size=20
        )
        rospy.Subscriber(
            "/m2_gazebo/contacts",
            ContactsState,
            self._contacts,
            queue_size=20,
        )
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=5)
        rospy.Subscriber(
            "/cmd_vel_navigation", Twist, self._command, queue_size=50
        )
        rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/local_plan",
            Path,
            self._local_plan,
            queue_size=5,
        )
        self.client = None
        if self.method == "actual_teb":
            self.client = actionlib.SimpleActionClient(
                "/move_base", MoveBaseAction
            )

    def _odom(self, message):
        self.odom = message

    def _models(self, message):
        self.model_states = message
        self.model_index = {
            name: index for index, name in enumerate(message.name)
        }
        if self.actor_model in self.model_index:
            index = self.model_index[self.actor_model]
            pose = message.pose[index]
            now_s = rospy.Time.now().to_sec()
            if (
                not self.actor_history
                or now_s > self.actor_history[-1][0] + 0.009
            ):
                self.actor_history.append(
                    (now_s, pose.position.x, pose.position.y)
                )
                while (
                    len(self.actor_history) > 2
                    and now_s - self.actor_history[0][0] > 1.0
                ):
                    self.actor_history.popleft()

    def _contacts(self, message):
        if self.collecting:
            self.contact_count += len(message.states)

    def _scan(self, message):
        self.scan = message
        if self.collecting:
            finite = [
                float(value)
                for value in message.ranges
                if math.isfinite(float(value))
            ]
            if finite:
                self.minimum_scan_m = min(
                    self.minimum_scan_m, min(finite)
                )

    def _command(self, _message):
        if not self.collecting:
            return
        now_s = rospy.Time.now().to_sec()
        if self.last_cmd_time is not None:
            self.max_cmd_gap_s = max(
                self.max_cmd_gap_s, now_s - self.last_cmd_time
            )
        self.last_cmd_time = now_s
        self.cmd_count += 1

    def _local_plan(self, message):
        if self.collecting and message.poses:
            self.local_plan_count += 1

    def actor_track(self):
        if not self.actor_history:
            raise RuntimeError("actor track unavailable")
        now_s, x_m, y_m = self.actor_history[-1]
        history = list(self.actor_history)

        def interpolate(target_s, field_index):
            if target_s <= history[0][0]:
                return float(history[0][field_index])
            for left, right in zip(history[:-1], history[1:]):
                if left[0] <= target_s <= right[0]:
                    span_s = right[0] - left[0]
                    if span_s <= 0.0:
                        return float(right[field_index])
                    ratio = (target_s - left[0]) / span_s
                    return float(left[field_index]) + ratio * (
                        float(right[field_index])
                        - float(left[field_index])
                    )
            return float(history[-1][field_index])

        derivative_half_window_s = 0.05
        if now_s - history[0][0] < 2.0 * derivative_half_window_s:
            velocity_x = velocity_y = 0.0
        else:
            x_previous = interpolate(
                now_s - derivative_half_window_s, 1
            )
            x_earlier = interpolate(
                now_s - 2.0 * derivative_half_window_s, 1
            )
            y_previous = interpolate(
                now_s - derivative_half_window_s, 2
            )
            y_earlier = interpolate(
                now_s - 2.0 * derivative_half_window_s, 2
            )
            denominator = 2.0 * derivative_half_window_s
            velocity_x = (
                3.0 * x_m - 4.0 * x_previous + x_earlier
            ) / denominator
            velocity_y = (
                3.0 * y_m - 4.0 * y_previous + y_earlier
            ) / denominator
        angle_rad = math.radians(self.actor_angle_deg)
        coordinate_m = x_m * math.cos(angle_rad) + y_m * math.sin(angle_rad)
        coordinate_speed_mps = (
            velocity_x * math.cos(angle_rad)
            + velocity_y * math.sin(angle_rad)
        )
        return {
            "stamp_s": now_s,
            "world_x_m": x_m,
            "world_y_m": y_m,
            "velocity_x_mps": velocity_x,
            "velocity_y_mps": velocity_y,
            "coordinate_m": coordinate_m,
            "coordinate_speed_mps": max(0.0, coordinate_speed_mps),
            "age_s": max(0.0, rospy.Time.now().to_sec() - now_s),
            "velocity_estimator": "backward_second_order_h0_05_s",
        }

    def robot_world_pose(self):
        if self.model_states is None or self.ego_model not in self.model_index:
            raise RuntimeError("ego model state unavailable")
        pose = self.model_states.pose[self.model_index[self.ego_model]]
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": execution_core.quaternion_yaw(pose.orientation),
        }

    def publish_track_for_teb(self, track):
        message = ObstacleArrayMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        obstacle = ObstacleMsg()
        obstacle.id = 1
        point = Point32()
        point.x = track["world_x_m"] - self.spawn_world_x
        point.y = track["world_y_m"]
        point.z = 0.0
        obstacle.polygon.points = [point]
        obstacle.radius = self.actor_radius_m
        obstacle.velocities.twist.linear.x = track["velocity_x_mps"]
        obstacle.velocities.twist.linear.y = track["velocity_y_mps"]
        yaw_rad = math.radians(self.actor_angle_deg)
        obstacle.orientation.z = math.sin(0.5 * yaw_rad)
        obstacle.orientation.w = math.cos(0.5 * yaw_rad)
        message.obstacles = [obstacle]
        self.obstacle_pub.publish(message)

    def publish_command(self, speed_mps=0.0, yaw_rate_radps=0.0):
        message = Twist()
        message.linear.x = float(speed_mps)
        message.angular.z = float(yaw_rate_radps)
        self.command_pub.publish(message)

    def wait_ready(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=25.0)
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=25.0)
        get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        unpause = rospy.ServiceProxy(
            "/gazebo/unpause_physics", EmptyService
        )
        spawn_deadline = time.monotonic() + 25.0
        model_spawned = False
        while not rospy.is_shutdown() and time.monotonic() < spawn_deadline:
            try:
                model_spawned = bool(
                    get_model_state(self.ego_model, "world").success
                )
            except rospy.ServiceException:
                model_spawned = False
            if model_spawned:
                break
            rospy.rostime.wallsleep(0.02)
        if not model_spawned:
            raise RuntimeError("M2 model did not spawn while Gazebo was paused")
        unpause()
        deadline = time.monotonic() + 35.0
        first_positive_clock_s = None
        positive_clock_progress = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now_s = rospy.Time.now().to_sec()
            if now_s > 0.0:
                if first_positive_clock_s is None:
                    first_positive_clock_s = now_s
                elif now_s > first_positive_clock_s + 0.02:
                    positive_clock_progress = True
            ready = (
                positive_clock_progress
                and self.odom is not None
                and self.scan is not None
                and self.model_states is not None
                and self.ego_model in self.model_index
                and self.actor_model in self.model_index
                and len(self.actor_history) >= 3
            )
            if ready:
                break
            rospy.rostime.wallsleep(0.02)
        if not ready:
            raise RuntimeError("simulation topics did not become ready")
        if self.client is not None and not self.client.wait_for_server(
            rospy.Duration(25.0)
        ):
            raise RuntimeError("move_base action server unavailable")

    def warmup(self):
        activation_rate = rospy.Rate(100)
        while (
            not rospy.is_shutdown()
            and rospy.Time.now().to_sec() < self.activation_time_s
        ):
            self.publish_track_for_teb(self.actor_track())
            self.publish_command(0.0, 0.0)
            activation_rate.sleep()
        activation_lateness_s = max(
            0.0, rospy.Time.now().to_sec() - self.activation_time_s
        )
        if activation_lateness_s > 0.075:
            raise RuntimeError(
                "shared activation barrier missed by {:.3f}s".format(
                    activation_lateness_s
                )
            )
        started_s = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        trace = []
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            pose = self.robot_world_pose()
            track = self.actor_track()
            self.publish_track_for_teb(track)
            speed_mps = float(self.odom.twist.twist.linear.x)
            trace.append(
                {
                    "time_s": now_s,
                    "ego_world_x_m": pose["x_m"],
                    "ego_speed_mps": speed_mps,
                    "actor_coordinate_m": track["coordinate_m"],
                    "actor_speed_mps": track["coordinate_speed_mps"],
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
            "gate_actor_track": self.actor_track(),
            "trace_digest": spatial_core_r1.digest(trace),
            "trace_tail": trace[-12:],
        }

    def plan_v3(self, gate):
        with open(self.planner_contract_path, "r", encoding="utf-8") as stream:
            contract = yaml.safe_load(stream)
        with open(self.predictor_model_path, "r", encoding="utf-8") as stream:
            model = yaml.safe_load(stream)
        actor = gate["gate_actor_track"]
        snapshot = {
            "scene_snapshot_id": "v3_11_{}_{}_{}".format(
                self.scene_id, self.seed, self.actor_policy
            ),
            "world_model_seq": 1,
            "snapshot_age_s": actor["age_s"],
            "predictor_latency_ms": 0.0,
            "verifier_latency_ms": 0.0,
            "ego": {
                # The common handoff gate is the canonical frozen V3 state.
                "x_m": -2.6,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "speed_mps": 0.8,
            },
            "actor": {
                "coordinate_m": actor["coordinate_m"],
                "speed_mps": actor["coordinate_speed_mps"],
                "path_angle_deg": self.actor_angle_deg,
                "monotone_coordinate_certified": True,
            },
            "health": {
                "localization_ok": True,
                "transform_ok": True,
                "actor_track_ok": actor["age_s"] <= 0.2,
                "command_chain_ok": True,
            },
        }
        started_ns = time.perf_counter_ns()
        result = spatial_core_r1.plan(snapshot, model, contract)
        latency_ms = (time.perf_counter_ns() - started_ns) / 1.0e6
        selected_id = result["decision"]["selected_candidate_id"]
        selected = next(
            item
            for item in result["candidate_set"]["candidates"]
            if item["candidate_id"] == selected_id
        )
        evaluation = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"] == selected_id
        )
        return {
            "snapshot": snapshot,
            "latency_ms": latency_ms,
            "result_digest": spatial_core_r1.digest(result),
            "selected_candidate_id": selected_id,
            "selected_candidate": selected,
            "selected_evaluation": evaluation,
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

    def execute(self, gate, v3_plan=None):
        if self.method == "actual_teb":
            self.send_teb_goal()
            points = None
        else:
            points = v3_plan["selected_candidate"]["points"]

        started_s = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_length_m = 0.0
        minimum_clearance_m = float("inf")
        tracking_errors = []
        path_index = 0
        goal_reached = False
        timeout = False
        rate = rospy.Rate(20)
        self.collecting = True
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            pose = self.robot_world_pose()
            track = self.actor_track()
            self.publish_track_for_teb(track)
            step = math.hypot(
                pose["x_m"] - previous_pose["x_m"],
                pose["y_m"] - previous_pose["y_m"],
            )
            if step <= 0.25:
                path_length_m += step
            previous_pose = pose
            minimum_clearance_m = min(
                minimum_clearance_m,
                math.hypot(
                    pose["x_m"] - track["world_x_m"],
                    pose["y_m"] - track["world_y_m"],
                )
                - self.combined_radius_m,
            )

            position_error = math.hypot(
                pose["x_m"] - self.goal_world_x, pose["y_m"]
            )
            yaw_error = abs(
                execution_core.angle_delta(0.0, pose["yaw_rad"])
            )
            if self.method == "actual_teb":
                action_state = self.client.get_state()
                goal_reached = (
                    action_state == GoalStatus.SUCCEEDED
                    and position_error <= self.goal_tolerance_m + 0.05
                )
            else:
                command = execution_core.pure_pursuit_command(
                    points,
                    pose["x_m"],
                    pose["y_m"],
                    pose["yaw_rad"],
                    minimum_index=path_index,
                    lookahead_m=self.lookahead_m,
                )
                path_index = max(path_index, command["nearest_index"])
                tracking_errors.append(command["tracking_error_m"])
                goal_reached = (
                    position_error <= self.goal_tolerance_m
                    and yaw_error <= 0.22
                )
                if not goal_reached:
                    self.publish_command(
                        command["linear_x_mps"],
                        command["angular_z_radps"],
                    )
            if goal_reached:
                break
            if now_s - started_s >= self.timeout_s:
                timeout = True
                break
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
            "command_count": self.cmd_count,
            "maximum_command_gap_s": self.max_cmd_gap_s,
            "local_plan_count": self.local_plan_count,
            "action_state": action_state,
            "action_state_name": STATUS_NAMES.get(action_state, "NOT_USED"),
            "final_pose": final_pose,
            "final_position_error_m": math.hypot(
                final_pose["x_m"] - self.goal_world_x,
                final_pose["y_m"],
            ),
            "final_yaw_error_rad": abs(
                execution_core.angle_delta(0.0, final_pose["yaw_rad"])
            ),
            "tracking_error_max_m": (
                max(tracking_errors) if tracking_errors else None
            ),
            "tracking_error_mean_m": (
                sum(tracking_errors) / len(tracking_errors)
                if tracking_errors
                else None
            ),
        }

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        with open(self.planner_contract_path, "r", encoding="utf-8") as stream:
            mode_contract = yaml.safe_load(stream)
        try:
            gate["inferred_actor_mode"] = spatial_core_r1.infer_mode(
                gate["gate_actor_track"]["coordinate_speed_mps"],
                mode_contract,
            )
            gate["actor_mode_inference_error"] = ""
        except Exception as error:
            gate["inferred_actor_mode"] = ""
            gate["actor_mode_inference_error"] = "{}:{}".format(
                type(error).__name__, error
            )
        gate["actor_mode_matches_evaluator_policy"] = (
            gate["inferred_actor_mode"] == self.actor_policy
        )
        v3_plan = self.plan_v3(gate) if self.method == "v3_spatial_safe" else None
        result = self.execute(gate, v3_plan)
        gate_pose = gate["gate_pose"]
        gate_physical_valid = (
            abs(gate_pose["x_m"] - self.gate_world_x) <= 0.03
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(gate["gate_speed_mps"] - 0.8) <= 0.04
        )
        report = {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": "V3-11",
            "simulation_only": True,
            "shared_vehicle_model": "autolabor_m2_ackermann",
            "shared_command_boundary": "/cmd_vel_navigation",
            "method": self.method,
            "scene_id": self.scene_id,
            "seed": self.seed,
            "actor_policy_evaluator_only": self.actor_policy,
            "actor_angle_deg": self.actor_angle_deg,
            "shared_oracle_actor_track": True,
            "planner_contract": {
                "path": self.planner_contract_path,
                "sha256": sha256_file(self.planner_contract_path),
            },
            "predictor_model": {
                "path": self.predictor_model_path,
                "sha256": sha256_file(self.predictor_model_path),
            },
            "warmup": gate,
            "gate_physical_valid": gate_physical_valid,
            "v3_plan": v3_plan,
            "result": result,
            "episode_pass": (
                gate_physical_valid
                and gate["actor_mode_matches_evaluator_policy"]
                and result["goal_reached"]
                and result["contact_state_count"] == 0
                and result["minimum_conservative_clearance_m"] >= 0.0
                and result["command_count"] > 0
                and (
                    self.method != "actual_teb"
                    or result["local_plan_count"] > 0
                )
                and (
                    self.method != "v3_spatial_safe"
                    or (
                        v3_plan["selected_candidate_id"]
                        == "spatial_left_v120"
                        and v3_plan["selected_evaluation"]["accepted"] is True
                    )
                )
            ),
            "training_steps": 0,
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
            "V3-11 fair episode complete method=%s scene=%s pass=%s time=%.3f",
            self.method,
            self.scene_id,
            report["episode_pass"],
            result["restricted_elapsed_s"],
        )
        return report


def main():
    rospy.init_node("v3_scope_actual_teb_fair_episode")
    try:
        FairEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-11 fair episode failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
