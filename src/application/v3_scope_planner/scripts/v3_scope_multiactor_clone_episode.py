#!/usr/bin/env python3
"""One bounded V3-13 multi-actor cloned-intervention episode."""

import collections
import hashlib
import math
import os
import sys
import time

import rospy
import yaml
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty as EmptyService

from v3_scope_planner import execution_core
from v3_scope_planner import multi_actor_core


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class CloneEpisode:
    def __init__(self):
        self.report_path = rospy.get_param("~report_path")
        self.scene_spec_path = rospy.get_param("~scene_spec_path")
        self.actor_specs_path = rospy.get_param("~actor_specs_path")
        self.planner_contract_path = rospy.get_param(
            "~planner_contract_path"
        )
        self.seed = int(rospy.get_param("~seed"))
        self.scene_id = rospy.get_param("~scene_id")
        self.block_index = int(rospy.get_param("~block_index"))
        self.forced_candidate_id = rospy.get_param(
            "~forced_candidate_id"
        )
        self.behavior_propensity = float(
            rospy.get_param("~behavior_propensity")
        )
        self.spawn_world_x = float(
            rospy.get_param("~spawn_world_x", -4.2)
        )
        self.gate_world_x = float(
            rospy.get_param("~gate_world_x", -2.6)
        )
        self.activation_time_s = float(
            rospy.get_param("~activation_time_s", 5.0)
        )
        self.warmup_timeout_s = float(
            rospy.get_param("~warmup_timeout_s", 12.0)
        )
        self.warmup_speed_mps = float(
            rospy.get_param("~warmup_speed_mps", 0.8)
        )
        self.horizon_s = float(rospy.get_param("~horizon_s", 7.0))
        self.lookahead_m = float(rospy.get_param("~lookahead_m", 0.70))
        self.combined_radius_m = float(
            rospy.get_param("~combined_radius_m", 1.15)
        )
        self.ego_model = rospy.get_param(
            "~ego_model_name", "autolabor_m2"
        )
        with open(
            self.actor_specs_path, "r", encoding="utf-8"
        ) as stream:
            actor_document = yaml.safe_load(stream)
        with open(
            self.scene_spec_path, "r", encoding="utf-8"
        ) as stream:
            self.scene = yaml.safe_load(stream)
        with open(
            self.planner_contract_path, "r", encoding="utf-8"
        ) as stream:
            self.contract = yaml.safe_load(stream)
        self.actor_specs = actor_document["actors"]
        if len(self.actor_specs) < 2:
            raise RuntimeError("at least two actors are required")
        if any(
            "evaluator_policy" in {
                key: value
                for key, value in spec.items()
                if key not in ("evaluator_policy",)
            }
            for spec in self.actor_specs
        ):
            raise RuntimeError("hidden evaluator policy firewall failed")

        self.odom = None
        self.scan = None
        self.model_states = None
        self.model_index = {}
        self.histories = {
            int(spec["track_id"]): collections.deque(maxlen=300)
            for spec in self.actor_specs
        }
        self.contact_count = 0
        self.minimum_scan_m = float("inf")
        self.command_count = 0
        self.maximum_command_gap_s = 0.0
        self.last_command_time_s = None
        self.collecting = False
        self.command_pub = rospy.Publisher(
            "/cmd_vel_navigation", Twist, queue_size=1
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

    def _odom(self, message):
        self.odom = message

    def _models(self, message):
        self.model_states = message
        self.model_index = {
            name: index for index, name in enumerate(message.name)
        }
        now_s = rospy.Time.now().to_sec()
        for spec in self.actor_specs:
            model_name = spec["model_name"]
            if model_name not in self.model_index:
                continue
            pose = message.pose[self.model_index[model_name]]
            history = self.histories[int(spec["track_id"])]
            if not history or now_s > history[-1][0] + 0.009:
                history.append(
                    (now_s, pose.position.x, pose.position.y)
                )
                while (
                    len(history) > 2
                    and now_s - history[0][0] > 1.0
                ):
                    history.popleft()

    def _contacts(self, message):
        if self.collecting:
            self.contact_count += len(message.states)

    def _scan(self, message):
        self.scan = message
        if not self.collecting:
            return
        finite = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
        ]
        if finite:
            self.minimum_scan_m = min(self.minimum_scan_m, min(finite))

    def _command(self, _message):
        if not self.collecting:
            return
        now_s = rospy.Time.now().to_sec()
        if self.last_command_time_s is not None:
            self.maximum_command_gap_s = max(
                self.maximum_command_gap_s,
                now_s - self.last_command_time_s,
            )
        self.last_command_time_s = now_s
        self.command_count += 1

    @staticmethod
    def interpolate(history, target_s, field_index):
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

    def actor_track(self, spec):
        history = list(self.histories[int(spec["track_id"])])
        if not history:
            raise RuntimeError("actor track unavailable")
        now_s, x_m, y_m = history[-1]
        half_window_s = 0.05
        if now_s - history[0][0] < 2.0 * half_window_s:
            velocity_x = velocity_y = 0.0
        else:
            x_previous = self.interpolate(
                history, now_s - half_window_s, 1
            )
            x_earlier = self.interpolate(
                history, now_s - 2.0 * half_window_s, 1
            )
            y_previous = self.interpolate(
                history, now_s - half_window_s, 2
            )
            y_earlier = self.interpolate(
                history, now_s - 2.0 * half_window_s, 2
            )
            denominator = 2.0 * half_window_s
            velocity_x = (
                3.0 * x_m - 4.0 * x_previous + x_earlier
            ) / denominator
            velocity_y = (
                3.0 * y_m - 4.0 * y_previous + y_earlier
            ) / denominator
        angle_rad = math.radians(float(spec["path_angle_deg"]))
        origin_x = float(spec["path_origin_x_m"])
        origin_y = float(spec["path_origin_y_m"])
        direction_x = math.cos(angle_rad)
        direction_y = math.sin(angle_rad)
        coordinate_m = (
            (x_m - origin_x) * direction_x
            + (y_m - origin_y) * direction_y
        )
        coordinate_speed_mps = (
            velocity_x * direction_x + velocity_y * direction_y
        )
        return {
            "track_id": int(spec["track_id"]),
            "stamp_s": now_s,
            "world_x_m": float(x_m),
            "world_y_m": float(y_m),
            "velocity_x_mps": velocity_x,
            "velocity_y_mps": velocity_y,
            "coordinate_m": coordinate_m,
            "coordinate_speed_mps": max(0.0, coordinate_speed_mps),
            "path_angle_deg": float(spec["path_angle_deg"]),
            "path_origin_x_m": origin_x,
            "path_origin_y_m": origin_y,
            "age_s": max(0.0, rospy.Time.now().to_sec() - now_s),
            "velocity_estimator": "backward_second_order_h0_05_s",
        }

    def actor_tracks(self):
        return [
            self.actor_track(spec)
            for spec in sorted(
                self.actor_specs,
                key=lambda item: int(item["track_id"]),
            )
        ]

    def robot_world_pose(self):
        if self.model_states is None or self.ego_model not in self.model_index:
            raise RuntimeError("ego model state unavailable")
        pose = self.model_states.pose[self.model_index[self.ego_model]]
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": execution_core.quaternion_yaw(pose.orientation),
        }

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
        deadline = time.monotonic() + 25.0
        spawned = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                spawned = bool(
                    get_model_state(self.ego_model, "world").success
                )
            except rospy.ServiceException:
                spawned = False
            if spawned:
                break
            rospy.rostime.wallsleep(0.02)
        if not spawned:
            raise RuntimeError("M2 model did not spawn while Gazebo paused")
        unpause()
        deadline = time.monotonic() + 35.0
        first_clock_s = None
        clock_progress = False
        ready = False
        actor_names = {spec["model_name"] for spec in self.actor_specs}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now_s = rospy.Time.now().to_sec()
            if now_s > 0.0:
                if first_clock_s is None:
                    first_clock_s = now_s
                elif now_s > first_clock_s + 0.02:
                    clock_progress = True
            ready = (
                clock_progress
                and self.odom is not None
                and self.scan is not None
                and self.model_states is not None
                and self.ego_model in self.model_index
                and actor_names.issubset(self.model_index)
                and all(
                    len(history) >= 3
                    for history in self.histories.values()
                )
            )
            if ready:
                break
            rospy.rostime.wallsleep(0.02)
        if not ready:
            raise RuntimeError("simulation topics did not become ready")

    def warmup(self):
        rate = rospy.Rate(100)
        while (
            not rospy.is_shutdown()
            and rospy.Time.now().to_sec() < self.activation_time_s
        ):
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
            "trace_digest": multi_actor_core.digest(trace),
            "trace_tail": trace[-12:],
        }

    def candidate(self, gate):
        snapshot = {
            "scene_snapshot_id": "v3_13_c1_{}_{}_{}".format(
                self.scene_id, self.seed, self.block_index
            ),
            "world_model_seq": 1,
            "ego": {
                "x_m": -2.6,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "speed_mps": 0.8,
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
                "forced candidate invalid: {}".format(failures)
            )
        return snapshot, candidate_set, candidate

    def execute(self, candidate):
        points = candidate["points"]
        started_s = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_index = 0
        path_length_m = 0.0
        minimum_clearance_m = float("inf")
        tracking_errors = []
        trace = []
        self.collecting = True
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            elapsed_s = now_s - started_s
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            step_m = math.hypot(
                pose["x_m"] - previous_pose["x_m"],
                pose["y_m"] - previous_pose["y_m"],
            )
            if step_m <= 0.25:
                path_length_m += step_m
            previous_pose = pose
            sample_clearances = [
                math.hypot(
                    pose["x_m"] - track["world_x_m"],
                    pose["y_m"] - track["world_y_m"],
                )
                - self.combined_radius_m
                for track in tracks
            ]
            minimum_clearance_m = min(
                minimum_clearance_m, min(sample_clearances)
            )
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
                    "actor_clearances_m": sample_clearances,
                }
            )
            if elapsed_s >= self.horizon_s:
                break
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
            self.publish_command(
                command["linear_x_mps"],
                command["angular_z_radps"],
            )
            rate.sleep()
        self.collecting = False
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        trace_duration_s = (
            trace[-1]["elapsed_s"] - trace[0]["elapsed_s"]
            if len(trace) >= 2
            else 0.0
        )
        return {
            "completed_horizon": trace_duration_s >= self.horizon_s - 0.15,
            "trace_duration_s": trace_duration_s,
            "trace_sample_count": len(trace),
            "trace": trace,
            "trace_digest": multi_actor_core.digest(trace),
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
            "tracking_error_max_m": (
                max(tracking_errors) if tracking_errors else None
            ),
            "tracking_error_mean_m": (
                sum(tracking_errors) / len(tracking_errors)
                if tracking_errors
                else None
            ),
            "final_pose": self.robot_world_pose(),
        }

    def run(self):
        self.wait_ready()
        gate = self.warmup()
        snapshot, candidate_set, candidate = self.candidate(gate)
        result = self.execute(candidate)
        gate_pose = gate["gate_pose"]
        gate_physical_valid = (
            abs(gate_pose["x_m"] - self.gate_world_x) <= 0.03
            and abs(gate_pose["y_m"]) <= 0.03
            and abs(gate_pose["yaw_rad"]) <= 0.03
            and abs(gate["gate_speed_mps"] - 0.8) <= 0.04
            and all(
                track["age_s"] <= 0.20
                for track in gate["gate_actor_tracks"]
            )
        )
        observable_modes = {}
        for track in gate["gate_actor_tracks"]:
            try:
                observable_modes[track["track_id"]] = (
                    multi_actor_core.infer_observable_mode(
                        track["coordinate_speed_mps"], self.contract
                    )
                )
            except Exception as error:
                observable_modes[track["track_id"]] = "ERROR:{}".format(
                    error
                )
        report = {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": "V3-13-C1",
            "simulation_only": True,
            "method": "cloned_intervention",
            "forced_candidate_id": self.forced_candidate_id,
            "behavior_propensity": self.behavior_propensity,
            "block_index": self.block_index,
            "seed": self.seed,
            "scene_id": self.scene_id,
            "actor_count": len(self.actor_specs),
            "actor_policies_evaluator_only": [
                spec["evaluator_policy"] for spec in self.actor_specs
            ],
            "hidden_policy_runtime_input": False,
            "observable_modes_at_gate": observable_modes,
            "shared_vehicle_model": "autolabor_m2_ackermann",
            "shared_command_boundary": "/cmd_vel_navigation",
            "planner_contract": {
                "path": self.planner_contract_path,
                "sha256": sha256_file(self.planner_contract_path),
            },
            "scene_spec": {
                "path": self.scene_spec_path,
                "sha256": sha256_file(self.scene_spec_path),
            },
            "actor_specs": {
                "path": self.actor_specs_path,
                "sha256": sha256_file(self.actor_specs_path),
            },
            "candidate_snapshot": snapshot,
            "candidate_set_digest": candidate_set[
                "candidate_set_digest"
            ],
            "candidate_digest": candidate["candidate_digest"],
            "warmup": gate,
            "gate_physical_valid": gate_physical_valid,
            "result": result,
            "episode_collection_pass": (
                gate_physical_valid
                and result["completed_horizon"]
                and result["trace_sample_count"] >= 130
                and result["command_count"] > 0
            ),
            "collision_outcome_retained": True,
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
            "V3-13-C1 clone complete block=%d action=%s pass=%s samples=%d",
            self.block_index,
            self.forced_candidate_id,
            report["episode_collection_pass"],
            result["trace_sample_count"],
        )
        return report


def main():
    rospy.init_node("v3_scope_multiactor_clone_episode")
    try:
        CloneEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-C1 clone failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())

