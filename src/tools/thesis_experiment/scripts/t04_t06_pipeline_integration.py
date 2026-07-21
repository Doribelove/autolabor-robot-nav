#!/usr/bin/env python3
"""Gazebo-only fixed-policy acceptance for the T04--T06 pipeline."""

import json
import math
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.episode_state_machine import EpisodeStateMachine
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.reward_cost import (
    FeedbackSample,
    RewardWeights,
    WindowEvents,
    calculate_reward_and_cost,
)
from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.state_builder import HistoryWindow, StateBuilder
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from teb_rl_tuner.timing import ActivationTracker
from thesis_experiment.run_artifacts import (
    RunValidator,
    sha256_file,
    write_checksums,
    write_episode_csv,
    write_run_manifest,
    write_step_csv,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
THETA_JSON_KWARGS = {"sort_keys": True, "separators": (",", ":")}


def _stamp(message):
    value = message.header.stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


def _yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


def _git_output(*args):
    return subprocess.check_output(
        ["git"] + list(args), cwd=str(WORKSPACE), text=True, stderr=subprocess.DEVNULL
    ).strip()


class PipelineAcceptance:
    def __init__(self):
        # Stop before the world's fixed box: this acceptance tests the data and
        # parameter pipeline, while obstacle detours remain covered by T02.
        self.goal_xy = (1.5, 0.0)
        self.odom = None
        self.scan = None
        self.global_plan = None
        self.local_plan = None
        self.local_plan_generation = 0
        self.path_length = 0.0
        self.previous_xy = None
        self.samples = []
        self.history = HistoryWindow(k=4)
        self.minimum_clearance = float("inf")
        self.planner_cycles = 0
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / "artifacts/t06/t04_t06_pipeline_run")
        ))
        self.seed = int(rospy.get_param("~seed", 42))
        self.timeout_s = float(rospy.get_param("~goal_timeout_s", 40.0))
        self.pipeline_config_path = Path(rospy.get_param(
            "~pipeline_config",
            str(WORKSPACE / "src/application/teb_rl_tuner/config/t04_t06_simulation_validation.yaml"),
        ))
        self.safety_config_path = Path(rospy.get_param(
            "~safety_config",
            str(WORKSPACE / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml"),
        ))
        self.world_path = WORKSPACE / "src/simulation/m2_gazebo/worlds/obstacle_test.world"
        self.episode_schema = WORKSPACE / "docs/thesis_experiment/schemas/episode_metrics_schema.csv"
        self.step_schema = WORKSPACE / "docs/thesis_experiment/schemas/step_metrics_schema.csv"
        self.contract_path = WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"
        self._load_configs()
        self.state_builder = StateBuilder(
            feature_order=(
                "footprint_clearance", "obstacle_density", "approximate_ttc",
                "goal_distance", "goal_bearing_sin", "goal_bearing_cos",
                "path_cross_track_error", "path_heading_error", "linear_velocity",
                "angular_velocity", "linear_acceleration", "planner_valid",
                "sensor_valid", "tf_valid", "localization_valid", "interface_valid",
            ) + tuple("theta_{}".format(name) for name in EXPECTED_THETA_ORDER),
            required_streams=("scan", "odom", "local_plan"),
            sector_count=36,
            max_sync_skew_s=0.6,
        )
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=10)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", NavPath,
                         self._global_plan_cb, queue_size=5)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", NavPath,
                         self._local_plan_cb, queue_size=20)
        self.cmd_stop = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def _load_configs(self):
        self.pipeline_config = yaml.safe_load(self.pipeline_config_path.read_text(encoding="utf-8"))
        self.safety_config = yaml.safe_load(self.safety_config_path.read_text(encoding="utf-8"))
        if (self.pipeline_config.get("scope") != "t02_gazebo_only" or
                self.safety_config.get("scope") != "t02_gazebo_only" or
                self.safety_config.get("real_vehicle_use_forbidden") is not True):
            raise RuntimeError("pipeline acceptance requires explicit simulation-only configs")

    def _odom_cb(self, message):
        if self.odom is not None:
            before = self.odom.pose.pose.position
            after = message.pose.pose.position
            delta = math.hypot(after.x - before.x, after.y - before.y)
            if delta < 0.25:
                self.path_length += delta
        self.odom = message

    def _scan_cb(self, message):
        self.scan = message

    def _global_plan_cb(self, message):
        if message.poses:
            self.global_plan = message

    def _local_plan_cb(self, message):
        if message.poses:
            self.local_plan = message
            self.local_plan_generation += 1

    def _wait_ready(self):
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise RuntimeError("move_base action server unavailable")
        rospy.wait_for_message("/odom", Odometry, timeout=15.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=15.0)
        deadline = time.monotonic() + 5.0
        while (self.odom is None or self.scan is None) and time.monotonic() < deadline:
            rospy.sleep(0.02)
        if self.odom is None or self.scan is None:
            raise RuntimeError("initial Gazebo observations unavailable")

    def _send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_xy[0]
        goal.target_pose.pose.position.y = self.goal_xy[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

    def _footprint_clearance(self):
        if self.scan is None:
            return float("inf")
        best = float("inf")
        angle = self.scan.angle_min
        for distance in self.scan.ranges:
            if math.isfinite(distance) and self.scan.range_min <= distance <= self.scan.range_max:
                cosine, sine = abs(math.cos(angle)), abs(math.sin(angle))
                x_exit = float("inf") if cosine < 1e-9 else 0.52 / cosine
                y_exit = float("inf") if sine < 1e-9 else 0.35 / sine
                best = min(best, max(0.0, distance - min(x_exit, y_exit)))
            angle += self.scan.angle_increment
        return best if math.isfinite(best) else self.scan.range_max

    def _path_errors(self):
        if self.odom is None or self.global_plan is None or not self.global_plan.poses:
            return 0.0, 0.0
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        nearest = min(
            self.global_plan.poses,
            key=lambda pose: (pose.pose.position.x - x) ** 2 + (pose.pose.position.y - y) ** 2,
        )
        cross = math.hypot(nearest.pose.position.x - x, nearest.pose.position.y - y)
        return cross, abs(_angle_delta(_yaw(self.odom.pose.pose.orientation),
                                       _yaw(nearest.pose.orientation)))

    def _observation(self, theta, previous_velocity, previous_stamp):
        now = rospy.Time.now().to_sec()
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        dx, dy = self.goal_xy[0] - x, self.goal_xy[1] - y
        distance = math.hypot(dx, dy)
        bearing = _angle_delta(math.atan2(dy, dx), yaw)
        velocity = float(self.odom.twist.twist.linear.x)
        angular = float(self.odom.twist.twist.angular.z)
        dt = max(now - previous_stamp, 1e-6)
        acceleration = (velocity - previous_velocity) / dt
        clearance = self._footprint_clearance()
        cross, heading = self._path_errors()
        valid_rays = [value for value in self.scan.ranges if math.isfinite(value)]
        density = (sum(value < 2.0 for value in valid_rays) / float(len(valid_rays))
                   if valid_rays else 0.0)
        ttc = clearance / max(abs(velocity), 0.05)
        features = {
            "footprint_clearance": clearance, "obstacle_density": density,
            "approximate_ttc": ttc, "goal_distance": distance,
            "goal_bearing_sin": math.sin(bearing), "goal_bearing_cos": math.cos(bearing),
            "path_cross_track_error": cross, "path_heading_error": heading,
            "linear_velocity": velocity, "angular_velocity": angular,
            "linear_acceleration": acceleration, "planner_valid": 1.0,
            "sensor_valid": 1.0, "tf_valid": 1.0, "localization_valid": 1.0,
            "interface_valid": 1.0,
        }
        features.update({"theta_{}".format(name): theta[name] for name in EXPECTED_THETA_ORDER})
        frame = self.state_builder.build(
            stamps={"scan": _stamp(self.scan), "odom": _stamp(self.odom),
                    "local_plan": _stamp(self.local_plan)},
            ranges=self.scan.ranges, range_min=self.scan.range_min,
            range_max=self.scan.range_max, features=features,
            validity={"scan": True, "tf": True, "localization": True,
                      "interface": True, "planner": True},
        )
        sample = FeedbackSample(
            stamp=now, goal_distance=distance, path_error=cross, clearance=clearance,
            linear_acceleration=acceleration, angular_acceleration=0.0,
            near_collision=False, fallback_active=False, emergency_active=False,
        )
        return frame, sample, velocity, now, angular, density, ttc, heading

    def _projector(self):
        bounds = self.safety_config["theta_bounds"]
        rates = self.safety_config["max_delta_per_step"]
        return ParameterProjector({
            name: ParameterLimit(bounds[name][0], bounds[name][1], rates[name], True)
            for name in EXPECTED_THETA_ORDER
        }, min_turning_radius=1.2)

    def _safety_filter(self):
        values = self.safety_config["safety_margin"]
        return SafetyMarginFilter(SafetyMarginConfig(
            a_brake_lower=values["a_brake_lower_mps2"],
            tau_total_upper=values["total_latency_upper_s"],
            d_margin=values["distance_margin_m"],
            warning_margin=values["warning_margin_m"],
            emergency_margin=values["emergency_margin_m"],
            hysteresis_margin=values["recovery_margin_m"],
            recovery_healthy_s=values["recovery_healthy_duration_s"],
        ))

    def run(self):
        self._wait_ready()
        self._send_goal()
        deadline = time.monotonic() + 10.0
        while self.local_plan is None and time.monotonic() < deadline:
            rospy.sleep(0.02)
        if self.local_plan is None:
            raise RuntimeError("TEB local plan unavailable after goal")

        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True,
            use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        parameter_client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0
        )
        rospy.on_shutdown(parameter_client.close)
        parameter_client.initialize()
        theta_previous = parameter_client.snapshot

        # T05 dry probes: malformed/dangerous candidates are rejected or filtered
        # before any ROS write, and emergency fallback is a complete atomic vector.
        projector = self._projector()
        fixed_projection = projector.project(theta_previous, theta_previous)
        safety_filter = self._safety_filter()
        health = {name: True for name in
                  ("sensor", "tf", "localization", "parameter_interface", "planner")}
        normal_decision = safety_filter.update(
            self._footprint_clearance(), abs(self.odom.twist.twist.linear.x),
            rospy.Time.now().to_sec(), health,
        )
        fallback = ConservativeFallbackPolicy(self.safety_config["conservative_theta"])
        fallback.confirm_applied_safe(theta_previous)
        fallback_decision = fallback.decide(normal_decision.mode, fixed_projection.projected, True)
        dry_emergency = self._safety_filter().update(0.0, 1.0, 0.0, health)
        dry_fallback = fallback.decide(dry_emergency.mode, fixed_projection.projected, True)
        if len(dry_fallback.theta) != 9 or not dry_fallback.use_fallback:
            raise RuntimeError("T05 emergency dry probe did not select atomic conservative fallback")

        episode = EpisodeStateMachine()
        run_id = "t04_t06_gazebo_seed{}".format(self.seed)
        episode_id = "{}-episode-0001".format(run_id)
        episode.start(episode_id)
        t_decision = rospy.Time.now().to_sec()
        t_request = rospy.Time.now().to_sec()
        transaction = parameter_client.apply(fallback_decision.theta)
        t_ack = rospy.Time.now().to_sec()
        tracker = ActivationTracker(transaction["config_seq"], t_decision, t_request,
                                    t_ack, self.pipeline_config["timing"]["activation_timeout_s"])
        generation = self.local_plan_generation
        while tracker.active is None:
            if self.local_plan_generation > generation:
                generation = self.local_plan_generation
                tracker.observe_local_plan(_stamp(self.local_plan), bool(self.local_plan.poses))
            disposition = episode.observe_activation_deadline(tracker, rospy.Time.now().to_sec())
            if not disposition.transition_stored:
                raise RuntimeError(disposition.transition_drop_reason)
            rospy.sleep(0.01)
        t_active = tracker.active.t_active

        previous_velocity = float(self.odom.twist.twist.linear.x)
        previous_stamp = rospy.Time.now().to_sec()
        observations = []
        window_end_target = t_active + self.pipeline_config["timing"]["reward_window_s"]
        last_aux = None
        last_plan_generation = self.local_plan_generation
        while rospy.Time.now().to_sec() <= window_end_target or len(observations) < 4:
            if self.local_plan_generation > last_plan_generation:
                self.planner_cycles += self.local_plan_generation - last_plan_generation
                last_plan_generation = self.local_plan_generation
            try:
                frame, sample, previous_velocity, previous_stamp, angular, density, ttc, heading = \
                    self._observation(fallback_decision.theta, previous_velocity, previous_stamp)
                if not observations or frame.timestamp > observations[-1].timestamp:
                    self.history.append(frame)
                    observations.append(frame)
                    self.samples.append(sample)
                    last_aux = (angular, density, ttc, heading)
                    self.minimum_clearance = min(self.minimum_clearance, sample.clearance)
            except Exception as exc:
                rospy.logdebug("waiting for synchronized T04 frame: %s", exc)
            rospy.sleep(0.08)
        stacked_state = self.history.stacked()
        if not stacked_state:
            raise RuntimeError("empty K=4 state")
        t_window_end = self.samples[-1].stamp
        rewards = calculate_reward_and_cost(
            self.samples, t_active, t_window_end, [0.0] * 9, WindowEvents(),
            RewardWeights(), warning_distance=1.0,
        )

        finished = self.client.wait_for_result(rospy.Duration(self.timeout_s))
        state = self.client.get_state()
        if not finished or state != GoalStatus.SUCCEEDED:
            self.client.cancel_goal()
            outcome = episode.finish("timeout" if not finished else "planner_failure")
        else:
            outcome = episode.finish("goal")
        self.cmd_stop.publish(Twist())
        restoration = parameter_client.restore()
        parameter_client.close()

        report = self._write_bundle(
            run_id, episode_id, transaction, restoration, theta_previous,
            fixed_projection, normal_decision, fallback_decision, dry_emergency,
            dry_fallback, rewards, outcome, t_decision, t_request, t_ack,
            t_active, t_window_end, last_aux,
        )
        return report

    def _write_bundle(
        self, run_id, episode_id, transaction, restoration, theta_previous,
        projection, safety, fallback, dry_emergency, dry_fallback, rewards,
        outcome, t_decision, t_request, t_ack, t_active, t_window_end, last_aux,
    ):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        angular, density, ttc, heading = last_aux
        theta_json = lambda value: json.dumps(value, **THETA_JSON_KWARGS)
        fields = rewards.step_fields()
        path_errors = [sample.path_error for sample in self.samples]
        velocities = [frame.named_features["linear_velocity"] for frame in self.history.frames]
        step = {
            "run_id": run_id, "episode_id": episode_id, "step_id": 0,
            "config_seq": transaction["config_seq"],
            "t_observation": self.history.frames[-1].timestamp,
            "t_decision": t_decision, "t_request": t_request, "t_ack": t_ack,
            "t_active": t_active, "t_window_end": t_window_end,
            "planner_cycle_count": self.planner_cycles,
            "valid_feedback_duration": fields["valid_feedback_duration"],
            "state_valid": True, "invalid_reason": "",
            "goal_distance_start": fields["goal_distance_start"],
            "goal_distance_end": fields["goal_distance_end"],
            "path_error_mean": sum(path_errors) / len(path_errors),
            "path_heading_error_mean": heading,
            "d_obs_min": self.minimum_clearance,
            "obstacle_density_mean": density, "ttc_min": ttc,
            "linear_velocity_mean": sum(velocities) / len(velocities),
            "angular_velocity_mean": angular,
            "eta_before_json": None, "action_raw_json": "[]", "eta_after_json": None,
            "theta_previous_json": theta_json(theta_previous),
            "theta_candidate_json": theta_json(projection.candidate),
            "theta_projected_json": theta_json(projection.projected),
            "theta_safe_json": theta_json(fallback.theta),
            "theta_applied_json": theta_json(transaction["readback"]),
            "projection_modified": projection.intervened,
            "projection_reason": "|".join(projection.reasons),
            "safety_modified": fallback.theta != projection.projected,
            "safety_mode": safety.mode.value,
            "safety_reason": "|".join(safety.reasons),
            "fallback_active": fallback.use_fallback,
            "fallback_reason": "|".join(fallback.reasons),
            "inference_latency": 0.0,
            "parameter_write_latency": (t_ack - t_request) * 1000.0,
            "parameter_activation_latency": (t_active - t_request) * 1000.0,
            "transition_stored": True, "transition_drop_reason": "",
        }
        step.update(rewards.components)
        step.update(rewards.costs)

        commit = _git_output("rev-parse", "HEAD")
        dirty = bool(_git_output("status", "--porcelain"))
        scenario_hash = sha256_file(self.world_path)
        navigation_time = max(0.0, rospy.Time.now().to_sec() - t_decision)
        episode_row = {
            "run_id": run_id, "episode_id": episode_id, "algorithm": "TEB-Default",
            "scene_id": "t04_t06_pipeline_straight", "scene_split": "validation",
            "training_seed": None, "seed": self.seed,
            "config_version": "t04_t06_pipeline_validation_v1",
            "git_commit": commit, "git_dirty": dirty,
            "submodule_commits_json": "{}", "policy_checkpoint_sha256": None,
            "scenario_manifest_sha256": scenario_hash, "localization_mode": "gazebo",
            "success": outcome.success, "collision": outcome.collision,
            "terminated": outcome.terminated, "truncated": outcome.truncated,
            "termination_reason": outcome.termination_reason,
            "path_length": self.path_length, "navigation_time": navigation_time,
            "path_efficiency": (1.5 / self.path_length if self.path_length > 0 else None),
            "smoothness": -rewards.components["reward_smoothness"],
            "linear_acc_rms": 0.0, "angular_acc_rms": 0.0,
            "min_obstacle_distance": self.minimum_clearance,
            "near_collision_time_ratio": 0.0,
            "parameter_adjustment_count": 1,
            "parameter_total_variation": 0.0,
            "projection_intervention_count": int(projection.intervened),
            "safety_filter_intervention_count": int(fallback.theta != projection.projected),
            "safety_fallback_count": int(fallback.use_fallback), "fallback_duration": 0.0,
            "fallback_recovery_count": 0, "planner_failure_count": 0,
            "candidate_parameter_violation_count": 0,
            "semantic_direction_consistency": None, "inference_latency_mean": 0.0,
            "parameter_write_latency_mean": step["parameter_write_latency"],
            "parameter_activation_latency_mean": step["parameter_activation_latency"],
            "operator_intervention_count": 0, "bag_uri": None,
            "notes": "pipeline validation only; excluded from formal results",
        }
        episode_csv = self.output_dir / "episodes.csv"
        step_csv = self.output_dir / "steps.csv"
        checksums_path = self.output_dir / "checksums.sha256"
        manifest_path = self.output_dir / "run_manifest.yaml"
        write_episode_csv(episode_csv, [episode_row], self.episode_schema)
        write_step_csv(step_csv, [step], self.step_schema)
        write_checksums(checksums_path, [episode_csv, step_csv], self.output_dir)
        manifest = {
            "schema_version": "1.0", "run_id": run_id,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "algorithm": "TEB-Default", "mode": "gazebo",
            "scene_id": "t04_t06_pipeline_straight", "scene_split": "validation",
            "training_seed": None, "evaluation_seed": self.seed,
            "source": {"main_commit": commit, "main_dirty": dirty,
                       "submodule_commits": {},
                       "ros_version": str(rospy.get_param("/rosversion", "noetic")).strip(),
                       "gazebo_version": "11", "python_version": sys.version.split()[0],
                       "host_id": os.uname().nodename},
            "configuration": {
                "experiment_contract_path": str(self.contract_path),
                "experiment_contract_sha256": sha256_file(self.contract_path),
                "scene_manifest_path": str(self.world_path),
                "scene_manifest_sha256": sha256_file(self.world_path),
                "theta_bounds_path": str(self.safety_config_path),
                "theta_bounds_sha256": sha256_file(self.safety_config_path),
                "A_TEB_path": None, "A_TEB_sha256": None,
                "reward_config_path": str(self.pipeline_config_path),
                "reward_config_sha256": sha256_file(self.pipeline_config_path),
                "safety_config_path": str(self.safety_config_path),
                "safety_config_sha256": sha256_file(self.safety_config_path),
                "policy_checkpoint_path": None, "policy_checkpoint_sha256": None,
            },
            "topics": {"scan": "/scan", "odom": "/odom", "cmd_vel": "/cmd_vel",
                       "global_plan": "/move_base/TebLocalPlannerROS/global_plan",
                       "local_plan": "/move_base/TebLocalPlannerROS/local_plan",
                       "status": "/move_base/status"},
            "safety": {"allow_motion": False, "allow_parameter_write": False,
                       "speed_limit_mps": 1.2, "human_operator": None,
                       "emergency_stop_checked": False, "fence_checked": False,
                       "conservative_fallback_checked": True},
            "artifacts": {"episode_csv": "episodes.csv", "step_log": "steps.csv",
                          "rosbag": None, "stdout_log": None, "failure_index": None,
                          "checksums_file": "checksums.sha256"},
            "completion": {"validated": True, "validation_report": None,
                           "excluded_from_formal_results": True,
                           "exclusion_reason": "pipeline_validation_only"},
        }
        write_run_manifest(manifest_path, manifest)
        validation = RunValidator(self.episode_schema, self.step_schema).validate(manifest_path)
        acceptance = {
            "schema_version": 1, "tasks": ["T04", "T05", "T06"],
            "simulation_only": True, "real_driver_started": False,
            "serial_or_can_accessed": False, "real_vehicle_motion": False,
            "formal_experiment": False, "passed": validation["valid"],
            "episode_success": outcome.success,
            "history_k": len(self.history.frames), "state_dimension": len(self.history.stacked()),
            "t_request": t_request, "t_ack": t_ack, "t_active": t_active,
            "planner_cycle_count": self.planner_cycles,
            "reward_components": dict(rewards.components), "costs": dict(rewards.costs),
            "projection_reasons": list(projection.reasons),
            "safety_mode": safety.mode.value,
            "dry_emergency_mode": dry_emergency.mode.value,
            "dry_emergency_fallback_atomic_count": len(dry_fallback.theta),
            "snapshot_restored": restoration["readback"] == theta_previous,
            "run_validation": validation,
        }
        report_path = self.output_dir.parent / "t04_t06_pipeline_acceptance.yaml"
        report_path.write_text(yaml.safe_dump(acceptance, sort_keys=False), encoding="utf-8")
        return acceptance


def main():
    rospy.init_node("t04_t06_pipeline_integration", anonymous=False)
    runner = PipelineAcceptance()
    try:
        report = runner.run()
        if (not report["passed"] or not report["snapshot_restored"] or
                not report["episode_success"]):
            return 1
        rospy.loginfo("T04--T06 pipeline acceptance passed")
        return 0
    finally:
        runner.client.cancel_all_goals()
        runner.cmd_stop.publish(Twist())


if __name__ == "__main__":
    if "--rostest" in sys.argv:
        import rostest

        class PipelineIntegrationTest(unittest.TestCase):
            def test_fixed_policy_bundle(self):
                self.assertEqual(main(), 0)

        rostest.rosrun("thesis_experiment", "t04_t06_pipeline_integration", PipelineIntegrationTest)
    else:
        sys.exit(main())
