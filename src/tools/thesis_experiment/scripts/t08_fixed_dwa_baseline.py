#!/usr/bin/env python3
"""Run one frozen T08 baseline group with shared episode/CSV semantics."""

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rospy
import yaml
from actionlib_msgs.msg import GoalStatus

from teb_rl_tuner.baseline_policy import CausalRuleTebPolicy, ObservationLayout, RuleThresholds
from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.episode_state_machine import EpisodeStateMachine
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.state_builder import StateFrame
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from thesis_experiment.baseline_evaluator import load_baseline_contract
from thesis_experiment.gazebo_training_adapter import GazeboTrainingAdapter, TrainingSafetyAdapter
from thesis_experiment.run_artifacts import (
    RunValidator, sha256_file, write_checksums, write_episode_csv,
    write_run_manifest, write_step_csv,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _git(*args):
    return subprocess.check_output(["git"] + list(args), cwd=str(WORKSPACE), text=True,
                                   stderr=subprocess.DEVNULL).strip()


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _trapz(samples, key):
    return sum(0.5 * (right["stamp"] - left["stamp"]) *
               (left[key] + right[key]) for left, right in zip(samples, samples[1:]))


class BaselineRunner:
    def __init__(self):
        self.seed = int(rospy.get_param("~seed", 42))
        self.algorithm_name = str(rospy.get_param("~algorithm", "Fixed-DWA"))
        self.run_id = str(rospy.get_param("~run_id", "t08_fixed_dwa_seed42"))
        self.planner_namespace = str(rospy.get_param(
            "~planner_namespace",
            "DWAPlannerROS" if self.algorithm_name == "Fixed-DWA" else "TebLocalPlannerROS",
        ))
        self.contract_path = Path(rospy.get_param(
            "~baseline_contract", str(WORKSPACE / "config/thesis_experiments/t08_baselines.yaml")
        ))
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / "artifacts/t08" / self.run_id)
        ))
        self.contract = load_baseline_contract(self.contract_path)
        self.algorithm = next(item for item in self.contract["algorithms"]
                              if item["name"] == self.algorithm_name)
        self.episode_schema = WORKSPACE / self.contract["logging"]["episode_schema"]
        self.step_schema = WORKSPACE / self.contract["logging"]["step_schema"]
        self.safety_path = WORKSPACE / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml"
        self.safety_data = yaml.safe_load(self.safety_path.read_text(encoding="utf-8"))
        self.dwa_path = WORKSPACE / "src/simulation/m2_gazebo/config/fixed_dwa.yaml"
        self.client = None
        self.adapter = None
        self.projector = None
        self.safety = None
        self.rule = None

    def _connect_teb(self):
        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True, use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        self.client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0
        )
        self.client.initialize()

    def _configure_teb_logic(self):
        self.projector = ParameterProjector({
            name: ParameterLimit(
                self.safety_data["theta_bounds"][name][0],
                self.safety_data["theta_bounds"][name][1],
                self.safety_data["max_delta_per_step"][name], True,
            ) for name in EXPECTED_THETA_ORDER
        }, min_turning_radius=1.2)
        values = self.safety_data["safety_margin"]
        self.safety = TrainingSafetyAdapter(
            SafetyMarginFilter(SafetyMarginConfig(
                a_brake_lower=values["a_brake_lower_mps2"],
                tau_total_upper=values["total_latency_upper_s"],
                d_margin=values["distance_margin_m"],
                warning_margin=values["warning_margin_m"],
                emergency_margin=values["emergency_margin_m"],
                hysteresis_margin=values["recovery_margin_m"],
                recovery_healthy_s=values["recovery_healthy_duration_s"],
            )), ConservativeFallbackPolicy(self.safety_data["conservative_theta"]),
        )
        if self.algorithm_name == "Rule-TEB":
            self.rule = CausalRuleTebPolicy(
                self.algorithm["profiles"], RuleThresholds(**self.algorithm["thresholds"]),
                ObservationLayout(1, ("footprint_clearance", "approximate_ttc",
                                      "path_cross_track_error"), 1),
            )

    def run(self):
        scenes = [dict(item) for item in self.contract["evaluation_matrix"]["scenes"]]
        if self.algorithm_name != "Fixed-DWA":
            self._connect_teb()
            self._configure_teb_logic()
        self.adapter = GazeboTrainingAdapter(
            self.client, scenes, self.safety_data["theta_bounds"] if self.client else None,
            planner_namespace=self.planner_namespace,
        )
        episodes, steps, failures = [], [], []
        try:
            for index, scene in enumerate(scenes):
                episode, step = self._episode(index, scene)
                episodes.append(episode)
                steps.append(step)
                if not episode["success"]:
                    failures.append({"episode_id": episode["episode_id"],
                                     "scene_id": scene["scene_id"],
                                     "termination_reason": episode["termination_reason"]})
            return self._write(episodes, steps, failures)
        finally:
            if self.adapter is not None:
                self.adapter.close()

    def _initial_theta(self):
        if self.algorithm_name == "Rule-TEB":
            return dict(self.algorithm["profiles"]["efficient"])
        if self.client:
            return dict(self.algorithm["theta"])
        return {}

    def _episode(self, index, scene):
        machine = EpisodeStateMachine()
        episode_id = "{}-episode-{:04d}".format(self.run_id, index)
        machine.start(episode_id)
        initial = self._initial_theta()
        if self.client:
            self.adapter.write_parameters(initial, 0)
            self.safety.reset(self.seed)
            self.safety.fallback_policy.confirm_applied_safe(initial)
            if self.rule:
                self.rule.reset(self.seed)
        self.adapter.reset(self.seed)
        started = rospy.Time.now().to_sec()
        start_goal_distance = math.hypot(float(scene["goal"][0]), float(scene["goal"][1]))
        previous_linear = previous_angular = 0.0
        previous_stamp = started
        samples = []
        reason = ""
        adjustment_count = 0
        total_variation = 0.0
        activation_latencies = []
        last_action = dict(initial)
        next_rule_decision = started
        last_safety_mode = "NORMAL"
        while rospy.Time.now().to_sec() - started < float(scene["timeout_s"]):
            metrics = self.adapter._metrics()
            if self.rule and metrics["stamp"] >= next_rule_decision:
                observation = (0.0, metrics["clearance"], metrics["ttc"], metrics["path_error"])
                action = self.rule.act(observation)
                current = self.adapter.current_theta()
                projected = self.projector.project(action, current)
                frame = StateFrame(metrics["stamp"], (), True, (), {
                    "footprint_clearance": metrics["clearance"],
                    "linear_velocity": metrics["linear_velocity"],
                })
                safe = self.safety.filter(projected.projected, current, frame, metrics["stamp"])
                last_action = dict(action)
                last_safety_mode = self.safety.last_decision.mode.value
                if safe.request_stop:
                    reason = "emergency_stop"
                    self.adapter.request_stop("t08_rule_safety_emergency")
                    break
                if any(abs(safe.theta[name] - current[name]) > 1e-12
                       for name in EXPECTED_THETA_ORDER):
                    receipt = self.adapter.write_parameters(safe.theta, adjustment_count + 1)
                    poll = self.adapter.poll_activation(adjustment_count + 1, receipt.t_ack, 2.0)
                    active = next((stamp for stamp, complete in poll.plans
                                   if complete and stamp > receipt.t_ack), None)
                    if active is None:
                        reason = "interface_fault"
                        self.adapter.request_stop("t08_rule_activation_timeout")
                        break
                    activation_latencies.append((active - receipt.t_request) * 1000.0)
                    for name in EXPECTED_THETA_ORDER:
                        low, high = self.safety_data["theta_bounds"][name]
                        total_variation += abs(safe.theta[name] - current[name]) / (high - low)
                    adjustment_count += 1
                next_rule_decision = metrics["stamp"] + 1.0
            dt = max(metrics["stamp"] - previous_stamp, 1e-6)
            linear_acc = (metrics["linear_velocity"] - previous_linear) / dt
            angular_acc = (metrics["angular_velocity"] - previous_angular) / dt
            samples.append({
                "stamp": metrics["stamp"], "goal_distance": metrics["goal_distance"],
                "path_error": metrics["path_error"], "heading_error": metrics["path_heading_error"],
                "clearance": metrics["clearance"], "density": metrics["obstacle_density"],
                "ttc": metrics["ttc"], "linear": metrics["linear_velocity"],
                "angular": metrics["angular_velocity"], "linear_acc_sq": linear_acc ** 2,
                "angular_acc_sq": angular_acc ** 2, "smoothness": linear_acc ** 2 + angular_acc ** 2,
                "near": float(metrics["clearance"] < 1.0),
            })
            previous_linear, previous_angular, previous_stamp = (
                metrics["linear_velocity"], metrics["angular_velocity"], metrics["stamp"]
            )
            if metrics["clearance"] < 0.2:
                reason = "collision"
                self.adapter.request_stop("t08_collision")
                break
            state = self.adapter.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                reason = "goal"
                break
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                reason = "planner_failure"
                break
            rospy.sleep(0.05)
        if not reason:
            reason = "timeout"
            self.adapter.request_stop("t08_timeout")
        outcome = machine.finish(reason)
        ended = rospy.Time.now().to_sec()
        if len(samples) < 2:
            # Retain an immediate safety/interface termination as a finite row.
            metrics = self.adapter._metrics()
            sample = {"stamp": ended, "goal_distance": metrics["goal_distance"],
                      "path_error": metrics["path_error"], "heading_error": metrics["path_heading_error"],
                      "clearance": metrics["clearance"], "density": metrics["obstacle_density"],
                      "ttc": metrics["ttc"], "linear": metrics["linear_velocity"],
                      "angular": metrics["angular_velocity"], "linear_acc_sq": 0.0,
                      "angular_acc_sq": 0.0, "smoothness": 0.0, "near": 0.0}
            samples = [dict(sample, stamp=started), sample]
        return self._rows(
            episode_id, scene, outcome, reason, started, ended, start_goal_distance,
            samples, adjustment_count, total_variation, activation_latencies,
            last_action, last_safety_mode,
        )

    def _rows(self, episode_id, scene, outcome, reason, started, ended,
              start_goal_distance, samples, adjustment_count, total_variation,
              activation_latencies, last_action, last_safety_mode):
        duration = max(ended - started, 1e-6)
        smoothness = _trapz(samples, "smoothness")
        near_ratio = _trapz(samples, "near") / duration
        linear_acc_rms = math.sqrt(max(0.0, _trapz(samples, "linear_acc_sq") / duration))
        angular_acc_rms = math.sqrt(max(0.0, _trapz(samples, "angular_acc_sq") / duration))
        path_length = self.adapter.path_length
        commit = _git("rev-parse", "HEAD")
        applied = self.adapter.current_theta() if self.client else {}
        activation_mean = (sum(activation_latencies) / len(activation_latencies)
                           if activation_latencies else 0.0)
        episode = {
            "run_id": self.run_id, "episode_id": episode_id, "algorithm": self.algorithm_name,
            "scene_id": scene["scene_id"], "scene_split": scene["split"],
            "training_seed": None, "seed": self.seed,
            "config_version": self.algorithm["config_version"], "git_commit": commit,
            "git_dirty": bool(_git("status", "--porcelain")), "submodule_commits_json": "{}",
            "policy_checkpoint_sha256": None, "scenario_manifest_sha256": sha256_file(self.contract_path),
            "localization_mode": "gazebo", "success": outcome.success, "collision": outcome.collision,
            "terminated": outcome.terminated, "truncated": outcome.truncated,
            "termination_reason": reason, "path_length": path_length, "navigation_time": duration,
            "path_efficiency": start_goal_distance / path_length if path_length > 0.0 else None,
            "smoothness": smoothness, "linear_acc_rms": linear_acc_rms,
            "angular_acc_rms": angular_acc_rms, "min_obstacle_distance": self.adapter.minimum_clearance,
            "near_collision_time_ratio": near_ratio, "parameter_adjustment_count": adjustment_count,
            "parameter_total_variation": total_variation, "projection_intervention_count": 0,
            "safety_filter_intervention_count": int(last_safety_mode != "NORMAL"),
            "safety_fallback_count": int(reason == "emergency_stop"), "fallback_duration": 0.0,
            "fallback_recovery_count": 0, "planner_failure_count": int(reason == "planner_failure"),
            "candidate_parameter_violation_count": 0, "semantic_direction_consistency": None,
            "inference_latency_mean": 0.0, "parameter_write_latency_mean": 0.0,
            "parameter_activation_latency_mean": activation_mean,
            "operator_intervention_count": 0, "bag_uri": None,
            "notes": "{}; T08 validation only".format(self.algorithm_name),
        }
        last = samples[-1]
        step = {
            "run_id": self.run_id, "episode_id": episode_id, "step_id": 0,
            "config_seq": adjustment_count, "t_observation": started, "t_decision": started,
            "t_request": None, "t_ack": None, "t_active": None, "t_window_end": ended,
            "planner_cycle_count": 0, "valid_feedback_duration": duration,
            "state_valid": reason not in ("sensor_fault", "tf_fault", "interface_fault"),
            "invalid_reason": reason if reason in ("sensor_fault", "tf_fault", "interface_fault") else "",
            "goal_distance_start": start_goal_distance, "goal_distance_end": last["goal_distance"],
            "path_error_mean": _trapz(samples, "path_error") / duration,
            "path_heading_error_mean": _trapz(samples, "heading_error") / duration,
            "d_obs_min": self.adapter.minimum_clearance,
            "obstacle_density_mean": _trapz(samples, "density") / duration,
            "ttc_min": min(item["ttc"] for item in samples),
            "linear_velocity_mean": _trapz(samples, "linear") / duration,
            "angular_velocity_mean": _trapz(samples, "angular") / duration,
            "eta_before_json": None,
            "action_raw_json": _json(last_action or {"planner": "Fixed-DWA"}),
            "eta_after_json": None, "theta_previous_json": _json(applied),
            "theta_candidate_json": _json(last_action), "theta_projected_json": _json(last_action),
            "theta_safe_json": _json(applied), "theta_applied_json": _json(applied) if self.client else None,
            "projection_modified": False, "projection_reason": "", "safety_modified": reason == "emergency_stop",
            "safety_mode": last_safety_mode, "safety_reason": reason if reason == "emergency_stop" else "",
            "fallback_active": reason == "emergency_stop", "fallback_reason": reason if reason == "emergency_stop" else "",
            "reward_total": start_goal_distance - last["goal_distance"] - duration,
            "reward_progress": start_goal_distance - last["goal_distance"], "reward_time": -duration,
            "reward_near_obstacle": -near_ratio * duration,
            "reward_path_error": -_trapz(samples, "path_error"), "reward_smoothness": -smoothness,
            "reward_planner_failure": -float(reason == "planner_failure"),
            "reward_parameter_adjustment": -total_variation, "reward_terminal": float(reason == "goal"),
            "cost_collision": float(reason == "collision"), "cost_near_collision": near_ratio * duration,
            "cost_parameter_violation": 0.0, "cost_planner_failure": float(reason == "planner_failure"),
            "cost_emergency_or_fallback": float(reason == "emergency_stop"), "inference_latency": 0.0,
            "parameter_write_latency": 0.0 if self.client else None,
            "parameter_activation_latency": activation_mean if self.client else None,
            "transition_stored": reason != "interface_fault",
            "transition_drop_reason": reason if reason == "interface_fault" else "",
        }
        return episode, step

    def _write(self, episodes, steps, failures):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        episode_csv, step_csv = self.output_dir / "episodes.csv", self.output_dir / "steps.csv"
        failure_path = self.output_dir / "failure_index.yaml"
        checksum_path, manifest_path = self.output_dir / "checksums.sha256", self.output_dir / "run_manifest.yaml"
        write_episode_csv(episode_csv, episodes, self.episode_schema)
        write_step_csv(step_csv, steps, self.step_schema)
        failure_path.write_text(yaml.safe_dump(failures, sort_keys=False), encoding="utf-8")
        write_checksums(checksum_path, [episode_csv, step_csv, failure_path], self.output_dir)
        config_path = self.safety_path if self.client else self.dwa_path
        manifest = {
            "schema_version": "1.0", "run_id": self.run_id,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "algorithm": self.algorithm_name, "mode": "gazebo", "scene_id": "t08_baseline_matrix",
            "scene_ids": [item["scene_id"] for item in self.contract["evaluation_matrix"]["scenes"]],
            "scene_split": "validation", "training_seed": None, "evaluation_seed": self.seed,
            "source": {"main_commit": _git("rev-parse", "HEAD"), "main_dirty": True,
                       "submodule_commits": {}, "ros_version": str(rospy.get_param("/rosversion", "noetic")),
                       "gazebo_version": "11", "python_version": sys.version.split()[0],
                       "host_id": os.uname().nodename},
            "configuration": {
                "experiment_contract_path": str(WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"),
                "experiment_contract_sha256": sha256_file(WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"),
                "scene_manifest_path": str(self.contract_path), "scene_manifest_sha256": sha256_file(self.contract_path),
                "theta_bounds_path": str(self.safety_path) if self.client else None,
                "theta_bounds_sha256": sha256_file(self.safety_path) if self.client else None,
                "A_TEB_path": None, "A_TEB_sha256": None,
                "reward_config_path": str(self.contract_path), "reward_config_sha256": sha256_file(self.contract_path),
                "safety_config_path": str(config_path), "safety_config_sha256": sha256_file(config_path),
                "policy_checkpoint_path": None, "policy_checkpoint_sha256": None,
            },
            "topics": {"scan": "/scan", "odom": "/odom", "cmd_vel": "/cmd_vel",
                       "global_plan": "/move_base/{}/global_plan".format(self.planner_namespace),
                       "local_plan": "/move_base/{}/local_plan".format(self.planner_namespace),
                       "status": "/move_base/status"},
            "safety": {"allow_motion": False, "allow_parameter_write": False,
                       "speed_limit_mps": 0.8 if not self.client else 1.2,
                       "human_operator": None, "emergency_stop_checked": False,
                       "fence_checked": False, "conservative_fallback_checked": bool(self.client)},
            "artifacts": {"episode_csv": "episodes.csv", "step_log": "steps.csv", "rosbag": None,
                          "stdout_log": None, "failure_index": "failure_index.yaml",
                          "checksums_file": "checksums.sha256"},
            "completion": {"validated": True, "validation_report": None,
                           "excluded_from_formal_results": True,
                           "exclusion_reason": "t08_pipeline_validation_only"},
        }
        write_run_manifest(manifest_path, manifest)
        return RunValidator(self.episode_schema, self.step_schema).validate(manifest_path)


def main():
    rospy.init_node("t08_baseline_runner", anonymous=False)
    report = BaselineRunner().run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
