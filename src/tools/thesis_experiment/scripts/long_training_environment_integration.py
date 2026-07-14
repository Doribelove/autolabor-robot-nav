#!/usr/bin/env python3
"""Persistent multi-episode Gazebo acceptance for the SAC environment skeleton."""

import json
import math
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

import rospy
import yaml

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.baseline_policy import CausalRuleTebPolicy, ObservationLayout, RuleThresholds
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.reward_cost import RewardWeights
from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.state_builder import StateBuilder
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from teb_rl_tuner.training_environment import EnvironmentConfig, FixedPolicy, TrainingEnvironment
from thesis_experiment.gazebo_training_adapter import (
    FEATURE_ORDER,
    GazeboTrainingAdapter,
    TrainingSafetyAdapter,
)
from thesis_experiment.run_artifacts import (
    RunValidator,
    sha256_file,
    write_checksums,
    write_episode_csv,
    write_run_manifest,
    write_step_csv,
)
from thesis_experiment.baseline_evaluator import load_baseline_contract


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _git(*args):
    return subprocess.check_output(
        ["git"] + list(args), cwd=str(WORKSPACE), text=True,
        stderr=subprocess.DEVNULL,
    ).strip()


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _mean(values):
    return sum(values) / len(values) if values else 0.0


class LongRunAcceptance:
    def __init__(self):
        self.episode_count = int(rospy.get_param("~episode_count", 5))
        self.seed = int(rospy.get_param("~seed", 42))
        self.algorithm = str(rospy.get_param("~algorithm", "TEB-Default"))
        self.run_id = str(rospy.get_param(
            "~run_id", "training_skeleton_seed{}".format(self.seed)
        ))
        baseline_contract_value = str(rospy.get_param("~baseline_contract", "")).strip()
        self.baseline_contract_path = Path(baseline_contract_value) if baseline_contract_value else None
        self.baseline_contract = (load_baseline_contract(self.baseline_contract_path)
                                  if self.baseline_contract_path else None)
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / "artifacts/t07/training_environment_run")
        ))
        self.safety_path = WORKSPACE / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml"
        self.pipeline_path = WORKSPACE / "src/application/teb_rl_tuner/config/t04_t06_simulation_validation.yaml"
        self.contract_path = WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"
        self.world_path = WORKSPACE / "src/simulation/m2_gazebo/worlds/obstacle_test.world"
        self.episode_schema = WORKSPACE / "docs/thesis_experiment/schemas/episode_metrics_schema.csv"
        self.step_schema = WORKSPACE / "docs/thesis_experiment/schemas/step_metrics_schema.csv"
        self.safety_data = yaml.safe_load(self.safety_path.read_text(encoding="utf-8"))
        self.pipeline_data = yaml.safe_load(self.pipeline_path.read_text(encoding="utf-8"))
        if self.safety_data.get("real_vehicle_use_forbidden") is not True:
            raise RuntimeError("long-run acceptance requires simulation-only safety config")
        self.parameter_client = None
        self.adapter = None

    def _projector(self):
        bounds = self.safety_data["theta_bounds"]
        rates = self.safety_data["max_delta_per_step"]
        return ParameterProjector({
            name: ParameterLimit(bounds[name][0], bounds[name][1], rates[name], True)
            for name in EXPECTED_THETA_ORDER
        }, min_turning_radius=1.2)

    def _safety(self, baseline):
        values = self.safety_data["safety_margin"]
        filter_core = SafetyMarginFilter(SafetyMarginConfig(
            a_brake_lower=values["a_brake_lower_mps2"],
            tau_total_upper=values["total_latency_upper_s"],
            d_margin=values["distance_margin_m"],
            warning_margin=values["warning_margin_m"],
            emergency_margin=values["emergency_margin_m"],
            hysteresis_margin=values["recovery_margin_m"],
            recovery_healthy_s=values["recovery_healthy_duration_s"],
        ))
        fallback = ConservativeFallbackPolicy(self.safety_data["conservative_theta"])
        fallback.confirm_applied_safe(baseline)
        return TrainingSafetyAdapter(filter_core, fallback)

    def _connect(self):
        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True,
            use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        self.parameter_client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0
        )
        self.parameter_client.initialize()
        if self.baseline_contract is None:
            scenarios = [
                {"scene_id": "long_clear_straight", "layout": "clear", "goal": [1.5, 0.0, 0.0]},
                {"scene_id": "long_clear_offset", "layout": "clear", "goal": [1.3, 0.35, 0.15]},
            ]
        else:
            scenarios = [dict(item) for item in self.baseline_contract["evaluation_matrix"]["scenes"]]
        self.adapter = GazeboTrainingAdapter(
            self.parameter_client, scenarios, self.safety_data["theta_bounds"]
        )

    def run(self):
        self._connect()
        baseline = self.parameter_client.snapshot
        safety = self._safety(baseline)
        builder = StateBuilder(
            FEATURE_ORDER, ("scan", "odom", "local_plan"), sector_count=36,
            max_sync_skew_s=0.6,
        )
        environment = TrainingEnvironment(
            adapter=self.adapter, state_builder=builder, projector=self._projector(),
            reward_weights=RewardWeights(),
            config=EnvironmentConfig(
                history_length=4, activation_timeout_s=2.0, reward_window_s=0.5,
                max_steps=100 if self.baseline_contract else 20,
                max_ros_duration_s=(
                    max(float(item["timeout_s"])
                        for item in self.baseline_contract["evaluation_matrix"]["scenes"])
                    if self.baseline_contract else 20.0
                ),
                warning_distance=1.0,
            ),
            safety_adapter=safety,
        )
        policy = self._policy(baseline)
        if self.baseline_contract is not None:
            initial_theta = policy.act(()) if isinstance(policy, FixedPolicy) else policy.profiles["efficient"]
            self.adapter.write_parameters(initial_theta, 0)
        run_id = self.run_id
        episode_rows, step_rows, episode_summaries = [], [], []
        try:
            for episode_index in range(self.episode_count):
                if hasattr(policy, "reset"):
                    policy.reset(self.seed)
                observation, reset_info = environment.reset(self.seed + episode_index)
                episode_id = "{}-episode-{:04d}".format(run_id, episode_index)
                reset_info["core_episode_id"] = reset_info["episode_id"]
                step_id = 0
                latencies_write, latencies_active = [], []
                total_variation = 0.0
                smoothness = 0.0
                started = rospy.Time.now().to_sec()
                last_planner_cycles = self.adapter.planner_cycle_count
                while True:
                    action = environment.action_from(policy, observation)
                    observation, reward, terminated, truncated, info = environment.step(action)
                    metrics = dict(self.adapter.last_metrics)
                    write_latency = (info.get("t_ack", 0.0) - info.get("t_request", 0.0)) * 1000.0
                    active_latency = (info.get("t_active", 0.0) - info.get("t_request", 0.0)) * 1000.0
                    latencies_write.append(write_latency)
                    latencies_active.append(active_latency)
                    planner_cycles = self.adapter.planner_cycle_count - last_planner_cycles
                    last_planner_cycles = self.adapter.planner_cycle_count
                    reward_fields = info.get("reward_fields", {})
                    smoothness += -float(reward_fields.get("reward_smoothness", 0.0))
                    projection_reasons = info.get("projection_reasons", ())
                    safety_reasons = info.get("safety_reasons", ())
                    decision = safety.last_decision
                    fallback = safety.last_fallback
                    previous = self.adapter._last_written_previous
                    applied = info.get("applied_theta", self.adapter.current_theta())
                    for name in EXPECTED_THETA_ORDER:
                        low, high = self.safety_data["theta_bounds"][name]
                        total_variation += abs(applied[name] - previous[name]) / (high - low)
                    row = {
                        "run_id": run_id, "episode_id": episode_id, "step_id": step_id,
                        "config_seq": info.get("config_seq", environment.config_seq),
                        "t_observation": metrics["stamp"],
                        "t_decision": info.get("t_decision", metrics["stamp"]),
                        "t_request": info.get("t_request"), "t_ack": info.get("t_ack"),
                        "t_active": info.get("t_active"),
                        "t_window_end": info.get("t_window_end", metrics["stamp"]),
                        "planner_cycle_count": max(0, planner_cycles),
                        "valid_feedback_duration": reward_fields.get("valid_feedback_duration", 0.0),
                        "state_valid": True, "invalid_reason": "",
                        "goal_distance_start": reward_fields.get("goal_distance_start", metrics["goal_distance"]),
                        "goal_distance_end": reward_fields.get("goal_distance_end", metrics["goal_distance"]),
                        "path_error_mean": metrics["path_error"],
                        "path_heading_error_mean": metrics["path_heading_error"],
                        "d_obs_min": self.adapter.minimum_clearance,
                        "obstacle_density_mean": metrics["obstacle_density"],
                        "ttc_min": metrics["ttc"],
                        "linear_velocity_mean": metrics["linear_velocity"],
                        "angular_velocity_mean": metrics["angular_velocity"],
                        "eta_before_json": None, "action_raw_json": _json(action),
                        "eta_after_json": None,
                        "theta_previous_json": _json(previous),
                        "theta_candidate_json": _json(info.get("candidate_theta", action)),
                        "theta_projected_json": _json(info.get("projected_theta", action)),
                        "theta_safe_json": _json(info.get("safe_theta", applied)),
                        "theta_applied_json": _json(applied),
                        "projection_modified": bool(info.get("projection_modified", False)),
                        "projection_reason": "|".join(projection_reasons),
                        "safety_modified": bool(fallback and fallback.theta != info.get("projected_theta", action)),
                        "safety_mode": decision.mode.value if decision else "FAULT",
                        "safety_reason": "|".join(safety_reasons),
                        "fallback_active": bool(fallback and fallback.use_fallback),
                        "fallback_reason": "|".join(fallback.reasons if fallback else ()),
                        "inference_latency": 0.0,
                        "parameter_write_latency": write_latency,
                        "parameter_activation_latency": active_latency,
                        "transition_stored": bool(info.get("transition_stored", False)),
                        "transition_drop_reason": info.get("transition_drop_reason", ""),
                    }
                    for name in (
                        "reward_total", "reward_progress", "reward_time", "reward_near_obstacle",
                        "reward_path_error", "reward_smoothness", "reward_planner_failure",
                        "reward_parameter_adjustment", "reward_terminal", "cost_collision",
                        "cost_near_collision", "cost_parameter_violation", "cost_planner_failure",
                        "cost_emergency_or_fallback",
                    ):
                        row[name] = float(reward_fields.get(name, 0.0))
                    step_rows.append(row)
                    step_id += 1
                    if terminated or truncated:
                        break
                outcome = environment.current_outcome
                if outcome is None:
                    raise RuntimeError("episode ended without an outcome")
                elapsed = rospy.Time.now().to_sec() - started
                episode_rows.append(self._episode_row(
                    run_id, episode_id, outcome, elapsed, step_id, total_variation,
                    smoothness, latencies_write, latencies_active,
                ))
                episode_summaries.append({
                    "episode_id": episode_id,
                    "core_episode_id": reset_info["core_episode_id"],
                    "scenario": dict(self.adapter.current_scenario),
                    "steps": step_id, "outcome": outcome.termination_reason,
                    "path_length_m": self.adapter.path_length,
                    "minimum_clearance_m": self.adapter.minimum_clearance,
                })
            return self._write_bundle(run_id, episode_rows, step_rows, episode_summaries)
        finally:
            if self.adapter is not None:
                self.adapter.close()

    def _algorithm_config(self):
        if self.baseline_contract is None:
            return None
        return next(item for item in self.baseline_contract["algorithms"]
                    if item["name"] == self.algorithm)

    def _policy(self, runtime_baseline):
        config = self._algorithm_config()
        if config is None:
            return FixedPolicy(runtime_baseline)
        if self.algorithm in ("TEB-Default", "TEB-Tuned"):
            return FixedPolicy(config["theta"])
        if self.algorithm == "Rule-TEB":
            thresholds = config["thresholds"]
            return CausalRuleTebPolicy(
                config["profiles"], RuleThresholds(**thresholds),
                ObservationLayout(36, FEATURE_ORDER, 4),
            )
        raise RuntimeError("long training environment cannot execute {}".format(self.algorithm))

    def _episode_row(
        self, run_id, episode_id, outcome, elapsed, step_count, total_variation,
        smoothness, write_latencies, active_latencies,
    ):
        commit = _git("rev-parse", "HEAD")
        scenario_hash = sha256_file(self.world_path)
        return {
            "run_id": run_id, "episode_id": episode_id, "algorithm": self.algorithm,
            "scene_id": self.adapter.current_scenario.get("scene_id", "training_skeleton_cycle"),
            "scene_split": self.adapter.current_scenario.get("split", "validation"),
            "training_seed": None, "seed": self.seed,
            "config_version": (self._algorithm_config() or {}).get(
                "config_version", "training_skeleton_v1"), "git_commit": commit,
            "git_dirty": bool(_git("status", "--porcelain")),
            "submodule_commits_json": "{}", "policy_checkpoint_sha256": None,
            "scenario_manifest_sha256": scenario_hash, "localization_mode": "gazebo",
            "success": outcome.success, "collision": outcome.collision,
            "terminated": outcome.terminated, "truncated": outcome.truncated,
            "termination_reason": outcome.termination_reason,
            "path_length": self.adapter.path_length, "navigation_time": elapsed,
            "path_efficiency": None, "smoothness": smoothness,
            "linear_acc_rms": None, "angular_acc_rms": None,
            "min_obstacle_distance": self.adapter.minimum_clearance,
            "near_collision_time_ratio": 0.0,
            "parameter_adjustment_count": step_count,
            "parameter_total_variation": total_variation,
            "projection_intervention_count": 0,
            "safety_filter_intervention_count": 0, "safety_fallback_count": 0,
            "fallback_duration": 0.0, "fallback_recovery_count": 0,
            "planner_failure_count": int(outcome.termination_reason == "planner_failure"),
            "candidate_parameter_violation_count": 0,
            "semantic_direction_consistency": None, "inference_latency_mean": 0.0,
            "parameter_write_latency_mean": _mean(write_latencies),
            "parameter_activation_latency_mean": _mean(active_latencies),
            "operator_intervention_count": 0, "bag_uri": None,
            "notes": "persistent {} baseline; pipeline validation only".format(self.algorithm),
        }

    def _write_bundle(self, run_id, episodes, steps, summaries):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        episode_csv = self.output_dir / "episodes.csv"
        step_csv = self.output_dir / "steps.csv"
        summary_path = self.output_dir / "episode_summaries.yaml"
        checksum_path = self.output_dir / "checksums.sha256"
        manifest_path = self.output_dir / "run_manifest.yaml"
        write_episode_csv(episode_csv, episodes, self.episode_schema)
        write_step_csv(step_csv, steps, self.step_schema)
        summary_path.write_text(yaml.safe_dump(summaries, sort_keys=False), encoding="utf-8")
        write_checksums(checksum_path, [episode_csv, step_csv, summary_path], self.output_dir)
        commit = _git("rev-parse", "HEAD")
        manifest = {
            "schema_version": "1.0", "run_id": run_id,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "algorithm": self.algorithm, "mode": "gazebo",
            "scene_id": "t08_baseline_matrix" if self.baseline_contract else "training_skeleton_cycle",
            "scene_split": "validation",
            "training_seed": None, "evaluation_seed": self.seed,
            "source": {"main_commit": commit, "main_dirty": True, "submodule_commits": {},
                       "ros_version": str(rospy.get_param("/rosversion", "noetic")).strip(),
                       "gazebo_version": "11", "python_version": sys.version.split()[0],
                       "host_id": os.uname().nodename},
            "configuration": {
                "experiment_contract_path": str(self.contract_path),
                "experiment_contract_sha256": sha256_file(self.contract_path),
                "scene_manifest_path": str(self.baseline_contract_path or self.world_path),
                "scene_manifest_sha256": sha256_file(self.baseline_contract_path or self.world_path),
                "theta_bounds_path": str(self.safety_path),
                "theta_bounds_sha256": sha256_file(self.safety_path),
                "A_TEB_path": None, "A_TEB_sha256": None,
                "reward_config_path": str(self.pipeline_path),
                "reward_config_sha256": sha256_file(self.pipeline_path),
                "safety_config_path": str(self.safety_path),
                "safety_config_sha256": sha256_file(self.safety_path),
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
                          "rosbag": None, "stdout_log": None,
                          "failure_index": "episode_summaries.yaml",
                          "checksums_file": "checksums.sha256"},
            "completion": {"validated": True, "validation_report": None,
                           "excluded_from_formal_results": True,
                           "exclusion_reason": "training_skeleton_validation_only"},
        }
        if self.baseline_contract is not None:
            manifest["scene_ids"] = [item["scene_id"]
                                     for item in self.baseline_contract["evaluation_matrix"]["scenes"]]
        write_run_manifest(manifest_path, manifest)
        validation = RunValidator(self.episode_schema, self.step_schema).validate(manifest_path)
        report = {
            "schema_version": 1,
            "task": "T08_{}_baseline".format(self.algorithm) if self.baseline_contract else
                    "T04_T06_training_skeleton",
            "simulation_only": True, "formal_experiment": False,
            "episode_count": len(episodes), "step_count": len(steps),
            "all_episodes_success": all(row["success"] for row in episodes),
            "all_transitions_stored": all(row["transition_stored"] for row in steps),
            "config_seq_resets_each_episode": all(
                [row["config_seq"] for row in steps if row["episode_id"] == episode["episode_id"]]
                == list(range(1, 1 + sum(row["episode_id"] == episode["episode_id"] for row in steps)))
                for episode in episodes
            ),
            "snapshot_restored": self.parameter_client.audit_records[-1]["readback"]
            == self.parameter_client.snapshot,
            "run_validation": validation,
            "passed": bool(validation["valid"] and (
                self.baseline_contract is not None or all(row["success"] for row in episodes)
            )),
        }
        report_path = self.output_dir.parent / "training_environment_acceptance.yaml"
        report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        return report


def main():
    rospy.init_node("long_training_environment_integration", anonymous=False)
    report = LongRunAcceptance().run()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    if "--rostest" in sys.argv:
        import rostest

        class LongRunTest(unittest.TestCase):
            def test_persistent_environment(self):
                self.assertEqual(main(), 0)

        rostest.rosrun("thesis_experiment", "long_training_environment_integration", LongRunTest)
    else:
        sys.exit(main())
