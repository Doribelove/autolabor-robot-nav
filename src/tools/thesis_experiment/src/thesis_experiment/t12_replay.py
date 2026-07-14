"""T12 offline telemetry replay through the read-only shadow runtime."""

import csv
import json
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.shadow_runtime import (
    FeatureEnvelope, ShadowRuntime, ShadowRuntimeConfig,
)


class T12ReplayError(ValueError):
    pass


def _float(row: Mapping[str, Any], name: str, default: float = 0.0) -> float:
    value = row.get(name, "")
    if value in (None, ""):
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        raise T12ReplayError("{} must be numeric".format(name))


def _bool(value: Any) -> bool:
    if value in (True, "true", "True", "1", 1):
        return True
    if value in (False, "false", "False", "0", 0, ""):
        return False
    raise T12ReplayError("invalid boolean {}".format(value))


def _json_mapping(row: Mapping[str, Any], name: str) -> Dict[str, float]:
    try:
        value = json.loads(row[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise T12ReplayError("invalid {}: {}".format(name, exc))
    if not isinstance(value, dict):
        raise T12ReplayError("{} must contain a JSON mapping".format(name))
    return {key: float(value[key]) for key in EXPECTED_THETA_ORDER}


def _read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def build_runtime(config: Mapping[str, Any], root: Path) -> ShadowRuntime:
    theta_path = root / config["theta_config"]
    theta_data = yaml.safe_load(theta_path.read_text(encoding="utf-8"))
    limits = {
        name: ParameterLimit(
            float(theta_data["theta_bounds"][name][0]),
            float(theta_data["theta_bounds"][name][1]),
            float(theta_data["max_delta_per_step"][name]),
        ) for name in EXPECTED_THETA_ORDER
    }
    safety = config["safety"]
    safety_filter = SafetyMarginFilter(SafetyMarginConfig(
        a_brake_lower=float(safety["a_brake_lower_mps2"]),
        tau_total_upper=float(safety["total_latency_upper_s"]),
        d_margin=float(safety["distance_margin_m"]),
        warning_margin=float(safety["warning_margin_m"]),
        emergency_margin=float(safety["emergency_margin_m"]),
        hysteresis_margin=float(safety["recovery_margin_m"]),
        recovery_healthy_s=float(safety["recovery_healthy_duration_s"]),
        emergency_distance_cap=float(safety["emergency_distance_cap_m"]),
        emergency_confirmation_s=float(safety["emergency_confirmation_s"]),
    ))
    runtime = config["runtime"]
    return ShadowRuntime(
        ShadowRuntimeConfig(
            float(runtime["ema_alpha"]),
            float(runtime["ood_warning_score"]),
            float(runtime["ood_fallback_score"]),
        ),
        ParameterProjector(limits, min_turning_radius=1.2),
        safety_filter,
        ConservativeFallbackPolicy(theta_data["conservative_theta"]),
        FeatureEnvelope(config["feature_envelope"]),
    )


def _discover_runs(config: Mapping[str, Any], root: Path) -> List[Path]:
    paths = sorted(root.glob(config["source"]["run_glob"]))
    if not paths:
        raise T12ReplayError("T12 source run_glob matched no directories")
    return paths


def evaluate_replay(config_path: Any, workspace: Any) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    root = Path(workspace)
    path = Path(config_path)
    if not path.is_absolute():
        path = root / path
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if config.get("mode") != "shadow" or config.get("read_only") is not True:
        raise T12ReplayError("T12 replay must be read-only shadow mode")
    if config.get("allow_motion") is not False or config.get("allow_parameter_write") is not False:
        raise T12ReplayError("T12 replay cannot authorize motion or parameter writes")
    runtime = build_runtime(config, root)
    included_splits = set(config["source"]["include_splits"])
    decisions, episode_rows = [], []
    last_episode = None
    invalid_rows = 0
    original_projection = 0
    candidate_l1, smoothed_l1, projected_l1, recommendation_l1 = [], [], [], []
    activation_latencies = []
    inference_latencies = []
    for run_dir in _discover_runs(config, root):
        episodes = _read_csv(run_dir / config["source"]["episode_file"])
        metadata = {row["episode_id"]: row for row in episodes}
        episode_rows.extend(row for row in episodes if row["scene_split"] in included_splits)
        for row in _read_csv(run_dir / config["source"]["step_file"]):
            episode = metadata.get(row["episode_id"])
            if episode is None or episode["scene_split"] not in included_splits:
                continue
            current = _json_mapping(row, "theta_previous_json")
            candidate = _json_mapping(row, "theta_candidate_json")
            if row["episode_id"] != last_episode:
                runtime.reset(current)
                last_episode = row["episode_id"]
            valid = _bool(row.get("state_valid", False))
            if not valid:
                invalid_rows += 1
            features = {
                "footprint_clearance": _float(row, "d_obs_min", 30.0),
                "linear_velocity": _float(row, "linear_velocity_mean"),
                "approximate_ttc": _float(row, "ttc_min", 30.0),
                "obstacle_density": _float(row, "obstacle_density_mean"),
                "path_error": abs(_float(row, "path_error_mean")),
                "goal_distance": _float(row, "goal_distance_start"),
            }
            health = dict(sensor=valid, tf=valid, localization=valid,
                          parameter_interface=True, planner=valid)
            decision = runtime.evaluate(
                candidate, current, features, health,
                _float(row, "t_observation"),
            )
            original_projection += int(_bool(row.get("projection_modified", False)))
            def l1(theta):
                return sum(abs(theta[name] - current[name])
                           for name in EXPECTED_THETA_ORDER)
            candidate_delta = l1(decision.candidate_theta)
            smoothed_delta = l1(decision.smoothed_theta)
            projected_delta = l1(decision.projected_theta)
            delta = l1(decision.recommended_theta)
            candidate_l1.append(candidate_delta)
            smoothed_l1.append(smoothed_delta)
            projected_l1.append(projected_delta)
            recommendation_l1.append(delta)
            activation_latencies.append(_float(row, "parameter_activation_latency"))
            inference_latencies.append(_float(row, "inference_latency"))
            decisions.append({
                "run_id": row["run_id"], "episode_id": row["episode_id"],
                "step_id": int(row["step_id"]), "scene_id": episode["scene_id"],
                "scene_split": episode["scene_split"],
                "training_seed": int(episode["training_seed"]),
                "evaluation_seed": int(episode["seed"]),
                "clearance": features["footprint_clearance"],
                "speed": features["linear_velocity"], "ttc": features["approximate_ttc"],
                "original_safety_mode": row.get("safety_mode", ""),
                "optimized_safety_mode": decision.safety.mode.value,
                "ood_score": decision.ood_score,
                "projection_modified": bool(decision.projection_reasons),
                "candidate_l1": candidate_delta,
                "smoothed_l1": smoothed_delta,
                "projected_l1": projected_delta,
                "recommendation_l1": delta,
                "write_allowed": decision.write_allowed,
                "motion_allowed": decision.motion_allowed,
                "reasons_json": json.dumps(decision.reasons, separators=(",", ":")),
                "recommended_theta_json": json.dumps(
                    decision.recommended_theta, sort_keys=True, separators=(",", ":")),
            })
    if not decisions:
        raise T12ReplayError("T12 replay selected no step rows")
    cap = float(config["safety"]["emergency_distance_cap_m"])
    emergency_episodes = [row for row in episode_rows if row["termination_reason"] == "emergency_stop"]
    avoidable = [row for row in emergency_episodes
                 if _float(row, "min_obstacle_distance") > cap]
    optimized_emergency = sum(row["optimized_safety_mode"] == "EMERGENCY" for row in decisions)
    optimized_fault = sum(row["optimized_safety_mode"] == "FAULT" for row in decisions)
    ood_fallback = sum(float(row["ood_score"]) >= config["runtime"]["ood_fallback_score"]
                       for row in decisions)
    acceptance = config["acceptance"]
    row_count = len(decisions)
    false_stop_fraction = len(avoidable) / float(max(1, len(emergency_episodes)))
    passed = (
        row_count >= int(acceptance["minimum_step_rows"]) and
        optimized_emergency / float(row_count) <=
        float(acceptance["maximum_optimized_emergency_rate"]) and
        false_stop_fraction >=
        float(acceptance["minimum_counterfactual_false_stop_fraction"]) and
        invalid_rows / float(row_count) <=
        float(acceptance["maximum_invalid_row_fraction"])
    )
    report = {
        "schema_version": "1.0", "task": "T12", "status": "passed" if passed else "failed",
        "passed": passed, "mode": "offline_telemetry_replay", "shadow_only": True,
        "allow_motion": False, "allow_parameter_write": False,
        "source_run_count": len(_discover_runs(config, root)),
        "episode_count": len(episode_rows), "step_row_count": row_count,
        "invalid_row_count": invalid_rows,
        "original_emergency_episode_count": len(emergency_episodes),
        "counterfactual_false_stop_candidate_count": len(avoidable),
        "counterfactual_false_stop_fraction": false_stop_fraction,
        "optimized_emergency_step_count": optimized_emergency,
        "optimized_fault_step_count": optimized_fault,
        "ood_fallback_step_count": ood_fallback,
        "original_projection_step_count": original_projection,
        "optimized_projection_step_count": sum(row["projection_modified"] for row in decisions),
        "mean_candidate_l1": mean(candidate_l1),
        "mean_smoothed_l1": mean(smoothed_l1),
        "mean_projected_l1": mean(projected_l1),
        "mean_recommendation_l1": mean(recommendation_l1),
        "mean_action_l1_reduction_after_smoothing_projection":
            1.0 - mean(projected_l1) / max(mean(candidate_l1), 1e-12),
        "mean_inference_latency_ms": mean(inference_latencies),
        "mean_simulated_activation_latency_ms": mean(activation_latencies),
        "acceptance": acceptance,
        "limitations": [
            "telemetry_csv_replay_does_not_reconstruct_raw_lidar",
            "counterfactual_safety_result_requires_live_shadow_confirmation",
            "simulation_theta_bounds_are_not_real_vehicle_calibration",
        ],
    }
    return report, decisions
