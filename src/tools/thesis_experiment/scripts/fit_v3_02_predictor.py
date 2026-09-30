#!/usr/bin/env python3
"""Fit and split-conformal calibrate the V3-02 latent-mode predictor."""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
HORIZONS = ("t1_5", "t2_5", "t3_5", "t4_5")
POLICIES = ("cautious", "assertive", "hesitant")


def module():
    spec = importlib.util.spec_from_file_location("v3_02_predictor", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def prediction(model, policy, horizon_index, features):
    coefficients = np.asarray(
        model["mode_heads"][policy][HORIZONS[horizon_index]]["coefficients"],
        dtype=float,
    )
    value = float(np.dot(coefficients, np.asarray(features, dtype=float)))
    physical = model.get("physical_coordinate_clip")
    if physical:
        value = max(float(features[1]), value)
        value = min(float(physical["upper_bound_m"]), value)
    return value


def model_features(v, metrics):
    if hasattr(v, "model_features"):
        return v.model_features(metrics)
    return metrics["feature_vector"]


def collect(v):
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    expected_count = int(schedule["row_count"])
    journal_rows = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        len([row for row in journal_rows if row["event"] == "unit_complete"])
        == expected_count
        and journal_rows[-1]["event"] == "execution_complete",
        "execution journal is incomplete",
    )
    episodes = []
    for row in schedule["rows"]:
        root = W / v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt = v.load_yaml_path(root / "attempt_receipt.yaml")
        raw = v.read_file_once(root / "raw_pose_trace.csv")
        v.require(
            receipt["raw_csv_sha256"] == v.sha256_bytes(raw),
            "raw hash differs",
        )
        metrics = v.trace_metrics(v.load_trace(root / "raw_pose_trace.csv"), row)
        v.require(metrics == receipt["metrics_preview"], "metric replay differs")
        episodes.append({"row": dict(row), "metrics": metrics})
    return episodes


def fit(v):
    stage_contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    v.validate_contract(
        stage_contract, v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    )
    contract = (
        v.effective_contract(stage_contract)
        if hasattr(v, "effective_contract")
        else stage_contract
    )
    if hasattr(v, "DATA_QUALIFICATION_RELATIVE"):
        qualification = v.load_yaml_relative(
            W, v.DATA_QUALIFICATION_RELATIVE
        )
        v.require(
            qualification["overall_pass"] is True
            and all(qualification["gates"].values()),
            "data qualification gate failed",
        )
    episodes = collect(v)
    training = [
        item for item in episodes if item["row"]["role_evaluator_only"] == "training"
    ]
    calibration = [
        item
        for item in episodes
        if item["row"]["role_evaluator_only"] == "calibration"
    ]
    gates = contract["model_calibration_gates"]
    v.require(
        len(training) == int(gates["training_episode_count"])
        and len(calibration) == int(gates["calibration_episode_count"]),
        "split differs",
    )
    ridge = float(contract["predictor_design"]["ridge_lambda"])
    heads = {}
    for policy in POLICIES:
        policy_rows = [
            item
            for item in training
            if item["row"]["actor_policy_evaluator_only"] == policy
        ]
        x = np.asarray(
            [model_features(v, item["metrics"]) for item in policy_rows],
            dtype=float,
        )
        heads[policy] = {}
        for horizon_index, horizon in enumerate(HORIZONS):
            y = np.asarray(
                [
                    item["metrics"]["target_actor_conflict_coordinates_m"][
                        horizon_index
                    ]
                    for item in policy_rows
                ],
                dtype=float,
            )
            penalty = ridge * np.eye(x.shape[1])
            penalty[0, 0] = 0.0
            coefficients = np.linalg.solve(x.T @ x + penalty, x.T @ y)
            heads[policy][horizon] = {
                "coefficients": [round(float(value), 12) for value in coefficients],
                "training_sample_count": len(policy_rows),
            }
    model = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "model_id": "{}_candidate_conditioned_latent_mode_ridge_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": "fit_before_held_out_test",
        "runtime_hidden_policy_input": False,
        "runtime_outputs_all_mode_heads": True,
        "feature_names": contract["predictor_design"]["scalar_features"],
        "observation_time_s": contract["scene"]["observation_time_s"],
        "prediction_absolute_times_s": contract["scene"][
            "prediction_absolute_times_s"
        ],
        "ridge_lambda": ridge,
        "mode_heads": heads,
        "calibration": {
            "method": getattr(
                v,
                "CONFORMAL_SCORE",
                "global_split_conformal_outer_envelope",
            ),
            "alpha": float(contract["predictor_design"]["conformal_alpha"]),
            "radius_m": None,
        },
        "actor_physical_radius_m": float(
            contract["predictor_design"]["actor_physical_radius_m"]
        ),
        "training_episode_count": len(training),
        "calibration_episode_count": len(calibration),
        "reserved_test_seeds_not_accessed": contract["authority_boundary"][
            "held_out_test_seeds_reserved"
        ],
    }
    if getattr(v, "PHYSICAL_CLIP", False):
        model["physical_coordinate_clip"] = {
            "lower_bound": "actor_coordinate_at_observation",
            "upper_bound_m": float(
                contract["predictor_design"]["physical_coordinate_clip"][
                    "upper_bound_m"
                ]
            ),
        }
    nonconformity = []
    calibration_points = []
    mode_errors = []
    for item in calibration:
        features = model_features(v, item["metrics"])
        actual_policy = item["row"]["actor_policy_evaluator_only"]
        for horizon_index, horizon in enumerate(HORIZONS):
            modes = [
                prediction(model, policy, horizon_index, features)
                for policy in POLICIES
            ]
            truth = item["metrics"]["target_actor_conflict_coordinates_m"][
                horizon_index
            ]
            lower, upper = min(modes), max(modes)
            actual_mode_error = abs(
                prediction(model, actual_policy, horizon_index, features)
                - truth
            )
            residual = (
                actual_mode_error
                if getattr(v, "CONFORMAL_SCORE", "")
                == "actual_mode_absolute_residual"
                else max(lower - truth, truth - upper, 0.0)
            )
            nonconformity.append(residual)
            mode_errors.append(actual_mode_error)
            calibration_points.append(
                {
                    "horizon": horizon,
                    "truth": truth,
                    "lower_before_calibration": lower,
                    "upper_before_calibration": upper,
                    "nonconformity": residual,
                }
            )
    alpha = float(model["calibration"]["alpha"])
    rank = int(math.ceil((len(nonconformity) + 1) * (1.0 - alpha)))
    rank = min(max(rank, 1), len(nonconformity))
    radius = sorted(nonconformity)[rank - 1]
    model["calibration"]["radius_m"] = round(float(radius), 12)
    model["calibration"]["sample_count"] = len(nonconformity)
    model["calibration"]["finite_sample_rank"] = rank

    covered = []
    widths = []
    horizon_coverage = {}
    for point in calibration_points:
        lower = point["lower_before_calibration"] - radius
        upper = point["upper_before_calibration"] + radius
        is_covered = lower - 1e-12 <= point["truth"] <= upper + 1e-12
        covered.append(is_covered)
        widths.append(upper - lower)
        horizon_coverage.setdefault(point["horizon"], []).append(is_covered)
    marginal_coverage = sum(covered) / len(covered)
    per_horizon = {
        horizon: sum(values) / len(values)
        for horizon, values in horizon_coverage.items()
    }
    sensitivities = []
    for block_id in sorted(
        {item["row"]["block_id"] for item in calibration}
    ):
        block = [item for item in calibration if item["row"]["block_id"] == block_id]
        for policy in POLICIES:
            values = [
                prediction(
                    model,
                    policy,
                    3,
                    model_features(v, item["metrics"]),
                )
                for item in block
                if item["row"]["actor_policy_evaluator_only"] == policy
            ]
            sensitivities.append(max(values) - min(values))
    actual_mode_mae = sum(mode_errors) / len(mode_errors)
    mean_width = sum(widths) / len(widths)
    minimum_sensitivity = min(sensitivities)
    gate_values = {
        "training_episode_count_pass": len(training)
        == int(gates["training_episode_count"]),
        "calibration_episode_count_pass": len(calibration)
        == int(gates["calibration_episode_count"]),
        "model_head_count_pass": len(POLICIES) * len(HORIZONS)
        == int(gates["model_head_count"]),
        "marginal_coverage_pass": marginal_coverage
        >= float(gates["calibration_marginal_coverage_min"]),
        "per_horizon_coverage_pass": min(per_horizon.values())
        >= float(gates["calibration_per_horizon_coverage_min"]),
        "mean_width_pass": mean_width
        <= float(gates["calibration_mean_full_width_max_m"]),
        "actual_mode_mae_pass": actual_mode_mae
        <= float(gates["actual_mode_mae_max_m"]),
        "candidate_sensitivity_pass": minimum_sensitivity
        >= float(gates["candidate_sensitivity_min_m"]),
        "hidden_label_runtime_firewall_pass": model[
            "runtime_hidden_policy_input"
        ]
        is False
        and model["runtime_outputs_all_mode_heads"] is True,
    }
    prefix_coordinate_spreads = []
    prefix_speed_spreads = []
    for block_id in sorted({item["row"]["block_id"] for item in episodes}):
        for policy in POLICIES:
            group = [
                item
                for item in episodes
                if item["row"]["block_id"] == block_id
                and item["row"]["actor_policy_evaluator_only"] == policy
            ]
            coordinates = [
                item["metrics"]["actor_conflict_coordinate_at_observation_m"]
                for item in group
            ]
            speeds = [
                item["metrics"]["actor_observed_speed_mps"] for item in group
            ]
            prefix_coordinate_spreads.append(max(coordinates) - min(coordinates))
            prefix_speed_spreads.append(max(speeds) - min(speeds))
    maximum_prefix_coordinate_spread = max(prefix_coordinate_spreads)
    maximum_prefix_speed_spread = max(prefix_speed_spreads)
    if "common_prefix_actor_coordinate_spread_max_m" in gates:
        gate_values["common_prefix_actor_coordinate_alignment_pass"] = (
            maximum_prefix_coordinate_spread
            <= float(gates["common_prefix_actor_coordinate_spread_max_m"])
        )
        gate_values["common_prefix_actor_speed_alignment_pass"] = (
            maximum_prefix_speed_spread
            <= float(gates["common_prefix_actor_speed_spread_max_mps"])
        )
    overall = all(gate_values.values())
    model["status"] = (
        "candidate_conditioned_predictor_calibrated"
        if overall
        else "candidate_conditioned_predictor_calibration_failed"
    )
    report = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "report_id": "{}_calibration_report_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_predictor_calibration_pass".format(
                contract["stage"].lower().replace("-", "_")
            )
            if overall
            else "{}_predictor_calibration_fail".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "training_episode_count": len(training),
        "calibration_episode_count": len(calibration),
        "calibration_point_count": len(calibration_points),
        "conformal_radius_m": radius,
        "marginal_coverage": marginal_coverage,
        "per_horizon_coverage": per_horizon,
        "mean_full_envelope_width_m": mean_width,
        "actual_mode_mae_m": actual_mode_mae,
        "minimum_candidate_sensitivity_m": minimum_sensitivity,
        "maximum_common_prefix_actor_coordinate_spread_m": (
            maximum_prefix_coordinate_spread
        ),
        "maximum_common_prefix_actor_speed_spread_mps": (
            maximum_prefix_speed_spread
        ),
        "gates": gate_values,
        "overall_pass": overall,
        "held_out_test_accessed": False,
        "reserved_test_seeds": contract["authority_boundary"][
            "held_out_test_seeds_reserved"
        ],
        "claim_scope": "parameterized_crossing_train_calibration_only",
    }
    return model, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    model, report = fit(v)
    model_bytes = v.yaml_bytes(model)
    report_bytes = v.yaml_bytes(report)
    outputs = {
        v.MODEL_RELATIVE: model_bytes,
        v.CALIBRATION_RELATIVE: report_bytes,
    }
    for relative, payload in outputs.items():
        path = W / relative
        if args.check:
            v.require(path.is_file() and path.read_bytes() == payload, "model differs")
        else:
            write_once(path, payload)
    if not args.check:
        schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
        resources = [
            v.SCHEDULE_RELATIVE,
            v.JOURNAL_RELATIVE,
            v.MODEL_RELATIVE,
            v.CALIBRATION_RELATIVE,
        ]
        if hasattr(v, "DATA_QUALIFICATION_RELATIVE"):
            resources.append(v.DATA_QUALIFICATION_RELATIVE)
        resources.extend(getattr(v, "REPAIR_SOURCE_RESOURCES", ()))
        for row in schedule["rows"]:
            root = v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
            resources.extend(
                [root / "raw_pose_trace.csv", root / "attempt_receipt.yaml"]
            )
        freeze = ("\n".join(
            "{}  {}".format(
                v.sha256_bytes(v.read_file_once(W / item)), item.as_posix()
            )
            for item in resources
        ) + "\n").encode("utf-8")
        write_once(W / v.MODEL_FREEZE_RELATIVE, freeze)
    print(
        "status={} overall_pass={} model_sha256={} report_sha256={}".format(
            report["status"],
            report["overall_pass"],
            v.sha256_bytes(model_bytes),
            v.sha256_bytes(report_bytes),
        )
    )


if __name__ == "__main__":
    main()
