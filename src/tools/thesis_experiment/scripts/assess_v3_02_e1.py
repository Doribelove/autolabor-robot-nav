#!/usr/bin/env python3
"""Assess the frozen observed-mode predictor on fresh V3-02-E1 data."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_e1.py"
FIT_PATH = W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

HORIZONS = ("t1_5", "t2_5", "t3_5", "t4_5")
POLICIES = ("cautious", "assertive", "hesitant")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def mode_configuration(contract, planner_contract):
    mode = planner_contract["predictor_runtime"]["mode_inference"]
    declared = contract["observed_mode_gate"]
    for key in (
        "mode_inference_id",
        "cautious_hesitant_threshold_mps",
        "hesitant_assertive_threshold_mps",
        "ambiguity_half_width_mps",
    ):
        if isinstance(mode[key], float):
            core.require(
                abs(float(mode[key]) - float(declared[key])) <= 1e-12,
                "mode gate binding differs: {}".format(key),
            )
        else:
            core.require(
                mode[key] == declared[key],
                "mode gate binding differs: {}".format(key),
            )
    core.require(
        mode["hidden_policy_runtime_input"] is False,
        "hidden policy runtime input enabled",
    )
    return mode


def feasibility_metrics(v, contract, episodes):
    maximum_trace_error = 0.0
    maximum_acceleration = 0.0
    maximum_deceleration = 0.0
    stop_terminal_error = 0.0
    stop_post_terminal_motion = 0.0
    for episode in episodes:
        row = episode["row"]
        root = W / v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        trace = v.load_trace(root / "raw_pose_trace.csv")
        points = v.ego_waypoints(contract, row)
        trace_error = max(
            math.hypot(
                raw["ego_x_m"]
                - float(v.interpolate_plan(points, raw["sim_time_s"])[0]),
                raw["ego_y_m"]
                - float(v.interpolate_plan(points, raw["sim_time_s"])[1]),
            )
            for raw in trace
        )
        kinematics = v.planned_kinematics(contract, row)
        maximum_trace_error = max(maximum_trace_error, trace_error)
        maximum_acceleration = max(
            maximum_acceleration, kinematics["maximum_acceleration_mps2"]
        )
        maximum_deceleration = max(
            maximum_deceleration, kinematics["maximum_deceleration_mps2"]
        )
        if row["certified_stop"]:
            stop_terminal_error = max(
                stop_terminal_error,
                abs(trace[-1]["ego_x_m"] - kinematics["terminal_x_m"]),
            )
            post = [
                item["ego_x_m"]
                for item in trace
                if item["sim_time_s"] >= 1.5 - 1e-9
            ]
            stop_post_terminal_motion = max(
                stop_post_terminal_motion, max(post) - min(post)
            )
    return {
        "maximum_ego_trace_position_error_m": maximum_trace_error,
        "maximum_planned_acceleration_mps2": maximum_acceleration,
        "maximum_planned_deceleration_mps2": maximum_deceleration,
        "maximum_stop_terminal_position_error_m": stop_terminal_error,
        "maximum_stop_terminal_motion_after_1_5_s_m": (
            stop_post_terminal_motion
        ),
    }


def assess(v, fitter):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    preregistration = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, preregistration)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    model_path = W / contract["frozen_model"]["path"]
    model_bytes = v.read_file_once(model_path)
    v.require(
        v.sha256_bytes(model_bytes) == contract["frozen_model"]["sha256"],
        "frozen model changed",
    )
    model = v.load_yaml_path(model_path)
    planner_path = W / contract["observed_mode_gate"][
        "effective_planner_contract_path"
    ]
    planner_bytes = v.read_file_once(planner_path)
    v.require(
        v.sha256_bytes(planner_bytes)
        == contract["observed_mode_gate"][
            "effective_planner_contract_sha256"
        ],
        "effective planner contract changed",
    )
    planner_contract = v.load_yaml_path(planner_path)
    mode_gate = mode_configuration(contract, planner_contract)
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        len([row for row in journal if row["event"] == "unit_complete"])
        == int(schedule["row_count"])
        and journal[-1]["event"] == "execution_complete",
        "test journal incomplete",
    )
    radius = float(model["calibration"]["radius_m"])
    margin = float(
        planner_contract["predictor_runtime"][
            "additional_coordinate_margin_m"
        ]
    )
    episodes = []
    covered = []
    widths = []
    mode_errors = []
    per_horizon = {horizon: [] for horizon in HORIZONS}
    physical_violations = 0
    classifier_correct = 0
    ambiguous_count = 0
    for row in schedule["rows"]:
        root = W / v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt = v.load_yaml_path(root / "attempt_receipt.yaml")
        raw = v.read_file_once(root / "raw_pose_trace.csv")
        v.require(
            receipt["raw_csv_sha256"] == v.sha256_bytes(raw),
            "raw hash differs",
        )
        trace = v.load_trace(root / "raw_pose_trace.csv")
        metrics = v.trace_metrics(trace, row)
        v.require(metrics == receipt["metrics_preview"], "metric replay differs")
        features = v.model_features(metrics)
        try:
            inferred = core.infer_observed_mode(
                float(features[2]), mode_gate
            )
        except core.PlannerError:
            inferred = ""
            ambiguous_count += 1
        label = row["actor_policy_evaluator_only"]
        classifier_correct += inferred == label
        points = []
        for horizon_index, horizon in enumerate(HORIZONS):
            prediction = (
                fitter.prediction(
                    model, inferred, horizon_index, features
                )
                if inferred
                else float("nan")
            )
            truth = float(
                metrics["target_actor_conflict_coordinates_m"][
                    horizon_index
                ]
            )
            physical_ok = (
                math.isfinite(prediction)
                and prediction >= float(features[1]) - 1e-12
                and prediction <= 3.5 + 1e-12
            )
            physical_violations += not physical_ok
            lower = prediction - radius - margin
            upper = prediction + radius + margin
            hit = (
                math.isfinite(lower)
                and lower - 1e-12 <= truth <= upper + 1e-12
            )
            covered.append(hit)
            per_horizon[horizon].append(hit)
            widths.append(upper - lower)
            if math.isfinite(prediction):
                mode_errors.append(abs(prediction - truth))
            points.append(
                {
                    "horizon": horizon,
                    "truth_m": truth,
                    "inferred_mode": inferred,
                    "prediction_m": prediction,
                    "lower_m": lower,
                    "upper_m": upper,
                    "covered": hit,
                }
            )
        episodes.append(
            {
                "row": dict(row),
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "features": features,
                "inferred_mode": inferred,
                "mode_correct": inferred == label,
                "points": points,
            }
        )
    sensitivities = []
    for block_id in sorted({row["block_id"] for row in schedule["rows"]}):
        for policy in POLICIES:
            values = [
                episode["points"][3]["prediction_m"]
                for episode in episodes
                if episode["row"]["block_id"] == block_id
                and episode["row"]["actor_policy_evaluator_only"] == policy
            ]
            sensitivities.append(max(values) - min(values))
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
            prefix_coordinate_spreads.append(
                max(item["features"][1] for item in group)
                - min(item["features"][1] for item in group)
            )
            prefix_speed_spreads.append(
                max(item["features"][2] for item in group)
                - min(item["features"][2] for item in group)
            )
    metrics = {
        "classifier_correct_count": classifier_correct,
        "classifier_accuracy": classifier_correct / len(episodes),
        "ambiguous_episode_count": ambiguous_count,
        "marginal_coverage": sum(covered) / len(covered),
        "per_horizon_coverage": {
            horizon: sum(values) / len(values)
            for horizon, values in per_horizon.items()
        },
        "mean_full_envelope_width_m": sum(widths) / len(widths),
        "actual_mode_mae_m": sum(mode_errors) / len(mode_errors),
        "minimum_candidate_sensitivity_m": min(sensitivities),
        "physical_coordinate_violation_count": physical_violations,
        "maximum_common_prefix_actor_coordinate_spread_m": max(
            prefix_coordinate_spreads
        ),
        "maximum_common_prefix_actor_speed_spread_mps": max(
            prefix_speed_spreads
        ),
    }
    metrics.update(feasibility_metrics(v, contract, episodes))
    gates = contract["heldout_gates"]
    gate_values = {
        "episode_count_pass": len(episodes) == int(gates["episode_count"]),
        "point_count_pass": len(covered) == int(gates["point_count"]),
        "classifier_accuracy_pass": metrics["classifier_accuracy"]
        >= float(gates["classifier_accuracy_min"]),
        "ambiguity_pass": metrics["ambiguous_episode_count"]
        <= int(gates["ambiguous_episode_count_max"]),
        "marginal_coverage_pass": metrics["marginal_coverage"]
        >= float(gates["marginal_coverage_min"]),
        "per_horizon_coverage_pass": min(
            metrics["per_horizon_coverage"].values()
        )
        >= float(gates["per_horizon_coverage_min"]),
        "mean_width_pass": metrics["mean_full_envelope_width_m"]
        <= float(gates["mean_full_envelope_width_max_m"]),
        "actual_mode_mae_pass": metrics["actual_mode_mae_m"]
        <= float(gates["actual_mode_mae_max_m"]),
        "candidate_sensitivity_pass": metrics[
            "minimum_candidate_sensitivity_m"
        ]
        >= float(gates["candidate_sensitivity_min_m"]),
        "physical_coordinate_pass": physical_violations
        <= int(gates["physical_coordinate_violation_count_max"]),
        "hidden_policy_runtime_firewall_pass": mode_gate[
            "hidden_policy_runtime_input"
        ]
        is False,
        "common_prefix_actor_coordinate_alignment_pass": metrics[
            "maximum_common_prefix_actor_coordinate_spread_m"
        ]
        <= float(gates["common_prefix_actor_coordinate_spread_max_m"]),
        "common_prefix_actor_speed_alignment_pass": metrics[
            "maximum_common_prefix_actor_speed_spread_mps"
        ]
        <= float(gates["common_prefix_actor_speed_spread_max_mps"]),
        "ego_trace_conformity_pass": metrics[
            "maximum_ego_trace_position_error_m"
        ]
        <= float(gates["maximum_ego_trace_position_error_m"]),
        "planned_acceleration_pass": metrics[
            "maximum_planned_acceleration_mps2"
        ]
        <= float(gates["maximum_planned_acceleration_mps2"]),
        "planned_deceleration_pass": metrics[
            "maximum_planned_deceleration_mps2"
        ]
        <= float(gates["maximum_planned_deceleration_mps2"]),
        "stop_terminal_position_pass": metrics[
            "maximum_stop_terminal_position_error_m"
        ]
        <= float(gates["stop_terminal_position_error_m"]),
        "stop_terminal_hold_pass": metrics[
            "maximum_stop_terminal_motion_after_1_5_s_m"
        ]
        <= float(gates["stop_terminal_motion_after_1_5_s_m"]),
    }
    overall = all(gate_values.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-02-E1",
        "assessment_id": "v3_02_e1_heldout_assessment_1",
        "status": (
            "v3_02_e1_observed_mode_heldout_predictor_pass"
            if overall
            else "v3_02_e1_observed_mode_heldout_predictor_fail"
        ),
        "formal_predictor_result": True,
        "frozen_model_sha256": contract["frozen_model"]["sha256"],
        "frozen_mode_gate_sha256": contract["observed_mode_gate"][
            "effective_planner_contract_sha256"
        ],
        "execution": {
            "authorized_units": schedule["row_count"],
            "consumed_units": schedule["row_count"],
            "completed_units": len(episodes),
            "retry_units": 0,
        },
        "metrics": metrics,
        "gates": gate_values,
        "overall_pass": overall,
        "model_or_gate_changed_after_test": False,
        "episodes": episodes,
        "claim": {
            "fresh_observed_mode_predictor_heldout_pass": overall,
            "general_mode_identification": False,
            "general_scene_prediction": False,
            "planner_safety": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_02_e1")
    fitter = load(FIT_PATH, "v3_02_e1_fitter")
    value = assess(v, fitter)
    payload = v.yaml_bytes(value)
    output = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "assessment differs",
        )
    else:
        write_once(output, payload)
        resources = [
            v.JOURNAL_RELATIVE,
            v.ASSESSMENT_RELATIVE,
            Path(v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                "frozen_model"
            ]["path"]),
            Path(v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                "observed_mode_gate"
            ]["effective_planner_contract_path"]),
        ]
        for row in v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)["rows"]:
            root = v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
            resources.extend(
                [root / "raw_pose_trace.csv", root / "attempt_receipt.yaml"]
            )
        freeze = (
            "\n".join(
                "{}  {}".format(
                    v.sha256_bytes(v.read_file_once(W / item)),
                    item.as_posix(),
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} overall_pass={} assessment_sha256={}".format(
            value["status"], value["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
