#!/usr/bin/env python3
"""Assess the frozen V3-02 predictor on held-out test blocks."""

import argparse
import importlib.util
import json
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"
FIT_PATH = W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"
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


def assess(v, fitter):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    model_path = W / contract["frozen_model"]["path"]
    model_bytes = v.read_file_once(model_path)
    v.require(
        v.sha256_bytes(model_bytes) == contract["frozen_model"]["sha256"],
        "frozen model changed",
    )
    model = v.load_yaml_path(model_path)
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
    episodes = []
    covered = []
    widths = []
    mode_errors = []
    physical_violations = 0
    per_horizon = {horizon: [] for horizon in HORIZONS}
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
        features = (
            v.model_features(metrics)
            if hasattr(v, "model_features")
            else metrics["feature_vector"]
        )
        points = []
        for horizon_index, horizon in enumerate(HORIZONS):
            modes = {
                policy: fitter.prediction(
                    model, policy, horizon_index, features
                )
                for policy in POLICIES
            }
            physical_violations += sum(
                value < features[1] - 1e-12 or value > 3.5 + 1e-12
                for value in modes.values()
            )
            truth = metrics["target_actor_conflict_coordinates_m"][
                horizon_index
            ]
            lower = min(modes.values()) - radius
            upper = max(modes.values()) + radius
            hit = lower - 1e-12 <= truth <= upper + 1e-12
            covered.append(hit)
            per_horizon[horizon].append(hit)
            widths.append(upper - lower)
            mode_errors.append(
                abs(modes[row["actor_policy_evaluator_only"]] - truth)
            )
            points.append(
                {
                    "horizon": horizon,
                    "truth_m": truth,
                    "mode_predictions_m": modes,
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
                "points": points,
            }
        )
    sensitivities = []
    for block_id in sorted({row["block_id"] for row in schedule["rows"]}):
        for policy in POLICIES:
            values = []
            for episode in episodes:
                if (
                    episode["row"]["block_id"] == block_id
                    and episode["row"]["actor_policy_evaluator_only"] == policy
                ):
                    values.append(
                        episode["points"][3]["mode_predictions_m"][policy]
                    )
            sensitivities.append(max(values) - min(values))
    metrics = {
        "marginal_coverage": sum(covered) / len(covered),
        "per_horizon_coverage": {
            horizon: sum(values) / len(values)
            for horizon, values in per_horizon.items()
        },
        "mean_full_envelope_width_m": sum(widths) / len(widths),
        "actual_mode_mae_m": sum(mode_errors) / len(mode_errors),
        "minimum_candidate_sensitivity_m": min(sensitivities),
        "physical_coordinate_violation_count": physical_violations,
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
            prefix_coordinate_spreads.append(
                max(item["features"][1] for item in group)
                - min(item["features"][1] for item in group)
            )
            prefix_speed_spreads.append(
                max(item["features"][2] for item in group)
                - min(item["features"][2] for item in group)
            )
    metrics["maximum_common_prefix_actor_coordinate_spread_m"] = max(
        prefix_coordinate_spreads
    )
    metrics["maximum_common_prefix_actor_speed_spread_mps"] = max(
        prefix_speed_spreads
    )
    gates = contract["heldout_gates"]
    gate_values = {
        "episode_count_pass": len(episodes) == int(gates["episode_count"]),
        "point_count_pass": len(covered) == int(gates["point_count"]),
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
        "hidden_policy_runtime_firewall_pass": model[
            "runtime_hidden_policy_input"
        ]
        is False,
    }
    if "common_prefix_actor_coordinate_spread_max_m" in gates:
        gate_values["common_prefix_actor_coordinate_alignment_pass"] = (
            metrics["maximum_common_prefix_actor_coordinate_spread_m"]
            <= float(gates["common_prefix_actor_coordinate_spread_max_m"])
        )
        gate_values["common_prefix_actor_speed_alignment_pass"] = (
            metrics["maximum_common_prefix_actor_speed_spread_mps"]
            <= float(gates["common_prefix_actor_speed_spread_max_mps"])
        )
    if hasattr(v, "heldout_additional_assessment"):
        additional = v.heldout_additional_assessment(
            contract, schedule, episodes
        )
        metrics.update(additional["metrics"])
        gate_values.update(additional["gates"])
    overall = all(gate_values.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "assessment_id": "{}_heldout_assessment_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_heldout_predictor_pass".format(
                contract["stage"].lower().replace("-", "_")
            )
            if overall
            else "{}_heldout_predictor_fail".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "formal_predictor_result": True,
        "frozen_model_sha256": contract["frozen_model"]["sha256"],
        "execution": {
            "authorized_units": schedule["row_count"],
            "consumed_units": schedule["row_count"],
            "completed_units": len(episodes),
            "retry_units": 0,
        },
        "metrics": metrics,
        "gates": gate_values,
        "overall_pass": overall,
        "model_refit_or_recalibrated_after_test": False,
        "episodes": episodes,
        "claim": {
            "parameterized_crossing_predictor_heldout_pass": overall,
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
    v = load(V3_PATH, "v3_02_t1")
    fitter = load(FIT_PATH, "v3_02_fitter")
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    value = assess(v, fitter)
    payload = v.yaml_bytes(value)
    output = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(output.is_file() and output.read_bytes() == payload, "assessment differs")
    else:
        write_once(output, payload)
        resources = [
            v.JOURNAL_RELATIVE,
            v.ASSESSMENT_RELATIVE,
            Path(contract["frozen_model"]["path"]),
        ]
        for row in v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)["rows"]:
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
        write_once(
            W / v.EVIDENCE_FREEZE_RELATIVE,
            freeze,
        )
    print(
        "status={} overall_pass={} assessment_sha256={}".format(
            value["status"], value["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
