#!/usr/bin/env python3
"""Assess frozen V3-07-B0-R1 predictor on fresh B1 held-out traces."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_b1.py"


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


def trace(path):
    with path.open(newline="") as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def features(metrics):
    coordinate = float(metrics["actor_conflict_coordinate_at_observation_m"])
    speed = float(metrics["actor_observed_speed_mps"])
    return [1.0, coordinate, speed, coordinate * coordinate, speed * speed,
            coordinate * speed]


def predictions(model, metrics):
    head = model["supported_action_heads"][metrics["observed_mode"]][
        metrics["candidate_id"]
    ]
    x = features(metrics)
    return [
        sum(a * b for a, b in zip(head[key]["coefficients"], x))
        for key in ("t1_5", "t2_5", "t3_5", "t4_5")
    ]


def assess(module):
    contract = module.load_yaml_relative(W, module.CONTRACT_RELATIVE)
    preregistration = module.load_yaml_relative(W, module.PREREGISTRATION_RELATIVE)
    module.validate_contract(contract, preregistration)
    schedule = module.load_yaml_relative(W, module.SCHEDULE_RELATIVE)
    module.require(schedule == module.exact_schedule(contract), "B1 schedule differs")
    model_bytes = module.read_file_once(W / module.MODEL_RELATIVE)
    module.require(
        module.sha256_bytes(model_bytes) == contract["upstream"]["frozen_model_sha256"],
        "B1 model differs",
    )
    model = module.load_yaml_relative(W, module.MODEL_RELATIVE)
    journal = [
        json.loads(line)
        for line in (W / module.JOURNAL_RELATIVE).read_text().splitlines()
        if line.strip()
    ]
    module.require(
        journal[-1]["event"] == "execution_complete"
        and len([x for x in journal if x["event"] == "unit_complete"]) == 20,
        "B1 journal incomplete",
    )
    rules = contract["heldout_qualification_gates"]
    collision_radius = float(rules["conservative_combined_radius_m"])
    interval_radius = float(model["calibration"]["radius_m"])
    head_counts = Counter()
    head_covered = Counter()
    horizon_covered = [0, 0, 0, 0]
    scores = []
    absolute_errors = []
    mode_matches = 0
    prefix_error = 0.0
    minimum_clearance = float("inf")
    clearance_violations = 0
    nonfinite = 0
    evidence = [module.JOURNAL_RELATIVE, module.MODEL_RELATIVE]
    for row in schedule["rows"]:
        root = module.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt_path = root / "attempt_receipt.yaml"
        raw_path = root / "raw_pose_trace.csv"
        world_path = root / "world.sdf"
        receipt = module.load_yaml_relative(W, receipt_path)
        raw_bytes = module.read_file_once(W / raw_path)
        module.require(
            receipt["row"] == row
            and receipt["raw_csv_sha256"] == module.sha256_bytes(raw_bytes),
            "B1 receipt differs",
        )
        rows = trace(W / raw_path)
        metrics = module.trace_metrics(rows, row)
        module.require(receipt["metrics_preview"] == metrics, "B1 preview differs")
        mode_matches += metrics["observed_mode"] == row["actor_policy_evaluator_only"]
        key = (metrics["observed_mode"], metrics["candidate_id"])
        predicted = predictions(model, metrics)
        observed = [float(x) for x in metrics["target_actor_conflict_coordinates_m"]]
        errors = [abs(a - b) for a, b in zip(observed, predicted)]
        score = max(errors)
        head_counts[key] += 1
        head_covered[key] += score <= interval_radius + 1.0e-12
        scores.append(score)
        absolute_errors.extend(errors)
        for index, error in enumerate(errors):
            horizon_covered[index] += error <= interval_radius + 1.0e-12
        prefix_error = max(
            prefix_error,
            math.hypot(
                module.interpolate(rows, "ego_x_m", 0.5) + 2.6,
                module.interpolate(rows, "ego_y_m", 0.5),
            ),
        )
        clearance = min(
            math.hypot(
                sample["ego_x_m"] - sample["actor_x_m"],
                sample["ego_y_m"] - sample["actor_y_m"],
            ) - collision_radius
            for sample in rows
        )
        minimum_clearance = min(minimum_clearance, clearance)
        clearance_violations += clearance < 0.0
        finite_values = (
            [metrics["actor_conflict_coordinate_at_observation_m"],
             metrics["actor_observed_speed_mps"], clearance, score]
            + observed + predicted + errors
        )
        nonfinite += sum(not math.isfinite(float(value)) for value in finite_values)
        evidence.extend((receipt_path, raw_path, world_path))
    eligible = [
        ("cautious", "certified_stop_v000"),
        ("cautious", "spatial_left_v120"),
        ("hesitant", "certified_stop_v000"),
        ("hesitant", "spatial_left_v120"),
        ("assertive", "certified_stop_v000"),
    ]
    per_head_coverage = {
        "{}__{}".format(*key): head_covered[key] / float(head_counts[key])
        for key in eligible
    }
    metrics = {
        "completed_rows": 20,
        "observed_mode_matches_evaluator_rate": mode_matches / 20.0,
        "eligible_head_count": len(eligible),
        "minimum_rows_per_eligible_head": min(head_counts[key] for key in eligible),
        "assertive_spatial_row_count": sum(
            value for (mode, candidate), value in head_counts.items()
            if mode == "assertive" and candidate == "spatial_left_v120"
        ),
        "interval_radius_m": interval_radius,
        "interval_coverage_rate": sum(score <= interval_radius + 1.0e-12 for score in scores)
        / 20.0,
        "per_head_interval_coverage_rate": per_head_coverage,
        "minimum_per_head_interval_coverage_rate": min(per_head_coverage.values()),
        "per_horizon_interval_coverage_rate": [
            value / 20.0 for value in horizon_covered
        ],
        "maximum_row_score_m": max(scores),
        "mean_absolute_error_m": sum(absolute_errors) / len(absolute_errors),
        "maximum_absolute_error_m": max(absolute_errors),
        "common_prefix_position_error_max_m": prefix_error,
        "minimum_conservative_clearance_m": minimum_clearance,
        "conservative_clearance_violation_count": clearance_violations,
        "nonfinite_metric_count": nonfinite,
    }
    gates = {
        "completed_pass": metrics["completed_rows"] == rules["completed_rows"],
        "mode_pass": metrics["observed_mode_matches_evaluator_rate"]
        >= float(rules["observed_mode_matches_evaluator_rate_min"]),
        "head_count_pass": metrics["eligible_head_count"] == rules["eligible_head_count"]
        and metrics["minimum_rows_per_eligible_head"]
        == rules["rows_per_eligible_head"],
        "assertive_fail_closed_pass": metrics["assertive_spatial_row_count"]
        <= rules["assertive_spatial_row_count_max"],
        "global_coverage_pass": metrics["interval_coverage_rate"]
        >= float(rules["interval_coverage_rate_min"]),
        "per_head_coverage_pass": metrics["minimum_per_head_interval_coverage_rate"]
        >= float(rules["per_head_interval_coverage_rate_min"]),
        "prefix_pass": prefix_error <= float(rules["common_prefix_position_error_max_m"]),
        "clearance_pass": clearance_violations
        <= rules["conservative_clearance_violation_count_max"],
        "finite_pass": nonfinite <= rules["nonfinite_metric_count_max"],
    }
    passed = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-B1",
        "status": (
            "spatial_predictor_heldout_qualification_pass"
            if passed else "spatial_predictor_heldout_qualification_fail"
        ),
        "formal_result": False,
        "execution": {
            "authorized_units": 20, "consumed_units": 20,
            "completed_units": 20, "retry_units": 0,
        },
        "frozen_model_sha256": contract["upstream"]["frozen_model_sha256"],
        "metrics": metrics,
        "gates": gates,
        "qualified_for_spatial_safe_planner_integration": passed,
        "performance_claim_authorized": False,
        "runtime_ready": False,
        "real_vehicle_authorized": False,
        "overall_pass": passed,
        "_evidence": evidence,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    module = load(MODULE, "v3_07_b1_assessment")
    value = assess(module)
    evidence = value.pop("_evidence")
    payload = module.yaml_bytes(value)
    output = W / module.ASSESSMENT_RELATIVE
    if args.check:
        module.require(output.is_file() and output.read_bytes() == payload,
                       "B1 assessment differs")
    else:
        write_once(output, payload)
        freeze = (
            "\n".join(
                "{}  {}".format(
                    module.sha256_bytes(module.read_file_once(W / item)), item
                )
                for item in evidence + [module.ASSESSMENT_RELATIVE]
            ) + "\n"
        ).encode()
        write_once(W / module.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} qualified={} assessment_sha256={}".format(
            value["status"],
            value["qualified_for_spatial_safe_planner_integration"],
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
