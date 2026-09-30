#!/usr/bin/env python3
"""Assess V3-07-A3 observed-mode-gated safe overlap."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"


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


def assess(m):
    contract = m.load_yaml_relative(W, m.CONTRACT_RELATIVE)
    prereg = m.load_yaml_relative(W, m.PREREGISTRATION_RELATIVE)
    m.validate_contract(contract, prereg)
    schedule = m.load_yaml_relative(W, m.SCHEDULE_RELATIVE)
    m.require(schedule == m.exact_schedule(contract), "A3 schedule differs")
    journal = [
        json.loads(line)
        for line in (W / m.JOURNAL_RELATIVE).read_text().splitlines()
        if line.strip()
    ]
    m.require(
        journal[-1]["event"] == "execution_complete"
        and len([row for row in journal if row["event"] == "unit_complete"]) == 48,
        "A3 journal incomplete",
    )
    rules = contract["qualification_gates"]
    radius = float(rules["conservative_combined_radius_m"])
    groups = {}
    evidence = [m.JOURNAL_RELATIVE]
    mode_matches = 0
    speed_differences = []
    minimum_clearance = float("inf")
    clearance_violations = 0
    assertive_spatial = 0
    spatial_goals = 0
    spatial_units = 0
    nonfinite = 0
    for row in schedule["rows"]:
        root = m.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt_path = root / "attempt_receipt.yaml"
        raw_path = root / "raw_pose_trace.csv"
        world_path = root / "world.sdf"
        receipt = m.load_yaml_relative(W, receipt_path)
        raw = m.read_file_once(W / raw_path)
        m.require(
            receipt["row"] == row
            and receipt["runtime"]["raw_csv_sha256"] == m.sha256_bytes(raw)
            and receipt["selector_input"]["hidden_policy_used"] is False,
            "A3 receipt differs",
        )
        rows = trace(W / raw_path)
        key = "{}:{}".format(row["block_id"], row["actor_policy_evaluator_only"])
        groups.setdefault(key, {})[row["phase"]] = receipt
        inferred = receipt["selector_input"]["inferred_mode"]
        mode_matches += inferred == row["actor_policy_evaluator_only"]
        if row["phase"] != "probe":
            probe_speed = float(receipt["selector_input"]["actor_observed_speed_mps"])
            treatment_speed = float(receipt["treatment_prefix_observed_speed_mps"])
            speed_differences.append(abs(probe_speed - treatment_speed))
            clearance = min(
                math.hypot(
                    item["ego_x_m"] - item["actor_x_m"],
                    item["ego_y_m"] - item["actor_y_m"],
                )
                - radius
                for item in rows
            )
            minimum_clearance = min(minimum_clearance, clearance)
            clearance_violations += clearance < 0.0
            if receipt["runtime_candidate_id"] == "spatial_left_v120":
                spatial_units += 1
                spatial_goals += float(rows[-1]["ego_x_m"]) >= float(
                    rules["spatial_goal_x_m"]
                )
                assertive_spatial += inferred == "assertive"
            nonfinite += sum(
                not math.isfinite(value)
                for value in (
                    clearance,
                    probe_speed,
                    treatment_speed,
                    rows[-1]["ego_x_m"],
                    rows[-1]["ego_y_m"],
                )
            )
        evidence.extend((receipt_path, raw_path, world_path))
    eligible_overlap = []
    for key, phases in groups.items():
        policy = key.split(":")[1]
        expected = (
            {"probe", "certified_stop", "conditional_spatial_left"}
            if policy in ("cautious", "hesitant")
            else {"probe", "certified_stop"}
        )
        eligible_overlap.append(set(phases) == expected)
    metrics = {
        "completed_unit_count": 48,
        "observed_mode_matches_evaluator_rate": mode_matches / 48.0,
        "maximum_probe_treatment_observed_speed_difference_mps": max(speed_differences),
        "eligible_mode_action_overlap_rate": sum(eligible_overlap) / len(eligible_overlap),
        "assertive_spatial_execution_count": assertive_spatial,
        "spatial_unit_count": spatial_units,
        "eligible_spatial_goal_rate": spatial_goals / spatial_units,
        "minimum_conservative_clearance_m": minimum_clearance,
        "conservative_clearance_violation_count": clearance_violations,
        "nonfinite_metric_count": nonfinite,
    }
    gates = {
        "completed_pass": metrics["completed_unit_count"] == int(rules["completed_unit_count"]),
        "mode_inference_pass": metrics["observed_mode_matches_evaluator_rate"]
        >= float(rules["observed_mode_matches_evaluator_rate_min"]),
        "clone_repeatability_pass": metrics[
            "maximum_probe_treatment_observed_speed_difference_mps"
        ]
        <= float(rules["probe_treatment_observed_speed_difference_max_mps"]),
        "eligible_overlap_pass": metrics["eligible_mode_action_overlap_rate"]
        >= float(rules["eligible_mode_action_overlap_rate_min"]),
        "assertive_fail_closed_pass": metrics["assertive_spatial_execution_count"]
        <= int(rules["assertive_spatial_execution_count_max"]),
        "clearance_pass": metrics["conservative_clearance_violation_count"]
        <= int(rules["conservative_clearance_violation_count_max"])
        and metrics["minimum_conservative_clearance_m"]
        >= float(rules["minimum_conservative_clearance_m_min"]),
        "spatial_goal_pass": metrics["eligible_spatial_goal_rate"]
        >= float(rules["eligible_spatial_goal_rate_min"]),
        "finite_pass": metrics["nonfinite_metric_count"]
        <= int(rules["nonfinite_metric_count_max"]),
    }
    passed = all(gates.values())
    return {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-07-A3",
        "status": (
            "mode_gated_safe_spatial_overlap_qualification_pass"
            if passed else "mode_gated_safe_spatial_overlap_qualification_fail"
        ),
        "formal_result": False,
        "execution": {
            "authorized_units": 48, "consumed_units": 48,
            "completed_units": 48, "retry_units": 0,
        },
        "metrics": metrics, "gates": gates,
        "qualified_for_spatial_predictor_data_design": passed,
        "overall_pass": passed,
        "claim": {
            "formal_performance_improvement": False,
            "runtime_safety_qualified": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "_evidence": evidence,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    m = load(MODULE, "v3_07_a3_assessor_module")
    value = assess(m)
    evidence = value.pop("_evidence")
    payload = m.yaml_bytes(value)
    output = W / m.ASSESSMENT_RELATIVE
    if args.check:
        m.require(output.is_file() and output.read_bytes() == payload, "A3 assessment differs")
    else:
        write_once(output, payload)
        freeze = (
            "\n".join(
                "{}  {}".format(m.sha256_bytes(m.read_file_once(W / path)), path)
                for path in evidence + [m.ASSESSMENT_RELATIVE]
            )
            + "\n"
        ).encode()
        write_once(W / m.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} qualified={} assessment_sha256={}".format(
            value["status"],
            value["qualified_for_spatial_predictor_data_design"],
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
