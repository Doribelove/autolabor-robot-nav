#!/usr/bin/env python3
"""Assess V3-07-A1 cloned spatial-intervention identifiability."""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"


def load_module():
    spec = importlib.util.spec_from_file_location("v3_07_a1_assessor_module", MODULE)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def actor_coordinate(m, trace, row, time_s):
    dx, dy = m.actor_direction(row)
    return (
        m.interpolate(trace, "actor_x_m", time_s) * dx
        + m.interpolate(trace, "actor_y_m", time_s) * dy
    )


def assess(m):
    contract = m.load_yaml_relative(W, m.CONTRACT_RELATIVE)
    prereg = m.load_yaml_relative(W, m.PREREGISTRATION_RELATIVE)
    m.validate_contract(contract, prereg)
    schedule = m.load_yaml_relative(W, m.SCHEDULE_RELATIVE)
    m.require(schedule == m.exact_schedule(contract), "A1 schedule differs")
    journal = [
        json.loads(line)
        for line in (W / m.JOURNAL_RELATIVE).read_text().splitlines()
        if line.strip()
    ]
    m.require(
        journal[-1]["event"] == "execution_complete"
        and len([row for row in journal if row["event"] == "unit_complete"])
        == 36,
        "A1 journal incomplete",
    )
    rules = contract["qualification_gates"]
    radius = float(rules["conservative_combined_radius_m"])
    groups = {}
    evidence = [m.JOURNAL_RELATIVE]
    nonfinite = 0
    prefix_error = 0.0
    minimum_clearance = float("inf")
    clearance_violations = 0
    for row in schedule["rows"]:
        root = m.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt_path = root / "attempt_receipt.yaml"
        raw_path = root / "raw_pose_trace.csv"
        world_path = root / "world.sdf"
        receipt = m.load_yaml_relative(W, receipt_path)
        raw = m.read_file_once(W / raw_path)
        m.require(
            receipt["row"] == row
            and receipt["raw_csv_sha256"] == m.sha256_bytes(raw),
            "A1 receipt differs",
        )
        trace = m.load_trace(W / raw_path)
        prefix_error = max(
            prefix_error,
            math.hypot(
                m.interpolate(trace, "ego_x_m", 0.5) + 2.6,
                m.interpolate(trace, "ego_y_m", 0.5),
            ),
        )
        clearance = min(
            math.hypot(
                float(item["ego_x_m"]) - float(item["actor_x_m"]),
                float(item["ego_y_m"]) - float(item["actor_y_m"]),
            )
            - radius
            for item in trace
        )
        minimum_clearance = min(minimum_clearance, clearance)
        clearance_violations += clearance < 0.0
        metrics = m.trace_metrics(trace, row)
        horizons = [
            actor_coordinate(m, trace, row, stamp)
            for stamp in (1.5, 2.5, 3.5, 4.5)
        ]
        values = horizons + [
            clearance,
            m.interpolate(trace, "ego_x_m", 3.5),
            m.interpolate(trace, "ego_y_m", 3.5),
        ]
        nonfinite += sum(not math.isfinite(value) for value in values)
        key = "{}:{}".format(
            row["block_id"], row["actor_policy_evaluator_only"]
        )
        groups.setdefault(key, {})[row["candidate_id_evaluator_only"]] = {
            "actor_coordinates_m": horizons,
            "ego_xy_at_3_5_s": [
                m.interpolate(trace, "ego_x_m", 3.5),
                m.interpolate(trace, "ego_y_m", 3.5),
            ],
            "minimum_conservative_clearance_m": clearance,
            "metrics_preview": metrics,
        }
        evidence.extend((receipt_path, raw_path, world_path))
    sensitivities = []
    mirror_actor_differences = []
    spatial_ego_separations = []
    pair_rows = []
    for key, branches in sorted(groups.items()):
        m.require(
            set(branches)
            == {"straight_v120", "spatial_left_v120", "spatial_right_v120"},
            "A1 cloned branch incomplete",
        )
        straight = branches["straight_v120"]
        left = branches["spatial_left_v120"]
        right = branches["spatial_right_v120"]
        mirror = max(
            abs(a - b)
            for a, b in zip(
                left["actor_coordinates_m"], right["actor_coordinates_m"]
            )
        )
        sensitivity = max(
            max(abs(a - b) for a, b in zip(
                straight["actor_coordinates_m"], spatial["actor_coordinates_m"]
            ))
            for spatial in (left, right)
        )
        separation = min(
            math.hypot(
                spatial["ego_xy_at_3_5_s"][0]
                - straight["ego_xy_at_3_5_s"][0],
                spatial["ego_xy_at_3_5_s"][1]
                - straight["ego_xy_at_3_5_s"][1],
            )
            for spatial in (left, right)
        )
        sensitivities.append(sensitivity)
        mirror_actor_differences.append(mirror)
        spatial_ego_separations.append(separation)
        pair_rows.append(
            {
                "pair_id": key,
                "spatial_vs_straight_actor_sensitivity_m": sensitivity,
                "left_right_actor_coordinate_difference_m": mirror,
                "spatial_ego_separation_at_3_5_s_m": separation,
            }
        )
    sensitive_fraction = sum(
        value >= float(rules["pair_sensitivity_threshold_m"])
        for value in sensitivities
    ) / len(sensitivities)
    metrics = {
        "completed_unit_count": 36,
        "cloned_pair_count": len(groups),
        "common_prefix_position_error_max_m": prefix_error,
        "minimum_spatial_ego_separation_at_3_5_s_m": min(
            spatial_ego_separations
        ),
        "maximum_left_right_actor_coordinate_difference_m": max(
            mirror_actor_differences
        ),
        "mean_spatial_vs_straight_actor_sensitivity_m": sum(
            sensitivities
        )
        / len(sensitivities),
        "candidate_sensitive_pair_fraction": sensitive_fraction,
        "minimum_conservative_clearance_m": minimum_clearance,
        "conservative_clearance_violation_count": clearance_violations,
        "nonfinite_metric_count": nonfinite,
    }
    gates = {
        "completed_pass": metrics["completed_unit_count"]
        == int(rules["completed_unit_count"]),
        "common_prefix_pass": metrics["common_prefix_position_error_max_m"]
        <= float(rules["common_prefix_position_error_max_m"]),
        "ego_intervention_separation_pass": metrics[
            "minimum_spatial_ego_separation_at_3_5_s_m"
        ]
        >= float(rules["spatial_ego_separation_at_3_5_s_min_m"]),
        "mirror_consistency_pass": metrics[
            "maximum_left_right_actor_coordinate_difference_m"
        ]
        <= float(rules["left_right_actor_coordinate_difference_max_m"]),
        "candidate_sensitivity_mean_pass": metrics[
            "mean_spatial_vs_straight_actor_sensitivity_m"
        ]
        >= float(rules["mean_spatial_vs_straight_actor_sensitivity_min_m"]),
        "candidate_sensitivity_fraction_pass": metrics[
            "candidate_sensitive_pair_fraction"
        ]
        >= float(rules["candidate_sensitive_pair_fraction_min"]),
        "clearance_pass": metrics["conservative_clearance_violation_count"]
        <= int(rules["conservative_clearance_violation_count_max"])
        and metrics["minimum_conservative_clearance_m"]
        >= float(rules["minimum_conservative_clearance_m_min"]),
        "finite_pass": metrics["nonfinite_metric_count"]
        <= int(rules["nonfinite_metric_count_max"]),
    }
    passed = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-A1",
        "status": (
            "spatial_intervention_identifiability_qualification_pass"
            if passed
            else "spatial_intervention_identifiability_qualification_fail"
        ),
        "formal_result": False,
        "execution": {
            "authorized_units": 36,
            "consumed_units": 36,
            "completed_units": 36,
            "retry_units": 0,
        },
        "metrics": metrics,
        "gates": gates,
        "pair_rows": pair_rows,
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
    m = load_module()
    value = assess(m)
    evidence = value.pop("_evidence")
    payload = m.yaml_bytes(value)
    output = W / m.ASSESSMENT_RELATIVE
    if args.check:
        m.require(output.is_file() and output.read_bytes() == payload, "A1 assessment differs")
    else:
        write_once(output, payload)
        freeze = (
            "\n".join(
                "{}  {}".format(
                    m.sha256_bytes(m.read_file_once(W / path)), path
                )
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
