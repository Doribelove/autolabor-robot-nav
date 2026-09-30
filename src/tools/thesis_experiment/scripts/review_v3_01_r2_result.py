#!/usr/bin/env python3
"""Independent terminal review of V3-01-R2 raw evidence."""

import argparse
import importlib.util
import json
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_01_r2", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def review(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    assessment = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    complete = [row for row in journal if row["event"] == "unit_complete"]
    v.require(len(complete) == 36, "journal completion count differs")
    v.require(journal[-1]["event"] == "execution_complete", "journal not terminal")
    positions = {}
    signatures = {}
    initial_ego_error = []
    initial_actor_error = []
    ego_crossing_errors = []
    for row in schedule["rows"]:
        root = W / v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt = v.load_yaml_path(root / "attempt_receipt.yaml")
        raw = v.read_file_once(root / "raw_pose_trace.csv")
        v.require(
            v.sha256_bytes(raw) == receipt["raw_csv_sha256"],
            "raw receipt differs",
        )
        metrics = v.trace_metrics(v.load_trace(root / "raw_pose_trace.csv"))
        key = (row["seed"], row["actor_policy_evaluator_only"])
        positions.setdefault(key, []).append(metrics["actor_y_at_4s_m"])
        signatures.setdefault(key, set()).add(metrics["response_signature"])
        initial_ego_error.append(
            ((metrics["initial_ego_xy_m"][0] + 3.0) ** 2
             + metrics["initial_ego_xy_m"][1] ** 2) ** 0.5
        )
        initial_actor_error.append(
            (metrics["initial_actor_xy_m"][0] ** 2
             + (metrics["initial_actor_xy_m"][1] + 3.0) ** 2) ** 0.5
        )
        ego_crossing_errors.append(
            abs(metrics["ego_crossing_time_s"] - 3.0 / row["ego_speed_mps"])
        )
    block_spreads = [max(values) - min(values) for values in positions.values()]
    repeat_spreads = []
    for policy in ["cautious", "assertive", "hesitant"]:
        for candidate in ["probe_v060", "probe_v080", "probe_v100", "probe_v120"]:
            values = []
            for episode in assessment["episodes"]:
                if (
                    episode["row"]["actor_policy_evaluator_only"] == policy
                    and episode["row"]["candidate_id_evaluator_only"] == candidate
                ):
                    values.append(episode["metrics"]["actor_y_at_4s_m"])
            repeat_spreads.append(max(values) - min(values))
    gates = contract["qualification_gates"]
    independent_pass = (
        len(positions) == 9
        and min(block_spreads)
        >= float(gates["minimum_actor_position_spread_at_4s_m"])
        and min(len(value) for value in signatures.values())
        >= int(gates["minimum_distinct_response_signatures_per_policy_seed"])
        and max(initial_ego_error + initial_actor_error)
        <= float(gates["clone_initial_position_tolerance_m"])
        and max(ego_crossing_errors)
        <= float(gates["ego_crossing_time_absolute_error_max_s"])
        and max(repeat_spreads)
        <= float(gates["maximum_cross_seed_response_spread_m"])
    )
    v.require(
        independent_pass == assessment["gates"]["overall_pass"],
        "independent result differs from assessor",
    )
    return {
        "schema_version": "1.0",
        "stage": "V3-01-R2",
        "review_id": "v3_scope_v3_01_r2_independent_result_review_1",
        "status": (
            "v3_01_r2_independent_result_review_pass"
            if independent_pass
            else "v3_01_r2_independent_result_review_fail"
        ),
        "raw_trace_count_replayed": 36,
        "journal_complete_count": len(complete),
        "policy_seed_block_count": len(positions),
        "minimum_actor_position_spread_m": min(block_spreads),
        "maximum_actor_position_spread_m": max(block_spreads),
        "minimum_distinct_signature_count": min(
            len(value) for value in signatures.values()
        ),
        "maximum_initial_position_error_m": max(
            initial_ego_error + initial_actor_error
        ),
        "maximum_ego_crossing_time_error_s": max(ego_crossing_errors),
        "maximum_cross_seed_response_spread_m": max(repeat_spreads),
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "execution_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EXECUTION_FREEZE_RELATIVE)
        ),
        "overall_pass": independent_pass,
        "claim_scope": "deterministic_crossing_counterfactual_data_generation_only",
        "general_multi_family_identifiability": False,
        "stochastic_actor_calibrated": False,
        "predictor_or_planner_validated": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    output = (
        W
        / v.EXECUTION_RELATIVE
        / "v3_01_r2_independent_result_review.yaml"
    )
    payload = v.yaml_bytes(review(v))
    if args.check:
        v.require(output.is_file() and output.read_bytes() == payload, "review differs")
    else:
        write_once(output, payload)
    print("review_sha256={}".format(v.sha256_bytes(payload)))


if __name__ == "__main__":
    main()
