#!/usr/bin/env python3
"""Replay and assess all V3-01-R1 raw traces."""

import argparse
import importlib.util
import json
import math
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_01_r1", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def assess(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    gates = contract["qualification_gates"]
    seeds = contract["experimental_unit"]["fresh_seeds"]
    root = W / v.EXECUTION_RELATIVE
    journal_rows = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        len([row for row in journal_rows if row["event"] == "unit_complete"])
        == 36,
        "complete journal count differs",
    )
    v.require(
        journal_rows[-1]["event"] == "execution_complete",
        "journal terminal event differs",
    )
    episodes = []
    for row in schedule["rows"]:
        unit_root = root / "attempts" / row["unit_id"]
        receipt = v.load_yaml_path(unit_root / "attempt_receipt.yaml")
        raw = unit_root / "raw_pose_trace.csv"
        raw_bytes = v.read_file_once(raw)
        v.require(
            receipt["raw_csv_sha256"] == v.sha256_bytes(raw_bytes),
            "raw hash differs",
        )
        metrics = v.trace_metrics(v.load_trace(raw))
        v.require(metrics == receipt["metrics_preview"], "metric replay differs")
        expected = 3.0 / float(row["ego_speed_mps"])
        episodes.append(
            {
                "row": dict(row),
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "metrics": metrics,
                "expected_ego_crossing_time_s": expected,
                "ego_crossing_time_error_s": abs(
                    metrics["ego_crossing_time_s"] - expected
                ),
            }
        )

    initial_tolerance = float(gates["clone_initial_position_tolerance_m"])
    initial_pass = all(
        math.hypot(
            item["metrics"]["initial_ego_xy_m"][0] + 3.0,
            item["metrics"]["initial_ego_xy_m"][1],
        )
        <= initial_tolerance
        and math.hypot(
            item["metrics"]["initial_actor_xy_m"][0],
            item["metrics"]["initial_actor_xy_m"][1] + 3.0,
        )
        <= initial_tolerance
        for item in episodes
    )
    ego_max_error = max(item["ego_crossing_time_error_s"] for item in episodes)
    ego_pass = ego_max_error <= float(
        gates["ego_crossing_time_absolute_error_max_s"]
    )
    blocks = []
    for seed in seeds:
        for policy in ["cautious", "assertive", "hesitant"]:
            group = [
                item
                for item in episodes
                if item["row"]["seed"] == seed
                and item["row"]["actor_policy_evaluator_only"] == policy
            ]
            positions = [item["metrics"]["actor_y_at_4s_m"] for item in group]
            signatures = {
                item["metrics"]["response_signature"] for item in group
            }
            spread = max(positions) - min(positions)
            blocks.append(
                {
                    "seed": seed,
                    "actor_policy": policy,
                    "candidate_count": len(group),
                    "actor_y_at_4s_spread_m": spread,
                    "distinct_response_signature_count": len(signatures),
                    "position_spread_gate_pass": spread
                    >= float(gates["minimum_actor_position_spread_at_4s_m"]),
                    "signature_gate_pass": len(signatures)
                    >= int(
                        gates[
                            "minimum_distinct_response_signatures_per_policy_seed"
                        ]
                    ),
                }
            )
    block_pass = all(
        row["position_spread_gate_pass"] and row["signature_gate_pass"]
        for row in blocks
    )
    pre_spreads = []
    for seed in seeds:
        for policy in ["cautious", "assertive", "hesitant"]:
            values = [
                item["metrics"]["actor_y_at_0_05s_m"]
                for item in episodes
                if item["row"]["seed"] == seed
                and item["row"]["actor_policy_evaluator_only"] == policy
            ]
            pre_spreads.append(max(values) - min(values))
    repeat_spreads = []
    for policy in ["cautious", "assertive", "hesitant"]:
        for candidate in ["probe_v060", "probe_v080", "probe_v100", "probe_v120"]:
            values = [
                item["metrics"]["actor_y_at_4s_m"]
                for item in episodes
                if item["row"]["actor_policy_evaluator_only"] == policy
                and item["row"]["candidate_id_evaluator_only"] == candidate
            ]
            repeat_spreads.append(max(values) - min(values))
    pre_max = max(pre_spreads)
    repeat_max = max(repeat_spreads)
    pre_pass = pre_max <= float(
        gates["maximum_pre_intervention_actor_position_spread_m"]
    )
    repeat_pass = repeat_max <= float(
        gates["maximum_cross_seed_response_spread_m"]
    )
    overall = initial_pass and ego_pass and block_pass and pre_pass and repeat_pass
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "assessment_id": "{}_crossing_assessment_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_crossing_reactive_actor_qualification_pass".format(
                contract["stage"].lower().replace("-", "_")
            )
            if overall
            else "{}_crossing_reactive_actor_qualification_fail".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "formal_result": False,
        "simulation_evidence": True,
        "execution": {
            "authorized_units": 36,
            "consumed_units": 36,
            "completed_units": len(episodes),
            "forfeited_units": 0,
            "retry_units": 0,
        },
        "gates": {
            "initial_clone_pass": initial_pass,
            "ego_crossing_time_pass": ego_pass,
            "ego_crossing_time_max_error_s": ego_max_error,
            "all_policy_seed_blocks_pass": block_pass,
            "pre_intervention_clone_spread_pass": pre_pass,
            "maximum_pre_intervention_spread_m": pre_max,
            "cross_seed_repeatability_pass": repeat_pass,
            "maximum_cross_seed_response_spread_m": repeat_max,
            "overall_pass": overall,
        },
        "policy_seed_blocks": blocks,
        "episodes": episodes,
        "claim": {
            "crossing_family_counterfactual_data_qualified": overall,
            "general_multi_family_identifiability": False,
            "predictor_accuracy_claimed": False,
            "planner_safety_claimed": False,
            "performance_improvement_claimed": False,
            "real_vehicle_ready": False,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    result = assess(v)
    payload = v.yaml_bytes(result)
    output = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(output.is_file() and output.read_bytes() == payload, "replay differs")
    else:
        write_once(output, payload)
        resources = [v.JOURNAL_RELATIVE, v.ASSESSMENT_RELATIVE]
        for row in v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)["rows"]:
            unit = v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
            resources.extend(
                [
                    unit / "world.sdf",
                    unit / "raw_pose_trace.csv",
                    unit / "gzserver.log",
                    unit / "attempt_receipt.yaml",
                ]
            )
        freeze = ("\n".join(
            "{}  {}".format(
                v.sha256_bytes(v.read_file_once(W / item)), item.as_posix()
            )
            for item in resources
        ) + "\n").encode("utf-8")
        write_once(W / v.EXECUTION_FREEZE_RELATIVE, freeze)
    print(
        "status={} overall_pass={} assessment_sha256={}".format(
            result["status"], result["gates"]["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
