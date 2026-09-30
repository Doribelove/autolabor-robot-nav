#!/usr/bin/env python3
"""Replay and independently assess all V3-01 raw pose traces."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("v3_01_qualification", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_once(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing to overwrite different file: {}".format(path))
        return
    path.write_bytes(payload)


def assess(module):
    contract = module.load_yaml_relative(WORKSPACE, module.CONTRACT_RELATIVE)
    schedule = module.load_yaml_relative(WORKSPACE, module.SCHEDULE_RELATIVE)
    gates = contract["qualification_gates"]
    execution_root = WORKSPACE / module.EXECUTION_RELATIVE
    journal_path = WORKSPACE / module.JOURNAL_RELATIVE
    journal_rows = [
        json.loads(line)
        for line in journal_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    complete_events = [
        row for row in journal_rows if row["event"] == "unit_complete"
    ]
    module.require(len(complete_events) == 36, "complete journal count differs")
    module.require(
        journal_rows[-1]["event"] == "execution_complete",
        "journal lacks terminal complete event",
    )

    episodes = []
    for row in schedule["rows"]:
        unit_root = execution_root / "attempts" / row["unit_id"]
        receipt = module.load_yaml_path(unit_root / "attempt_receipt.yaml")
        raw_path = unit_root / "raw_pose_trace.csv"
        raw_payload = module.read_file_once(raw_path)
        module.require(
            receipt["raw_csv_sha256"] == module.sha256_bytes(raw_payload),
            "raw receipt differs: {}".format(row["unit_id"]),
        )
        metrics = module.trace_metrics(module.load_trace(raw_path))
        module.require(
            metrics == receipt["metrics_preview"],
            "metric replay differs: {}".format(row["unit_id"]),
        )
        expected_ego_crossing = 3.0 / float(row["ego_speed_mps"])
        episodes.append(
            {
                "row": dict(row),
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "metrics": metrics,
                "expected_ego_crossing_time_s": expected_ego_crossing,
                "ego_crossing_time_error_s": abs(
                    metrics["ego_crossing_time_s"] - expected_ego_crossing
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
    ego_crossing_max_error = max(
        item["ego_crossing_time_error_s"] for item in episodes
    )
    ego_crossing_pass = (
        ego_crossing_max_error
        <= float(gates["ego_crossing_time_absolute_error_max_s"])
    )

    block_results = []
    for seed in [6101, 6102, 6103]:
        for policy in ["cautious", "assertive", "hesitant"]:
            block = [
                item
                for item in episodes
                if item["row"]["seed"] == seed
                and item["row"]["actor_policy_evaluator_only"] == policy
            ]
            positions = [
                item["metrics"]["actor_y_at_4s_m"] for item in block
            ]
            signatures = {
                item["metrics"]["response_signature"] for item in block
            }
            spread = max(positions) - min(positions)
            block_results.append(
                {
                    "seed": seed,
                    "actor_policy": policy,
                    "candidate_count": len(block),
                    "actor_y_at_4s_spread_m": spread,
                    "distinct_response_signature_count": len(signatures),
                    "position_spread_gate_pass": spread
                    >= float(
                        gates["minimum_actor_position_spread_at_4s_m"]
                    ),
                    "signature_gate_pass": len(signatures)
                    >= int(
                        gates[
                            "minimum_distinct_response_signatures_per_policy_seed"
                        ]
                    ),
                }
            )
    block_gate_pass = all(
        row["position_spread_gate_pass"] and row["signature_gate_pass"]
        for row in block_results
    )

    pre_spreads = []
    cross_seed_spreads = []
    for seed in [6101, 6102, 6103]:
        for policy in ["cautious", "assertive", "hesitant"]:
            values = [
                item["metrics"]["actor_y_at_0_05s_m"]
                for item in episodes
                if item["row"]["seed"] == seed
                and item["row"]["actor_policy_evaluator_only"] == policy
            ]
            pre_spreads.append(max(values) - min(values))
    for policy in ["cautious", "assertive", "hesitant"]:
        for candidate in ["probe_v060", "probe_v080", "probe_v100", "probe_v120"]:
            values = [
                item["metrics"]["actor_y_at_4s_m"]
                for item in episodes
                if item["row"]["actor_policy_evaluator_only"] == policy
                and item["row"]["candidate_id_evaluator_only"] == candidate
            ]
            cross_seed_spreads.append(max(values) - min(values))
    maximum_pre_spread = max(pre_spreads)
    maximum_cross_seed_spread = max(cross_seed_spreads)
    pre_gate_pass = maximum_pre_spread <= float(
        gates["maximum_pre_intervention_actor_position_spread_m"]
    )
    repeatability_pass = maximum_cross_seed_spread <= float(
        gates["maximum_cross_seed_response_spread_m"]
    )
    overall = (
        initial_pass
        and ego_crossing_pass
        and block_gate_pass
        and pre_gate_pass
        and repeatability_pass
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-01",
        "assessment_id": "v3_scope_v3_01_crossing_qualification_assessment_1",
        "status": (
            "v3_01_crossing_reactive_actor_qualification_pass"
            if overall
            else "v3_01_crossing_reactive_actor_qualification_fail"
        ),
        "formal_result": False,
        "simulation_evidence": True,
        "execution": {
            "authorized_units": 36,
            "consumed_units": 36,
            "completed_units": len(episodes),
            "forfeited_units": 0,
            "retry_units": 0,
            "unique_seed_count": 3,
        },
        "gates": {
            "initial_clone_pass": initial_pass,
            "ego_crossing_time_pass": ego_crossing_pass,
            "ego_crossing_time_max_error_s": ego_crossing_max_error,
            "all_policy_seed_blocks_pass": block_gate_pass,
            "pre_intervention_clone_spread_pass": pre_gate_pass,
            "maximum_pre_intervention_spread_m": maximum_pre_spread,
            "cross_seed_repeatability_pass": repeatability_pass,
            "maximum_cross_seed_response_spread_m": maximum_cross_seed_spread,
            "overall_pass": overall,
        },
        "policy_seed_blocks": block_results,
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
    module = load_module()
    value = assess(module)
    payload = module.yaml_bytes(value)
    assessment_path = WORKSPACE / module.ASSESSMENT_RELATIVE
    if args.check:
        if not assessment_path.is_file() or assessment_path.read_bytes() != payload:
            raise module.V301Error("assessment replay differs")
    else:
        write_once(assessment_path, payload)
        resources = [module.JOURNAL_RELATIVE, module.ASSESSMENT_RELATIVE]
        for row in module.load_yaml_relative(
            WORKSPACE, module.SCHEDULE_RELATIVE
        )["rows"]:
            root = (
                module.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
            )
            resources.extend(
                [
                    root / "world.sdf",
                    root / "raw_pose_trace.csv",
                    root / "gzserver.log",
                    root / "attempt_receipt.yaml",
                ]
            )
        lines = [
            "{}  {}".format(
                module.sha256_bytes(
                    module.read_file_once(WORKSPACE / relative)
                ),
                relative.as_posix(),
            )
            for relative in resources
        ]
        write_once(
            WORKSPACE / module.EXECUTION_FREEZE_RELATIVE,
            ("\n".join(lines) + "\n").encode("utf-8"),
        )
    print(
        module.yaml_bytes(
            {
                "status": value["status"],
                "overall_pass": value["gates"]["overall_pass"],
                "completed_units": value["execution"]["completed_units"],
                "assessment_sha256": module.sha256_bytes(payload),
            }
        ).decode("utf-8").strip()
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("V3-01 assessment failed closed: {}".format(exc), file=sys.stderr)
        raise
