#!/usr/bin/env python3
"""Consume the single non-evidence V3-13-C1 integration preflight."""

import argparse
from pathlib import Path
import sys

import yaml

import run_v3_13_c1_multiactor_clones as runner
import v3_13_multiactor_design as design


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    output_root = Path(arguments.output_root).resolve()
    if output_root.exists():
        raise RuntimeError("preflight output exists; retry is forbidden")
    contract_payload = runner.read_bytes(runner.CONTRACT_PATH)
    contract_sha256 = runner.sha256_bytes(contract_payload)
    if arguments.contract_sha256 != contract_sha256:
        raise RuntimeError("contract SHA does not match")
    contract = yaml.safe_load(contract_payload)
    authority = contract["authority"]
    if not authority["exact_bounded_simulation_collection_authorized"]:
        raise RuntimeError("simulation is not authorized")
    if int(authority["authorized_preflight_units"]) != 1:
        raise RuntimeError("preflight unit authority differs")
    preflight = contract["integration_preflight"]
    if preflight["evidence_identity"]:
        raise RuntimeError("preflight must be non-evidence")
    scene = next(
        item
        for item in contract["scene_population"]
        if item["scene_id"] == preflight["scene_id"]
    )
    row = {
        "sequence": 1,
        "block_index": 0,
        "seed": int(preflight["seed"]),
        "scene_id": preflight["scene_id"],
        "actor_policies_evaluator_only": list(
            preflight["actor_policies_evaluator_only"]
        ),
        "method": "cloned_intervention",
        "forced_candidate_id": preflight["forced_candidate_id"],
        "behavior_propensity": float(
            preflight["behavior_propensity"]
        ),
        "attempt": 1,
        "evidence_identity": False,
    }
    if row["seed"] in {
        scheduled["seed"] for scheduled in design.build_schedule(contract)
    }:
        raise RuntimeError("preflight seed overlaps evidence")
    output_root.mkdir(parents=True)
    runner.atomic_yaml(
        output_root / "preflight_identity.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-C1-PREFLIGHT",
            "contract_sha256": contract_sha256,
            "row": row,
            "result_cannot_enter_predictor_fit_or_performance_analysis": True,
        },
    )
    journal = output_root / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    runner.append_jsonl(
        journal,
        {
            "event": "preflight_unit_consumed",
            "row": row,
            "authorized_units": 1,
        },
    )
    try:
        receipt = runner.run_episode(
            row, scene, output_root / "episode", 27990
        )
    except Exception as error:
        runner.append_jsonl(
            journal,
            {
                "event": "preflight_terminal_failure",
                "row": row,
                "error": "{}:{}".format(type(error).__name__, error),
                "consumed_units": 1,
            },
        )
        raise
    report = receipt["episode_report"]
    result = {
        "schema_version": "1.0",
        "stage": "V3-13-C1-PREFLIGHT",
        "status": (
            "integration_preflight_pass"
            if report["episode_collection_pass"]
            else "integration_preflight_fail"
        ),
        "contract_sha256": contract_sha256,
        "consumed_units": 1,
        "completed_units": 1,
        "evidence_units_created": 0,
        "episode_collection_pass": report["episode_collection_pass"],
        "actor_count": report["actor_count"],
        "trace_sample_count": report["result"]["trace_sample_count"],
        "trace_duration_s": report["result"]["trace_duration_s"],
        "gate_physical_valid": report["gate_physical_valid"],
        "credential_environment_scrubbed": receipt[
            "credential_environment_scrubbed"
        ],
        "result_cannot_enter_predictor_fit_or_performance_analysis": True,
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    runner.atomic_yaml(output_root / "preflight_result.yaml", result)
    runner.append_jsonl(
        journal,
        {
            "event": "preflight_complete",
            "status": result["status"],
            "consumed_units": 1,
            "evidence_units_created": 0,
        },
    )
    print(
        "status={} result_sha256={}".format(
            result["status"],
            runner.sha256_bytes(
                runner.read_bytes(output_root / "preflight_result.yaml")
            ),
        )
    )
    return 0 if report["episode_collection_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())

