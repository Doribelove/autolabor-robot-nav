#!/usr/bin/env python3
"""Repair only V3-16-S1's raw-report path lookup and reassess offline."""
import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_16_s1_response_gated_support"
OUT = ROOT / "assessment_r1.yaml"
MANIFEST = ROOT / "evidence_manifest_r1.sha256"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    contract_path = W / "config/thesis_experiments/v3/v3_16_s1_response_gated_support_contract.yaml"
    repair_contract_path = W / "config/thesis_experiments/v3/v3_16_s1_result_assessor_r1_contract.yaml"
    original_assessor_path = W / "src/tools/thesis_experiment/scripts/assess_v3_16_s1_response_gated_support.py"
    original_review_path = W / "src/tools/thesis_experiment/scripts/review_v3_16_s1_result.py"
    schedule_path = ROOT / "exact_schedule.yaml"
    terminal_path = ROOT / "terminal_result.yaml"
    summary_path = ROOT / "evidence_summary.yaml"
    schedule = yaml.safe_load(schedule_path.read_bytes())
    terminal = yaml.safe_load(terminal_path.read_bytes())
    summary = yaml.safe_load(summary_path.read_bytes())
    contract = yaml.safe_load(contract_path.read_bytes())

    binding_path = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"
    spec = importlib.util.spec_from_file_location("v3_16_assess_binding_r1", binding_path)
    binding = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(binding)
    allowed = tuple(contract["population"]["actions"])

    resources = [
        repair_contract_path,
        contract_path,
        original_assessor_path,
        original_review_path,
        ROOT / "assessment.yaml",
        ROOT / "evidence_manifest.sha256",
        ROOT / "independent_result_review.yaml",
        ROOT / "terminal_freeze.sha256",
        schedule_path,
        ROOT / "attempt_journal.jsonl",
        ROOT / "preflight_summary.yaml",
        summary_path,
        terminal_path,
    ]
    receipts = []
    raw_report_hashes_match = True
    for row in schedule["rows"]:
        paths = list((ROOT / "episodes").glob("{:03d}_*/matrix_receipt.yaml".format(row["sequence"])))
        if len(paths) != 1:
            raw_report_hashes_match = False
            continue
        receipt_path = paths[0]
        receipt = yaml.safe_load(receipt_path.read_bytes())
        report_path = receipt_path.parent / "episode_report.yaml"
        binding.verify_runtime_report_binding(row, receipt["episode_report"], allowed)
        if (
            receipt["row"] != row
            or not receipt["scientific_inclusion_binding_pass"]
            or not receipt["response_gate_binding_pass"]
            or receipt["episode_report"]["single_factor"]["treatment"] != 12.55
        ):
            raise RuntimeError("receipt identity")
        raw_report_hashes_match = raw_report_hashes_match and (
            receipt["episode_report_sha256"] == sha(report_path.read_bytes())
        )
        receipts.append(receipt)
        resources.extend(
            [
                receipt_path,
                report_path,
                receipt_path.parent / "world.sdf",
                receipt_path.parent / "scene_spec.yaml",
                receipt_path.parent / "actor_specs.yaml",
                receipt_path.parent / "roslaunch.log",
            ]
        )

    checks = {
        "original_failure_preserved": yaml.safe_load((ROOT / "assessment.yaml").read_bytes())["status"]
        == "v3_16_s1_deterministic_result_assessment_fail",
        "receipt_count_matches_terminal": len(receipts) == terminal["completed_units"] == 45,
        "all_action_response_bindings_replay": len(receipts) == 45,
        "all_raw_report_hashes_match_repaired_lookup": raw_report_hashes_match,
        "preflight_excluded": terminal["scientific_episode_count"] == 40,
        "evidence_gate_consistent": terminal["evidence_collection_gate_pass"]
        == summary["collection_gate_pass"]
        == False,
        "terminal_negative_preserved": terminal["status"] == "terminal_negative_response_gated_support",
        "no_hardware": not terminal["hardware_accessed"] and not terminal["real_vehicle_used"],
    }
    passed = all(checks.values())
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-16-S1-R1",
        "status": "v3_16_s1_r1_deterministic_result_assessment_pass" if passed else "v3_16_s1_r1_deterministic_result_assessment_fail",
        "terminal_status": terminal["status"],
        "completed_units": len(receipts),
        "scientific_episode_count": terminal["scientific_episode_count"],
        "evidence_collection_gate_pass": False,
        "maximum_decision_actor_coordinate_range_m": summary["maximum_decision_actor_coordinate_range_m"],
        "maximum_decision_actor_speed_range_mps": summary["maximum_decision_actor_speed_range_mps"],
        "checks": checks,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode()
    manifest = "".join(
        "{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in resources
    ).encode()
    atomic(OUT, body, args.check)
    atomic(MANIFEST, manifest, args.check)
    print(
        "status={} assessment_sha256={} manifest_sha256={}".format(
            value["status"], sha(body), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()

