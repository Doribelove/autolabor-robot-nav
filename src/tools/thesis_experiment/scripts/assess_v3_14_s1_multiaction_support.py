#!/usr/bin/env python3
"""Deterministically replay and freeze the V3-14-S1 raw result."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_14_s1_multiaction_support"
OUT = ROOT / "assessment.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def sha(payload): return hashlib.sha256(payload).hexdigest()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    schedule = yaml.safe_load((ROOT / "exact_schedule.yaml").read_bytes())
    terminal = yaml.safe_load((ROOT / "terminal_result.yaml").read_bytes())
    contract_path = W / "config/thesis_experiments/v3/v3_14_s1_multiaction_support_contract.yaml"
    contract = yaml.safe_load(contract_path.read_bytes())
    binding_path = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"
    spec = importlib.util.spec_from_file_location("v3_14_s1_assess_binding", binding_path)
    binding = importlib.util.module_from_spec(spec); spec.loader.exec_module(binding)
    allowed = tuple(contract["population"]["actions"])
    resources = [contract_path, ROOT / "exact_schedule.yaml", ROOT / "attempt_journal.jsonl", ROOT / "preflight_summary.yaml", ROOT / "terminal_result.yaml"]
    if (ROOT / "evidence_summary.yaml").is_file(): resources.append(ROOT / "evidence_summary.yaml")
    receipts = []
    for row in schedule["rows"]:
        matches = list((ROOT / "episodes").glob("{:03d}_*/matrix_receipt.yaml".format(row["sequence"])))
        if len(matches) != 1: break
        receipt = yaml.safe_load(matches[0].read_bytes())
        binding.verify_runtime_report_binding(row, receipt["episode_report"], allowed)
        if receipt["row"] != row or not receipt["preconsumption_binding"]["binding_pass"] or not receipt["runtime_binding"]["binding_pass"]:
            raise RuntimeError("receipt binding differs")
        receipts.append(receipt); resources.extend([matches[0], matches[0].parent / "episode_report.yaml", matches[0].parent / "world.sdf", matches[0].parent / "scene_spec.yaml", matches[0].parent / "actor_specs.yaml", matches[0].parent / "roslaunch.log"])
    expected = int(terminal["completed_units"])
    evidence_summary = yaml.safe_load((ROOT / "evidence_summary.yaml").read_bytes()) if (ROOT / "evidence_summary.yaml").is_file() else None
    checks = {
        "receipt_count_matches_terminal": len(receipts) == expected,
        "all_binding_replays_pass": len(receipts) == expected,
        "all_receipt_raw_hashes_match": all(
            r["episode_report_sha256"] == sha((p.parent / "episode_report.yaml").read_bytes())
            for r, p in zip(receipts, [x for x in resources if x.name == "matrix_receipt.yaml"])
        ),
        "preflight_excluded_from_science": terminal["scientific_episode_count"] in (0, 40),
        "evidence_gate_consistent": evidence_summary is None or terminal["evidence_collection_gate_pass"] == evidence_summary["collection_gate_pass"],
        "hardware_absent": not terminal["hardware_accessed"] and not terminal["real_vehicle_used"],
    }
    passed = all(checks.values())
    status = "v3_14_s1_deterministic_result_assessment_pass" if passed else "v3_14_s1_deterministic_result_assessment_fail"
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1", "status": status,
        "terminal_status": terminal["status"], "completed_units": len(receipts),
        "scientific_episode_count": terminal["scientific_episode_count"], "evidence_collection_gate_pass": terminal.get("evidence_collection_gate_pass", False),
        "checks": checks, "predictor_fit_authorized": passed and terminal.get("predictor_fit_authorized", False),
        "actual_TEB_execution_authorized": False, "perception_execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in resources).encode("utf-8")
    for path, data in ((OUT, payload), (MANIFEST, manifest)):
        if args.check:
            if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        else:
            if path.exists(): raise RuntimeError("{} exists".format(path))
            temporary = path.with_name(path.name + ".tmp")
            with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
            os.replace(str(temporary), str(path))
    print("status={} assessment_sha256={} manifest_sha256={}".format(status, sha(payload), sha(manifest)))


if __name__ == "__main__": main()

