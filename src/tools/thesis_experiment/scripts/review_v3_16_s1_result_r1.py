#!/usr/bin/env python3
"""Independent offline review of the V3-16-S1 assessor-only R1 repair."""
import argparse
import hashlib
import os
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_16_s1_response_gated_support"
OUT = ROOT / "independent_result_review_r1.yaml"
FREEZE = ROOT / "terminal_freeze_r1.sha256"


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
    assessment_path = ROOT / "assessment_r1.yaml"
    manifest_path = ROOT / "evidence_manifest_r1.sha256"
    original_assessment_path = ROOT / "assessment.yaml"
    original_review_path = ROOT / "independent_result_review.yaml"
    assessment = yaml.safe_load(assessment_path.read_bytes())
    entries = [line.split("  ", 1) for line in manifest_path.read_text().splitlines() if line]
    checks = {
        "r1_assessment_pass": assessment["status"] == "v3_16_s1_r1_deterministic_result_assessment_pass",
        "manifest_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in entries),
        "original_failed_assessor_preserved": yaml.safe_load(original_assessment_path.read_bytes())["status"]
        == "v3_16_s1_deterministic_result_assessment_fail",
        "original_failed_review_preserved": yaml.safe_load(original_review_path.read_bytes())["status"]
        == "v3_16_s1_independent_result_review_fail",
        "binding_replay": assessment["checks"]["all_action_response_bindings_replay"],
        "raw_report_hashes_replayed": assessment["checks"]["all_raw_report_hashes_match_repaired_lookup"],
        "negative_scientific_result_preserved": not assessment["evidence_collection_gate_pass"]
        and assessment["terminal_status"] == "terminal_negative_response_gated_support",
        "authority_scoped": not assessment["predictor_fit_authorized"]
        and not assessment["actual_TEB_execution_authorized"]
        and not assessment["perception_execution_authorized"]
        and not assessment["training_authorized"],
        "no_hardware": not assessment["hardware_accessed"] and not assessment["real_vehicle_used"],
    }
    passed = all(checks.values())
    value = {
        "schema_version": "1.0",
        "stage": "V3-16-S1-R1",
        "status": "v3_16_s1_r1_independent_result_review_pass" if passed else "v3_16_s1_r1_independent_result_review_fail",
        "assessment_sha256": sha(assessment_path.read_bytes()),
        "evidence_manifest_sha256": sha(manifest_path.read_bytes()),
        "terminal_status": assessment["terminal_status"],
        "checks": checks,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode()
    atomic(OUT, body, args.check)
    freeze_resources = [
        W / "config/thesis_experiments/v3/v3_16_s1_result_assessor_r1_contract.yaml",
        assessment_path,
        manifest_path,
        OUT,
        original_assessment_path,
        original_review_path,
    ]
    freeze = "".join(
        "{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in freeze_resources
    ).encode()
    atomic(FREEZE, freeze, args.check)
    print(
        "status={} review_sha256={} terminal_freeze_sha256={}".format(
            value["status"], sha(body), sha(freeze)
        )
    )


if __name__ == "__main__":
    main()

