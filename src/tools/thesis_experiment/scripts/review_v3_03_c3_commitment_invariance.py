#!/usr/bin/env python3
"""Independent replay review for V3-03-C3."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_03_c3_commitment_invariance.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_03_c3_builder_review", BUILDER
    )
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def review(builder):
    outputs = builder.build()
    for path, payload in outputs.items():
        builder.core.require(
            (W / path).is_file() and (W / path).read_bytes() == payload,
            "builder replay differs: {}".format(path),
        )
    report = yaml.safe_load(outputs[builder.REPORT])
    planner = yaml.safe_load(outputs[builder.EFFECTIVE_CONTRACT])
    closure = yaml.safe_load(outputs[builder.CLOSURE])
    for item in closure["resources"]:
        payload = (W / item["path"]).read_bytes()
        builder.core.require(
            sha(payload) == item["sha256"]
            and len(payload) == item["size_bytes"],
            "resource receipt differs: {}".format(item["path"]),
        )
    builder.core.require(
        report["overall_pass"] is True
        and all(report["gates"].values())
        and planner["hard_verifier"]["control_commitment_horizon_s"]
        == 0.25
        and report["metrics"]["c2_failure_state_selected_candidate_id"]
        == "certified_stop_v000"
        and report["metrics"]["c2_failure_state_selected_accepted"] is True,
        "C3 offline gate failed",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C3",
        "review_id": "v3_03_c3_independent_review_1",
        "status": "v3_03_c3_commitment_invariance_review_pass",
        "report_sha256": sha(outputs[builder.REPORT]),
        "effective_planner_contract_sha256": sha(
            outputs[builder.EFFECTIVE_CONTRACT]
        ),
        "evidence_freeze_sha256": sha(outputs[builder.EVIDENCE_FREEZE]),
        "property_case_count": report["metrics"]["property_case_count"],
        "dependency_resource_count": len(closure["resources"]),
        "all_gates_pass": True,
        "simulation_authorized": False,
        "training_authorized": False,
        "performance_claim_authorized": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    builder = load_builder()
    value = review(builder)
    payload = builder.yaml_bytes(value)
    output = W / builder.INDEPENDENT_REVIEW
    if args.check:
        builder.core.require(
            output.is_file() and output.read_bytes() == payload,
            "review differs",
        )
    else:
        write_once(output, payload)
    print("status={} review_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__":
    main()
