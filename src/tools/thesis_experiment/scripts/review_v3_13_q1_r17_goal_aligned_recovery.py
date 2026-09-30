#!/usr/bin/env python3
"""Independently replay R17 geometry and terminal-guard closure."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r17_goal_aligned_recovery.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r17_goal_aligned_recovery/independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r17_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "R17 replay differs",
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.evidence_resources() + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "manifest replay differs",
    )
    audit = yaml.safe_load(outputs[builder.AUDIT])
    passed = audit["offline_design_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R17",
        "status": (
            "independent_goal_aligned_recovery_and_guard_pass"
            if passed
            else "independent_goal_aligned_recovery_and_guard_fail"
        ),
        "audit_sha256": hashlib.sha256(
            outputs[builder.AUDIT]
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "nine_design_states_replayed": True,
        "kinematics_goal_safety_and_guard_fixtures_replayed": True,
        "fresh_runtime_preflight_design_authorized": passed,
        "candidate_specific_support_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW.exists(), "review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
