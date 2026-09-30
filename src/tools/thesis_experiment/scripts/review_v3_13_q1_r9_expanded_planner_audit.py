#!/usr/bin/env python3
"""Independently replay the terminal R9 integration audit."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r9_expanded_planner_audit.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r9_planner_audit/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r9_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "R9 audit replay differs",
        )
    audit = yaml.safe_load(outputs[builder.AUDIT])
    if audit["planner_integration_pass"]:
        raise RuntimeError("R9 unexpectedly passed")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R9",
        "status": "independent_terminal_planner_integration_fail_confirmed",
        "audit_sha256": hashlib.sha256(
            outputs[builder.AUDIT]
        ).hexdigest(),
        "evidence_manifest_sha256": builder.sha(
            builder.read(builder.MANIFEST)
        ),
        "all_eight_states_replayed": True,
        "dynamic_and_static_verdicts_replayed": True,
        "runtime_trace_corridor_containment_replayed": True,
        "candidate_geometry_redesign_authorized": True,
        "direct_actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = builder.yaml_bytes(value)
    if REVIEW.exists():
        raise RuntimeError("review exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
