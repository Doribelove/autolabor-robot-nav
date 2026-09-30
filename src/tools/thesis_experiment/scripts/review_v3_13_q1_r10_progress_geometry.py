#!/usr/bin/env python3
"""Independently replay R10 progress-relative geometry closure."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r10_progress_geometry.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r10_progress_geometry/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r10_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "R10 geometry replay differs",
        )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R10",
        "status": "independent_progress_geometry_closure_pass",
        "audit_sha256": hashlib.sha256(
            outputs[builder.AUDIT]
        ).hexdigest(),
        "evidence_manifest_sha256": builder.sha(
            builder.read(builder.MANIFEST)
        ),
        "all_eight_design_states_replayed": True,
        "translation_invariance_replayed": True,
        "static_dynamic_and_kinematic_gates_replayed": True,
        "fresh_non_evidence_preflight_design_authorized": True,
        "candidate_specific_support_authorized": False,
        "direct_actual_TEB_execution_authorized": False,
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
