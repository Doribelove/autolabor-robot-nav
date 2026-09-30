#!/usr/bin/env python3
"""Independent replay of V3-13-Q1-R4-D1/R4-R1."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r4_r1_recovery_geometry.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r4_r1_recovery_geometry/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r4_r1_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        target = builder.W / path
        if not target.is_file() or target.read_bytes() != payload:
            raise RuntimeError("offline replay differs: {}".format(path))
    audit = yaml.safe_load(outputs[builder.AUDIT])
    consistency = yaml.safe_load(outputs[builder.CONSISTENCY])
    if not audit["all_offline_geometry_gates_pass"]:
        raise RuntimeError("geometry gates failed")
    if consistency["status"] != "cross_layer_clearance_mismatch_audit_pass":
        raise RuntimeError("consistency audit failed")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R4-R1",
        "status": "independent_runtime_radius_aligned_geometry_review_pass",
        "consistency_audit_sha256": hashlib.sha256(
            outputs[builder.CONSISTENCY]
        ).hexdigest(),
        "candidate_geometry_audit_sha256": hashlib.sha256(
            outputs[builder.AUDIT]
        ).hexdigest(),
        "manifest_sha256": hashlib.sha256(
            outputs[builder.MANIFEST]
        ).hexdigest(),
        "R4_cross_layer_gap_replayed": True,
        "R4_bytes_preserved": True,
        "revised_geometry_replayed": True,
        "planner_and_runtime_radius_surrogates_replayed": True,
        "new_candidate_predictor_qualified": False,
        "fresh_non_evidence_preflight_design_authorized": True,
        "simulator_execution_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    if REVIEW.exists():
        raise RuntimeError("independent review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
