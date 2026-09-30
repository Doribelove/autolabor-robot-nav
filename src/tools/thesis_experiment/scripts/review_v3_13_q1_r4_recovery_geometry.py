#!/usr/bin/env python3
"""Independent deterministic replay of V3-13-Q1-R4."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r4_recovery_geometry.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r4_recovery_geometry/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r4_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "offline output replay differs: {}".format(relative),
        )
    audit = builder.load_yaml(builder.AUDIT)
    builder.require(
        audit["all_offline_geometry_gates_pass"],
        "offline geometry gates failed",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R4",
        "status": "independent_offline_recovery_geometry_review_pass",
        "audit_sha256": builder.sha(builder.read(builder.AUDIT)),
        "manifest_sha256": builder.sha(builder.read(builder.MANIFEST)),
        "legacy_candidate_geometry_replayed_unchanged": True,
        "single_recovery_only_extension_replayed": True,
        "M2_simulation_envelope_replayed": True,
        "R2_surrogate_design_clearance_replayed": True,
        "new_candidate_predictor_qualified": False,
        "fresh_non_evidence_preflight_design_authorized": True,
        "simulator_execution_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW.exists(), "independent review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
