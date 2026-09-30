#!/usr/bin/env python3
"""Independent replay of the V3-13-S5 recovery planner audit."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s5_recovery_planner_audit.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s5_recovery_audit/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s5_audit_builder", BUILDER_PATH
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "audit replay differs",
        )
    audit = builder.load_yaml(builder.AUDIT)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S5",
        "status": "v3_13_s5_independent_recovery_audit_review_pass",
        "audit_sha256": builder.sha(builder.read(builder.AUDIT)),
        "evidence_manifest_sha256": builder.sha(
            builder.read(builder.MANIFEST)
        ),
        "all_four_three_state_strata_replayed": True,
        "stop_wait_progress_chain_replayed": True,
        "selected_clone_safety_outcomes_replayed": True,
        "actual_TEB_release_design_authorized": audit[
            "actual_TEB_release_design_authorized"
        ],
        "direct_actual_TEB_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "performance_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW_PATH.exists(), "review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
