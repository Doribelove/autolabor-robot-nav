#!/usr/bin/env python3
"""Independent replay of the V3-13-D0 offline foundation."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_d0_foundation.py"
)
REVIEW = (
    W
    / "artifacts/v3/multiactor/v3_13_d0_foundation/"
    "independent_review.yaml"
)


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_13_d0_builder", BUILDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    builder = load_builder()
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "offline output replay differs: {}".format(relative),
        )
    resources = [
        builder.CONTRACT,
        builder.CORE,
        builder.BUILDER,
        builder.REVIEWER,
        builder.TEST,
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in resources
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "foundation manifest differs",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-D0",
        "status": "v3_13_d0_independent_offline_foundation_review_pass",
        "manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "candidate_bank_replayed": True,
        "multi_actor_aggregation_replayed": True,
        "differential_action_fixtures_replayed": True,
        "joint_actor_opposite_spatial_block_attribution_replayed": True,
        "actor_order_invariance_replayed": True,
        "stale_track_fail_closed_replayed": True,
        "clone_schema_full_support_fixture_replayed": True,
        "simulation_evidence_created": False,
        "predictor_qualified": False,
        "performance_claim_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW.exists(), "independent review exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
