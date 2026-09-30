#!/usr/bin/env python3
"""Independent deterministic review of V3-13-S1 execution readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s1_readiness.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s1_readiness/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s1_readiness_builder", BUILDER_PATH
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "readiness replay differs: {}".format(relative),
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "readiness manifest differs",
    )
    release_sha = builder.sha(builder.read(builder.RELEASE))
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "v3_13_s1_independent_readiness_review_pass",
        "execution_release_sha256": release_sha,
        "evidence_manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "fresh_32_unit_schedule_replayed": True,
        "eight_matched_context_blocks_replayed": True,
        "four_action_positive_propensity_replayed": True,
        "early_and_late_wait_depths_replayed": True,
        "old_seed_or_budget_reuse": False,
        "credential_environment_scrub_replayed": True,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
        "execution_ready": True,
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
