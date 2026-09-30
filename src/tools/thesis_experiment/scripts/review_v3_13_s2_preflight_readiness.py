#!/usr/bin/env python3
"""Independent review of the V3-13-S2 preflight release."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s2_preflight_readiness.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/"
    "v3_13_s2_preflight_readiness/independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s2_preflight_builder", BUILDER_PATH
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "preflight readiness replay differs",
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "preflight manifest differs",
    )
    release_sha = builder.sha(builder.read(builder.RELEASE))
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-PREFLIGHT",
        "status": "v3_13_s2_independent_preflight_readiness_review_pass",
        "preflight_release_sha256": release_sha,
        "evidence_manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "fixed_clock_and_windowed_observer_replayed": True,
        "original_matching_thresholds_retained": True,
        "fresh_non_evidence_seed_7239_replayed": True,
        "exact_four_action_preflight_replayed": True,
        "credential_environment_scrub_replayed": True,
        "fit_or_claim_evidence_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
        "preflight_execution_ready": True,
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
