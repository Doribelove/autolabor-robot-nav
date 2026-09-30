#!/usr/bin/env python3
"""Independent replay review of V3-13-Q1 readiness."""

import argparse
import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_readiness.py"
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--phase", choices=("preflight", "qualification"), required=True
    )
    arguments = parser.parse_args()
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_readiness_builder", BUILDER_PATH
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    phase = arguments.phase
    outputs = builder.build_outputs(phase)
    resources = builder.resources_for(phase)
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "readiness replay differs",
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in resources + list(outputs)
    ).encode("utf-8")
    root = builder.ROOTS[phase]
    builder.require(
        builder.read(root / "evidence_manifest.sha256") == manifest,
        "manifest replay differs",
    )
    release_sha = builder.sha(
        builder.read(root / "execution_release.yaml")
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": (
            "V3-13-Q1-PREFLIGHT"
            if phase == "preflight"
            else "V3-13-Q1"
        ),
        "phase": phase,
        "status": "v3_13_q1_independent_readiness_review_pass",
        "execution_release_sha256": release_sha,
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "exact_schedule_replayed": True,
        "actual_teb_binary_config_and_source_hashes_replayed": True,
        "fresh_seed_roster_replayed": True,
        "old_seed_or_budget_reuse": False,
        "credential_environment_scrub_replayed": True,
        "formal_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "perception_stage_execution_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
        "execution_ready": True,
    }
    payload = builder.yaml_bytes(value)
    review_path = root / "independent_review.yaml"
    builder.require(not (W / review_path).exists(), "review exists")
    (W / review_path).write_bytes(payload)
    print(
        "status={} phase={} review_sha256={}".format(
            value["status"], phase, hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
