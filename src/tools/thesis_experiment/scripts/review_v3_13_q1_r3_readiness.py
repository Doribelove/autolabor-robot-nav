#!/usr/bin/env python3
"""Independent replay review of Q1-R3 readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r3_readiness.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r3_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload, "readiness replay differs"
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "manifest replay differs",
    )
    release_sha = builder.sha(builder.read(builder.RELEASE))
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3",
        "status": "midpoint_clone_independent_readiness_review_pass",
        "execution_release_sha256": release_sha,
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "fresh_16_unit_schedule_replayed": True,
        "single_timing_factor_replayed": True,
        "four_action_positive_propensity_replayed": True,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
        "execution_ready": True,
    }
    payload = builder.yaml_bytes(value)
    path = W / builder.ROOT / "independent_review.yaml"
    builder.require(not path.exists(), "review exists")
    path.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
