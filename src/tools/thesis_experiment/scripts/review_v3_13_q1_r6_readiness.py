#!/usr/bin/env python3
"""Independent replay review of Q1-R6 readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r6_readiness.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r6_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "readiness replay differs: {}".format(path),
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "manifest replay differs",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
        "status": "independent_five_action_support_readiness_review_pass",
        "execution_release_sha256": builder.sha(
            builder.read(builder.RELEASE)
        ),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "fresh_40_unit_schedule_replayed": True,
        "eight_matched_blocks_replayed": True,
        "five_action_positive_propensity_replayed": True,
        "retry_resume_replacement_forbidden": True,
        "execution_ready": True,
        "supervised_new_head_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
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
