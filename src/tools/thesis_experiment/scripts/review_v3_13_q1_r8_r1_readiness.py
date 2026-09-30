#!/usr/bin/env python3
"""Independently replay R8-R1 repair readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r8_r1_readiness.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r8_r1_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.BASE.require(
            builder.BASE.read(path) == payload,
            "readiness replay differs: {}".format(path),
        )
    manifest = "".join(
        "{}  {}\n".format(
            builder.BASE.sha(builder.BASE.read(path)), path
        )
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.BASE.require(
        builder.BASE.read(builder.MANIFEST) == manifest,
        "manifest replay differs",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R1",
        "status": "independent_runner_repair_readiness_pass",
        "execution_release_sha256": builder.BASE.sha(
            builder.BASE.read(builder.RELEASE)
        ),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "frozen_failed_release_preserved": True,
        "failed_release_execute_invoked": False,
        "runner_arity_repair_replayed": True,
        "semantic_gate_or_model_change_count": 0,
        "execution_ready": True,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = builder.BASE.yaml_bytes(value)
    path = W / builder.ROOT / "independent_review.yaml"
    builder.BASE.require(not path.exists(), "review exists")
    path.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
