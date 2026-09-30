#!/usr/bin/env python3
"""Independently replay R19-R5-R1 readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r19_r5_r1_readiness.py"
)


def main():
    spec = importlib.util.spec_from_file_location("r19_r5_r1_builder", BUILDER)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.BASE.require(
            builder.BASE.read(path) == payload, "readiness replay differs"
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
        "stage": "V3-13-Q1-R19-R5-R1",
        "status": "independent_repaired_binding_preflight_readiness_pass",
        "execution_release_sha256": builder.BASE.sha(
            builder.BASE.read(builder.RELEASE)
        ),
        "evidence_manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "fresh_seed7696_attempt1_replayed": True,
        "behavior_propensity_0_20_replayed": True,
        "execution_ready": True,
        "predictor_fit_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = builder.BASE.yaml_bytes(value)
    target = W / builder.ROOT / "independent_review.yaml"
    builder.BASE.require(not target.exists(), "review exists")
    target.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
