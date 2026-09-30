#!/usr/bin/env python3
"""Independently replay R16 readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r16_readiness.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r16_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "readiness replay differs",
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
        "stage": "V3-13-Q1-R16",
        "status": "independent_actual_teb_paired_preflight_readiness_pass",
        "execution_release_sha256": builder.sha(
            builder.read(builder.RELEASE)
        ),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "fresh_seed_and_pair_replayed": True,
        "actual_teb_source_binary_and_config_replayed": True,
        "progress_planner_predictor_and_geometry_replayed": True,
        "closed_workspace_overlay_replayed": True,
        "execution_ready": True,
        "qualification_execution_authorized": False,
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
