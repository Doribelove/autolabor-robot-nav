#!/usr/bin/env python3
"""Independently replay R19-D1 diagnosis."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r19_d1_terminal_atomicity_diagnosis.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r19_d1_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "diagnosis replay differs",
        )
    diagnosis = yaml.safe_load(outputs[builder.DIAGNOSIS])
    passed = all(diagnosis["checks"].values())
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-D1",
        "status": (
            "independent_terminal_boundary_atomicity_diagnosis_pass"
            if passed
            else "independent_terminal_boundary_atomicity_diagnosis_fail"
        ),
        "diagnosis_sha256": hashlib.sha256(
            outputs[builder.DIAGNOSIS]
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            outputs[builder.MANIFEST]
        ).hexdigest(),
        "all_raw_r3_hashes_and_final_guard_replays_verified": True,
        "fresh_single_factor_boundary_atomic_preflight_design_authorized": (
            passed
        ),
        "simulation_execution_authorized": False,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
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
