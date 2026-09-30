#!/usr/bin/env python3
"""Independently replay R16-D1 failure attribution."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r16_d1_execution_diagnosis.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r16_d1_diagnosis/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r16_d1_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for path, payload in outputs.items():
        builder.require(
            builder.read(path) == payload,
            "diagnosis replay differs",
        )
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in builder.RESOURCES + list(outputs)
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "manifest replay differs",
    )
    diagnosis = yaml.safe_load(outputs[builder.DIAGNOSIS])
    confirmed = diagnosis["attribution_confirmed"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16-D1",
        "status": (
            "independent_goal_and_terminal_execution_attribution_pass"
            if confirmed
            else "independent_failure_attribution_inconclusive"
        ),
        "diagnosis_sha256": hashlib.sha256(
            outputs[builder.DIAGNOSIS]
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "frozen_R16_report_and_receipt_rehashed": True,
        "goal_terminal_tracking_and_safety_metrics_replayed": True,
        "single_repair_design_authorized": confirmed,
        "new_simulation_execution_authorized": False,
        "actual_TEB_qualification_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW.exists(), "review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
