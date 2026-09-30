#!/usr/bin/env python3
"""Independent replay of the R19-R4-D1 terminal-margin diagnosis."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
DIAGNOSER = W / (
    "src/tools/thesis_experiment/scripts/"
    "diagnose_v3_13_q1_r19_r4_terminal_margin.py"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r19_r4_d1_diagnoser", DIAGNOSER
    )
    diagnoser = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(diagnoser)
    report_payload, manifest_payload = diagnoser.build_outputs()
    diagnoser.require(
        diagnoser.read(diagnoser.REPORT) == report_payload,
        "diagnosis replay differs",
    )
    diagnoser.require(
        diagnoser.read(diagnoser.MANIFEST) == manifest_payload,
        "manifest replay differs",
    )
    report = yaml.safe_load(report_payload)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R4-D1",
        "status": "independent_terminal_margin_diagnosis_pass",
        "diagnosis_sha256": hashlib.sha256(
            report_payload
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest_payload
        ).hexdigest(),
        "all_eight_r3_boundaries_replayed": True,
        "single_1_16_mm_margin_miss_replayed": True,
        "tolerance_0_15_m_scope_replayed": True,
        "R19_R4_remains_failed": report["R19_R4_remains_failed"],
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = diagnoser.yaml_bytes(value)
    target = W / diagnoser.ROOT / "independent_review.yaml"
    diagnoser.require(not target.exists(), "review exists")
    target.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
