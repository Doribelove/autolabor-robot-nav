#!/usr/bin/env python3
"""Freeze the post-execution namespace-only analysis repair."""

import hashlib
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT = (
    WORKSPACE
    / "artifacts/v3/actual_teb/v3_11_f1_release/"
    "formal_analysis_r1_release.yaml"
)
I0_ANALYSIS_RELEASE = (
    WORKSPACE
    / "artifacts/v3/actual_teb/v3_11_f1_release/"
    "formal_analysis_release.yaml"
)
EXPECTED_I0_ANALYSIS_RELEASE_SHA256 = (
    "61342983f229dd739a4f0330cebaecc94b0ea981be8982d3f0915bc1d6f69929"
)
ROSTER = [
    "artifacts/v3/actual_teb/v3_11_f1_release/formal_analysis_release.yaml",
    "artifacts/v3/actual_teb/v3_11_f1_release/formal_analysis_i0_failure.yaml",
    "src/tools/thesis_experiment/scripts/assess_v3_11_f1_actual_teb_formal.py",
    "src/tools/thesis_experiment/scripts/review_v3_11_f1_actual_teb_formal.py",
    "src/tools/thesis_experiment/scripts/assess_v3_11_f1_actual_teb_formal_r1.py",
    "src/tools/thesis_experiment/scripts/review_v3_11_f1_actual_teb_formal_r1.py",
    "config/thesis_experiments/v3/v3_11_f1_actual_teb_formal_contract.yaml",
    "experiments/manifests/v3/v3_11_f1_actual_teb_formal_release.yaml",
    "artifacts/v3/actual_teb/v3_11_f1_execution/attempt_journal.jsonl",
    "artifacts/v3/actual_teb/v3_11_f1_execution/matrix_summary.yaml",
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    require(path.is_file(), "required file absent: {}".format(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    require(not OUTPUT.exists(), "immutable R1 analysis release already exists")
    require(
        digest(I0_ANALYSIS_RELEASE) == EXPECTED_I0_ANALYSIS_RELEASE_SHA256,
        "I0 analysis release changed",
    )
    require(
        not (
            WORKSPACE
            / "artifacts/v3/actual_teb/v3_11_f1_assessment/"
            "formal_assessment.yaml"
        ).exists(),
        "assessment already exists before repair release",
    )
    resources = []
    for relative in ROSTER:
        path = WORKSPACE / relative
        resources.append(
            {
                "path": relative,
                "sha256": digest(path),
                "size_bytes": path.stat().st_size,
            }
        )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1-ANALYSIS-R1",
        "status": "immutable_offline_namespace_repair_release",
        "i0_analysis_release_sha256": EXPECTED_I0_ANALYSIS_RELEASE_SHA256,
        "preserved_failure_path": (
            "artifacts/v3/actual_teb/v3_11_f1_release/"
            "formal_analysis_i0_failure.yaml"
        ),
        "repair_semantics": {
            "cautious": "cautious",
            "hesitant": "hesitant",
        },
        "repair_scope": (
            "inject two intended string metadata labels into the frozen "
            "assessor module namespace"
        ),
        "statistic_threshold_or_execution_evidence_change_authorized": False,
        "simulation_rerun_or_selective_exclusion_authorized": False,
        "resources": resources,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    temporary = OUTPUT.with_suffix(OUTPUT.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(OUTPUT))
    print(
        "status={} resources={} repair_release_sha256={}".format(
            value["status"],
            len(resources),
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
