#!/usr/bin/env python3
"""Independently review deterministic V3-12-P1 paper exports."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_12_p1_actual_teb_paper_closure.py"
)
REVIEW = (
    W
    / "artifacts/v3/actual_teb/v3_12_p1_paper_closure/"
    "v3_12_p1_independent_paper_review.yaml"
)


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_12_p1_builder", BUILDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    builder = load_builder()
    outputs = builder.compute_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "paper export replay differs: {}".format(relative),
        )
    resources = [
        builder.CONTRACT,
        builder.ASSESSMENT,
        builder.REVIEW,
        builder.EVIDENCE_FREEZE,
        builder.TERMINAL_FREEZE,
        builder.EXECUTION_RELEASE,
        builder.ANALYSIS_REPAIR_RELEASE,
        builder.BUILDER,
        builder.REVIEWER,
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(builder.sha(builder.read(path)), path)
        for path in resources
    ).encode("utf-8")
    builder.require(
        builder.read(builder.MANIFEST) == manifest,
        "paper evidence manifest differs",
    )
    assessment = builder.load_yaml(builder.ASSESSMENT)
    claims = yaml.safe_load(outputs[builder.CLAIMS])
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-12-P1",
        "status": "v3_12_p1_actual_teb_independent_paper_closure_pass",
        "formal_result": assessment["formal_result"],
        "performance_improvement_demonstrated": assessment[
            "performance_improvement_demonstrated"
        ],
        "qualified_claim_matches_formal_result": (
            claims["formal_result"] is True
            and claims["performance_improvement_demonstrated"] is True
        ),
        "export_count": len(outputs),
        "paper_manifest_sha256": hashlib.sha256(manifest).hexdigest(),
        "formal_assessment_sha256": builder.EXPECTED_ASSESSMENT_SHA,
        "formal_review_sha256": builder.EXPECTED_REVIEW_SHA,
        "all_exports_replayed_exactly": True,
        "execution_or_training_performed": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW.exists(), "paper review already exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
