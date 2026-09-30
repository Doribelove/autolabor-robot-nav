#!/usr/bin/env python3
"""Independent deterministic review of V3-11-Q1-R1 assessment and freeze."""

import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
ASSESSOR_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_11_q1_r1_actual_teb.py"
)
REVIEW_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "terminal_freeze.yaml"
)


def load_assessor():
    spec = importlib.util.spec_from_file_location("v3_11_q1_r1_assessor", ASSESSOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_once(module, path, payload):
    module.require(not path.exists(), "immutable output already exists: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    assessor = load_assessor()
    assessment, evidence_paths = assessor.recompute_assessment()
    assessment_payload = assessor.yaml_bytes(assessment)
    freeze = assessor.build_evidence_freeze(evidence_paths, assessment_payload)
    freeze_payload = assessor.yaml_bytes(freeze)
    assessor.require(
        assessor.read_once(WORKSPACE / assessor.ASSESSMENT_RELATIVE)
        == assessment_payload,
        "independent assessment replay differs",
    )
    assessor.require(
        assessor.read_once(WORKSPACE / assessor.EVIDENCE_FREEZE_RELATIVE)
        == freeze_payload,
        "independent evidence-freeze replay differs",
    )
    for resource in freeze["resources"]:
        path = WORKSPACE / resource["path"]
        assessor.require(
            assessor.sha256_file(path) == resource["sha256"],
            "frozen resource differs: {}".format(resource["path"]),
        )
        assessor.require(
            path.stat().st_size == resource["size_bytes"],
            "frozen resource size differs: {}".format(resource["path"]),
        )
    passed = assessment["overall_pass"] is True
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-Q1-R1",
        "status": (
            "v3_11_q1_r1_independent_actual_teb_qualification_review_pass"
            if passed
            else "v3_11_q1_r1_independent_actual_teb_qualification_review_valid_negative"
        ),
        "review_method": (
            "independent deterministic replay and complete evidence-manifest rehash"
        ),
        "assessment_sha256": hashlib.sha256(assessment_payload).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(freeze_payload).hexdigest(),
        "evidence_resource_count": freeze["resource_count"],
        "all_assessment_gates_pass": passed,
        "qualified_for_fresh_formal_actual_teb_evaluation": passed,
        "formal_performance_claim_authorized": False,
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
    }
    review_payload = assessor.yaml_bytes(review)
    review_path = WORKSPACE / REVIEW_RELATIVE
    write_once(assessor, review_path, review_payload)
    terminal = {
        "schema_version": "1.0",
        "stage": "V3-11-Q1-R1",
        "status": "qualification_terminal_freeze",
        "resources": [
            {
                "path": str(assessor.ASSESSMENT_RELATIVE),
                "sha256": hashlib.sha256(assessment_payload).hexdigest(),
            },
            {
                "path": str(assessor.EVIDENCE_FREEZE_RELATIVE),
                "sha256": hashlib.sha256(freeze_payload).hexdigest(),
            },
            {
                "path": str(REVIEW_RELATIVE),
                "sha256": hashlib.sha256(review_payload).hexdigest(),
            },
            {
                "path": str(ASSESSOR_PATH.relative_to(WORKSPACE)),
                "sha256": assessor.sha256_file(ASSESSOR_PATH),
            },
            {
                "path": str(Path(__file__).resolve().relative_to(WORKSPACE)),
                "sha256": assessor.sha256_file(Path(__file__).resolve()),
            },
        ],
    }
    terminal_payload = yaml.safe_dump(
        terminal, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    write_once(
        assessor,
        WORKSPACE / TERMINAL_FREEZE_RELATIVE,
        terminal_payload,
    )
    print(
        "status={} review_sha256={} terminal_freeze_sha256={}".format(
            review["status"],
            hashlib.sha256(review_payload).hexdigest(),
            hashlib.sha256(terminal_payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
