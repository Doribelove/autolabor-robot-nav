#!/usr/bin/env python3
"""Independently replay and freeze the V3-11-F1 formal assessment."""

import hashlib
import importlib.util
import os
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
ASSESSOR_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_11_f1_actual_teb_formal.py"
)
REVIEW_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "terminal_freeze.yaml"
)


def load_assessor():
    spec = importlib.util.spec_from_file_location(
        "v3_11_f1_assessor", ASSESSOR_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_once(module, path, payload):
    module.require(not path.exists(), "immutable output exists: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    assessor = load_assessor()
    prior_assessment = assessor.load_yaml(
        WORKSPACE / assessor.ASSESSMENT_RELATIVE
    )
    analysis_release_sha = prior_assessment["analysis_release_sha256"]
    assessment, evidence_paths = assessor.recompute_assessment(
        analysis_release_sha
    )
    assessment_payload = assessor.yaml_bytes(assessment)
    freeze = assessor.evidence_freeze(evidence_paths, assessment_payload)
    freeze_payload = assessor.yaml_bytes(freeze)
    assessor.require(
        assessor.read_bytes(WORKSPACE / assessor.ASSESSMENT_RELATIVE)
        == assessment_payload,
        "independent formal assessment replay differs",
    )
    assessor.require(
        assessor.read_bytes(WORKSPACE / assessor.EVIDENCE_FREEZE_RELATIVE)
        == freeze_payload,
        "independent formal freeze replay differs",
    )
    for resource in freeze["resources"]:
        path = WORKSPACE / resource["path"]
        assessor.require(
            assessor.sha256_file(path) == resource["sha256"],
            "formal frozen resource changed: {}".format(resource["path"]),
        )
        assessor.require(
            path.stat().st_size == resource["size_bytes"],
            "formal frozen resource size changed: {}".format(resource["path"]),
        )
    demonstrated = assessment["performance_improvement_demonstrated"] is True
    formal_result = assessment["formal_result"] is True
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1",
        "status": (
            "independent_actual_teb_formal_meaningful_improvement_review_pass"
            if demonstrated
            else (
                "independent_actual_teb_formal_valid_negative_review_pass"
                if formal_result
                else "independent_actual_teb_formal_integrity_review_fail"
            )
        ),
        "review_method": (
            "full raw-evidence replay, deterministic bootstrap repetition, "
            "and complete evidence-manifest rehash"
        ),
        "execution_release_sha256": assessment["execution_release_sha256"],
        "analysis_release_sha256": analysis_release_sha,
        "assessment_sha256": hashlib.sha256(assessment_payload).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(freeze_payload).hexdigest(),
        "evidence_resource_count": freeze["resource_count"],
        "all_frozen_gates_pass": all(assessment["gates"].values()),
        "formal_result": formal_result,
        "performance_improvement_demonstrated": demonstrated,
        "point_reduction_percent": assessment["metrics"][
            "point_reduction_percent"
        ],
        "one_sided_lower_reduction_percent": assessment["metrics"][
            "bootstrap"
        ]["one_sided_lower_reduction_percent"],
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
    }
    review_payload = assessor.yaml_bytes(review)
    write_once(assessor, WORKSPACE / REVIEW_RELATIVE, review_payload)
    terminal = {
        "schema_version": "1.0",
        "stage": "V3-11-F1",
        "status": "formal_terminal_freeze",
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
    terminal_payload = assessor.yaml_bytes(terminal)
    write_once(
        assessor,
        WORKSPACE / TERMINAL_FREEZE_RELATIVE,
        terminal_payload,
    )
    print(
        "status={} demonstrated={} review_sha256={} "
        "terminal_freeze_sha256={}".format(
            review["status"],
            demonstrated,
            hashlib.sha256(review_payload).hexdigest(),
            hashlib.sha256(terminal_payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
