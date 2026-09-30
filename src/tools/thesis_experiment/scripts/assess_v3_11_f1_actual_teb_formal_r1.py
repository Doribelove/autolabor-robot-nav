#!/usr/bin/env python3
"""Namespace-only repair wrapper for the frozen V3-11-F1 assessor."""

import argparse
import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_11_f1_actual_teb_formal.py"
)
REPAIR_RELEASE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/"
    "formal_analysis_r1_release.yaml"
)
EXPECTED_BASE_SHA256 = (
    "050d86f230b647d94fd2f697f9e18d33bb2856548efbc73e7c3957e808e81bc3"
)
EXPECTED_I0_ANALYSIS_RELEASE_SHA256 = (
    "61342983f229dd739a4f0330cebaecc94b0ea981be8982d3f0915bc1d6f69929"
)


def load_base():
    spec = importlib.util.spec_from_file_location("v3_11_f1_assessor_i0", BASE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = load_base()
BASE.require(
    BASE.sha256_file(BASE_PATH) == EXPECTED_BASE_SHA256,
    "frozen I0 assessor changed",
)
# The only semantic repair: resolve two intended YAML metadata labels as strings.
BASE.cautious = "cautious"
BASE.hesitant = "hesitant"

# Re-export the frozen implementation for the R1 independent reviewer.
ASSESSMENT_RELATIVE = BASE.ASSESSMENT_RELATIVE
EVIDENCE_FREEZE_RELATIVE = BASE.EVIDENCE_FREEZE_RELATIVE
require = BASE.require
read_bytes = BASE.read_bytes
sha256_bytes = BASE.sha256_bytes
sha256_file = BASE.sha256_file
load_yaml = BASE.load_yaml
yaml_bytes = BASE.yaml_bytes
write_once = BASE.write_once
evidence_freeze = BASE.evidence_freeze


def validate_repair_release(repair_release_sha256):
    path = WORKSPACE / REPAIR_RELEASE_RELATIVE
    require(
        sha256_file(path) == repair_release_sha256,
        "analysis R1 release hash differs",
    )
    value = load_yaml(path)
    require(
        value["status"] == "immutable_offline_namespace_repair_release",
        "analysis R1 release status differs",
    )
    require(
        value["i0_analysis_release_sha256"]
        == EXPECTED_I0_ANALYSIS_RELEASE_SHA256,
        "analysis R1 binds a different I0 release",
    )
    require(
        value["repair_semantics"]
        == {
            "cautious": "cautious",
            "hesitant": "hesitant",
        },
        "analysis R1 repair semantics differ",
    )
    for resource in value["resources"]:
        resource_path = WORKSPACE / resource["path"]
        require(
            sha256_file(resource_path) == resource["sha256"],
            "analysis R1 resource changed: {}".format(resource["path"]),
        )
        require(
            resource_path.stat().st_size == resource["size_bytes"],
            "analysis R1 resource size changed: {}".format(resource["path"]),
        )
    return value


def recompute_assessment(i0_analysis_release_sha256, repair_release_sha256):
    release = validate_repair_release(repair_release_sha256)
    assessment, evidence_paths = BASE.recompute_assessment(
        i0_analysis_release_sha256
    )
    assessment["analysis_repair_release_sha256"] = repair_release_sha256
    assessment["analysis_implementation_repair"] = {
        "stage": "V3-11-F1-ANALYSIS-R1",
        "type": "namespace_only_string_label_injection",
        "preserved_i0_failure_path": (
            "artifacts/v3/actual_teb/v3_11_f1_release/"
            "formal_analysis_i0_failure.yaml"
        ),
        "repair_semantics": release["repair_semantics"],
        "statistic_threshold_or_evidence_changed": False,
    }
    evidence_paths.extend(
        [
            REPAIR_RELEASE_RELATIVE,
            Path(
                "artifacts/v3/actual_teb/v3_11_f1_release/"
                "formal_analysis_i0_failure.yaml"
            ),
        ]
    )
    return assessment, sorted(set(evidence_paths), key=str)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-release-sha256", required=True)
    parser.add_argument("--repair-release-sha256", required=True)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    require(
        arguments.analysis_release_sha256
        == EXPECTED_I0_ANALYSIS_RELEASE_SHA256,
        "unexpected I0 analysis release",
    )
    assessment, evidence_paths = recompute_assessment(
        arguments.analysis_release_sha256,
        arguments.repair_release_sha256,
    )
    assessment_payload = yaml_bytes(assessment)
    freeze = evidence_freeze(evidence_paths, assessment_payload)
    freeze_payload = yaml_bytes(freeze)
    assessment_path = WORKSPACE / ASSESSMENT_RELATIVE
    freeze_path = WORKSPACE / EVIDENCE_FREEZE_RELATIVE
    if arguments.check:
        require(
            read_bytes(assessment_path) == assessment_payload,
            "formal R1 assessment replay differs",
        )
        require(
            read_bytes(freeze_path) == freeze_payload,
            "formal R1 evidence freeze replay differs",
        )
    else:
        write_once(assessment_path, assessment_payload)
        write_once(freeze_path, freeze_payload)
    print(
        "status={} formal_result={} demonstrated={} point={:.6f}% "
        "lower={:.6f}% assessment_sha256={} evidence_freeze_sha256={}".format(
            assessment["status"],
            assessment["formal_result"],
            assessment["performance_improvement_demonstrated"],
            assessment["metrics"]["point_reduction_percent"],
            assessment["metrics"]["bootstrap"][
                "one_sided_lower_reduction_percent"
            ],
            sha256_bytes(assessment_payload),
            sha256_bytes(freeze_payload),
        )
    )


if __name__ == "__main__":
    main()
