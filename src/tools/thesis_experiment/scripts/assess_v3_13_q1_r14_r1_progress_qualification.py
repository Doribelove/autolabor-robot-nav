#!/usr/bin/env python3
"""Assess the fresh R14-R1 progress-predictor qualification."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r8_recovery_qualification.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_assessor", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
BASE.RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r14_r1_progress_qualification.py"
)
BASE.CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_r1_progress_predictor_qualification_contract.yaml"
)
BASE.MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "expanded_progress_predictor.yaml"
)
BASE.READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_readiness"
)
BASE.EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_r1_progress_qualification"
)
BASE.ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_assessment"
)
BASE.ASSESSMENT = BASE.ROOT / "predictor_qualification.yaml"
BASE.FREEZE = BASE.ROOT / "evidence_freeze.yaml"
BASE.ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r14_r1_progress_qualification.py"
)
BASE.REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r14_r1_qualification_result.py"
)
STAGE = "V3-13-Q1-R14-R1"


def build_outputs():
    original_read = BASE.read
    summary_path = BASE.EXECUTION / "collection_summary.yaml"
    summary_reads = [0]

    def stage_compatible_read(path):
        payload = original_read(path)
        if Path(path) == summary_path:
            summary_reads[0] += 1
            if summary_reads[0] == 1:
                value = yaml.safe_load(payload)
                value["stage"] = "V3-13-Q1-R8"
                value["status"] = (
                    "fresh_expanded_predictor_"
                    "qualification_collection_pass"
                    if value["collection_gate_pass"]
                    else
                    "fresh_expanded_predictor_"
                    "qualification_collection_fail"
                )
                return BASE.yaml_bytes(value)
        return payload

    BASE.read = stage_compatible_read
    try:
        outputs = BASE.build_outputs()
    finally:
        BASE.read = original_read
    BASE.require(
        summary_reads[0] >= 2,
        "summary compatibility and evidence hash were not both exercised",
    )
    assessment = yaml.safe_load(outputs[BASE.ASSESSMENT])
    contract = BASE.load_yaml(BASE.CONTRACT)
    roots = sorted(
        path.relative_to(W)
        for path in (W / BASE.EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    candidate_static = []
    for root in roots:
        report = BASE.load_yaml(root / "episode_report.yaml")
        if report["forced_candidate_id"] == (
            "recovery_evasive_right_v120_r2"
        ):
            candidate_static.append(
                {
                    "block_index": report["block_index"],
                    "scene_id": report["scene_id"],
                    "minimum_trace_static_clearance_m": report[
                        "minimum_trace_static_clearance_m"
                    ],
                }
            )
    BASE.require(len(candidate_static) == 8, "progress candidate count")
    static_pass = all(
        float(row["minimum_trace_static_clearance_m"])
        >= float(
            contract["predictor_gates"][
                "progress_candidate_every_block_static_clearance_m_min"
            ]
        )
        for row in candidate_static
    )
    assessment["stage"] = STAGE
    assessment["gate_checks"][
        "progress_candidate_every_block_static_clearance"
    ] = static_pass
    passed = all(assessment["gate_checks"].values())
    assessment["status"] = (
        "fresh_stage_repair_progress_predictor_qualification_pass"
        if passed
        else "fresh_stage_repair_progress_predictor_qualification_fail"
    )
    assessment["progress_candidate_static_statistics"] = candidate_static
    assessment["minimum_progress_candidate_static_clearance_m"] = min(
        float(row["minimum_trace_static_clearance_m"])
        for row in candidate_static
    )
    assessment["released_summary_stage_compatibility_repaired"] = True
    assessment["qualification_pass"] = passed
    assessment[
        "progress_recovery_planner_integration_design_authorized"
    ] = passed
    assessment.pop(
        "expanded_recovery_planner_integration_design_authorized",
        None,
    )
    assessment_payload = BASE.yaml_bytes(assessment)
    freeze = yaml.safe_load(outputs[BASE.FREEZE])
    freeze["stage"] = STAGE
    freeze["status"] = (
        "terminal_fresh_stage_repair_progress_qualification_freeze"
    )
    freeze["qualification_pass"] = passed
    freeze["seeds7658_through_7665_reusable"] = False
    freeze.pop("seeds7625_through_7632_reusable", None)
    freeze["resources"][-1]["sha256"] = BASE.sha(assessment_payload)
    return {
        BASE.ASSESSMENT: assessment_payload,
        BASE.FREEZE: BASE.yaml_bytes(freeze),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        BASE.write_or_check(path, payload, args.check)
    assessment = yaml.safe_load(outputs[BASE.ASSESSMENT])
    print(
        "status={} assessment_sha256={} freeze_sha256={}".format(
            assessment["status"],
            BASE.sha(outputs[BASE.ASSESSMENT]),
            BASE.sha(outputs[BASE.FREEZE]),
        )
    )


if __name__ == "__main__":
    main()
