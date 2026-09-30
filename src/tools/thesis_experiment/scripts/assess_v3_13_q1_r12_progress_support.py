#!/usr/bin/env python3
"""Assess R12 by replaying the frozen R6 support assessor."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r6_recovery_support.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r6_frozen_assessor", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
BASE.RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r12_progress_support.py"
)
BASE.CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r12_progress_recovery_support_contract.yaml"
)
BASE.READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r12_readiness"
)
BASE.EXECUTION = Path(
    "artifacts/v3/multiactor/v3_13_q1_r12_progress_support"
)
BASE.ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r12_assessment"
)
BASE.ASSESSMENT = BASE.ROOT / "support_assessment.yaml"
BASE.FREEZE = BASE.ROOT / "evidence_freeze.yaml"
BASE.ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r12_progress_support.py"
)
BASE.REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r12_support_result.py"
)


def build_outputs():
    outputs = BASE.build_outputs()
    assessment = yaml.safe_load(outputs[BASE.ASSESSMENT])
    assessment["stage"] = "V3-13-Q1-R12"
    assessment["status"] = (
        "fresh_progress_five_action_support_assessment_pass"
        if assessment["collection_gate_pass"]
        else "fresh_progress_five_action_support_assessment_fail"
    )
    assessment["fit_scope"] = (
        "assertive_head_for_recovery_evasive_right_v120_r2_only"
    )
    assessment["all_R7_heads_must_remain_byte_equivalent"] = (
        assessment.pop("all_S3_heads_must_remain_byte_equivalent")
    )
    reports = [
        receipt["episode_report"]
        for receipt in [
            BASE.load_yaml(root / "matrix_receipt.yaml")
            for root in sorted(
                path.relative_to(W)
                for path in (W / BASE.EXECUTION / "episodes").iterdir()
                if path.is_dir()
            )
        ]
    ]
    assessment["minimum_trace_static_clearance_m"] = min(
        float(report["minimum_trace_static_clearance_m"])
        for report in reports
    )
    assessment_payload = BASE.yaml_bytes(assessment)
    freeze = yaml.safe_load(outputs[BASE.FREEZE])
    freeze["stage"] = "V3-13-Q1-R12"
    freeze["status"] = "terminal_progress_support_evidence_freeze"
    freeze["seeds7642_through_7649_reusable"] = False
    freeze.pop("seeds7617_through_7624_reusable")
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
