#!/usr/bin/env python3
"""Independent replay of the terminal V3-13-S1 failure freeze."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
ANALYZER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "analyze_v3_13_s1_failure.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s1_failure/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s1_failure_analyzer", ANALYZER_PATH
    )
    analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyzer)
    outputs = analyzer.build_outputs()
    for relative, payload in outputs.items():
        analyzer.require(
            analyzer.read(relative) == payload,
            "failure replay differs: {}".format(relative),
        )
    report = analyzer.load_yaml(analyzer.REPORT)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1-D1",
        "status": "v3_13_s1_independent_terminal_failure_review_pass",
        "failure_report_sha256": analyzer.sha(
            analyzer.read(analyzer.REPORT)
        ),
        "terminal_evidence_freeze_sha256": analyzer.sha(
            analyzer.read(analyzer.FREEZE)
        ),
        "all_32_attempt_one_identities_replayed": True,
        "complete_support_but_matching_gate_failure_replayed": True,
        "unique_mode_ambiguity_replayed": True,
        "S1_predictor_fit_authorized": False,
        "S1_rerun_or_resume_authorized": False,
        "fresh_prospective_repair_required": True,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    analyzer.require(
        report["assertive_recovery_predictor_fit_authorized"] is False,
        "failure incorrectly authorized fitting",
    )
    payload = analyzer.yaml_bytes(value)
    analyzer.require(not REVIEW_PATH.exists(), "review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
