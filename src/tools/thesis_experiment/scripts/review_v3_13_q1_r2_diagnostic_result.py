#!/usr/bin/env python3
"""Independent replay of the Q1-R2 diagnostic analysis."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ANALYZER = W / (
    "src/tools/thesis_experiment/scripts/"
    "analyze_v3_13_q1_r2_diagnostic.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r2_analyzer", ANALYZER
    )
    analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyzer)
    outputs = analyzer.build_outputs()
    for path, payload in outputs.items():
        if (W / path).read_bytes() != payload:
            raise RuntimeError("analysis replay differs")
    analysis = yaml.safe_load(outputs[analyzer.OUTPUT])
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R2-D1",
        "status": "diagnostic_independent_result_review_pass",
        "analysis_sha256": hashlib.sha256(
            outputs[analyzer.OUTPUT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[analyzer.FREEZE]
        ).hexdigest(),
        "root_cause_replayed": analysis["root_cause"],
        "actual_TEB_qualification_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = analyzer.yaml_bytes(value)
    if REVIEW.exists():
        raise RuntimeError("review exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
