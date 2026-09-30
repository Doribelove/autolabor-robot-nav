#!/usr/bin/env python3
"""Freeze the terminal V3-07-A1 short-trace parser failure."""

import hashlib
import json
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/spatial/v3_07_a1")
EXECUTION = ROOT / "execution"
UNIT = (
    EXECUTION
    / "attempts/v3_07_a1_ident_6541_cautious_spatial_left_v120"
)
REPORT = EXECUTION / "v3_07_a1_terminal_failure_report.yaml"
FREEZE = ROOT / "v3_07_a1_terminal_freeze.sha256"
RESOURCES = [
    Path("config/thesis_experiments/v3/v3_07_a1_spatial_identifiability_contract.yaml"),
    Path("experiments/manifests/v3/v3_07_a1_spatial_identifiability_preregistration.yaml"),
    Path("experiments/manifests/v3/v3_07_a1_simulation_authorization.yaml"),
    ROOT / "offline_readiness/v3_07_a1_execution_release.yaml",
    ROOT / "offline_readiness/v3_07_a1_independent_release_review.yaml",
    ROOT / "offline_readiness/v3_07_a1_readiness_freeze.sha256",
    EXECUTION / "v3_07_a1_attempt_journal.jsonl",
    UNIT / "world.sdf",
    UNIT / "raw_pose_trace.csv",
    UNIT / "gzserver.log",
    UNIT / "done.marker",
]


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def dump(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode()


def write_once(path, payload):
    target = W / path
    if target.exists():
        if target.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        target.write_bytes(payload)


def main():
    journal = [
        json.loads(line)
        for line in read(EXECUTION / "v3_07_a1_attempt_journal.jsonl")
        .decode()
        .splitlines()
        if line.strip()
    ]
    if [row["event"] for row in journal] != [
        "execution_opened",
        "unit_consumed",
    ]:
        raise RuntimeError("A1 journal state differs")
    raw_line_count = len(read(UNIT / "raw_pose_trace.csv").splitlines())
    if raw_line_count != 102:
        raise RuntimeError("A1 raw trace length differs")
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-A1",
        "status": "terminal_short_trace_parser_failure",
        "failure_class": "post_gazebo_pre_receipt_evidence_parser_failure",
        "failed_unit": journal[-1]["unit_id"],
        "failed_sequence": 1,
        "authorized_units": 36,
        "consumed_units": 1,
        "completed_units": 0,
        "forfeited_units": 35,
        "retry_authorized": False,
        "resume_authorized": False,
        "observed_raw_csv_line_count_including_header": raw_line_count,
        "parser_minimum_data_rows": 150,
        "confirmed_cause": (
            "The preregistered 5.0 s scene at 0.05 s sampling produced 101 "
            "data rows, below the inherited load_trace minimum of 150."
        ),
        "gazebo_marker_present": (W / UNIT / "done.marker").is_file(),
        "attempt_receipt_present": (W / UNIT / "attempt_receipt.yaml").is_file(),
        "assessment_present": (W / EXECUTION / "v3_07_a1_identifiability_assessment.yaml").is_file(),
        "qualified_for_spatial_predictor_data_design": False,
        "execution_ready": False,
        "formal_result": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
        "repair_boundary": {
            "same_stage_retry_or_resume_forbidden": True,
            "seeds_6541_through_6544_retired": True,
            "new_stage_and_fresh_seeds_required": True,
            "semantic_factor_and_gates_must_remain_frozen": True,
        },
        "evidence_resources": [
            {
                "path": path.as_posix(),
                "sha256": sha(read(path)),
                "size_bytes": len(read(path)),
            }
            for path in RESOURCES
        ],
    }
    payload = dump(report)
    write_once(REPORT, payload)
    terminal_resources = RESOURCES + [REPORT]
    freeze = (
        "\n".join(
            "{}  {}".format(sha(read(path)), path)
            for path in terminal_resources
        )
        + "\n"
    ).encode()
    write_once(FREEZE, freeze)
    print(
        "status={} report_sha256={} terminal_freeze_sha256={}".format(
            report["status"], sha(payload), sha(freeze)
        )
    )


if __name__ == "__main__":
    main()
