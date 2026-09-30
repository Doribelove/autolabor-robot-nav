#!/usr/bin/env python3
"""Pure-offline attribution of the lone R19-R4 terminal-guard miss."""

import argparse
import hashlib
import math
import os
from pathlib import Path

import yaml

from v3_scope_planner import progress_recovery_candidate_core


W = Path("/home/robot/robot_ws_base_rl")
EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r4_prefix_aligned_support"
)
ASSESSMENT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r4_assessment"
)
GEOMETRY = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r4_d1_terminal_margin_diagnosis"
)
REPORT = ROOT / "terminal_margin_diagnosis.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
INPUTS = [
    EXECUTION / "collection_summary.yaml",
    ASSESSMENT / "support_assessment.yaml",
    ASSESSMENT / "evidence_freeze.yaml",
    ASSESSMENT / "independent_result_review.yaml",
    GEOMETRY,
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "goal_aligned_execution_adapter_r1.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_boundary_atomic_preflight_episode.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "diagnose_v3_13_q1_r19_r4_terminal_margin.py"
    ),
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def build_outputs():
    summary = yaml.safe_load(read(INPUTS[0]))
    assessment = yaml.safe_load(read(INPUTS[1]))
    geometry = yaml.safe_load(read(GEOMETRY))
    require(summary["completed_units"] == 40, "collection incomplete")
    require(not summary["collection_gate_pass"], "unexpected pass")
    require(
        summary["r3_terminal_guard_execution_rate"] == 0.875
        and summary["r3_boundary_atomic_execution_rate"] == 0.875,
        "terminal rates differ",
    )
    roots = sorted(
        path
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.name.endswith(
            "recovery_evasive_right_v120_r3"
        )
    )
    require(len(roots) == 8, "R3 episode count differs")
    rows = []
    for root in roots:
        report = yaml.safe_load(
            (root / "episode_report.yaml").read_bytes()
        )
        planner = yaml.safe_load(
            Path(report["planner_contract"]["path"]).read_bytes()
        )
        candidate_set = (
            progress_recovery_candidate_core.generate_candidate_set(
                report["branch_snapshot"],
                planner,
                geometry,
                recovery_context=True,
            )
        )
        candidate = next(
            item
            for item in candidate_set["candidates"]
            if item["candidate_id"]
            == "recovery_evasive_right_v120_r3"
        )
        require(
            candidate["candidate_digest"] == report["candidate_digest"],
            "candidate replay differs",
        )
        endpoint = candidate["points"][-1]
        last = report["result"]["trace"][-1]["ego"]
        distance = math.hypot(
            float(endpoint["x_m"]) - float(last["x_m"]),
            float(endpoint["y_m"]) - float(last["y_m"]),
        )
        rows.append(
            {
                "block_index": int(report["block_index"]),
                "seed": int(report["seed"]),
                "terminal_distance_at_boundary_m": distance,
                "guard_activation_count": int(
                    report["terminal_guard_activation_count"]
                ),
                "boundary_evaluation_count": int(
                    report["boundary_terminal_evaluation_count"]
                ),
                "boundary_trigger_count": int(
                    report["boundary_terminal_trigger_count"]
                ),
                "would_trigger_at_tolerance_0_15_m": distance <= 0.15,
            }
        )
    misses = [
        row for row in rows if row["guard_activation_count"] == 0
    ]
    require(len(misses) == 1, "miss count differs")
    miss = misses[0]
    require(
        miss["block_index"] == 5
        and miss["seed"] == 7691
        and 0.12 < miss["terminal_distance_at_boundary_m"] < 0.122,
        "terminal-margin attribution differs",
    )
    require(
        all(row["would_trigger_at_tolerance_0_15_m"] for row in rows),
        "0.15 m replay does not close all rows",
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R4-D1",
        "status": "offline_terminal_margin_diagnosis_pass",
        "upstream": {
            "collection_summary_sha256": sha(read(INPUTS[0])),
            "support_assessment_sha256": sha(read(INPUTS[1])),
            "evidence_freeze_sha256": sha(read(INPUTS[2])),
            "independent_result_review_sha256": sha(read(INPUTS[3])),
        },
        "r3_rows_replayed": rows,
        "causal_attribution": {
            "failed_block": 5,
            "failed_seed": 7691,
            "frozen_terminal_tolerance_m": 0.12,
            "observed_terminal_distance_at_boundary_m": miss[
                "terminal_distance_at_boundary_m"
            ],
            "miss_margin_m": (
                miss["terminal_distance_at_boundary_m"] - 0.12
            ),
            "boundary_was_evaluated": True,
            "candidate_geometry_or_matching_failure": False,
            "mechanism": (
                "the boundary sample was 1.16 mm outside the frozen "
                "terminal tolerance, so the correctly ordered evaluator "
                "did not trigger before braking"
            ),
        },
        "single_factor_repair_design": {
            "factor": "r3_terminal_tolerance_m",
            "control": 0.12,
            "treatment": 0.15,
            "offline_all_eight_boundary_samples_trigger": True,
            "stop_occurs_earlier_not_later": True,
            "candidate_geometry_unchanged": True,
            "horizon_and_actor_model_unchanged": True,
            "legacy_action_execution_unchanged": True,
            "next_stage": "one_fresh_r3_only_runtime_preflight",
        },
        "R19_R4_remains_failed": True,
        "R19_R4_seed_reuse_authorized": False,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    report_payload = yaml_bytes(report)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in INPUTS
    )
    manifest += "{}  {}\n".format(sha(report_payload), REPORT)
    return report_payload, manifest.encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report, manifest = build_outputs()
    write_or_check(REPORT, report, args.check)
    write_or_check(MANIFEST, manifest, args.check)
    print(
        "status=offline_terminal_margin_diagnosis_pass "
        "diagnosis_sha256={} manifest_sha256={}".format(
            sha(report), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
