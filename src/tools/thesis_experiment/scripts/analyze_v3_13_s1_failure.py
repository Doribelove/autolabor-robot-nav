#!/usr/bin/env python3
"""Freeze the terminal V3-13-S1 sequential collection gate failure."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
EXECUTION = Path("artifacts/v3/multiactor/v3_13_s1_execution")
READINESS = Path("artifacts/v3/multiactor/v3_13_s1_readiness")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s1_sequential_clone_contract.yaml"
)
D0_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
ANALYZER = Path(
    "src/tools/thesis_experiment/scripts/"
    "analyze_v3_13_s1_failure.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s1_failure.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_s1_failure")
REPORT = ROOT / "failure_attribution.yaml"
FREEZE = ROOT / "terminal_evidence_freeze.yaml"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def episode_roots():
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 32, "episode root count")
    return roots


def raw_resources(roots):
    resources = [
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "collection_summary.yaml",
        CONTRACT,
        D0_CONTRACT,
        ANALYZER,
        REVIEWER,
    ]
    for root in roots:
        resources.extend(
            root / name
            for name in (
                "world.sdf",
                "scene_spec.yaml",
                "actor_specs.yaml",
                "episode_report.yaml",
                "matrix_receipt.yaml",
                "roslaunch.log",
            )
        )
    return resources


def build_outputs():
    roots = episode_roots()
    summary = load_yaml(EXECUTION / "collection_summary.yaml")
    require(
        summary["status"] == "sequential_clone_collection_gate_fail",
        "S1 did not terminate with the expected gate failure",
    )
    require(summary["completed_units"] == 32, "completed unit count")
    require(summary["complete_action_support_rate"] == 1.0, "support rate")
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(events[-1]["event"] == "execution_complete", "journal terminal")
    require(events[-1]["collection_gate_pass"] is False, "journal gate")
    require(
        not any(
            event["event"] in (
                "terminal_failure",
                "unit_retry",
                "unit_resume",
            )
            for event in events
        ),
        "retry or terminal process failure observed",
    )
    reports = [load_yaml(root / "episode_report.yaml") for root in roots]
    failed_reports = [
        report
        for report in reports
        if not report["episode_collection_pass"]
    ]
    require(len(failed_reports) == 1, "unexpected episode gate failure count")
    failed = failed_reports[0]
    require(failed["seed"] == 7224, "failed identity seed")
    require(failed["block_index"] == 4, "failed identity block")
    require(
        failed["forced_candidate_id"] == "spatial_right_v120",
        "failed identity action",
    )
    ambiguous = [
        {
            "track_id": int(track_id),
            "mode": mode,
            "coordinate_speed_mps": next(
                float(track["coordinate_speed_mps"])
                for track in failed["common_stop_prefix"][
                    "decision_actor_tracks"
                ]
                if int(track["track_id"]) == int(track_id)
            ),
        }
        for track_id, mode in failed[
            "observable_modes_at_branch"
        ].items()
        if str(mode).startswith("ERROR:")
    ]
    require(len(ambiguous) == 1, "ambiguous track count")
    d0 = load_yaml(D0_CONTRACT)
    mode_gate = d0["predictor"]["mode_inference"]
    assertive_lower = float(mode_gate["hesitant_max_speed_mps"]) + float(
        mode_gate["ambiguity_half_width_mps"]
    )
    ambiguous[0]["assertive_lower_bound_mps"] = assertive_lower
    ambiguous[0]["shortfall_to_assertive_mps"] = (
        assertive_lower - ambiguous[0]["coordinate_speed_mps"]
    )
    failed_checks = sorted(
        key for key, value in summary["gate_checks"].items() if not value
    )
    require(
        failed_checks
        == [
            "branch_modes_assertive",
            "paired_decision_actor_coordinate",
            "paired_decision_actor_speed",
        ],
        "unexpected failed gate roster",
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1-D1",
        "status": "terminal_sequential_collection_integrity_gate_fail",
        "execution_release_sha256": sha(
            read(READINESS / "execution_release.yaml")
        ),
        "collection_summary_sha256": sha(
            read(EXECUTION / "collection_summary.yaml")
        ),
        "authorized_units": 32,
        "consumed_units": 32,
        "completed_units": 32,
        "forfeited_units": 0,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "complete_action_support_rate": summary[
            "complete_action_support_rate"
        ],
        "episode_collection_pass_count": len(reports)
        - len(failed_reports),
        "episode_collection_fail_count": len(failed_reports),
        "failed_checks": failed_checks,
        "maximum_decision_ego_x_range_m": summary[
            "maximum_decision_ego_x_range_m"
        ],
        "maximum_decision_actor_coordinate_range_m": summary[
            "maximum_decision_actor_coordinate_range_m"
        ],
        "maximum_decision_actor_speed_range_mps": summary[
            "maximum_decision_actor_speed_range_mps"
        ],
        "unique_ambiguous_identity": {
            "seed": failed["seed"],
            "block_index": failed["block_index"],
            "scene_id": failed["scene_id"],
            "pre_action_wait_s": failed["pre_action_wait_s"],
            "forced_candidate_id": failed["forced_candidate_id"],
            "ambiguous_tracks": ambiguous,
        },
        "attribution": {
            "primary": (
                "branch decisions were triggered relative to a "
                "variable warmup-gate time"
            ),
            "secondary": (
                "the instantaneous backward derivative is phase-sensitive "
                "at the fail-closed observable-mode boundary"
            ),
            "process_or_gazebo_crash": False,
            "candidate_execution_failure": False,
            "support_population_missing": False,
            "posthoc_gate_lowering_allowed": False,
        },
        "prospective_single_repair": {
            "fixed_absolute_sim_clock_branch_barrier": True,
            "causal_windowed_actor_speed_estimate": True,
            "all_original_matching_thresholds_retained": True,
            "fresh_seed_and_budget_required": True,
            "S1_identity_or_seed_reuse_forbidden": True,
        },
        "assertive_recovery_predictor_fit_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "perception_claim_authorized": False,
        "performance_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    resources = raw_resources(roots)
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1-D1",
        "status": "terminal_failed_sequential_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "S1_rerun_resume_repair_or_reinterpretation_allowed": False,
    }
    return {REPORT: yaml_bytes(report), FREEZE: yaml_bytes(freeze)}


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    print(
        "status=v3_13_s1_failure_{} report_sha256={} "
        "freeze_sha256={}".format(
            "reproducible" if arguments.check else "frozen",
            sha(read(REPORT)),
            sha(read(FREEZE)),
        )
    )


if __name__ == "__main__":
    main()
