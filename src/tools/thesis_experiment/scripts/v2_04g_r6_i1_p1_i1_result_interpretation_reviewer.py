#!/usr/bin/env python3
"""Pure-offline deterministic reviewer for the P1 result/paper closure."""

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p1_i1_offline_result_interpretation_paper_contract.yaml"
)
REPORT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_i1_interpretation/"
    "v2_04g_p1_i1_result_interpretation_review.yaml"
)
SCHEDULE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
JOURNAL_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/journals"
)
RELEASE_SHA256 = "05a6efe21a4c110be667b0873d8ac1b26ac692feefea8932f40a506ca9b9027b"
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class ReviewError(ValueError):
    """Raised when the frozen offline interpretation closure differs."""


def require(condition, message):
    if not condition:
        raise ReviewError(message)


def sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def safe_relative(value):
    require(type(value) is str and value != "", "binding path is not a non-empty string")
    relative = Path(value)
    require(not relative.is_absolute() and ".." not in relative.parts, "binding path is not safe")
    return relative


def stable_read(workspace, relative):
    relative = safe_relative(relative.as_posix() if isinstance(relative, Path) else relative)
    current = workspace
    for component in relative.parts:
        current = current / component
        require(not current.is_symlink(), "symbolic link forbidden: {}".format(relative))
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(workspace / relative), flags)
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "binding is not a regular file: {}".format(relative))
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "binding changed during read: {}".format(relative),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(relative))
        return payload
    finally:
        os.close(descriptor)


def load_yaml(payload, label):
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise ReviewError("{} YAML parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} is not a mapping".format(label))
    return value


def verify_binding(workspace, binding, label):
    require(type(binding) is dict and set(binding) == {"path", "sha256"}, "{} binding schema differs".format(label))
    require(type(binding["sha256"]) is str and HEX_SHA256.fullmatch(binding["sha256"]), "{} SHA differs".format(label))
    relative = safe_relative(binding["path"])
    payload = stable_read(workspace, relative)
    require(sha256(payload) == binding["sha256"], "{} hash differs".format(label))
    return relative, payload


def verify_sha_manifest(workspace, binding, expected_count, label):
    _, payload = verify_binding(workspace, binding, label)
    entries = []
    for line in payload.decode("utf-8").splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, "{} line schema differs".format(label))
        expected, path_text = match.groups()
        relative = safe_relative(path_text)
        actual = stable_read(workspace, relative)
        require(sha256(actual) == expected, "{} target hash differs: {}".format(label, relative))
        entries.append({"path": relative.as_posix(), "sha256": expected})
    require(len(entries) == expected_count, "{} entry count differs".format(label))
    require(len({item["path"] for item in entries}) == expected_count, "{} contains duplicate paths".format(label))
    return entries


def expected_contract_schema(contract):
    required = {
        "schema_version", "architecture_generation", "stage", "status", "offline_only",
        "formal_result", "runtime_ready", "execution_ready", "execution_authorized",
        "evidence_budget_authorized", "training_allowed", "real_vehicle_use_forbidden",
        "source_evidence", "paper_artifacts", "result_interpretation", "claim_closure",
        "forbidden_side_effects", "reviewer",
    }
    require(set(contract) == required, "contract top-level schema differs")
    require(contract["schema_version"] == "1.0", "contract schema version differs")
    require(contract["architecture_generation"] == "v2", "architecture generation differs")
    require(contract["stage"] == "V2-04G-P1-I1", "contract stage differs")
    require(contract["status"] == "offline_result_interpretation_paper_contract_frozen", "contract status differs")
    require(contract["offline_only"] is True, "closure is not offline-only")
    require(contract["formal_result"] is True, "formal completed result was not preserved")
    require(contract["runtime_ready"] is False, "runtime readiness was upgraded")
    require(contract["execution_ready"] is False and contract["execution_authorized"] is False, "execution authority exists")
    require(contract["evidence_budget_authorized"] == 0, "closure has evidence budget")
    require(contract["training_allowed"] is False, "closure permits training")
    require(contract["real_vehicle_use_forbidden"] is True, "closure permits real vehicle use")

    require(set(contract["source_evidence"]) == {
        "p1_terminal_evidence_freeze", "p1_release", "p1_schedule", "p1_stage_report",
        "p1_normalized_matrix", "p1_raw_replay_report", "p1_four_gate_assessment",
        "p1_execution_handoff", "i5_evidence_freeze", "i5_execution_report",
    }, "source evidence roster differs")
    require(set(contract["paper_artifacts"]) == {
        "draft_readme", "system_design_chapter", "integrity_method_chapter",
        "i5_results_chapter", "p1_results_chapter", "claims_evidence_matrix",
    }, "paper artifact roster differs")
    require(set(contract["reviewer"]) == {"path", "sha256"}, "reviewer binding differs")

    interpretation = contract["result_interpretation"]
    require(set(interpretation) == {
        "study_status", "formal_study_complete", "performance_improvement_demonstrated",
        "claim_authorized", "four_gate_decision", "efficiency", "mechanism", "safety_and_task",
        "decision_reason",
    }, "result interpretation schema differs")
    require(interpretation["study_status"] == "formal_complete_performance_improvement_not_demonstrated", "study status differs")
    require(interpretation["formal_study_complete"] is True, "formal study completion differs")
    require(interpretation["performance_improvement_demonstrated"] is False, "performance claim was upgraded")
    require(interpretation["claim_authorized"] is False, "claim authority was upgraded")
    require(interpretation["four_gate_decision"] == {
        "integrity": True,
        "safety_and_task_noninferiority": True,
        "efficiency_superiority": False,
        "mechanism_reduction": True,
    }, "four-gate decision differs")
    require(interpretation["efficiency"] == {
        "geometric_time_reduction_fraction": 0.03762364346211233,
        "one_sided_95_percent_lower_fraction": 0.027903858836313078,
        "point_threshold_min": 0.05,
        "fixed_sequence_adjusted_p_value": 9.99990000099999e-06,
        "pass": False,
    }, "efficiency interpretation differs")
    require(interpretation["mechanism"] == {
        "control_mean_fraction": 0.17672110140675926,
        "treatment_mean_fraction": 0.0,
        "relative_reduction_fraction": 1.0,
        "paired_absolute_decrease_one_sided_95_percent_lower": 0.17176223658064604,
        "fixed_sequence_adjusted_p_value": 9.99990000099999e-06,
        "pass": True,
    }, "mechanism interpretation differs")
    require(interpretation["safety_and_task"] == {
        "legacy_successes": 90,
        "treatment_successes": 90,
        "legacy_collisions": 0,
        "treatment_collisions": 0,
        "harmful_discordances": 0,
        "clearance_paired_mean_difference_m": -0.029086489773633224,
        "clearance_one_sided_95_percent_lower_m": -0.04289094138217517,
        "true_conflict_control_match_fraction": 1.0,
        "true_conflict_treatment_match_fraction": 1.0,
        "pass": True,
    }, "safety/task interpretation differs")
    require(
        interpretation["decision_reason"]
        == "efficiency_point_estimate_below_preregistered_five_percent_minimum_effect",
        "decision reason differs",
    )

    claim = contract["claim_closure"]
    require(set(claim) == {
        "claim_population", "fixed_teb_status", "allowed_claims", "forbidden_claims",
        "required_thesis_wording",
    }, "claim closure schema differs")
    require(
        claim["claim_population"]
        == "three_frozen_dynamic_templates_under_fresh_simulator_stochastic_seeds_5201_5290",
        "claim population differs",
    )
    require(claim["fixed_teb_status"] == "contextual_only_excluded_from_confirmatory_claim", "fixed TEB status differs")
    require(claim["allowed_claims"] == [
        "p1_exact_270_row_formal_simulation_study_completed",
        "p1_safety_and_task_noninferiority_gate_passed_within_frozen_simulation_distribution",
        "p1_semantic_clear_overlay_mechanism_reduction_gate_passed",
        "p1_restricted_completion_time_directional_reduction_was_3_762_percent",
        "p1_performance_improvement_was_not_demonstrated_because_the_five_percent_minimum_effect_was_not_met",
    ], "allowed claims differ")
    require(claim["forbidden_claims"] == [
        "navigation_performance_improvement",
        "whole_system_superiority",
        "safety_superiority",
        "cross_family_geometry_or_timing_generalization",
        "learning_or_training_gain",
        "real_vehicle_performance_or_safety",
        "formal_safety_guarantee",
        "winner_or_best_profile",
        "deployment_readiness",
    ], "forbidden claims differ")

    side_effects = contract["forbidden_side_effects"]
    require(set(side_effects) == {
        "rerun_or_resume_i5", "rerun_resume_or_extend_p1", "create_execution_release",
        "create_attempt_consumption_or_journal", "start_ros_or_gazebo", "start_training",
        "consume_seed_or_evidence_budget", "change_preregistered_thresholds", "select_or_drop_samples",
        "connect_real_vehicle", "write_online_teb_parameters",
    }, "forbidden side-effect schema differs")
    require(all(value is True for value in side_effects.values()), "a forbidden side effect was enabled")


def verify_terminal_state(workspace, source):
    schedule = load_yaml(stable_read(workspace, safe_relative(source["p1_schedule"]["path"])), "schedule")
    stage = load_yaml(stable_read(workspace, safe_relative(source["p1_stage_report"]["path"])), "stage report")
    replay = load_yaml(stable_read(workspace, safe_relative(source["p1_raw_replay_report"]["path"])), "raw replay")
    assessment = load_yaml(stable_read(workspace, safe_relative(source["p1_four_gate_assessment"]["path"])), "assessment")
    matrix = json.loads(stable_read(workspace, safe_relative(source["p1_normalized_matrix"]["path"])).decode("utf-8"))
    require(type(matrix) is list and len(matrix) == 270, "normalized matrix row count differs")
    require(type(schedule.get("episodes")) is list and len(schedule["episodes"]) == 270, "schedule row count differs")
    identity_keys = ("sequence", "method_id", "scene_id", "seed", "attempt", "block_id", "scene_role")
    for planned, observed in zip(schedule["episodes"], matrix):
        require(
            tuple(planned[key] for key in identity_keys) == tuple(observed[key] for key in identity_keys),
            "normalized identity differs at sequence {}".format(planned["sequence"]),
        )
        require(observed["terminal_status"] == "completed", "terminal status differs")
        require(observed["consumed"] is True and observed["raw_resources_complete"] is True, "row completeness differs")
        require(observed["integrity_failures"] == [], "row has integrity failure")
    require([row["sequence"] for row in matrix] == list(range(1, 271)), "sequence order differs")
    require(Counter(row["method_id"] for row in matrix) == Counter({
        "r6_semantics_legacy_control": 90,
        "r6_semantics_circle_contact": 90,
        "fixed_teb_reference": 90,
    }), "method counts differ")
    require(Counter(row["scene_role"] for row in matrix) == Counter({
        "single_true_conflict": 90,
        "multi_true_conflict": 90,
        "time_separated_semantic_clear": 90,
    }), "role counts differ")
    require({row["seed"] for row in matrix} == set(range(5201, 5291)), "fresh seed firewall differs")
    require({row["attempt"] for row in matrix} == {1}, "attempt policy differs")
    require(len({row["block_id"] for row in matrix}) == 90, "block count differs")

    require(stage["status"] == "exact_270_row_execution_and_raw_replay_complete", "stage status differs")
    require(stage["release_sha256"] == RELEASE_SHA256, "stage release differs")
    require(stage["evidence_units_consumed"] == 270, "consumed count differs")
    require(stage["evidence_units_completed"] == 270, "completed count differs")
    require(stage["evidence_units_forfeited"] == 0, "forfeited count differs")
    require(stage["terminal_integrity_failures"] == [], "terminal integrity failures exist")
    require(len(stage["attempt_ledger"]) == 270, "attempt ledger count differs")
    require(stage["training_budget_steps"] == 0, "training budget differs")
    require(stage["real_vehicle_used"] is False and stage["i5_rerun_or_resume_performed"] is False, "boundary violation recorded")

    gates = {
        "integrity": True,
        "safety_and_task_noninferiority": True,
        "efficiency_superiority": False,
        "mechanism_reduction": True,
    }
    require(assessment["assessment_status"] == "performance_improvement_not_demonstrated", "assessment status differs")
    require(assessment["formal_study_complete"] is True and assessment["formal_result"] is True, "formal study status differs")
    require(assessment["four_gate_decision"] == gates, "assessment gates differ")
    require(assessment["performance_improvement_demonstrated"] is False, "performance claim differs")
    require(assessment["claim_authorized"] is False and assessment["runtime_ready"] is False, "claim boundary differs")
    require(replay["status"] == "raw_replay_complete", "raw replay status differs")
    require(replay["episode_count"] == 270 and replay["raw_resources_rehashed"] == 1890, "raw replay counts differ")
    require(len(replay["episode_audits"]) == 270, "episode audit count differs")
    require(all(set(item["raw_sha256"]) == {
        "activation", "evaluation", "trace", "clearance", "overlay", "teardown_receipt", "process_log"
    } for item in replay["episode_audits"]), "raw resource roster differs")
    return schedule, matrix, replay, assessment


def deterministic_replay(workspace, schedule, source):
    from thesis_experiment.v2_04g_r6_i1_p1_e1_release import validate_release
    from thesis_experiment.v2_04g_r6_i1_p1_e1_replay import build_terminal_assessment

    release = validate_release(RELEASE_SHA256, workspace, require_fresh_state=False)
    normalized, replay, assessment = build_terminal_assessment(
        schedule, JOURNAL_RELATIVE, RELEASE_SHA256, workspace
    )
    expected = {
        "p1_normalized_matrix": (
            json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
        ).encode("utf-8"),
        "p1_raw_replay_report": yaml.safe_dump(replay, sort_keys=False).encode("utf-8"),
        "p1_four_gate_assessment": yaml.safe_dump(assessment, sort_keys=False).encode("utf-8"),
    }
    for label, payload in expected.items():
        actual = stable_read(workspace, safe_relative(source[label]["path"]))
        require(payload == actual, "deterministic replay byte comparison failed: {}".format(label))
    return release, replay, assessment


def build_report(workspace):
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    contract = load_yaml(contract_payload, "contract")
    expected_contract_schema(contract)
    source_verified = {}
    for label, binding in contract["source_evidence"].items():
        relative, payload = verify_binding(workspace, binding, label)
        source_verified[label] = {"path": relative.as_posix(), "sha256": sha256(payload)}
    paper_verified = {}
    for label, binding in contract["paper_artifacts"].items():
        relative, payload = verify_binding(workspace, binding, label)
        paper_verified[label] = {"path": relative.as_posix(), "sha256": sha256(payload)}
    verify_binding(workspace, contract["reviewer"], "reviewer")
    p1_freeze = verify_sha_manifest(
        workspace, contract["source_evidence"]["p1_terminal_evidence_freeze"], 10,
        "P1 terminal evidence freeze",
    )
    i5_freeze = verify_sha_manifest(
        workspace, contract["source_evidence"]["i5_evidence_freeze"], 13,
        "I5 evidence freeze",
    )
    schedule, matrix, replay, assessment = verify_terminal_state(workspace, contract["source_evidence"])
    release_validation, replay_rebuilt, assessment_rebuilt = deterministic_replay(
        workspace, schedule, contract["source_evidence"]
    )
    require(replay == replay_rebuilt and assessment == assessment_rebuilt, "rebuilt terminal objects differ")
    require(contract["result_interpretation"]["four_gate_decision"] == assessment["four_gate_decision"], "contract gate interpretation differs")
    require(contract["result_interpretation"]["efficiency"]["geometric_time_reduction_fraction"] == assessment["efficiency"]["geometric_time_reduction_fraction"], "contract efficiency differs")
    require(contract["result_interpretation"]["mechanism"]["relative_reduction_fraction"] == assessment["mechanism"]["relative_reduction_fraction"], "contract mechanism differs")

    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P1-I1",
        "review_id": "fam_teb_v2_04g_p1_i1_result_interpretation_paper_review_1",
        "status": "offline_result_interpretation_paper_closure_pass",
        "review_result": "pass",
        "offline_only": True,
        "formal_result": True,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_started": False,
        "real_vehicle_used": False,
        "contract": {
            "path": CONTRACT_RELATIVE.as_posix(),
            "sha256": sha256(contract_payload),
        },
        "source_evidence_verified": source_verified,
        "freeze_verification": {
            "p1_terminal_resources_verified": len(p1_freeze),
            "i5_frozen_resources_verified": len(i5_freeze),
        },
        "terminal_replay_verification": {
            "release_sha256": RELEASE_SHA256,
            "release_local_resources_verified": release_validation["local_resources_verified"],
            "release_external_resources_verified": len(release_validation["closure"]["external_resources"]),
            "schedule_rows": len(schedule["episodes"]),
            "normalized_rows": len(matrix),
            "paired_blocks": len({row["block_id"] for row in matrix}),
            "fresh_seed_count": len({row["seed"] for row in matrix}),
            "journals_replayed": len(replay["episode_audits"]),
            "raw_resources_rehashed": replay["raw_resources_rehashed"],
            "normalized_matrix_raw_replay_and_assessment_byte_equal": True,
            "terminal_integrity_failures": [],
        },
        "result_interpretation": {
            "study_status": contract["result_interpretation"]["study_status"],
            "four_gate_decision": assessment["four_gate_decision"],
            "performance_improvement_demonstrated": False,
            "claim_authorized": False,
            "efficiency_geometric_reduction_fraction": assessment["efficiency"]["geometric_time_reduction_fraction"],
            "efficiency_preregistered_point_threshold": 0.05,
            "efficiency_decision": "fail_below_preregistered_minimum_effect",
            "mechanism_relative_reduction_fraction": assessment["mechanism"]["relative_reduction_fraction"],
            "claim_population": contract["claim_closure"]["claim_population"],
            "fixed_teb_status": contract["claim_closure"]["fixed_teb_status"],
            "allowed_claims": contract["claim_closure"]["allowed_claims"],
            "forbidden_claims": contract["claim_closure"]["forbidden_claims"],
            "required_thesis_wording": contract["claim_closure"]["required_thesis_wording"],
        },
        "paper_artifacts_verified": paper_verified,
        "side_effect_audit": {
            "i5_rerun_or_resume": False,
            "p1_rerun_resume_or_extension": False,
            "ros_or_gazebo_started_by_closure": False,
            "training_started": False,
            "seed_or_evidence_budget_consumed_by_closure": False,
            "new_execution_release_attempt_consumption_or_journal_created": False,
            "real_vehicle_connected": False,
            "online_teb_parameters_written": False,
        },
        "closure_pass": True,
    }


def serialized(report):
    return yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(WORKSPACE))
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--emit", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    report = build_report(workspace)
    payload = serialized(report)
    if args.emit:
        sys.stdout.buffer.write(payload)
        return 0
    persisted = stable_read(workspace, REPORT_RELATIVE)
    require(persisted == payload, "persisted review differs from deterministic rebuild")
    print(report["status"])
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ReviewError as exc:
        print("P1-I1 review failed: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
