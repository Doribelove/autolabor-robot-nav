#!/usr/bin/env python3
"""Offline-only I6 evidence freeze and result-interpretation reviewer."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


STAGE = "V2-04G-R6-I6"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_r6_i6_"
    "offline_result_interpretation_design_contract.yaml"
)
PERFORMANCE_DESIGN_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i6_future_multiseed_performance_design.yaml"
)
OUTPUT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/"
    "v2_04g_r6_i6_result_interpretation_review.yaml"
)

EXPECTED_ALLOWED_CLAIMS = [
    "i5_completed_all_six_fresh_simulation_integration_units",
    "i5_expected_and_observed_ttc_status_matched_all_six_units",
    "i5_direct_readiness_bootstrap_scene_snapshot_and_teardown_gates_passed",
    "circle_contact_semantics_suppressed_legacy_non_none_overlay_in_the_single_semantic_clear_i5_scene",
    "i5_supports_semantic_and_execution_integration_correctness_only",
]
EXPECTED_FORBIDDEN_CLAIMS = [
    "navigation_performance_improvement",
    "safety_superiority",
    "generalization",
    "winner_or_best_profile",
    "whole_system_superiority",
    "learning_or_training_gain",
    "real_vehicle_validity",
    "deployment_readiness",
]
EXPECTED_SEMANTIC_COUNTS = [
    (1, "r6_semantics_legacy_control", 31, 25),
    (2, "r6_semantics_circle_contact", 30, 19),
    (3, "r6_semantics_circle_contact", 32, 19),
    (4, "r6_semantics_legacy_control", 28, 30),
    (5, "r6_semantics_legacy_control", 0, 18),
    (6, "r6_semantics_circle_contact", 0, 0),
]


class InterpretationReviewError(ValueError):
    """Raised when the frozen I5 evidence or I6 design drifts."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise InterpretationReviewError(
                "duplicate YAML key: {!r}".format(key)
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require(condition, message):
    if not condition:
        raise InterpretationReviewError(message)


def _sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def _read_regular_file(workspace, relative):
    root = Path(workspace).resolve()
    path = root / Path(relative)
    resolved = path.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise InterpretationReviewError(
            "path escapes workspace: {}".format(relative)
        ) from exc
    _require(not path.is_symlink(), "symlink forbidden: {}".format(relative))
    _require(resolved.is_file(), "regular file required: {}".format(relative))
    return resolved.read_bytes()


def _load_yaml_bytes(data, label):
    try:
        document = yaml.load(data.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise InterpretationReviewError(
            "invalid YAML {}: {}".format(label, exc)
        ) from exc
    _require(isinstance(document, dict), "mapping required: {}".format(label))
    return document


def _snapshot_yaml(workspace, relative, expected_sha256=None):
    data = _read_regular_file(workspace, relative)
    digest = _sha256_bytes(data)
    if expected_sha256 is not None:
        _require(
            digest == expected_sha256,
            "SHA256 mismatch for {}".format(relative),
        )
    return {
        "path": Path(relative).as_posix(),
        "sha256": digest,
        "document": _load_yaml_bytes(data, str(relative)),
    }


def _verify_binding(workspace, binding, label):
    _require(isinstance(binding, dict), "binding mapping required: {}".format(label))
    _require(set(binding) >= {"path", "sha256"}, "incomplete binding: {}".format(label))
    data = _read_regular_file(workspace, binding["path"])
    actual = _sha256_bytes(data)
    _require(actual == binding["sha256"], "bound resource drift: {}".format(label))
    return {"path": binding["path"], "sha256": actual}


def _validate_contract(contract):
    expected_top = {
        "schema_version",
        "architecture_generation",
        "stage",
        "status",
        "offline_only",
        "formal_result",
        "runtime_ready",
        "execution_ready",
        "execution_authorized",
        "evidence_budget_authorized",
        "training_allowed",
        "real_vehicle_use_forbidden",
        "source_evidence",
        "claim_interpretation",
        "future_performance_design",
        "forbidden_side_effects",
        "reviewer_integrity",
    }
    _require(set(contract) == expected_top, "contract top-level schema drift")
    _require(contract["stage"] == STAGE, "contract stage drift")
    _require(
        contract["status"] == "offline_result_interpretation_design_preregistered",
        "contract status drift",
    )
    for key in (
        "offline_only",
        "real_vehicle_use_forbidden",
    ):
        _require(contract[key] is True, "contract flag must be true: {}".format(key))
    for key in (
        "formal_result",
        "runtime_ready",
        "execution_ready",
        "execution_authorized",
        "training_allowed",
    ):
        _require(contract[key] is False, "contract flag must be false: {}".format(key))
    _require(contract["evidence_budget_authorized"] == 0, "I6 budget must be zero")
    claims = contract["claim_interpretation"]
    _require(claims["allowed_claims"] == EXPECTED_ALLOWED_CLAIMS, "allowed claim drift")
    _require(claims["forbidden_claims"] == EXPECTED_FORBIDDEN_CLAIMS, "forbidden claim drift")


def _validate_i5_report(workspace, report):
    expected_scalars = {
        "stage": "V2-04G-R6-I5",
        "status": "simulation_integration_validation_pass",
        "assessment_result": "pass",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "winner_ranked_or_frozen": False,
        "downstream_authorized": False,
        "safety_performance_or_generalization_claimed": False,
        "planned_identity_count": 6,
        "completed_identity_count": 6,
        "evidence_budget_authorized": 6,
        "evidence_units_consumed": 6,
        "unattempted_budget_forfeited": 0,
        "ttc_status_matches_preregistration": True,
        "semantic_schedule_pass": True,
        "readiness_direct_counts_pass": True,
        "two_phase_teardown_restore_pass": True,
        "integration_validation_pass": True,
    }
    for key, value in expected_scalars.items():
        _require(report.get(key) == value, "I5 report drift: {}".format(key))
    _require(report.get("integrity_failures") == [], "I5 integrity failures present")
    attempts = report.get("attempt_replays")
    _require(isinstance(attempts, list) and len(attempts) == 6, "I5 replay count drift")
    verified_raw = 0
    verified_journals = 0
    observed = []
    for attempt, expected in zip(attempts, EXPECTED_SEMANTIC_COUNTS):
        sequence, profile, finite_count, overlay_count = expected
        identity = attempt.get("identity", {})
        _require(attempt.get("sequence") == sequence, "I5 sequence drift")
        _require(identity.get("profile_id") == profile, "I5 profile drift")
        _require(attempt.get("status") == "evidence_complete", "I5 incomplete attempt")
        _verify_binding(workspace, attempt.get("journal"), "journal_{}".format(sequence))
        verified_journals += 1
        raw = attempt.get("raw_resources")
        _require(isinstance(raw, dict) and len(raw) == 6, "I5 raw roster drift")
        for name in sorted(raw):
            _verify_binding(
                workspace,
                raw[name],
                "attempt_{}_{}".format(sequence, name),
            )
            verified_raw += 1
        readiness = attempt.get("readiness", {})
        _require(readiness.get("pass") is True, "I5 readiness failed")
        counts = readiness.get("direct_counts", {})
        _require(len(counts) == 4, "I5 readiness count roster drift")
        _require(all(value >= 20 for value in counts.values()), "I5 direct count below 20")
        _require(
            attempt.get("scene_snapshot_post_episode_verified") is True,
            "I5 scene snapshot verification drift",
        )
        _require(
            attempt.get("teardown_restore_verified") is True,
            "I5 teardown verification drift",
        )
        semantic = attempt.get("semantic_observation", {})
        _require(
            semantic.get("finite_ttc_sample_count") == finite_count,
            "I5 finite TTC drift",
        )
        _require(
            semantic.get("non_none_overlay_count") == overlay_count,
            "I5 overlay count drift",
        )
        observed.append(
            {
                "sequence": sequence,
                "profile_id": profile,
                "finite_ttc_sample_count": finite_count,
                "non_none_overlay_count": overlay_count,
            }
        )
    return {
        "journal_bindings_verified": verified_journals,
        "raw_resource_bindings_verified": verified_raw,
        "semantic_observations": observed,
    }


def _validate_performance_design(design):
    _require(design.get("stage") == "V2-04G-P1", "performance stage drift")
    _require(
        design.get("status") == "design_preregistered_execution_not_authorized",
        "performance design status drift",
    )
    for key in ("simulation_only", "real_vehicle_use_forbidden"):
        _require(design.get(key) is True, "performance flag must be true: {}".format(key))
    for key in (
        "formal_result",
        "runtime_ready",
        "execution_ready",
        "execution_authorized",
        "training_allowed",
        "checkpoint_selection_allowed",
    ):
        _require(design.get(key) is False, "performance flag must be false: {}".format(key))
    _require(design.get("evidence_budget_authorized") == 0, "performance budget authorized")
    _require(design.get("training_budget_steps") == 0, "training budget is nonzero")
    firewall = design.get("fresh_seed_firewall", {})
    block = firewall.get("scene_seed_block", {})
    _require(block == {"first": 5201, "last": 5290, "count": 90}, "seed block drift")
    families = design.get("scene_families")
    _require(isinstance(families, list) and len(families) == 3, "scene family drift")
    _require(sum(item["paired_blocks"] for item in families) == 90, "pair count drift")
    matrix = design.get("planned_matrix", {})
    _require(matrix.get("planned_episode_count") == 270, "episode budget drift")
    _require(matrix.get("planned_confirmatory_paired_episode_count") == 180, "pair budget drift")
    _require(matrix.get("retry_allowed") is False, "retry unexpectedly allowed")
    decision = design.get("confirmatory_decision_rule", {})
    _require(decision.get("all_conditions_required") is True, "gatekeeping drift")
    _require(
        decision.get("decision_if_any_condition_fails")
        == "performance_improvement_not_demonstrated",
        "failure decision drift",
    )
    release = design.get("future_release_gates", {})
    _require(release.get("separate_execution_authorization_required") is True, "authorization gate drift")
    _require(release.get("current_design_may_start_ros_or_gazebo") is False, "execution enabled")
    return {
        "scene_seed_blocks": 90,
        "planned_episode_count": 270,
        "confirmatory_pair_episode_count": 180,
        "training_budget_steps": 0,
        "execution_authorized": False,
        "primary_comparison": "r6_semantics_circle_contact_vs_r6_semantics_legacy_control",
        "claim_scope": "fresh_simulation_dynamic_interaction_only",
    }


def _assert_i6_execution_absent(workspace):
    root = Path(workspace).resolve()
    forbidden = [
        Path("experiments/manifests/v2/integration/v2_04g_r6_i6_execution_authorization.yaml"),
        Path("experiments/manifests/v2/integration/v2_04g_r6_i6_execution_release.yaml"),
        Path("artifacts/v2/integration/v2_04g_r6_i1/r6_i6_execution"),
    ]
    present = [item.as_posix() for item in forbidden if (root / item).exists()]
    _require(present == [], "I6 execution material unexpectedly exists: {}".format(present))


def build_review(workspace):
    workspace = Path(workspace).resolve()
    contract_snapshot = _snapshot_yaml(workspace, CONTRACT_RELATIVE)
    contract = contract_snapshot["document"]
    _validate_contract(contract)
    verified_sources = {}
    source_documents = {}
    for label, binding in contract["source_evidence"].items():
        verified_sources[label] = _verify_binding(workspace, binding, label)
        if str(binding["path"]).endswith((".yaml", ".yml")):
            source_documents[label] = _snapshot_yaml(
                workspace, binding["path"], binding["sha256"]
            )["document"]
    i5_report = source_documents["i5_execution_report"]
    i5_verification = _validate_i5_report(workspace, i5_report)
    performance_binding = contract["future_performance_design"]
    performance_snapshot = _snapshot_yaml(
        workspace,
        performance_binding["path"],
        performance_binding["sha256"],
    )
    performance_summary = _validate_performance_design(
        performance_snapshot["document"]
    )
    for label, binding in contract["reviewer_integrity"].items():
        _verify_binding(workspace, binding, "reviewer_integrity_{}".format(label))
    _assert_i6_execution_absent(workspace)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": "fam_teb_v2_04g_r6_i6_result_interpretation_review_1",
        "status": "offline_result_interpretation_design_closure_pass",
        "review_result": "pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_started": False,
        "real_vehicle_used": False,
        "contract": {
            "path": CONTRACT_RELATIVE.as_posix(),
            "sha256": contract_snapshot["sha256"],
        },
        "source_evidence_verified": verified_sources,
        "i5_replay_verification": i5_verification,
        "i5_interpretation": {
            "integration_claim_supported": True,
            "performance_claim_supported": False,
            "performance_improvement_established": False,
            "allowed_claims": EXPECTED_ALLOWED_CLAIMS,
            "forbidden_claims": EXPECTED_FORBIDDEN_CLAIMS,
            "claim_limit": "fresh_simulation_semantic_and_execution_integration_only",
        },
        "future_performance_design": {
            "path": performance_binding["path"],
            "sha256": performance_snapshot["sha256"],
            "summary": performance_summary,
            "capable_of_testing_targeted_improvement": True,
            "guarantees_positive_result": False,
            "separate_execution_review_and_authorization_required": True,
        },
        "side_effect_audit": {
            "i5_rerun_or_resume": False,
            "ros_or_gazebo_started": False,
            "training_started": False,
            "seed_or_evidence_consumed": False,
            "execution_material_created": False,
            "real_vehicle_connected": False,
        },
        "closure_pass": True,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--output", default=str(OUTPUT_RELATIVE))
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args(argv)
    workspace = Path(args.workspace).resolve()
    review = build_review(workspace)
    output = Path(args.output)
    if not output.is_absolute():
        output = workspace / output
    persisted = _snapshot_yaml(workspace, output.relative_to(workspace))["document"]
    _require(persisted == review, "persisted I6 review is not deterministic")
    print(review["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

