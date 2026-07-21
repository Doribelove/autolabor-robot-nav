#!/usr/bin/env python3
"""Offline-only reviewer for the independent R6-I3 authorization envelope.

This script has no ROS imports and no process-launching path.  It validates the
fresh seed/budget/schedule preregistration, reuses the frozen R6-I2
single-open authorization validator, regenerates the R6-I2 machine review in
process, and records that execution release, fresh scene materialization, and
an actual R6-I3 execution entrypoint are still absent.
"""

import argparse
import copy
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Mapping

import yaml


sys.dont_write_bytecode = True

STAGE = "V2-04G-R6-I3"
SOURCE_STAGE = "V2-04G-R6-I2"
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml"
)
OUTPUT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i3_authorization_review/"
    "v2_04g_r6_i3_authorization_review.yaml"
)
I2_REVIEW_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_integration_review.yaml"
)
I2_CLOSURE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "execution_dependency_closure.yaml"
)
I2_COMPONENT_REVIEW_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_authorization_assessment_review.yaml"
)
I2_AUTHORIZATION_MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_authorization.py"
)
I2_REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_reviewer.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py"
)
DIRECTED_TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_authorization_review.py"
)
EXECUTION_RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_release.yaml"
)
EXECUTION_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)
FRESH_SCENE_INDEX_RELATIVE = (
    EXECUTION_ROOT_RELATIVE / "compiled_scenes/compiled_scene_index.yaml"
)
EXECUTION_ENTRYPOINT_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py"
)
EXECUTION_CLOSURE_RELATIVE = (
    EXECUTION_ROOT_RELATIVE / "execution_dependency_closure.yaml"
)

EXPECTED_PREREGISTRATION_SHA256 = (
    "a8295c723c1cf973c2c35c86e5b2d5c07361bdf0e92f36a0e8d12d2364ce6268"
)
EXPECTED_AUTHORIZATION_SHA256 = (
    "ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2"
)
EXPECTED_SCHEDULE_SHA256 = (
    "ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c"
)
EXPECTED_I2_REVIEW_SHA256 = (
    "b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61"
)
EXPECTED_I2_CLOSURE_FILE_SHA256 = (
    "63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58"
)
EXPECTED_I2_CLOSURE_LOGICAL_SHA256 = (
    "2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6"
)
EXPECTED_I2_COMPONENT_REVIEW_SHA256 = (
    "55e7c3d7aebcb561edc9acd794347355d6f60df46868462fa1beb069c7eb4c59"
)

EXPECTED_RESOURCE_PATHS = {
    "preregistration": PREREGISTRATION_RELATIVE.as_posix(),
    "r6_i2_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml"
    ),
    "inherited_r6_i2_dependency_closure": (
        I2_CLOSURE_RELATIVE.as_posix()
    ),
    "r6_i2_integration_review": I2_REVIEW_RELATIVE.as_posix(),
    "r6_i2_authorization_component_review": (
        I2_COMPONENT_REVIEW_RELATIVE.as_posix()
    ),
    "r6_i2_authorization_module": (
        I2_AUTHORIZATION_MODULE_RELATIVE.as_posix()
    ),
    "r6_i1_scene_derivation": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_scene_derivation.yaml"
    ),
    "source_r6_i1_compiled_scene_index": (
        "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    "legacy_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_legacy_control/supervisor.yaml"
    ),
    "aligned_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_circle_contact/supervisor.yaml"
    ),
    "frozen_evaluator": (
        "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py"
    ),
    "r6_design_report": (
        "artifacts/v2/design_review/v2_04g_r6/"
        "v2_04g_r6_design_review.yaml"
    ),
}

HISTORICAL_SEED_RESOURCES = (
    Path(
        "experiments/manifests/v2/calibration/"
        "v2_04g_r5_preregistration.yaml"
    ),
    Path(
        "experiments/manifests/v2/calibration/"
        "v2_04g_r5_bounded_execution_authorization.yaml"
    ),
    Path(
        "artifacts/v2/calibration/v2_04g_r5/"
        "v2_04g_r5_stage_report.yaml"
    ),
    Path(
        "artifacts/v2/calibration/v2_04g_r5/ttc_readiness_compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    Path(
        "artifacts/v2/calibration/v2_04g_r5/compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    Path(
        "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
        "v2_04g_ttc_d1_report.yaml"
    ),
    Path(
        "artifacts/v2/design_review/v2_04g_r6/"
        "v2_04g_r6_design_review.yaml"
    ),
    Path(
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_alignment_preregistration.yaml"
    ),
    Path(
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_candidates.yaml"
    ),
    Path(
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_execution_preregistration.yaml"
    ),
    Path(
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_bounded_simulation_authorization.yaml"
    ),
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_stage_report.yaml"
    ),
    Path(
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_scene_derivation.yaml"
    ),
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_integration_review.yaml"
    ),
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_terminal_assessment.yaml"
    ),
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/execution/journals/"
        "attempt_73c14969a81b4dbd4837158a06c9d03abeef09033adf4702684a5427e9405f3f.yaml"
    ),
    Path(
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i2_stage_transition.yaml"
    ),
    Path(
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i2_repair_preregistration.yaml"
    ),
    I2_REVIEW_RELATIVE,
)

PREREGISTRATION_FIELDS = {
    "schema_version",
    "architecture_generation",
    "stage",
    "preregistration_id",
    "status",
    "preregistration_date",
    "independent_stage",
    "source_stage",
    "simulation_only",
    "formal_result",
    "runtime_ready",
    "execution_ready",
    "execution_authorized",
    "execution_release_required",
    "execution_release_received",
    "ros_or_gazebo_start_authorized",
    "training_allowed",
    "real_vehicle_use_forbidden",
    "objective",
    "claim_limit",
    "single_changed_factor",
    "frozen_common_values",
    "r6_i2_trust_boundary",
    "fresh_seed_firewall",
    "budget",
    "schedule",
    "fresh_scene_identity_plan",
    "execution_release_plan",
    "readiness_gate",
    "execution_order",
    "downstream_forbidden",
}

EXPECTED_EXECUTION_SEEDS = [5151, 5152, 5153]
EXPECTED_COMPILE_SUPPORT_SEEDS = [5154, 5155, 5156, 5157]
EXPECTED_PROFILES = [
    "r6_semantics_legacy_control",
    "r6_semantics_circle_contact",
]
EXPECTED_SCHEDULE = [
    {
        "sequence": 1,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i3-dynamic-conflict-single-s5151",
        "seed": 5151,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 2,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i3-dynamic-conflict-single-s5151",
        "seed": 5151,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 3,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i3-dynamic-conflict-multi-s5152",
        "seed": 5152,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 4,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i3-dynamic-conflict-multi-s5152",
        "seed": 5152,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 5,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i3-dynamic-semantic-clear-s5153",
        "seed": 5153,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "legacy_non_none_identifiability",
    },
    {
        "sequence": 6,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i3-dynamic-semantic-clear-s5153",
        "seed": 5153,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "none_iff_no_finite_ttc",
    },
]

EXPECTED_FACTOR = {
    "name": "dynamic_conflict_estimator_semantics",
    "runtime_field": "supervisor.dynamic.conflict_estimator_id",
    "factor_count": 1,
    "only_profile_field_allowed_to_differ": True,
    "atomic_subcomponents": [
        "conflict_eligibility_primitive",
        "tracked_footprint_radius_interpretation",
        "multi_track_conflict_selection_order",
    ],
    "levels": {
        "r6_semantics_legacy_control": (
            "legacy_class_conditioned_geometry_v1"
        ),
        "r6_semantics_circle_contact": (
            "shared_circle_envelope_first_contact_v1"
        ),
    },
    "winner_eligible_profiles": [],
}

EXPECTED_FROZEN_VALUES = {
    "predicted_ttc_max_s": 5.0,
    "evaluator_ttc_horizon_s": 5.0,
    "minimum_track_confidence": 0.45,
    "robot_radius_m": 0.62,
    "minimum_relative_speed_mps": 0.05,
    "world_model_prediction_horizon_s": 2.0,
    "closest_approach_max_m": 1.35,
    "overlay_release_confirmation_s": 0.20,
    "horizon_values_1_5_or_1_0_enabled": False,
    "evaluator_changed": False,
    "anchor_bank_changed": False,
    "mechanism_changed": False,
    "tracker_or_classifier_changed": False,
    "transaction_or_join_changed": False,
    "scene_behavior_changed": False,
    "scene_timing_posthoc_change_allowed": False,
}

EXPECTED_EXECUTION_RELEASE_PLAN = {
    "status": "required_not_created",
    "canonical_manifest_path": EXECUTION_RELEASE_RELATIVE.as_posix(),
    "canonical_execution_root": EXECUTION_ROOT_RELATIVE.as_posix(),
    "canonical_compiled_scene_index_path": (
        FRESH_SCENE_INDEX_RELATIVE.as_posix()
    ),
    "canonical_entrypoint_path": EXECUTION_ENTRYPOINT_RELATIVE.as_posix(),
    "canonical_dependency_closure_path": (
        EXECUTION_CLOSURE_RELATIVE.as_posix()
    ),
    "manifest_present": False,
    "creation_before_next_explicit_execution_instruction_allowed": False,
    "authorization_envelope_alone_sufficient_for_execution": False,
    "future_entrypoint_must_fail_closed_without_valid_release": True,
    "caller_supplied_exact_release_sha256_required": True,
    "dedicated_release_schema_and_validator_required": True,
    "release_schema_closed_and_type_sensitive": True,
    "release_hash_and_parse_single_open_no_follow": True,
    "release_validation_before_any_journal_directory_or_subprocess": True,
    "required_release_bindings": [
        "current_preregistration_path_and_sha256",
        "current_authorization_envelope_path_and_sha256",
        "fresh_compiled_scene_index_and_every_child_path_and_sha256",
        "actual_r6_i3_entrypoint_and_transitive_dependency_path_and_sha256",
        "dedicated_release_validator_and_negative_tests_path_and_sha256",
        (
            "independent_r6_i3_execution_dependency_closure_file_and_"
            "logical_sha256"
        ),
        (
            "independent_r6_i3_execution_integration_machine_review_"
            "path_and_sha256"
        ),
    ],
    "required_release_state": {
        "explicit_user_execution_instruction_received": True,
        "execution_release_authorized": True,
        "execution_ready_after_all_prejournal_checks": True,
    },
}

I3_CONTROLLED_SCAN_ROOTS = (
    Path("artifacts/v2"),
    Path("experiments/manifests/v2"),
    Path("config/thesis_experiments/v2"),
    Path("src"),
)
ALLOWED_NONEXECUTION_I3_PATHS = {
    PREREGISTRATION_RELATIVE,
    AUTHORIZATION_RELATIVE,
    OUTPUT_RELATIVE,
    REVIEWER_RELATIVE,
    DIRECTED_TEST_RELATIVE,
}


class R6I3AuthorizationReviewError(ValueError):
    """Raised when the R6-I3 authorization envelope fails closed."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise R6I3AuthorizationReviewError(message)


def _exact(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _exact(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _exact(left, right) for left, right in zip(actual, expected)
        )
    return actual == expected


def _closed(value: Any, fields, label: str) -> Mapping[str, Any]:
    _require(isinstance(value, Mapping), label + " must be a mapping")
    expected = set(fields)
    actual = set(value)
    _require(
        actual == expected,
        "{} keys drifted; missing={} extra={}".format(
            label, sorted(expected - actual), sorted(actual - expected)
        ),
    )
    return value


def _load_module(path: Path, name: str):
    specification = importlib.util.spec_from_file_location(name, path)
    _require(specification is not None, "cannot create module specification")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _authorization_module(workspace: Path):
    return _load_module(
        workspace / I2_AUTHORIZATION_MODULE_RELATIVE,
        "v2_04g_r6_i2_authorization_for_i3_review",
    )


def _i2_reviewer_module(workspace: Path):
    return _load_module(
        workspace / I2_REVIEWER_RELATIVE,
        "v2_04g_r6_i2_reviewer_for_i3_review",
    )


def _verify_preregistration_document(document, authorization_module):
    _closed(document, PREREGISTRATION_FIELDS, "preregistration")
    expected_scalars = {
        "schema_version": "2.1",
        "architecture_generation": "v2",
        "stage": STAGE,
        "preregistration_id": (
            "fam_teb_v2_04g_r6_i3_bounded_simulation_"
            "authorization_review_1"
        ),
        "status": (
            "bounded_simulation_preregistered_authorization_review_only"
        ),
        "preregistration_date": "2026-07-21",
        "independent_stage": True,
        "source_stage": SOURCE_STAGE,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "execution_release_required": True,
        "execution_release_received": False,
        "ros_or_gazebo_start_authorized": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "claim_limit": (
            "authorization_envelope_review_only_not_execution_evidence_"
            "or_readiness"
        ),
    }
    for key, expected in expected_scalars.items():
        _require(
            _exact(document[key], expected),
            "preregistration {} drifted".format(key),
        )
    _require(
        isinstance(document["objective"], str) and document["objective"],
        "preregistration objective is missing",
    )
    _require(
        _exact(document["single_changed_factor"], EXPECTED_FACTOR),
        "single factor drifted",
    )
    _require(
        _exact(document["frozen_common_values"], EXPECTED_FROZEN_VALUES),
        "frozen common values drifted",
    )

    trust = _closed(
        document["r6_i2_trust_boundary"],
        {
            "machine_review",
            "dependency_closure",
            "authorization_component_review",
            "required_repairs",
            "required_bootstrap_order",
        },
        "r6_i2_trust_boundary",
    )
    _require(
        _exact(
            trust["machine_review"],
            {
                "path": I2_REVIEW_RELATIVE.as_posix(),
                "sha256": EXPECTED_I2_REVIEW_SHA256,
                "required_status": (
                    "repair_integration_review_pass_execution_not_authorized"
                ),
            },
        ),
        "R6-I2 review trust binding drifted",
    )
    _require(
        _exact(
            trust["dependency_closure"],
            {
                "path": I2_CLOSURE_RELATIVE.as_posix(),
                "file_sha256": EXPECTED_I2_CLOSURE_FILE_SHA256,
                "logical_sha256": EXPECTED_I2_CLOSURE_LOGICAL_SHA256,
                "inherited_python_binding_coverage": 39,
                "runtime_binding_count": 5,
                "unresolved_dependency_count": 0,
            },
        ),
        "R6-I2 dependency trust binding drifted",
    )
    _require(
        _exact(
            trust["authorization_component_review"],
            {
                "path": I2_COMPONENT_REVIEW_RELATIVE.as_posix(),
                "sha256": EXPECTED_I2_COMPONENT_REVIEW_SHA256,
                "required_status": (
                    "repair_component_review_pass_execution_not_authorized"
                ),
            },
        ),
        "R6-I2 component review trust binding drifted",
    )
    _require(
        _exact(
            trust["required_repairs"],
            [
                "positive_clock_before_move_base_readiness",
                "canonical_path_and_sha_external_dependency_closure",
                "closed_authorization_schema_and_exact_schedule_enforcement",
                "single_open_no_follow_hash_and_parse",
                "deterministic_offline_assessor",
                "credential_safe_child_environment_and_log_redaction",
            ],
        ),
        "required repair list drifted",
    )
    _require(
        _exact(
            trust["required_bootstrap_order"],
            [
                "base_spawn",
                "unpause_request",
                "successful_unpause_ack",
                "first_strictly_positive_post_ack_clock",
                "second_strictly_greater_positive_post_ack_clock",
                "release_move_base_and_teb_service_wait",
            ],
        ),
        "bootstrap order drifted",
    )

    firewall = _closed(
        document["fresh_seed_firewall"],
        {
            "audit_scope",
            "historical_high_watermark",
            "execution_seeds",
            "compile_support_only_seeds",
            "compile_support_seeds_are_evidence",
            "held_out_5001_5010_forbidden",
            "r5_allocated_interval_5111_5135_forbidden",
            "r6_i1_allocated_interval_5141_5147_forbidden",
            "seed5111_or_r5_allocation_reused",
            "seed5141_or_r6_i1_allocation_reused",
            "prior_failed_seed_retry",
            "prior_identity_reused",
            "seed_substitution_after_authorization",
        },
        "fresh_seed_firewall",
    )
    expected_firewall = {
        "audit_scope": "selected_authoritative_pre_i3_yaml_seed_evidence",
        "historical_high_watermark": 5147,
        "execution_seeds": EXPECTED_EXECUTION_SEEDS,
        "compile_support_only_seeds": EXPECTED_COMPILE_SUPPORT_SEEDS,
        "compile_support_seeds_are_evidence": False,
        "held_out_5001_5010_forbidden": True,
        "r5_allocated_interval_5111_5135_forbidden": True,
        "r6_i1_allocated_interval_5141_5147_forbidden": True,
        "seed5111_or_r5_allocation_reused": False,
        "seed5141_or_r6_i1_allocation_reused": False,
        "prior_failed_seed_retry": False,
        "prior_identity_reused": False,
        "seed_substitution_after_authorization": False,
    }
    _require(_exact(dict(firewall), expected_firewall), "seed firewall drifted")

    budget = _closed(
        document["budget"],
        {
            "evidence_unit_definition",
            "evidence_units_authorizable",
            "evidence_units_authorized_before_separate_authorization",
            "evidence_units_consumed_before_authorization",
            "evidence_units_consumed_by_this_review",
            "compile_support_evidence_units",
            "attempt_limit_per_identity",
            "retry_allowed",
            "resume_allowed",
            "replacement_seed_allowed",
            "budget_expansion_allowed",
            "failure_stops_stage_and_forfeits_unattempted_units",
        },
        "budget",
    )
    _require(
        _exact(
            dict(budget),
            {
                "evidence_unit_definition": (
                    "one_profile_x_one_execution_scene_x_attempt_1"
                ),
                "evidence_units_authorizable": 6,
                "evidence_units_authorized_before_separate_authorization": 0,
                "evidence_units_consumed_before_authorization": 0,
                "evidence_units_consumed_by_this_review": 0,
                "compile_support_evidence_units": 0,
                "attempt_limit_per_identity": 1,
                "retry_allowed": False,
                "resume_allowed": False,
                "replacement_seed_allowed": False,
                "budget_expansion_allowed": False,
                "failure_stops_stage_and_forfeits_unattempted_units": True,
            },
        ),
        "budget boundary drifted",
    )
    _require(_exact(document["schedule"], EXPECTED_SCHEDULE), "schedule drifted")
    _require(
        authorization_module.canonical_document_sha256(document["schedule"])
        == EXPECTED_SCHEDULE_SHA256,
        "schedule canonical digest drifted",
    )

    scene_plan = _closed(
        document["fresh_scene_identity_plan"],
        {
            "status",
            "derivation_rule",
            "fresh_compiled_scene_index_present",
            "fresh_compiled_scene_children_present",
            "actual_r6_i3_execution_entrypoint_present",
            "materialization_before_next_explicit_execution_instruction_allowed",
            "execution_ready_claim_allowed",
            "required_before_any_journal_or_subprocess",
            "execution_scenes",
            "compile_support_identities",
        },
        "fresh_scene_identity_plan",
    )
    expected_scene_scalars = {
        "status": "identities_preregistered_materialization_not_authorized",
        "derivation_rule": (
            "clone_frozen_r6_i1_behavior_replace_only_stage_scene_id_and_seed"
        ),
        "fresh_compiled_scene_index_present": False,
        "fresh_compiled_scene_children_present": False,
        "actual_r6_i3_execution_entrypoint_present": False,
        "materialization_before_next_explicit_execution_instruction_allowed": (
            False
        ),
        "execution_ready_claim_allowed": False,
    }
    for key, expected in expected_scene_scalars.items():
        _require(_exact(scene_plan[key], expected), "scene plan drifted: " + key)
    _require(
        _exact(
            scene_plan["required_before_any_journal_or_subprocess"],
            [
                "materialize_and_compile_all_fresh_scene_identities",
                "verify_behavioral_scene_diff_is_empty",
                "bind_compiled_index_and_every_child_path_sha256",
                (
                    "integrate_r6_i2_bootstrap_and_authorization_guards_in_"
                    "actual_r6_i3_entrypoint"
                ),
                (
                    "mechanically_build_and_review_r6_i3_execution_"
                    "dependency_closure"
                ),
                "create_and_validate_separate_r6_i3_execution_release_manifest",
                "revalidate_the_authorization_and_all_closure_targets_in_process",
            ],
        ),
        "pre-execution requirements drifted",
    )
    expected_scene_rows = [
        {
            "source_scene_id": (
                "v2-04g-r6-i1-dynamic-conflict-single-s5141"
            ),
            "target_scene_id": (
                "v2-04g-r6-i3-dynamic-conflict-single-s5151"
            ),
            "seed": 5151,
            "role": "single_track_circle_contact",
            "behavior_fields_changed": [],
        },
        {
            "source_scene_id": "v2-04g-r6-i1-dynamic-conflict-multi-s5142",
            "target_scene_id": "v2-04g-r6-i3-dynamic-conflict-multi-s5152",
            "seed": 5152,
            "role": "multi_track_circle_contact",
            "behavior_fields_changed": [],
        },
        {
            "source_scene_id": (
                "v2-04g-r6-i1-dynamic-semantic-clear-s5143"
            ),
            "target_scene_id": (
                "v2-04g-r6-i3-dynamic-semantic-clear-s5153"
            ),
            "seed": 5153,
            "role": "time_separated_centerline_crossing",
            "behavior_fields_changed": [],
        },
    ]
    _require(
        _exact(scene_plan["execution_scenes"], expected_scene_rows),
        "fresh execution scene identities drifted",
    )
    _require(
        _exact(
            scene_plan["compile_support_identities"],
            [
                {
                    "family": "CRUISE",
                    "source_seed": 5144,
                    "target_seed": 5154,
                    "evidence_units": 0,
                },
                {
                    "family": "STATIC_DENSE",
                    "source_seed": 5145,
                    "target_seed": 5155,
                    "evidence_units": 0,
                },
                {
                    "family": "CORRIDOR",
                    "source_seed": 5146,
                    "target_seed": 5156,
                    "evidence_units": 0,
                },
                {
                    "family": "MANEUVER",
                    "source_seed": 5147,
                    "target_seed": 5157,
                    "evidence_units": 0,
                },
            ],
        ),
        "compile-support identities drifted",
    )
    _require(
        _exact(
            document["execution_release_plan"],
            EXPECTED_EXECUTION_RELEASE_PLAN,
        ),
        "execution release plan drifted",
    )
    _require(
        _exact(
            document["readiness_gate"],
            {
                "minimum_message_count_per_stream": 20,
                "warmup_timeout_s": 12.0,
                "measurement_duration_s": 6.0,
                "minimum_valid_fraction": 0.95,
                "required_consecutive_stable_count": 10,
                "maximum_expected_context_hold_count_per_probe": 3,
                "activation_tracker_and_context_counts_direct": True,
                "evaluation_tracker_and_context_counts_direct": True,
            },
        ),
        "readiness gate drifted",
    )
    _require(
        _exact(
            document["execution_order"],
            [
                "r6_i2_review_and_dependency_closure_revalidated",
                "preregistration_and_bounded_authorization_envelope_reviewed",
                "stop_and_wait_for_new_explicit_user_execution_instruction",
                "fresh_scene_and_actual_entrypoint_closure_review_before_any_journal",
                "separate_execution_release_manifest_exact_hash_validation",
                (
                    "caller_supplied_authorization_sha_and_full_in_process_"
                    "dependency_rehash"
                ),
                "exact_schedule_until_complete_or_first_terminal_failure",
                "persisted_journal_assessment",
            ],
        ),
        "execution order drifted",
    )
    _require(
        _exact(
            document["downstream_forbidden"],
            {
                "r5_retry_or_resume": True,
                "r5_remaining_68_units": True,
                "r6_i1_retry_or_resume": True,
                "r6_i1_forfeited_5_units": True,
                "held_out_5001_5010": True,
                "rank_or_freeze_winner": True,
                "v2_05": True,
                "sac_or_any_training": True,
                "real_vehicle": True,
                "real_vehicle_teb_write": True,
                "formal_or_runtime_ready_claim": True,
            },
        ),
        "downstream firewall drifted",
    )
    return {
        "execution_seeds": copy.deepcopy(EXPECTED_EXECUTION_SEEDS),
        "compile_support_only_seeds": copy.deepcopy(
            EXPECTED_COMPILE_SUPPORT_SEEDS
        ),
        "schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "identity_count": 6,
        "evidence_budget_units": 6,
    }


def _seed_values(value: Any, under_seed_key: bool = False):
    collected = set()
    if isinstance(value, Mapping):
        for key, child in value.items():
            seed_key = under_seed_key or "seed" in str(key).lower()
            collected.update(_seed_values(child, seed_key))
    elif isinstance(value, list):
        for child in value:
            collected.update(_seed_values(child, under_seed_key))
    elif type(value) is int and under_seed_key:
        collected.add(value)
    elif isinstance(value, str):
        if under_seed_key:
            collected.update(int(token) for token in re.findall(r"\d+", value))
        collected.update(
            int(token) for token in re.findall(r"(?:^|[-_])s(\d{4,})(?:\b|$)", value)
        )
    return collected


def _verify_historical_seed_firewall(workspace: Path, authorization_module):
    resources = []
    historical = set()
    for relative in HISTORICAL_SEED_RESOURCES:
        snapshot = authorization_module.read_workspace_yaml_once(
            workspace, relative.as_posix()
        )
        resources.append(
            {"path": relative.as_posix(), "sha256": snapshot.sha256}
        )
        historical.update(_seed_values(snapshot.document))
    positive = sorted(seed for seed in historical if 0 < seed <= 9999)
    new = set(EXPECTED_EXECUTION_SEEDS + EXPECTED_COMPILE_SUPPORT_SEEDS)
    _require(new.isdisjoint(historical), "fresh seed block was previously used")
    _require(positive and max(positive) == 5147, "historical seed high-watermark drifted")
    return {
        "resource_count": len(resources),
        "resources": resources,
        "experiment_seed_namespace": "integer_1_through_9999",
        "out_of_namespace_numeric_tokens_ignored": len(
            {seed for seed in historical if seed > 9999}
        ),
        "historical_seed_count": len(positive),
        "historical_high_watermark": max(positive),
        "fresh_seed_block": sorted(new),
        "fresh_seed_prior_reference_count": 0,
        "pass": True,
    }


def _verify_i2_review(workspace: Path, authorization_validation):
    reviewer = _i2_reviewer_module(workspace)
    rebuilt = reviewer.build_review(workspace)
    persisted = authorization_validation.bound_resources[
        "r6_i2_integration_review"
    ].document
    _require(_exact(rebuilt, persisted), "R6-I2 machine review does not reproduce")
    _require(
        rebuilt["stage"] == SOURCE_STAGE
        and rebuilt["status"]
        == "repair_integration_review_pass_execution_not_authorized"
        and rebuilt["review_result"] == "pass"
        and rebuilt["execution_authorized"] is False
        and rebuilt["execution_ready"] is False
        and rebuilt["all_repair_gates_pass"] is True,
        "R6-I2 review boundary drifted",
    )
    closure = rebuilt["dependency_closure_review"]
    _require(
        closure["local_file_count"] == 106
        and closure["local_edge_count"] == 146
        and closure["external_file_count"] == 301
        and closure["external_python_binding_count"] == 45
        and closure["inherited_python_binding_coverage_count"] == 39
        and closure["external_runtime_binding_count"] == 5
        and closure["unresolved_count"] == 0
        and closure["closure_sha256"] == EXPECTED_I2_CLOSURE_LOGICAL_SHA256
        and closure["mechanically_rehashed"] is True
        and closure["process_started"] is False
        and closure["pass"] is True,
        "R6-I2 closure revalidation drifted",
    )
    return {
        "machine_review_sha256": EXPECTED_I2_REVIEW_SHA256,
        "dependency_closure_file_sha256": (
            EXPECTED_I2_CLOSURE_FILE_SHA256
        ),
        "dependency_closure_logical_sha256": (
            EXPECTED_I2_CLOSURE_LOGICAL_SHA256
        ),
        "local_file_count": closure["local_file_count"],
        "local_edge_count": closure["local_edge_count"],
        "external_file_count": closure["external_file_count"],
        "external_python_binding_count": (
            closure["external_python_binding_count"]
        ),
        "inherited_python_binding_coverage_count": (
            closure["inherited_python_binding_coverage_count"]
        ),
        "runtime_binding_count": closure["external_runtime_binding_count"],
        "unresolved_count": closure["unresolved_count"],
        "mechanically_rehashed_in_process": True,
        "process_started": False,
        "pass": True,
    }


def _verify_execution_absence(workspace: Path):
    def belongs_to_i3(path: Path) -> bool:
        normalized = re.sub(r"[^a-z0-9]", "", path.as_posix().lower())
        return "r6i3" in normalized

    def is_allowed_or_allowed_parent(path: Path) -> bool:
        for allowed in ALLOWED_NONEXECUTION_I3_PATHS:
            if path == allowed:
                return True
            try:
                allowed.relative_to(path)
                return True
            except ValueError:
                pass
        return False

    unexpected = []
    for scan_root in I3_CONTROLLED_SCAN_ROOTS:
        absolute_root = workspace / scan_root
        if not absolute_root.exists():
            continue
        for candidate in absolute_root.rglob("*"):
            relative = candidate.relative_to(workspace)
            if "__pycache__" in relative.parts or relative.suffix == ".pyc":
                continue
            if belongs_to_i3(relative) and not is_allowed_or_allowed_parent(
                relative
            ):
                unexpected.append(relative.as_posix())
    _require(
        unexpected == [],
        "R6-I3 execution material unexpectedly exists: {}".format(
            sorted(unexpected)
        ),
    )
    return {
        "execution_release_received": False,
        "execution_release_manifest_present": False,
        "canonical_execution_release_path": (
            EXECUTION_RELEASE_RELATIVE.as_posix()
        ),
        "authorization_envelope_alone_sufficient_for_execution": False,
        "closed_world_stage_prefix_scan": True,
        "controlled_scan_roots": [
            path.as_posix() for path in I3_CONTROLLED_SCAN_ROOTS
        ],
        "unexpected_i3_owned_paths": [],
        "fresh_scene_index_present": False,
        "fresh_scene_children_present": False,
        "actual_execution_entrypoint_present": False,
        "execution_journal_present": False,
        "execution_receipt_present": False,
        "evidence_units_consumed": 0,
        "ros_started_by_review": False,
        "gazebo_started_by_review": False,
        "move_base_started_by_review": False,
        "execution_ready": False,
        "pass": True,
    }


def _review_source_integrity(workspace: Path, authorization_module):
    reviewer = authorization_module.read_workspace_file_once(
        workspace, REVIEWER_RELATIVE.as_posix(), parse_yaml=False
    )
    directed_test = authorization_module.read_workspace_file_once(
        workspace, DIRECTED_TEST_RELATIVE.as_posix(), parse_yaml=False
    )
    return {
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": reviewer.sha256,
        },
        "directed_test": {
            "path": DIRECTED_TEST_RELATIVE.as_posix(),
            "sha256": directed_test.sha256,
        },
        "single_open_no_follow": True,
    }


def build_review(workspace: Path, caller_authorization_sha256: str):
    root = Path(workspace).resolve()
    _require(root.is_absolute() and root == root.resolve(), "workspace is not canonical")
    _require(
        caller_authorization_sha256 == EXPECTED_AUTHORIZATION_SHA256,
        "caller authorization SHA256 differs from reviewed trust anchor",
    )
    authorization_module = _authorization_module(root)
    validation = authorization_module.load_and_validate_authorization(
        root,
        AUTHORIZATION_RELATIVE.as_posix(),
        caller_authorization_sha256,
        STAGE,
        "preregistration",
        EXPECTED_RESOURCE_PATHS,
        EXPECTED_I2_CLOSURE_LOGICAL_SHA256,
        dependency_closure_label="inherited_r6_i2_dependency_closure",
    )
    _require(
        validation.authorization.sha256 == EXPECTED_AUTHORIZATION_SHA256,
        "authorization file hash drifted",
    )
    _require(
        validation.preregistration.sha256 == EXPECTED_PREREGISTRATION_SHA256,
        "preregistration file hash drifted",
    )
    preregistration_review = _verify_preregistration_document(
        validation.preregistration.document, authorization_module
    )
    _require(
        validation.identity_count == 6
        and list(validation.execution_seeds) == EXPECTED_EXECUTION_SEEDS
        and validation.schedule_sha256 == EXPECTED_SCHEDULE_SHA256,
        "base authorization validation result drifted",
    )
    authorization = validation.authorization.document
    _require(
        authorization["execution_authorized"] is True
        and authorization["status"]
        == "bounded_fresh_seed_simulation_authorized",
        "bounded authorization envelope drifted",
    )
    _require(
        authorization["authorization_source"]
        == "explicit_user_instruction_after_independent_integration_review"
        and authorization["completion_boundary"]["maximum_claim"]
        == "fresh_simulation_runtime_evaluator_semantic_integration",
        "authorization release boundary drifted",
    )
    seed_review = _verify_historical_seed_firewall(root, authorization_module)
    i2_review = _verify_i2_review(root, validation)
    absence = _verify_execution_absence(root)
    source_integrity = _review_source_integrity(root, authorization_module)
    return {
        "schema_version": "2.1",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": (
            "fam_teb_v2_04g_r6_i3_bounded_simulation_"
            "authorization_review_1"
        ),
        "status": (
            "bounded_authorization_review_pass_execution_release_required"
        ),
        "review_result": "pass",
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "authorization_envelope_valid": True,
        "authorization_manifest_execution_authorized": True,
        "execution_release_required": True,
        "execution_release_received": False,
        "preregistration_review": preregistration_review,
        "authorization_review": {
            "path": AUTHORIZATION_RELATIVE.as_posix(),
            "sha256": validation.authorization.sha256,
            "single_open_no_follow_validation": True,
            "closed_type_sensitive_schema_validation": True,
            "exact_schedule_validation": True,
            "bound_resource_count": len(validation.bound_resources),
            "all_bound_resource_hashes_match": True,
            "caller_supplied_hash_trust_anchor_match": True,
            "evidence_budget_authorized": 6,
            "evidence_budget_consumed": 0,
        },
        "r6_i2_revalidation": i2_review,
        "review_source_integrity": source_integrity,
        "fresh_seed_firewall_review": seed_review,
        "execution_absence_review": absence,
        "side_effects": {
            "preregistration_created": True,
            "bounded_authorization_envelope_created": True,
            "fresh_seed_values_reserved": list(range(5151, 5158)),
            "seed_or_evidence_units_consumed": 0,
            "journal_created": False,
            "scene_materialized": False,
            "subprocess_started": False,
            "ros_started": False,
            "gazebo_started": False,
            "move_base_started": False,
            "training_started": False,
            "real_vehicle_used": False,
            "real_vehicle_teb_written": False,
        },
        "next_gate": {
            "new_explicit_user_execution_instruction_required": True,
            "fresh_scene_materialization_and_child_hash_binding_required": True,
            "actual_r6_i3_entrypoint_and_dependency_closure_review_required": True,
            "separate_execution_release_manifest_required": True,
            "dedicated_release_schema_and_validator_required": True,
            "release_schema_closed_and_type_sensitive": True,
            "release_hash_and_parse_single_open_no_follow_required": True,
            "release_validation_before_any_journal_directory_or_subprocess": True,
            "canonical_execution_release_path": (
                EXECUTION_RELEASE_RELATIVE.as_posix()
            ),
            "future_entrypoint_must_validate_caller_supplied_release_hash": True,
            "authorization_envelope_alone_may_start_execution": False,
            "authorization_and_full_dependency_rehash_before_journal_required": True,
            "execution_may_start_now": False,
        },
        "claim_limit": (
            "bounded_authorization_envelope_only_no_execution_evidence_"
            "performance_safety_generalization_or_readiness_claim"
        ),
    }


def _atomic_yaml(path: Path, value: Mapping[str, Any]) -> None:
    _require(path.parent.is_dir(), "review output directory is missing")
    _require(not path.is_symlink(), "review output is a symlink")
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path("/home/robot/robot_ws_base_rl"),
    )
    parser.add_argument(
        "--authorization-sha256",
        required=True,
        help="caller-supplied exact R6-I3 authorization file SHA256",
    )
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.check_only and args.output is not None:
        parser.error("--check-only and --output are mutually exclusive")
    review = build_review(args.workspace, args.authorization_sha256)
    root = args.workspace.resolve()
    canonical_output = (root / OUTPUT_RELATIVE).resolve()
    if args.output is not None:
        supplied = args.output
        if not supplied.is_absolute():
            supplied = root / supplied
        if supplied.resolve() != canonical_output:
            parser.error("output must be the canonical R6-I3 review report")
        _atomic_yaml(canonical_output, review)
    elif args.check_only:
        _require(canonical_output.is_file(), "persisted R6-I3 review is missing")
        persisted = yaml.safe_load(canonical_output.read_text(encoding="utf-8"))
        _require(_exact(persisted, review), "persisted R6-I3 review drifted")
    else:
        print(yaml.safe_dump(review, sort_keys=False, allow_unicode=True))
        return 0
    print(review["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
