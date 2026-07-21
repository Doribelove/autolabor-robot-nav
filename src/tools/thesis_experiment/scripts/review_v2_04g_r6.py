#!/usr/bin/env python3
"""Review the offline-only V2-04G-R6 semantic-alignment preregistration.

This reviewer is deliberately ROS-free.  It verifies the frozen R5/D1 inputs,
the one-factor design, evaluator parity on synthetic fixtures, and all six
fail-closed integrity protocols.  Its only persistent write is the canonical
deterministic design-review report; it cannot authorize or execute R6.
"""

import argparse
import copy
import hashlib
import math
import os
from pathlib import Path
import stat
import sys
import tempfile

import yaml


sys.dont_write_bytecode = True

STAGE = "V2-04G-R6-DESIGN"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_semantic_alignment_design_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v2/preregistrations/"
    "v2_04g_r6_semantic_alignment_preregistration.yaml"
)
CANDIDATE_BANK_RELATIVE = Path(
    "experiments/manifests/v2/preregistrations/"
    "v2_04g_r6_semantic_candidates.yaml"
)
SEMANTIC_REFERENCE_RELATIVE = Path(
    "src/application/teb_mode_manager/src/teb_mode_manager/"
    "r6_relative_ttc_supervisor.py"
)
INTEGRITY_PROTOCOL_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_integrity.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/review_v2_04g_r6.py"
)
TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6.py"
)
OUTPUT_RELATIVE = Path(
    "artifacts/v2/design_review/v2_04g_r6/"
    "v2_04g_r6_design_review.yaml"
)
R5_ARTIFACT_RELATIVE = Path("artifacts/v2/calibration/v2_04g_r5")
D1_ARTIFACT_RELATIVE = Path(
    "artifacts/v2/diagnosis/v2_04g_ttc_d1"
)
EXPECTED_R5_FILE_COUNT = 68
EXPECTED_R5_TREE_SHA256 = (
    "ecb1f33093dee469008c2ad2d783b3e8ffd1c0739db7903b5df273717e270984"
)
EXPECTED_D1_REPORT_SHA256 = (
    "e8983d6bb9fc805c807d289cb65949b5d08b4eab8984a72760febde91d6bb063"
)
EXPECTED_FACTOR = "dynamic_conflict_estimator_semantics"
EXPECTED_RUNTIME_FIELD = "supervisor.dynamic.conflict_estimator_id"
LEGACY_ESTIMATOR_ID = "legacy_class_conditioned_geometry_v1"
ALIGNED_ESTIMATOR_ID = "shared_circle_envelope_first_contact_v1"
EXPECTED_RESOURCE_PATHS = {
    "contract": str(CONTRACT_RELATIVE),
    "candidate_bank": str(CANDIDATE_BANK_RELATIVE),
    "semantic_reference": str(SEMANTIC_REFERENCE_RELATIVE),
    "integrity_protocol": str(INTEGRITY_PROTOCOL_RELATIVE),
    "d1_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
    ),
    "d1_report": (
        "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
        "v2_04g_ttc_d1_report.yaml"
    ),
    "d1_script": (
        "src/tools/thesis_experiment/scripts/diagnose_v2_04g_ttc_d1.py"
    ),
    "d1_handoff": (
        "docs/thesis_experiment/CURRENT_V2_04G_TTC_D1_HANDOFF.md"
    ),
    "r5_preregistration": (
        "experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml"
    ),
    "r5_stage_report": (
        "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_stage_report.yaml"
    ),
    "r5_assessment": (
        "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_assessment.yaml"
    ),
    "r5_readiness_summary": (
        "artifacts/v2/calibration/v2_04g_r5/readiness/"
        "v2_04g_r5_readiness_summary.yaml"
    ),
    "frozen_rule_supervisor": (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "rule_supervisor.py"
    ),
    "frozen_rule_supervisor_node": (
        "src/application/teb_mode_manager/scripts/"
        "rule_context_supervisor_node.py"
    ),
    "frozen_risk_evidence": (
        "src/perception/nav_world_model/src/nav_world_model/risk_evidence.py"
    ),
    "frozen_world_model_core": (
        "src/perception/nav_world_model/src/nav_world_model/core.py"
    ),
    "frozen_world_model_config": (
        "src/perception/nav_world_model/config/v2_03_candidate.yaml"
    ),
    "frozen_v2_evaluator": (
        "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py"
    ),
    "frozen_r5_readiness_batch": (
        "src/tools/thesis_experiment/scripts/v2_04g_r5_readiness_batch.py"
    ),
    "frozen_r5_assessor": (
        "src/tools/thesis_experiment/scripts/assess_v2_04g_r5.py"
    ),
    "frozen_transaction_node": (
        "src/application/teb_mode_manager/scripts/"
        "v2_04g_r1_typed_anchor_transaction_node.py"
    ),
}
EXPECTED_RISK_IDS = (
    "D1-RISK-READINESS-DIRECT-COUNTS",
    "D1-RISK-COMPILED-SCENE-TOCTOU",
    "D1-RISK-SIGINT-IN-PROGRESS",
    "D1-RISK-ASSESSMENT-RAW-BINDING",
    "D1-RISK-EXECUTION-HASH-CLOSURE",
    "D1-RISK-TEARDOWN-RESTORE",
)
R6_I1_TRANSITION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_stage_transition.yaml"
)
R6_I1_OWNED_PREFIXES = (
    "artifacts/v2/integration/v2_04g_r6_i1/",
    "experiments/manifests/v2/integration/",
    "config/thesis_experiments/v2/v2_04g_r6_i1_",
    "docs/thesis_experiment/CURRENT_V2_04G_R6_I1_",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_",
    "src/tools/thesis_experiment/scripts/assess_v2_04g_r6_i1.py",
    "src/tools/thesis_experiment/scripts/generate_v2_04g_r6_i1_",
    "src/tools/thesis_experiment/scripts/review_v2_04g_r6_i1.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py",
    "src/application/teb_mode_manager/scripts/r6_rule_context_supervisor_node.py",
    "src/application/teb_mode_manager/scripts/v2_04g_r6_typed_anchor_transaction_node.py",
    "src/application/teb_mode_manager/launch/v2_04g_r6_",
    "src/application/teb_mode_manager/src/teb_mode_manager/r6_execution_integration.py",
    "src/application/teb_mode_manager/tests/test_r6_execution_integration.py",
    "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_",
)


class R6ReviewError(ValueError):
    """Raised when the R6 design or its frozen boundary fails closed."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader rejecting duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise R6ReviewError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require(condition, message):
    if not condition:
        raise R6ReviewError(message)


def _type_exact_equal(actual, expected):
    """Compare nested YAML values without bool/int or int/float coercion."""

    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return (
            set(actual) == set(expected)
            and all(
                _type_exact_equal(actual[key], expected[key])
                for key in expected
            )
        )
    if isinstance(expected, list):
        return (
            len(actual) == len(expected)
            and all(
                _type_exact_equal(actual_value, expected_value)
                for actual_value, expected_value in zip(actual, expected)
            )
        )
    return actual == expected


def _require_exact(actual, expected, label):
    _require(
        _type_exact_equal(actual, expected),
        "{} schema or value drifted".format(label),
    )


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_yaml(path):
    source = Path(path)
    try:
        value = yaml.load(
            source.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader
        )
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6ReviewError(
            "cannot strictly load {}: {}".format(source, exc)
        ) from exc
    _require(isinstance(value, dict), "{} must contain a mapping".format(source))
    return value


def _inside(root, path, label):
    boundary = Path(root).resolve()
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise R6ReviewError(
            "{} leaves workspace: {}".format(label, resolved)
        ) from exc
    return resolved


def _canonical_path(root, supplied, relative, label):
    expected = (Path(root) / relative).resolve()
    candidate = Path(supplied)
    if not candidate.is_absolute():
        candidate = Path(root) / candidate
    actual = _inside(root, candidate, label)
    _require(actual == expected, "{} path drifted".format(label))
    _require(not (Path(root) / relative).is_symlink(), "{} is a symlink".format(label))
    return actual


def _tree_snapshot(workspace, relative_root):
    root = _inside(
        workspace, Path(workspace) / relative_root, str(relative_root)
    )
    _require(root.is_dir(), "{} is missing".format(relative_root))
    records = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise R6ReviewError(
                "{} contains a symlink: {}".format(relative_root, path)
            )
        if path.is_file():
            records.append({
                "path": str(path.relative_to(Path(workspace).resolve())),
                "sha256": _sha256(path),
            })
    canonical = "".join(
        "{} {}\n".format(row["path"], row["sha256"]) for row in records
    ).encode("utf-8")
    return {
        "file_count": len(records),
        "tree_sha256": hashlib.sha256(canonical).hexdigest(),
        "files": records,
    }


def _summary(snapshot):
    return {
        "file_count": snapshot["file_count"],
        "tree_sha256": snapshot["tree_sha256"],
    }


def _verify_contract(contract):
    expected_top_level = {
        "schema_version",
        "architecture_generation",
        "stage",
        "contract_id",
        "status",
        "design_only",
        "offline_only",
        "calibration_execution_allowed",
        "formal_result",
        "runtime_ready",
        "execution_ready",
        "training_allowed",
        "real_vehicle_use_forbidden",
        "scope",
        "single_changed_factor",
        "aligned_estimator",
        "frozen_non_factor_fields",
        "offline_identifiability",
        "common_integrity_repairs",
        "seed_and_budget_boundary",
        "frozen_inputs",
        "design_review_output",
        "forbidden_r6_artifacts_this_stage",
        "authorization",
    }
    _require(
        set(contract) == expected_top_level,
        "R6 contract top-level schema drifted",
    )
    _require_exact(
        {
            key: contract[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "contract_id",
                "status",
                "design_only",
                "offline_only",
                "calibration_execution_allowed",
                "formal_result",
                "runtime_ready",
                "execution_ready",
                "training_allowed",
                "real_vehicle_use_forbidden",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "contract_id": (
                "fam_teb_v2_04g_r6_runtime_evaluator_"
                "semantic_alignment_design_1"
            ),
            "status": (
                "design_preregistration_review_contract_"
                "execution_not_authorized"
            ),
            "design_only": True,
            "offline_only": True,
            "calibration_execution_allowed": False,
            "formal_result": False,
            "runtime_ready": False,
            "execution_ready": False,
            "training_allowed": False,
            "real_vehicle_use_forbidden": True,
        },
        "R6 contract safety boundary",
    )
    _require_exact(
        contract["scope"],
        {
            "source_stage": "V2-04G-TTC-D1",
            "frozen_execution_source_stage": "V2-04G-R5",
            "objective": (
                "Design exactly one categorical runtime conflict-estimator "
                "factor that makes non-NONE runtime conflict eligibility use "
                "the same relative circle-envelope first-contact primitive as "
                "the frozen evaluator, while implementing and offline-"
                "verifying all six D1 execution-integrity repairs."
            ),
            "d1_seed5111_use": "frozen_design_input_only_not_new_evidence",
            "new_evidence_units": 0,
            "execution_episode_count": 0,
            "runtime_config_count_persisted": 0,
            "scene_count_created": 0,
            "evaluator_files_changed": 0,
        },
        "R6 contract scope",
    )
    _require_exact(
        contract["single_changed_factor"],
        {
            "name": EXPECTED_FACTOR,
            "type": "categorical_atomic",
            "runtime_field": EXPECTED_RUNTIME_FIELD,
            "factor_count": 1,
            "control_value": LEGACY_ESTIMATOR_ID,
            "repair_value": ALIGNED_ESTIMATOR_ID,
            "only_profile_field_allowed_to_differ": True,
            "atomic_definition_includes": [
                "conflict_eligibility_primitive",
                "tracked_footprint_radius_interpretation",
                "multi_track_conflict_selection_order",
            ],
            "independently_tunable_subcomponents": False,
            "scientific_hypothesis": (
                "Using finite relative circle-envelope TTC as the necessary "
                "and sufficient runtime conflict-eligibility gate will remove "
                "the D1 mismatch in which centerline CROSSING overlays exist "
                "while evaluator finite TTC is absent."
            ),
        },
        "R6 contract single changed factor",
    )
    _require_exact(
        contract["aligned_estimator"],
        {
            "primitive": {
                "path": (
                    "src/perception/nav_world_model/src/nav_world_model/"
                    "risk_evidence.py"
                ),
                "callable": "relative_collision_ttc",
                "sha256": (
                    "96f20f43e5f764d8356725ee5b9d1598a4c9265ce228740"
                    "f69710fd085b7a0dc"
                ),
            },
            "track_frame": "current_robot_frame",
            "velocity_semantics": "relative_velocity",
            "actor_radius_source": (
                "tracked_obstacle_footprint_circumradius"
            ),
            "actor_radius_scene_truth_or_hardcode_allowed": False,
            "runtime_track_adapter": {
                "path": str(SEMANTIC_REFERENCE_RELATIVE),
                "callable": "runtime_track_from_footprint",
                "footprint_bearing_input_required": True,
                "precomputed_runtime_track_radius_accepted": False,
                "empty_footprint_frozen_default_m": 0.25,
            },
            "robot_radius_m": 0.62,
            "horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "minimum_relative_speed_mps": 0.05,
            "excluded_motion_classes": ["STATIONARY", "DEPARTING"],
            "conflict_present_iff_finite_ttc": True,
            "selected_conflict": (
                "earliest_ttc_then_frozen_overlay_priority_then_track_id"
            ),
            "no_finite_ttc_overlay": "NONE",
            "motion_class_use": "overlay_label_only_not_conflict_gate",
            "runtime_truth_access": False,
        },
        "R6 aligned estimator",
    )
    _require_exact(
        contract["frozen_non_factor_fields"],
        {
            "supervisor_predicted_ttc_max_s": 5.0,
            "evaluator_ttc_horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "robot_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "world_model_prediction_horizon_s": 2.0,
            "legacy_closest_approach_max_m": 1.35,
            "overlay_release_confirmation_s": 0.20,
            "horizon_values_1_5_and_1_0_enabled": False,
            "motion_classifier_thresholds_changed": False,
            "overlay_taxonomy_or_mapping_changed": False,
            "transition_hysteresis_or_dwell_changed": False,
            "anchor_bank_changed": False,
            "teb_parameters_changed": False,
            "mechanism_or_join_changed": False,
            "evaluator_changed": False,
            "scene_changed": False,
        },
        "R6 frozen non-factor fields",
    )
    _require_exact(
        contract["offline_identifiability"],
        {
            "source_report": {
                "path": str(
                    D1_ARTIFACT_RELATIVE / "v2_04g_ttc_d1_report.yaml"
                ),
                "sha256": EXPECTED_D1_REPORT_SHA256,
            },
            "trace_rows": 193,
            "legacy_proxy_non_none_count": 25,
            "legacy_proxy_crossing_count": 21,
            "legacy_proxy_overtake_or_yield_count": 4,
            "shared_circle_ttc_finite_count": 0,
            "expected_changed_rows_under_proposed_definition": 25,
            "source_reused_as_new_evidence": False,
            "claim_limit": "design_identifiability_only",
        },
        "R6 offline identifiability",
    )
    expected_common_integrity = {
        "candidate_symmetric": True,
        "parameter_surface_changed": [],
        "execution_proven": False,
        "readiness_direct_counts": {
            "risk_id": EXPECTED_RISK_IDS[0],
            "helper": (
                "thesis_experiment.v2_04g_r6_integrity."
                "validate_readiness_raw_evidence"
            ),
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "activation_and_evaluation_identity_directly_bound": True,
                "activation_tracker_message_count_min": 20,
                "activation_context_message_count_min": 20,
                "evaluation_tracker_message_count_min": 20,
                "evaluation_context_message_count_min": 20,
                "aggregate_boolean_alone_is_insufficient": True,
            },
        },
        "compiled_scene_snapshot": {
            "risk_id": EXPECTED_RISK_IDS[1],
            "helpers": [
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "acquire_compiled_scene_lease"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "materialize_scene_snapshot"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "revalidate_scene_snapshot"
                ),
            ],
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "index_and_child_hashes_verified": True,
                "source_bytes_captured_once": True,
                "attempt_local_content_addressed_snapshot": True,
                "exclusive_creation": True,
                "pre_spawn_revalidation": True,
                "post_episode_revalidation": True,
                "command_may_reference_only_bound_snapshot": True,
            },
        },
        "interruption_terminalization": {
            "risk_id": EXPECTED_RISK_IDS[2],
            "helpers": [
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "AtomicAttemptJournal"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "seal_orphaned_attempt"
                ),
            ],
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "outer_context_covers_start_work_evidence_and_teardown": True,
                "keyboard_interrupt_terminal_status": "terminal_interrupted",
                "orphan_nonterminal_status": "terminal_unclean_shutdown",
                "resume_after_interruption": False,
                "atomic_write_file_and_directory_fsync": True,
                "canonical_journal_root_required": True,
                "state_and_lock_path_derived_from_complete_identity": True,
                "caller_selected_state_filename_allowed": False,
                "concurrent_same_identity_allowed": False,
            },
        },
        "terminal_raw_evidence_binding": {
            "risk_id": EXPECTED_RISK_IDS[3],
            "helpers": [
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "bind_attempt_raw_evidence"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "bind_terminal_attempt_evidence"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "AtomicAttemptJournal.attach_terminal_evidence"
                ),
            ],
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "iterate_attempt_ledger_not_only_accepted_reports": True,
                (
                    "activation_evaluation_trace_clearance_log_and_"
                    "teardown_bound"
                ): True,
                "raw_hash_and_identity_cross_checks": True,
                "terminal_failure_may_have_incomplete_evidence": True,
                "not_produced_requires_phase_and_reason": True,
                (
                    "not_produced_phase_must_match_current_journal_lifecycle"
                ): True,
                "preattached_terminal_bundle_may_advance_lifecycle": False,
                (
                    "post_episode_terminal_requires_all_six_produced_"
                    "resources"
                ): True,
                "terminal_artifact_directory_inventory_must_be_exact": True,
                "silent_raw_evidence_omission_allowed": False,
            },
        },
        "complete_dependency_closure": {
            "risk_id": EXPECTED_RISK_IDS[4],
            "helper": (
                "thesis_experiment.v2_04g_r6_integrity."
                "verify_dependency_closure"
            ),
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "mechanically_generated_manifest_required": True,
                "dynamic_python_launch_config_and_scene_edges_required": True,
                "unresolved_dependencies_allowed": False,
                "guard_must_verify_before_ledger_or_subprocess": True,
                "regenerate_after_any_future_entrypoint_change": True,
                "current_execution_generator_status": (
                    "not_applicable_no_execution_entrypoint"
                ),
            },
        },
        "two_phase_teardown_restore": {
            "risk_id": EXPECTED_RISK_IDS[5],
            "helper": (
                "thesis_experiment.v2_04g_r6_integrity."
                "verify_teardown_restore"
            ),
            "additional_helpers": [
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "authorize_launch_stop"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "AtomicAttemptJournal.capture_startup_profile"
                ),
                (
                    "thesis_experiment.v2_04g_r6_integrity."
                    "AtomicAttemptJournal.verify_post_episode_scene"
                ),
            ],
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "future_gate": {
                "restore_requested_while_backend_alive": True,
                "transaction_ack_and_readback_receipt": True,
                "independent_final_readback_receipt": True,
                "startup_profile_exact_match": True,
                "launch_stop_only_after_restore_pass": True,
                "failure_terminal_status": "terminal_teardown_failure",
                "teardown_token_binds_complete_attempt_identity": True,
                "teardown_token_binds_journal_provenance": True,
                "launch_stop_gate_checks_expected_identity": True,
                "startup_profile_lease_binds_complete_attempt_identity": True,
            },
        },
    }
    _require_exact(
        contract["common_integrity_repairs"],
        expected_common_integrity,
        "R6 common integrity repairs",
    )
    _require_exact(
        contract["seed_and_budget_boundary"],
        {
            "seed_schedule_present": False,
            "seed_values": [],
            "seed_allocation_deferred": True,
            "seed_consumption": 0,
            "evidence_budget_authorized": 0,
            "d1_seed5111_reconsumed": False,
            "r5_remaining_68_units_consumed": False,
            "held_out_5001_5010_accessed": False,
            "future_seed_or_budget_amendment_requires_separate_review": True,
        },
        "R6 contract seed and budget boundary",
    )
    frozen_inputs = contract["frozen_inputs"]
    expected_frozen_labels = set(EXPECTED_RESOURCE_PATHS) - {
        "contract",
        "candidate_bank",
        "semantic_reference",
        "integrity_protocol",
    }
    _require(
        isinstance(frozen_inputs, dict)
        and set(frozen_inputs) == expected_frozen_labels,
        "R6 contract frozen-input label set drifted",
    )
    for label, row in frozen_inputs.items():
        _require(
            isinstance(row, dict)
            and set(row) == {"path", "sha256"}
            and row.get("path") == EXPECTED_RESOURCE_PATHS[label]
            and isinstance(row.get("sha256"), str)
            and len(row["sha256"]) == 64
            and all(character in "0123456789abcdef" for character in row["sha256"]),
            "R6 contract frozen-input declaration drifted: {}".format(label),
        )
    _require_exact(
        contract["design_review_output"],
        {
            "path": str(OUTPUT_RELATIVE),
            "deterministic": True,
            "atomic_write": True,
            "only_persistent_write_allowed": True,
        },
        "R6 contract design review output",
    )
    _require_exact(
        contract["forbidden_r6_artifacts_this_stage"],
        {
            "execution_authorization": True,
            "seed_schedule": True,
            "evidence_budget": True,
            "scene_manifest_or_compiled_scene": True,
            "persisted_runtime_candidate_config": True,
            "ros_node_or_launch": True,
            "episode_runner_or_batch": True,
            "assessment_claiming_execution_evidence": True,
        },
        "R6 contract forbidden artifacts",
    )
    _require_exact(
        contract["authorization"],
        {
            "design_and_preregistration_review": True,
            "pure_python_reference_implementation": True,
            "pure_python_unit_tests": True,
            "offline_d1_design_replay": True,
            "create_execution_authorization": False,
            "ros_or_gazebo_execution": False,
            "component_or_navigation_execution": False,
            "seed_or_evidence_consumption": False,
            "use_held_out_5001_5010": False,
            "candidate_ranking_or_winner_freeze": False,
            "start_v2_05": False,
            "sac_or_other_training": False,
            "connect_real_vehicle": False,
            "real_vehicle_motion": False,
            "write_real_vehicle_teb_parameters": False,
        },
        "R6 contract authorization",
    )


def _verify_preregistration(preregistration):
    _require(
        set(preregistration)
        == {
            "schema_version",
            "architecture_generation",
            "stage",
            "preregistration_id",
            "status",
            "design_only",
            "offline_only",
            "formal_result",
            "runtime_ready",
            "execution_ready",
            "execution_authorization_present",
            "training_allowed",
            "real_vehicle_use_forbidden",
            "research_question",
            "single_changed_factor",
            "candidate_ids",
            "candidate_roles",
            "shared_frozen_values",
            "offline_design_review",
            "integrity_repair_boundary",
            "seed_and_budget_boundary",
            "prospective_execution_boundary",
            "resources",
            "review_gates",
            "authorization_after_review",
        },
        "R6 preregistration top-level schema drifted",
    )
    _require_exact(
        {
            key: preregistration[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "preregistration_id",
                "status",
                "design_only",
                "offline_only",
                "formal_result",
                "runtime_ready",
                "execution_ready",
                "execution_authorization_present",
                "training_allowed",
                "real_vehicle_use_forbidden",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "preregistration_id": (
                "fam_teb_v2_04g_r6_semantic_alignment_design_review_1"
            ),
            "status": "design_preregistered_execution_not_authorized",
            "design_only": True,
            "offline_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "execution_ready": False,
            "execution_authorization_present": False,
            "training_allowed": False,
            "real_vehicle_use_forbidden": True,
        },
        "R6 preregistration safety boundary",
    )
    _require_exact(
        preregistration["research_question"],
        (
            "With all R5/D1 evidence frozen and no new execution, is a single "
            "atomic runtime conflict-estimator selector well specified and "
            "offline-identifiable when its repair level reuses the evaluator's "
            "circle-envelope first-contact primitive, and are all six D1 "
            "execution-integrity repairs implemented as candidate-symmetric "
            "fail-closed protocols that can gate a later review?"
        ),
        "R6 preregistration research question",
    )
    _require_exact(
        preregistration["single_changed_factor"],
        {
            "name": EXPECTED_FACTOR,
            "type": "categorical_atomic",
            "runtime_field": EXPECTED_RUNTIME_FIELD,
            "allowed_values": [
                LEGACY_ESTIMATOR_ID,
                ALIGNED_ESTIMATOR_ID,
            ],
            "factor_count": 1,
            "only_profile_field_allowed_to_differ": True,
            "independently_tunable_subcomponents": False,
            "horizon_scan_included": False,
            "horizon_values_1_5_or_1_0_included": False,
            "evaluator_or_scene_change_included": False,
        },
        "R6 preregistration single changed factor",
    )
    candidate_ids = [
        "r6_semantics_legacy_control",
        "r6_semantics_circle_contact",
    ]
    _require_exact(
        preregistration["candidate_ids"],
        candidate_ids,
        "R6 preregistration candidate IDs",
    )
    _require_exact(
        preregistration["candidate_roles"],
        {
            candidate_ids[0]: {
                "conflict_estimator_id": LEGACY_ESTIMATOR_ID,
                "role": "design_control",
                "winner_eligible": False,
            },
            candidate_ids[1]: {
                "conflict_estimator_id": ALIGNED_ESTIMATOR_ID,
                "role": "design_repair_candidate",
                "winner_eligible": False,
            },
        },
        "R6 preregistration candidate roles",
    )
    _require_exact(
        preregistration["shared_frozen_values"],
        {
            "predicted_ttc_max_s": 5.0,
            "evaluator_ttc_horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "robot_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "world_model_prediction_horizon_s": 2.0,
            "overlay_release_confirmation_s": 0.20,
            "runtime_truth_access": False,
            "empty_footprint_radius_default_m": 0.25,
            (
                "anchor_mechanism_transaction_join_evaluator_and_"
                "scene_unchanged"
            ): True,
        },
        "R6 preregistration shared frozen values",
    )
    _require_exact(
        preregistration["offline_design_review"],
        {
            "synthetic_semantic_fixture_count": 5,
            "required_fixture_classes": [
                "legacy_centerline_crossing_without_circle_contact",
                "aligned_head_on_circle_contact",
                "aligned_unknown_circle_contact",
                "excluded_stationary",
                "low_confidence_rejection",
            ],
            "frozen_d1_design_replay": {
                "trace_row_count": 193,
                "legacy_non_none_count": 25,
                "aligned_finite_ttc_count": 0,
                "changed_row_count": 25,
                "evidence_unit_count": 0,
                "seed5111_reconsumed": False,
            },
            "allowed_claims": [
                "single_factor_is_machine_specified",
                (
                    "semantic_candidate_is_offline_identifiable_on_"
                    "frozen_d1_input"
                ),
                "six_integrity_protocols_are_implemented_and_unit_verified",
            ],
            "forbidden_claims": [
                "runtime_or_evaluator_parity_execution_proven",
                "safety_performance_generalization_or_winner",
                "r5_resume_or_repair_result",
                "runtime_ready_or_formal_result",
            ],
        },
        "R6 preregistration offline design review",
    )
    _require_exact(
        preregistration["integrity_repair_boundary"],
        {
            "candidate_symmetric": True,
            "experimental_factor_count_contribution": 0,
            "required_risk_ids": list(EXPECTED_RISK_IDS),
            "design_fix_status_required": (
                "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
            ),
            "execution_validation_status_required": "NOT_RUN_NOT_AUTHORIZED",
            "future_runner_integration_required": True,
            "future_assessor_integration_required": True,
            "future_two_phase_teardown_integration_required": True,
            "future_dependency_closure_regeneration_required": True,
        },
        "R6 preregistration integrity repair boundary",
    )
    _require_exact(
        preregistration["seed_and_budget_boundary"],
        {
            "seed_schedule_present": False,
            "seed_values": [],
            "seed_allocation_deferred": True,
            "evidence_budget_authorized": 0,
            "evidence_units_consumed": 0,
            "r5_remaining_units_consumed": 0,
            "held_out_5001_5010_accessed": False,
            (
                "future_seed_and_budget_amendment_requires_separate_"
                "user_review"
            ): True,
        },
        "R6 preregistration seed and budget boundary",
    )
    _require_exact(
        preregistration["prospective_execution_boundary"],
        {
            "runtime_candidate_configs_persisted": 0,
            "scene_manifests_created": 0,
            "compiled_scenes_created": 0,
            "ros_nodes_or_launch_files_created": 0,
            "episode_runners_or_batches_created": 0,
            "execution_dependency_manifest_created": False,
            "reason_execution_dependency_manifest_absent": (
                "No R6 execution entrypoint exists in this design-only stage.  "
                "A mechanical execution closure must be generated after a "
                "future runner/node exists and before a separate authorization "
                "review."
            ),
            "execution_authorization_artifact_created": False,
        },
        "R6 preregistration prospective execution boundary",
    )
    _require_exact(
        preregistration["review_gates"],
        {
            "strict_yaml_duplicate_key_rejection": True,
            "exact_resource_path_and_sha256_closure": True,
            "exactly_one_candidate_field_diff": True,
            "aligned_primitive_is_frozen_relative_collision_ttc": True,
            "footprint_circumradius_matches_evaluator_semantics": True,
            "footprint_bearing_runtime_adapter_is_mandatory": True,
            "frozen_d1_replay_is_design_input_not_evidence": True,
            "all_six_integrity_protocols_unit_verified": True,
            "r5_and_d1_trees_unchanged": True,
            "execution_authorization_artifact_count": 0,
            "seed_schedule_present": False,
            "evidence_budget_authorized": 0,
            "all_downstream_authorizations_false": True,
        },
        "R6 preregistration review gates",
    )
    _require_exact(
        preregistration["authorization_after_review"],
        {
            "create_execution_authorization": False,
            "persist_runtime_candidate_config": False,
            "create_scene_or_compiled_scene": False,
            "create_ros_node_or_launch": False,
            "create_episode_runner_or_batch": False,
            "start_ros_or_gazebo": False,
            "execute_component_or_navigation": False,
            "allocate_or_consume_seed": False,
            "consume_r5_remaining_units": False,
            "use_held_out_5001_5010": False,
            "rank_candidate_or_freeze_winner": False,
            "start_v2_05": False,
            "train_sac_or_any_model": False,
            "connect_or_move_real_vehicle": False,
            "write_real_vehicle_teb_parameters": False,
        },
        "R6 preregistration authorization after review",
    )


def _verify_candidate_bank(bank):
    _require(
        set(bank)
        == {
            "schema_version",
            "architecture_generation",
            "stage",
            "candidate_bank_id",
            "status",
            "design_only",
            "offline_only",
            "formal_result",
            "runtime_ready",
            "execution_authorized",
            "training_allowed",
            "real_vehicle_use_forbidden",
            "single_changed_factor",
            "shared_frozen_values",
            "estimators",
            "candidates",
            "excluded_second_factors",
            "materialization_boundary",
        },
        "R6 candidate bank top-level schema drifted",
    )
    _require_exact(
        {
            key: bank[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "candidate_bank_id",
                "status",
                "design_only",
                "offline_only",
                "formal_result",
                "runtime_ready",
                "execution_authorized",
                "training_allowed",
                "real_vehicle_use_forbidden",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "candidate_bank_id": (
                "fam_teb_v2_04g_r6_dynamic_conflict_semantics_design_1"
            ),
            "status": "design_candidates_not_materialized_not_executable",
            "design_only": True,
            "offline_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "execution_authorized": False,
            "training_allowed": False,
            "real_vehicle_use_forbidden": True,
        },
        "R6 candidate bank safety boundary",
    )
    _require_exact(
        bank["single_changed_factor"],
        {
            "name": EXPECTED_FACTOR,
            "type": "categorical_atomic",
            "runtime_field": EXPECTED_RUNTIME_FIELD,
            "factor_count": 1,
            "control_value": LEGACY_ESTIMATOR_ID,
            "repair_value": ALIGNED_ESTIMATOR_ID,
            "independently_tunable_subcomponents": False,
            "only_profile_field_allowed_to_differ": True,
        },
        "R6 candidate bank single changed factor",
    )
    _require_exact(
        bank["shared_frozen_values"],
        {
            "predicted_ttc_max_s": 5.0,
            "evaluator_ttc_horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "robot_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "world_model_prediction_horizon_s": 2.0,
            "closest_approach_max_m_control_only": 1.35,
            "overlay_release_confirmation_s": 0.20,
            "footprint_input": "tracked_obstacle_footprint",
            "empty_footprint_radius_default_m": 0.25,
            "runtime_scene_or_gazebo_truth_access": False,
        },
        "R6 candidate bank shared frozen values",
    )
    _require_exact(
        bank["estimators"],
        {
            LEGACY_ESTIMATOR_ID: {
            "role": "frozen_behavior_control_not_winner_eligible",
            "conflict_gate": {
                "crossing": "centerline_crossing_time",
                "other_dynamic_classes": "point_closest_approach",
            },
            "footprint_radius_semantics": "maximum_absolute_x_extent",
            "evaluator_aligned": False,
            },
            ALIGNED_ESTIMATOR_ID: {
                "role": (
                    "semantic_alignment_repair_candidate_not_winner_eligible"
                ),
                "conflict_gate": {
                    "primitive": (
                        "nav_world_model.risk_evidence.relative_collision_ttc"
                    ),
                    "conflict_present_iff_finite_ttc": True,
                    "no_finite_ttc_overlay": "NONE",
                    "selected_conflict": (
                        "earliest_ttc_then_frozen_overlay_priority_then_"
                        "track_id"
                    ),
                    "motion_class_use": (
                        "overlay_label_only_not_conflict_gate"
                    ),
                },
                "footprint_radius_semantics": "maximum_xy_circumradius",
                "runtime_track_adapter": "runtime_track_from_footprint",
                "precomputed_legacy_runtime_track_radius_accepted": False,
                "excluded_motion_classes": ["STATIONARY", "DEPARTING"],
                "evaluator_aligned": True,
            },
        },
        "R6 candidate bank estimator definitions",
    )
    expected_candidate_rows = [
        {
            "candidate_id": "r6_semantics_legacy_control",
            "conflict_estimator_id": LEGACY_ESTIMATOR_ID,
            "role": "design_control",
            "winner_eligible": False,
            "runtime_config_materialized": False,
        },
        {
            "candidate_id": "r6_semantics_circle_contact",
            "conflict_estimator_id": ALIGNED_ESTIMATOR_ID,
            "role": "design_repair_candidate",
            "winner_eligible": False,
            "runtime_config_materialized": False,
        },
    ]
    _require_exact(
        bank["candidates"],
        expected_candidate_rows,
        "R6 design candidate declarations",
    )
    _require_exact(
        bank["excluded_second_factors"],
        {
            "horizon_values_1_5_or_1_0_present": False,
            "numeric_horizon_scan": False,
            "motion_classifier_threshold_change": False,
            "overlay_mapping_change": False,
            "transition_or_dwell_change": False,
            "anchor_bank_change": False,
            "teb_parameter_change": False,
            "mechanism_or_join_change": False,
            "evaluator_change": False,
            "scene_change": False,
        },
        "R6 candidate bank excluded second factors",
    )
    _require_exact(
        bank["materialization_boundary"],
        {
            "in_memory_design_comparison_allowed": True,
            "persist_runtime_config_allowed": False,
            "create_ros_node_or_launch_allowed": False,
            "create_episode_runner_or_batch_allowed": False,
            "execution_authorization_required_for_any_future_runtime_use": True,
        },
        "R6 candidate bank materialization boundary",
    )


def _verify_resources(workspace, preregistration):
    resources = preregistration.get("resources")
    _require(
        isinstance(resources, dict)
        and set(resources) == set(EXPECTED_RESOURCE_PATHS),
        "R6 preregistered resource label set drifted",
    )
    verified = {}
    paths = set()
    for label, row in resources.items():
        _require(
            isinstance(row, dict)
            and set(row) == {"path", "sha256"}
            and isinstance(row["path"], str)
            and not Path(row["path"]).is_absolute()
            and ".." not in Path(row["path"]).parts
            and isinstance(row["sha256"], str)
            and len(row["sha256"]) == 64,
            "R6 resource declaration drifted: {}".format(label),
        )
        _require(
            row["path"] == EXPECTED_RESOURCE_PATHS[label],
            "R6 resource canonical path drifted: {}".format(label),
        )
        path = _inside(workspace, Path(workspace) / row["path"], label)
        _require(
            path.is_file() and not (Path(workspace) / row["path"]).is_symlink(),
            "R6 resource is missing or unsafe: {}".format(label),
        )
        _require(row["path"] not in paths, "duplicate R6 resource path")
        paths.add(row["path"])
        digest = _sha256(path)
        _require(
            digest == row["sha256"],
            "R6 resource hash drifted: {}".format(label),
        )
        verified[label] = {
            "path": row["path"],
            "sha256": digest,
            "verified": True,
        }
    return verified


def _digest_workspace_relative_file_once(workspace, relative, label):
    """Hash one regular file through no-follow directory descriptors."""

    _require(
        isinstance(relative, str)
        and relative
        and not Path(relative).is_absolute()
        and relative == Path(relative).as_posix()
        and ".." not in Path(relative).parts
        and "." not in Path(relative).parts
        and "\\" not in relative,
        "{} path is not canonical workspace-relative".format(label),
    )
    parts = Path(relative).parts
    _require(parts, "{} path is empty".format(label))
    nofollow = getattr(os, "O_NOFOLLOW", None)
    directory_flag = getattr(os, "O_DIRECTORY", None)
    _require(
        nofollow is not None and directory_flag is not None,
        "platform lacks no-follow closure verification",
    )
    root = Path(workspace).resolve()
    descriptors = []
    try:
        current = os.open(
            str(root),
            os.O_RDONLY | directory_flag | nofollow,
        )
        descriptors.append(current)
        for part in parts[:-1]:
            current = os.open(
                part,
                os.O_RDONLY | directory_flag | nofollow,
                dir_fd=current,
            )
            descriptors.append(current)
        descriptor = os.open(
            parts[-1],
            os.O_RDONLY | nofollow,
            dir_fd=current,
        )
        descriptors.append(descriptor)
        metadata = os.fstat(descriptor)
        _require(
            stat.S_ISREG(metadata.st_mode),
            "{} is not a regular file".format(label),
        )
        digest = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        return digest.hexdigest(), metadata.st_size
    except OSError as exc:
        raise R6ReviewError(
            "{} is missing, unsafe, or symlinked: {}".format(label, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _verify_d1_declared_closure(workspace, d1_contract):
    """Verify every path/hash declared by the frozen D1 input contract."""

    frozen_inputs = d1_contract.get("frozen_inputs")
    dependency_chain = d1_contract.get("execution_dependency_chain")
    _require(
        isinstance(frozen_inputs, dict) and frozen_inputs,
        "D1 frozen_inputs must be a non-empty mapping",
    )
    _require(
        isinstance(dependency_chain, list) and dependency_chain,
        "D1 execution_dependency_chain must be a non-empty list",
    )
    declarations = []
    for label in sorted(frozen_inputs):
        declarations.append(
            (
                "frozen_inputs.{}".format(label),
                frozen_inputs[label],
            )
        )
    for index, row in enumerate(dependency_chain):
        declarations.append(
            ("execution_dependency_chain[{}]".format(index), row)
        )

    by_path = {}
    for source, row in declarations:
        _require(
            isinstance(row, dict)
            and set(row) == {"path", "sha256"},
            "D1 closure declaration schema drifted: {}".format(source),
        )
        relative = row.get("path")
        declared_digest = row.get("sha256")
        _require(
            isinstance(relative, str),
            "D1 closure path is not a string: {}".format(source),
        )
        _require(
            isinstance(declared_digest, str)
            and len(declared_digest) == 64
            and all(
                character in "0123456789abcdef"
                for character in declared_digest
            ),
            "D1 closure digest is not lowercase SHA256: {}".format(source),
        )
        existing = by_path.get(relative)
        if existing is not None:
            _require(
                existing["sha256"] == declared_digest,
                "D1 closure has conflicting declarations for {}".format(
                    relative
                ),
            )
            existing["declaration_sources"].append(source)
            continue
        actual_digest, size_bytes = _digest_workspace_relative_file_once(
            workspace,
            relative,
            "D1 closure {}".format(source),
        )
        _require(
            actual_digest == declared_digest,
            "D1 declared dependency hash drifted: {}".format(relative),
        )
        by_path[relative] = {
            "path": relative,
            "sha256": actual_digest,
            "size_bytes": size_bytes,
            "declaration_sources": [source],
        }

    files = []
    for relative in sorted(by_path):
        row = by_path[relative]
        files.append(
            {
                "path": row["path"],
                "sha256": row["sha256"],
                "size_bytes": row["size_bytes"],
                "declaration_sources": sorted(row["declaration_sources"]),
            }
        )
    canonical = "".join(
        "{} {}\n".format(row["path"], row["sha256"]) for row in files
    ).encode("utf-8")
    return {
        "frozen_input_declaration_count": len(frozen_inputs),
        "execution_dependency_declaration_count": len(dependency_chain),
        "total_declaration_count": len(declarations),
        "unique_file_count": len(files),
        "duplicate_identical_declaration_count": (
            len(declarations) - len(files)
        ),
        "all_paths_canonical_workspace_relative": True,
        "all_components_no_follow_verified": True,
        "all_declared_hashes_match_single_open_reads": True,
        "closure_sha256": hashlib.sha256(canonical).hexdigest(),
        "files": files,
    }


def _load_project_modules(workspace, verified_resources=None):
    if verified_resources is None:
        verified_resources = _verify_resources(
            workspace,
            _load_yaml(Path(workspace) / PREREGISTRATION_RELATIVE),
        )
    additions = (
        Path(workspace) / "src/perception/nav_world_model/src",
        Path(workspace) / "src/application/teb_mode_manager/src",
        Path(workspace) / "src/tools/thesis_experiment/src",
    )
    for addition in reversed(additions):
        value = str(addition)
        if value not in sys.path:
            sys.path.insert(0, value)
    # pylint: disable=import-outside-toplevel
    from nav_world_model import risk_evidence
    from teb_mode_manager import r6_relative_ttc_supervisor as semantic
    from teb_mode_manager import rule_supervisor
    from thesis_experiment import v2_04g_r6_integrity as integrity
    # pylint: enable=import-outside-toplevel

    module_boundaries = (
        (
            risk_evidence,
            "frozen_risk_evidence",
            EXPECTED_RESOURCE_PATHS["frozen_risk_evidence"],
        ),
        (
            rule_supervisor,
            "frozen_rule_supervisor",
            EXPECTED_RESOURCE_PATHS["frozen_rule_supervisor"],
        ),
        (
            semantic,
            "semantic_reference",
            str(SEMANTIC_REFERENCE_RELATIVE),
        ),
        (
            integrity,
            "integrity_protocol",
            str(INTEGRITY_PROTOCOL_RELATIVE),
        ),
    )
    for module, resource_label, expected_relative in module_boundaries:
        module_file = getattr(module, "__file__", None)
        _require(
            isinstance(module_file, str),
            "loaded {} module has no source provenance".format(
                resource_label
            ),
        )
        actual = Path(module_file).resolve()
        expected = (Path(workspace).resolve() / expected_relative).resolve()
        _require(
            actual == expected,
            "loaded {} module escaped verified workspace source".format(
                resource_label
            ),
        )
        _require(
            verified_resources[resource_label]["path"] == expected_relative
            and _sha256(actual)
            == verified_resources[resource_label]["sha256"],
            "loaded {} module hash no longer matches verification".format(
                resource_label
            ),
        )

    return {
        "RelativeTrack": risk_evidence.RelativeTrack,
        "relative_collision_ttc": risk_evidence.relative_collision_ttc,
        "R6RelativeTTCSupervisor": semantic.R6RelativeTTCSupervisor,
        "FootprintRuntimeTrack": semantic.FootprintRuntimeTrack,
        "evaluator_aligned_conflict": semantic.evaluator_aligned_conflict,
        "footprint_radius": semantic.footprint_radius,
        "runtime_track_from_footprint": semantic.runtime_track_from_footprint,
        "RuntimeTrack": rule_supervisor.RuntimeTrack,
        "integrity": integrity,
    }


def _in_memory_config(estimator_id):
    return {
        "transition": {
            "minimum_mode_confidence": 0.55,
            "minimum_dwell_s": 0.5,
            "enter_confirmation_s": 0.1,
            "exit_confirmation_s": 0.1,
            "blend_duration_s": 0.2,
            "overlay_release_confirmation_s": 0.20,
            "switch_score_margin": 0.05,
        },
        "dynamic": {
            "minimum_track_confidence": 0.45,
            "predicted_ttc_max_s": 5.0,
            "closest_approach_max_m": 1.35,
            "robot_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "conflict_estimator_id": estimator_id,
        },
    }


def _leaf_differences(first, second, prefix=""):
    _require(type(first) is type(second), "in-memory candidate type drifted")
    if isinstance(first, dict):
        _require(set(first) == set(second), "in-memory candidate keys drifted")
        differences = []
        for key in sorted(first):
            path = "{}.{}".format(prefix, key) if prefix else key
            differences.extend(_leaf_differences(first[key], second[key], path))
        return differences
    return [] if first == second else [prefix]


def _semantic_review(modules, candidate_bank):
    FootprintRuntimeTrack = modules["FootprintRuntimeTrack"]
    candidate_rows = candidate_bank["candidates"]
    control_config = _in_memory_config(
        candidate_rows[0]["conflict_estimator_id"]
    )
    repair_config = _in_memory_config(
        candidate_rows[1]["conflict_estimator_id"]
    )
    differences = _leaf_differences(control_config, repair_config)
    _require(
        differences == ["dynamic.conflict_estimator_id"],
        "in-memory candidates differ by more than the selector",
    )
    control = modules["R6RelativeTTCSupervisor"](control_config)
    repair = modules["R6RelativeTTCSupervisor"](repair_config)
    repair_config["dynamic"]["predicted_ttc_max_s"] = 1.0
    _require(
        repair.horizon_s == 5.0,
        "supervisor retained a mutable external config reference",
    )
    footprint_030 = ((0.30, 0.0), (-0.30, 0.0), (0.0, 0.30), (0.0, -0.30))

    def make_track(track_id, motion_class, x, y, vx, vy, confidence):
        return FootprintRuntimeTrack(
            track_id=track_id,
            motion_class=motion_class,
            x=x,
            y=y,
            vx=vx,
            vy=vy,
            footprint=footprint_030,
            confidence=confidence,
        )

    fixtures = (
        (
            "legacy_centerline_crossing_without_circle_contact",
            make_track(1, "CROSSING", 4.0, -2.0, 0.0, 1.0, 0.90),
            "CROSSING",
            "NONE",
        ),
        (
            "aligned_head_on_circle_contact",
            make_track(2, "HEAD_ON", 4.0, 0.0, -1.0, 0.0, 0.90),
            "HEAD_ON",
            "HEAD_ON",
        ),
        (
            "aligned_unknown_circle_contact",
            make_track(3, "UNKNOWN", 2.0, 0.0, -0.5, 0.0, 0.90),
            "OVERTAKE_OR_YIELD",
            "OVERTAKE_OR_YIELD",
        ),
        (
            "excluded_stationary",
            make_track(4, "STATIONARY", 0.5, 0.0, 0.0, 0.0, 0.90),
            "NONE",
            "NONE",
        ),
        (
            "low_confidence_rejection",
            make_track(5, "HEAD_ON", 2.0, 0.0, -1.0, 0.0, 0.40),
            "NONE",
            "NONE",
        ),
    )
    results = []
    finite_parity_count = 0
    for fixture_id, track, expected_control, expected_repair in fixtures:
        control_overlay, _ = control._overlay([track])
        repair_overlay, repair_reason = repair._overlay([track])
        adapted = modules["runtime_track_from_footprint"](
            track, ALIGNED_ESTIMATOR_ID
        )
        decision = modules["evaluator_aligned_conflict"](
            [adapted],
            robot_radius_m=0.62,
            horizon_s=5.0,
            minimum_track_confidence=0.45,
            minimum_relative_speed_mps=0.05,
        )
        _require(control_overlay == expected_control, "{} control drifted".format(fixture_id))
        _require(decision.overlay == expected_repair, "{} repair drifted".format(fixture_id))
        _require(
            repair_overlay == decision.overlay
            and (
                decision.track_id is None
                or repair_reason.endswith(str(decision.track_id))
            ),
            "{} supervisor wiring drifted".format(fixture_id),
        )
        evaluator_ttc = modules["relative_collision_ttc"](
            modules["RelativeTrack"](
                x=adapted.x,
                y=adapted.y,
                vx=adapted.vx,
                vy=adapted.vy,
                radius=adapted.radius,
                confidence=adapted.confidence,
                motion_class=adapted.motion_class,
            ),
            robot_radius_m=0.62,
            horizon_s=5.0,
            minimum_confidence=0.45,
            minimum_relative_speed_mps=0.05,
        )
        _require(
            (decision.ttc_s is None) == (evaluator_ttc is None),
            "{} evaluator finite-TTC parity drifted".format(fixture_id),
        )
        if decision.ttc_s is not None:
            _require(
                math.isclose(decision.ttc_s, evaluator_ttc, abs_tol=1.0e-12),
                "{} evaluator TTC value drifted".format(fixture_id),
            )
            finite_parity_count += 1
        results.append({
            "fixture_id": fixture_id,
            "legacy_overlay": control_overlay,
            "aligned_overlay": repair_overlay,
            "aligned_ttc_s": decision.ttc_s,
            "matches_frozen_evaluator": True,
            "pass": True,
        })
    footprint = ((-0.275, -0.275), (-0.275, 0.275), (0.275, 0.275), (0.275, -0.275))
    legacy_radius = modules["footprint_radius"](footprint, LEGACY_ESTIMATOR_ID)
    aligned_radius = modules["footprint_radius"](footprint, ALIGNED_ESTIMATOR_ID)
    legacy_default = modules["footprint_radius"]([], LEGACY_ESTIMATOR_ID)
    aligned_default = modules["footprint_radius"]([], ALIGNED_ESTIMATOR_ID)
    _require(
        math.isclose(legacy_radius, 0.275, abs_tol=1.0e-12)
        and math.isclose(
            aligned_radius, 0.3889087296526012, abs_tol=1.0e-12
        ),
        "tracked footprint radius semantics drifted",
    )
    _require(
        legacy_default == aligned_default == 0.25,
        "empty-footprint frozen fallback drifted",
    )

    def aligned_decision(pre_tracks):
        return modules["evaluator_aligned_conflict"](
            [
                modules["runtime_track_from_footprint"](
                    value, ALIGNED_ESTIMATOR_ID
                )
                for value in pre_tracks
            ],
            robot_radius_m=0.62,
            horizon_s=5.0,
            minimum_track_confidence=0.45,
            minimum_relative_speed_mps=0.05,
        )

    earliest = aligned_decision([
        make_track(10, "HEAD_ON", 4.0, 0.0, -1.0, 0.0, 0.90),
        make_track(11, "UNKNOWN", 2.0, 0.0, -1.0, 0.0, 0.90),
    ])
    same_ttc_priority = aligned_decision([
        make_track(20, "CROSSING", 4.0, 0.0, -1.0, 0.0, 0.90),
        make_track(21, "HEAD_ON", 4.0, 0.0, -1.0, 0.0, 0.90),
    ])
    same_ttc_track_id = aligned_decision([
        make_track(9, "UNKNOWN", 4.0, 0.0, -1.0, 0.0, 0.90),
        make_track(2, "UNKNOWN", 4.0, 0.0, -1.0, 0.0, 0.90),
    ])
    _require(
        earliest.track_id == 11
        and earliest.overlay == "OVERTAKE_OR_YIELD"
        and same_ttc_priority.track_id == 21
        and same_ttc_priority.overlay == "HEAD_ON"
        and same_ttc_track_id.track_id == 2,
        "aligned multi-track ordering drifted",
    )
    return {
        "factor_count": 1,
        "in_memory_candidate_count": 2,
        "only_differing_profile_field": EXPECTED_RUNTIME_FIELD,
        "persistent_runtime_config_count": 0,
        "fixture_count": len(results),
        "finite_ttc_evaluator_parity_fixture_count": finite_parity_count,
        "supervisor_candidate_wiring_fixture_count": len(results),
        "multi_track_ordering_fixture_count": 3,
        "fixtures": results,
        "footprint_radius_semantics": {
            "legacy_maximum_absolute_x_m": legacy_radius,
            "aligned_xy_circumradius_m": aligned_radius,
            "empty_footprint_frozen_default_m": aligned_default,
            "matches_frozen_evaluator_actor_radius": True,
        },
        "pass": True,
    }


def _expect_rejection(callable_value, integrity):
    try:
        callable_value()
    except integrity.R6IntegrityError:
        return True
    raise R6ReviewError("integrity negative fixture did not fail closed")


def _write_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        yaml.safe_dump(value, sort_keys=False), encoding="utf-8"
    )


def _write_bytes(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(value)


def _integrity_review(workspace, modules, preregistration):
    integrity = modules["integrity"]
    _require(
        tuple(integrity.RISK_REPAIR_IDS) == EXPECTED_RISK_IDS,
        "integrity implementation risk IDs drifted",
    )
    identity = {
        "stage": STAGE,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r5-readiness-dynamic-conflict-s5111",
        "seed": 0,
        "attempt": 1,
    }
    raw = dict(identity)
    raw.update({"tracker_message_count": 20, "context_message_count": 20})
    direct_counts = integrity.validate_readiness_raw_evidence(
        identity, raw, dict(raw), 20
    )
    direct_negative_count = 0
    for mutation in (
        lambda value: value.update({"tracker_message_count": 19}),
        lambda value: value.update({"context_message_count": True}),
        lambda value: value.update({"profile_id": "wrong_profile"}),
        lambda value: value.update({"seed": False, "attempt": True}),
    ):
        invalid = dict(raw)
        mutation(invalid)
        _expect_rejection(
            lambda invalid=invalid: integrity.validate_readiness_raw_evidence(
                identity, invalid, dict(raw), 20
            ),
            integrity,
        )
        direct_negative_count += 1

    d1_contract = _load_yaml(
        Path(workspace)
        / "config/thesis_experiments/v2/"
        "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
    )
    index = d1_contract["frozen_inputs"]["readiness_compiled_index"]
    with tempfile.TemporaryDirectory(prefix="v2_04g_r6_integrity_") as temporary:
        fixture_root = Path(temporary)
        lease = integrity.acquire_compiled_scene_lease(
            workspace,
            index["path"],
            index["sha256"],
            "v2-04g-r5-readiness-dynamic-conflict-s5111",
        )
        snapshot = integrity.materialize_scene_snapshot(
            lease, fixture_root / "snapshot"
        )
        tamper_snapshot = integrity.materialize_scene_snapshot(
            lease, fixture_root / "tamper_snapshot"
        )
        integrity.revalidate_scene_snapshot(tamper_snapshot, "pre_spawn")
        tamper_document = tamper_snapshot.as_document()
        snapshot_instance = Path(
            tamper_document["snapshot_instance"]["path"]
        )
        snapshot_instance.chmod(0o600)
        snapshot_instance.write_bytes(snapshot_instance.read_bytes() + b"\n")
        _expect_rejection(
            lambda: integrity.revalidate_scene_snapshot(
                tamper_snapshot, "post_episode"
            ),
            integrity,
        )

        artifact = fixture_root / "artifacts/attempt"
        trace = b"stamp,x,y\n0.0,0.0,0.0\n"
        trace_digest = hashlib.sha256(trace).hexdigest()
        activation = dict(identity)
        activation.update({
            "tracker_message_count": 20,
            "context_message_count": 20,
        })
        evaluation = dict(activation)
        evaluation["raw_trace_sha256"] = trace_digest
        clearance = dict(identity)
        clearance["contact_count"] = 0
        startup_profile = b"synthetic frozen startup profile\n"
        startup_hash = hashlib.sha256(startup_profile).hexdigest()
        teardown_receipt = dict(identity)
        teardown_receipt.update({
            "restore_requested_while_backend_alive": True,
            "transaction_acknowledged": True,
            "transaction_readback_match": True,
            "independent_readback_match": True,
            "startup_profile_sha256": startup_hash,
            "transaction_readback_sha256": startup_hash,
            "independent_readback_sha256": startup_hash,
        })
        payloads = {
            "activation": yaml.safe_dump(
                activation, sort_keys=False
            ).encode("utf-8"),
            "evaluation": yaml.safe_dump(
                evaluation, sort_keys=False
            ).encode("utf-8"),
            "trace": trace,
            "clearance": yaml.safe_dump(
                clearance, sort_keys=False
            ).encode("utf-8"),
            "process_log": b"synthetic offline protocol fixture\n",
            "teardown_receipt": yaml.safe_dump(
                teardown_receipt, sort_keys=False
            ).encode("utf-8"),
        }
        resources = {}
        for label, payload in payloads.items():
            suffix = ".yaml" if label in {
                "activation", "evaluation", "clearance", "teardown_receipt"
            } else (".csv" if label == "trace" else ".log")
            path = artifact / (label + suffix)
            _write_bytes(path, payload)
            relative = str(path.relative_to(fixture_root))
            resources[label] = {
                "path": relative,
                "sha256": hashlib.sha256(payload).hexdigest(),
            }

        complete_journal_root = fixture_root / "journals_complete"
        with integrity.AtomicAttemptJournal(
            complete_journal_root, identity
        ) as journal:
            journal_path = journal.path
            startup_lease = journal.capture_startup_profile(startup_profile)
            journal.bind_scene_snapshot(snapshot)
            journal.mark_execution_started()
            post_episode = journal.verify_post_episode_scene()
            binding = integrity.bind_attempt_raw_evidence(
                fixture_root,
                "artifacts/attempt",
                identity,
                resources,
                20,
                startup_lease,
                post_episode,
            )
            binding_document = binding.as_document()
            _expect_rejection(
                lambda: journal.complete({"unvalidated": True}), integrity
            )
            _expect_rejection(
                lambda: journal.complete(binding), integrity
            )
            teardown_token = binding.verified_teardown
            stop_gate = journal.authorize_launch_stop(teardown_token)
            _require(
                stop_gate.get("launch_stop_allowed") is True
                and stop_gate.get("identity") == identity,
                "verified teardown did not authorize identity-bound launch stop",
            )
            journal.complete(binding)
            _expect_rejection(lambda: journal.complete(binding), integrity)
        completed = integrity.strict_yaml(journal_path)
        _require(
            completed.get("status") == "evidence_complete"
            and completed.get("active_identity") is None
            and completed["evidence_binding"]["raw_evidence_bound"] is True,
            "validated evidence did not complete the journal",
        )
        _expect_rejection(
            lambda: integrity.authorize_launch_stop(
                teardown_token, identity, journal
            ),
            integrity,
        )
        invalid_receipt = dict(teardown_receipt)
        invalid_receipt["independent_readback_match"] = False
        _expect_rejection(
            lambda: integrity.verify_teardown_restore(
                invalid_receipt,
                startup_lease,
                post_episode,
                identity,
            ),
            integrity,
        )
        bool_identity_receipt = dict(teardown_receipt)
        bool_identity_receipt.update({"seed": False, "attempt": True})
        _expect_rejection(
            lambda: integrity.verify_teardown_restore(
                bool_identity_receipt,
                startup_lease,
                post_episode,
                identity,
            ),
            integrity,
        )

        alias_resources = copy.deepcopy(resources)
        alias_resources["process_log"] = dict(alias_resources["trace"])
        _expect_rejection(
            lambda: integrity.bind_attempt_raw_evidence(
                fixture_root,
                "artifacts/attempt",
                identity,
                alias_resources,
                20,
                startup_lease,
                post_episode,
            ),
            integrity,
        )
        trace_path = artifact / "trace.csv"
        trace_path.write_bytes(trace + b"1.0,1.0,1.0\n")
        _expect_rejection(
            lambda: integrity.bind_attempt_raw_evidence(
                fixture_root,
                "artifacts/attempt",
                identity,
                resources,
                20,
                startup_lease,
                post_episode,
            ),
            integrity,
        )
        trace_path.write_bytes(trace)

        terminal_root = fixture_root / "artifacts/terminal"
        terminal_root.mkdir(parents=True)
        not_produced = {
            label: {
                "status": "not_produced",
                "phase": "attempt_started",
                "reason": "synthetic_interrupt_fixture",
            }
            for label in integrity.RAW_EVIDENCE_LABELS
        }
        terminal_binding = integrity.bind_terminal_attempt_evidence(
            fixture_root,
            "artifacts/terminal",
            identity,
            not_produced,
        )
        impossible_phase = copy.deepcopy(not_produced)
        impossible_phase["activation"]["phase"] = "impossible_phase"
        _expect_rejection(
            lambda: integrity.bind_terminal_attempt_evidence(
                fixture_root,
                "artifacts/terminal",
                identity,
                impossible_phase,
            ),
            integrity,
        )
        late_not_produced = copy.deepcopy(not_produced)
        for row in late_not_produced.values():
            row["phase"] = "post_episode_scene_verified"
        _expect_rejection(
            lambda: integrity.bind_terminal_attempt_evidence(
                fixture_root,
                "artifacts/terminal",
                identity,
                late_not_produced,
            ),
            integrity,
        )
        interrupted_journal_root = fixture_root / "journals_interrupted"
        try:
            with integrity.AtomicAttemptJournal(
                interrupted_journal_root, identity
            ) as interrupted_journal:
                interrupted_path = interrupted_journal.path
                interrupted_journal.attach_terminal_evidence(
                    terminal_binding
                )
                raise KeyboardInterrupt()
        except KeyboardInterrupt:
            pass
        interrupted = integrity.strict_yaml(interrupted_path)
        _require(
            interrupted.get("status") == "terminal_interrupted"
            and interrupted.get("resume_forbidden") is True
            and interrupted["evidence_binding"][
                "terminal_raw_evidence_declared"
            ]
            is True,
            "SIGINT did not terminalize the attempt",
        )
        orphan_journal_root = fixture_root / "journals_orphan"
        orphan_path = integrity.canonical_attempt_state_path(
            orphan_journal_root, identity
        )
        orphan_path.parent.mkdir(parents=True)
        _write_yaml(orphan_path, {
            "schema_version": "2.0",
            "stage": STAGE,
            "identity": dict(identity),
            "status": "attempt_started",
            "lifecycle_phase": "attempt_started",
            "active_identity": dict(identity),
            "resume_forbidden": True,
        })
        orphan = integrity.seal_orphaned_attempt(
            orphan_journal_root, identity, terminal_binding
        )
        _require(
            orphan.get("status") == "terminal_unclean_shutdown"
            and orphan.get("resume_forbidden") is True
            and orphan.get("active_identity") is None,
            "orphan attempt was not sealed",
        )
        _expect_rejection(
            lambda: integrity.AtomicAttemptJournal(
                interrupted_journal_root, identity
            ).__enter__(),
            integrity,
        )
        active_journal_root = fixture_root / "journals_active"
        with integrity.AtomicAttemptJournal(
            active_journal_root, identity
        ) as active_journal:
            _require(
                integrity.AtomicAttemptJournal(
                    active_journal_root, identity
                ).path
                == active_journal.path,
                "canonical journal path is not identity-derived",
            )
            active_journal.attach_terminal_evidence(terminal_binding)
            _expect_rejection(
                lambda: active_journal.capture_startup_profile(
                    b"stale terminal bundle fixture\n"
                ),
                integrity,
            )
            _expect_rejection(
                lambda: integrity.AtomicAttemptJournal(
                    active_journal_root, identity
                ).__enter__(),
                integrity,
            )
        invalid_receipt_payload = yaml.safe_dump(
            invalid_receipt, sort_keys=False
        ).encode("utf-8")
        failure_payloads = dict(payloads)
        failure_payloads["teardown_receipt"] = invalid_receipt_payload
        failure_artifact = fixture_root / "artifacts/teardown_failure"
        failure_resources = {}
        for label, payload in failure_payloads.items():
            suffix = ".yaml" if label in {
                "activation",
                "evaluation",
                "clearance",
                "teardown_receipt",
            } else (".csv" if label == "trace" else ".log")
            path = failure_artifact / (label + suffix)
            _write_bytes(path, payload)
            failure_resources[label] = {
                "status": "produced",
                "path": str(path.relative_to(fixture_root)),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        teardown_failure_binding = (
            integrity.bind_terminal_attempt_evidence(
                fixture_root,
                "artifacts/teardown_failure",
                identity,
                failure_resources,
            )
        )
        teardown_journal_root = fixture_root / "journals_teardown"
        try:
            with integrity.AtomicAttemptJournal(
                teardown_journal_root, identity
            ) as failed_journal:
                teardown_failure_path = failed_journal.path
                failed_startup = failed_journal.capture_startup_profile(
                    startup_profile
                )
                failed_journal.bind_scene_snapshot(snapshot)
                failed_journal.mark_execution_started()
                failed_post = failed_journal.verify_post_episode_scene()
                failed_journal.attach_terminal_evidence(
                    teardown_failure_binding
                )
                integrity.verify_teardown_restore(
                    invalid_receipt,
                    failed_startup,
                    failed_post,
                    identity,
                )
        except integrity.R6TeardownFailure:
            pass
        failed_document = integrity.strict_yaml(teardown_failure_path)
        _require(
            failed_document.get("status") == "terminal_teardown_failure"
            and failed_document["evidence_binding"][
                "terminal_raw_evidence_declared"
            ]
            is True,
            "teardown failure did not produce its dedicated terminal state",
        )

    resource_rows = list(preregistration["resources"].values())
    paths = [row["path"] for row in resource_rows]
    semantic_path = preregistration["resources"]["semantic_reference"]["path"]
    risk_path = preregistration["resources"]["frozen_risk_evidence"]["path"]
    rule_path = preregistration["resources"]["frozen_rule_supervisor"]["path"]
    contract_path = preregistration["resources"]["contract"]["path"]
    edges = []
    for label, row in preregistration["resources"].items():
        if row["path"] == contract_path:
            continue
        kind = "frozen_input"
        if label == "candidate_bank":
            kind = "candidate_specification"
        elif label == "semantic_reference":
            kind = "design_reference"
        elif label == "integrity_protocol":
            kind = "integrity_protocol"
        edges.append({
            "from": contract_path,
            "to": row["path"],
            "kind": kind,
        })
    edges.extend([
        {
            "from": semantic_path,
            "to": risk_path,
            "kind": "python_import",
        },
        {
            "from": semantic_path,
            "to": rule_path,
            "kind": "python_import",
        },
    ])
    manifest = {
        "files": [dict(row) for row in resource_rows],
        "edges": edges,
        "entrypoints": [contract_path],
        "unresolved": [],
    }
    closure = integrity.verify_dependency_closure(
        workspace, manifest, paths
    )
    incomplete = copy.deepcopy(manifest)
    incomplete["files"] = incomplete["files"][:-1]
    _expect_rejection(
        lambda: integrity.verify_dependency_closure(
            workspace, incomplete, paths
        ),
        integrity,
    )
    unreachable = copy.deepcopy(manifest)
    d1_handoff_path = preregistration["resources"]["d1_handoff"]["path"]
    unreachable["edges"] = [
        edge
        for edge in unreachable["edges"]
        if not (
            edge["from"] == contract_path
            and edge["to"] == d1_handoff_path
        )
    ]
    _expect_rejection(
        lambda: integrity.verify_dependency_closure(
            workspace, unreachable, paths
        ),
        integrity,
    )

    fixture_results = {
        EXPECTED_RISK_IDS[0]: {
            "direct_minimum_count": direct_counts["minimum_message_count"],
            "negative_fixture_count": direct_negative_count,
            "pass": True,
        },
        EXPECTED_RISK_IDS[1]: {
            "index_and_children_bound": True,
            "content_addressed_snapshot_created_in_temporary_directory": True,
            "snapshot_tamper_rejected": True,
            "persistent_scene_created": False,
            "pass": True,
        },
        EXPECTED_RISK_IDS[2]: {
            "keyboard_interrupt_status": "terminal_interrupted",
            "orphan_status": "terminal_unclean_shutdown",
            "terminal_not_produced_bundle_verified": True,
            "concurrent_same_identity_within_canonical_root_rejected": True,
            "terminal_phase_lifecycle_binding_verified": True,
            "resume_rejected": True,
            "pass": True,
        },
        EXPECTED_RISK_IDS[3]: {
            "required_raw_resource_count": len(
                binding_document["resources"]
            ),
            "raw_trace_tamper_rejected": True,
            "resource_alias_rejected": True,
            "completion_requires_opaque_validated_binding": True,
            "post_episode_terminal_requires_six_produced_resources": True,
            "pass": True,
        },
        EXPECTED_RISK_IDS[4]: {
            "verified_file_count": closure["file_count"],
            "verified_edge_count": closure["edge_count"],
            "closure_sha256": closure["closure_sha256"],
            "missing_dependency_rejected": True,
            "unreachable_dependency_rejected": True,
            "prospective_only": True,
            "execution_manifest_persisted": False,
            "future_mechanical_generator_required": True,
            "pass": True,
        },
        EXPECTED_RISK_IDS[5]: {
            "two_phase_restore_verified": teardown_token.as_document()[
                "two_phase_restore_verified"
            ],
            "failed_independent_readback_rejected": True,
            "bool_int_identity_coercion_rejected": True,
            "launch_stop_requires_opaque_restore_token": True,
            "launch_stop_checks_identity_and_journal_provenance": True,
            "teardown_failure_terminal_status": (
                "terminal_teardown_failure"
            ),
            "execution_receipt_observed": False,
            "pass": True,
        },
    }
    return {
        "required_count": len(EXPECTED_RISK_IDS),
        "implemented_and_unit_verified_count": len(fixture_results),
        "candidate_symmetric": True,
        "experimental_factor_count_contribution": 0,
        "execution_validation_status": "NOT_RUN_NOT_AUTHORIZED",
        "repairs": [
            {
                "risk_id": risk_id,
                "design_fix_status": (
                    "OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED"
                ),
                "execution_validation_status": "NOT_RUN_NOT_AUTHORIZED",
                "fixture": fixture_results[risk_id],
            }
            for risk_id in EXPECTED_RISK_IDS
        ],
        "pass": True,
    }


def _verify_d1_design_input(workspace):
    report = _load_yaml(
        Path(workspace)
        / "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
        "v2_04g_ttc_d1_report.yaml"
    )
    seed = report.get("seed5111", {})
    semantic = report.get("semantic_comparison", {})
    ttc = report.get("ttc_and_circle_envelope", {})
    _require(
        seed.get("trace_row_count") == 193
        and seed.get("trace_finite_predicted_ttc_sample_count") == 0
        and semantic.get("semantic_difference_confirmed") is True
        and ttc.get("proxy_finite_ttc_sample_count") == 0,
        "D1 frozen semantic input drifted",
    )
    counts = seed.get("offline_truth_proxy_motion_class_counts", {})
    legacy_non_none = int(counts.get("CROSSING", -1)) + 4
    _require(
        counts.get("CROSSING") == 21
        and report["integrated_candidate_distinguishability"][
            "overlay_counts_by_horizon_s"
        ]["5.0"]["OVERTAKE_OR_YIELD"] == 4
        and legacy_non_none == 25,
        "D1 offline legacy proxy counts drifted",
    )
    return {
        "source_stage": "V2-04G-TTC-D1",
        "source_identity": "seed5111_frozen_failure_episode",
        "trace_row_count": 193,
        "legacy_proxy_non_none_count": 25,
        "legacy_proxy_crossing_count": 21,
        "legacy_proxy_overtake_or_yield_count": 4,
        "shared_circle_ttc_finite_count": 0,
        "expected_changed_rows_under_proposed_definition": 25,
        "evidence_units_consumed": 0,
        "seed5111_reconsumed": False,
        "claim_limit": "design_identifiability_only",
        "pass": True,
    }


def _forbidden_r6_artifacts(workspace):
    root = Path(workspace).resolve()
    transition_path = root / R6_I1_TRANSITION_RELATIVE
    transition_valid = False
    if transition_path.is_file() and not transition_path.is_symlink():
        transition = _load_yaml(transition_path)
        transition_valid = (
            transition.get("schema_version") == "2.0"
            and transition.get("source_stage") == STAGE
            and transition.get("target_stage") == "V2-04G-R6-I1"
            and transition.get("integration_stage_separate") is True
            and transition.get("design_stage_mutation_authorized") is False
            and transition.get("execution_authorization_created_by_transition")
            is False
            and transition.get("ros_or_gazebo_started_by_transition") is False
            and transition.get("seed_consumed_by_transition") is False
        )
    allowed = {
        str(CONTRACT_RELATIVE),
        str(PREREGISTRATION_RELATIVE),
        str(CANDIDATE_BANK_RELATIVE),
        str(SEMANTIC_REFERENCE_RELATIVE),
        str(INTEGRITY_PROTOCOL_RELATIVE),
        str(REVIEWER_RELATIVE),
        str(TEST_RELATIVE),
        str(OUTPUT_RELATIVE),
        "AGENTS.md",
        "docs/thesis_experiment/DEVELOPMENT_STATUS.md",
        "docs/thesis_experiment/CURRENT_V2_04G_R6_DESIGN_HANDOFF.md",
    }
    ignored_parts = {
        ".git",
        ".pytest_cache",
        "__pycache__",
        ".venv",
        "venv",
        "node_modules",
        "build",
        "devel",
        "install",
        "log",
    }
    unexpected = []
    structured_stage_paths = []
    for current_root, directories, filenames in os.walk(str(root)):
        current = Path(current_root)
        directories[:] = [
            name
            for name in directories
            if name not in ignored_parts
            and not (current / name / ".git").exists()
        ]
        for filename in filenames:
            path = current / filename
            relative = str(path.relative_to(root))
            later_stage_owned = (
                transition_valid
                and (
                    relative == str(R6_I1_TRANSITION_RELATIVE)
                    or any(
                        relative.startswith(prefix)
                        for prefix in R6_I1_OWNED_PREFIXES
                    )
                )
            )
            if later_stage_owned:
                continue
            normalized = relative.lower().replace("-", "_")
            relevant_name = (
                "v2_04g_r6" in normalized
                or (
                    "r6" in normalized
                    and any(
                        marker in normalized
                        for marker in (
                            "authorization",
                            "seed_schedule",
                            "evidence_budget",
                            "runtime_candidate",
                            "compiled_scene",
                            "scene_manifest",
                            "episode_runner",
                            "batch",
                            "assessment",
                        )
                    )
                )
            )
            relevant_stage = False
            if path.suffix.lower() in {".yaml", ".yml", ".json"}:
                try:
                    text = path.read_text(encoding="utf-8")
                    if "V2-04G-R6" in text:
                        value = yaml.safe_load(text)
                        relevant_stage = (
                            isinstance(value, dict)
                            and str(value.get("stage", "")).startswith(
                                "V2-04G-R6"
                            )
                        )
                except (OSError, UnicodeDecodeError, yaml.YAMLError):
                    if relevant_name:
                        unexpected.append(relative)
                    continue
            elif path.suffix.lower() in {
                ".py",
                ".sh",
                ".xml",
                ".launch",
                ".md",
                ".rst",
                ".txt",
                ".toml",
                ".ini",
                ".cfg",
                ".conf",
                ".csv",
                ".tsv",
            } or relevant_name:
                try:
                    relevant_stage = (
                        "V2-04G-R6" in path.read_text(encoding="utf-8")
                    )
                except (OSError, UnicodeDecodeError):
                    if relevant_name:
                        unexpected.append(relative)
                    continue
            if relevant_stage:
                structured_stage_paths.append(relative)
            if (relevant_name or relevant_stage) and relative not in allowed:
                unexpected.append(relative)
    unexpected = sorted(set(unexpected))
    _require(
        not unexpected,
        "R6 closed artifact allowlist rejected: {}".format(
            ", ".join(unexpected)
        ),
    )
    return {
        "closed_allowlist_unexpected": 0,
        "structured_r6_stage_unexpected": 0,
        "execution_authorization": 0,
        "ros_launch_or_runtime_entrypoint": 0,
        "scene_or_runtime_candidate_materialization": 0,
    }


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    _require(not target.is_symlink(), "R6 report target is a symlink")
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=100
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
        directory = os.open(str(target.parent), os.O_RDONLY)
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


def build_report(workspace, contract_path=None):
    """Build the deterministic design review without persistent writes."""

    root = Path(workspace).resolve()
    contract_path = _canonical_path(
        root,
        CONTRACT_RELATIVE if contract_path is None else contract_path,
        CONTRACT_RELATIVE,
        "R6 contract",
    )
    preregistration_path = _canonical_path(
        root,
        PREREGISTRATION_RELATIVE,
        PREREGISTRATION_RELATIVE,
        "R6 preregistration",
    )
    candidate_bank_path = _canonical_path(
        root,
        CANDIDATE_BANK_RELATIVE,
        CANDIDATE_BANK_RELATIVE,
        "R6 candidate bank",
    )
    reviewer_path = _canonical_path(
        root, REVIEWER_RELATIVE, REVIEWER_RELATIVE, "R6 reviewer"
    )
    test_path = _canonical_path(
        root, TEST_RELATIVE, TEST_RELATIVE, "R6 tests"
    )
    _require(reviewer_path.is_file(), "R6 reviewer is missing")
    _require(test_path.is_file(), "R6 tests are missing")

    r5_before = _tree_snapshot(root, R5_ARTIFACT_RELATIVE)
    d1_before = _tree_snapshot(root, D1_ARTIFACT_RELATIVE)
    _require(
        r5_before["file_count"] == EXPECTED_R5_FILE_COUNT
        and r5_before["tree_sha256"] == EXPECTED_R5_TREE_SHA256,
        "frozen R5 artifact tree drifted",
    )
    _require(
        d1_before["file_count"] == 1
        and d1_before["files"][0]["path"]
        == str(D1_ARTIFACT_RELATIVE / "v2_04g_ttc_d1_report.yaml")
        and d1_before["files"][0]["sha256"] == EXPECTED_D1_REPORT_SHA256,
        "frozen D1 artifact tree drifted",
    )

    contract = _load_yaml(contract_path)
    preregistration = _load_yaml(preregistration_path)
    bank = _load_yaml(candidate_bank_path)
    _verify_contract(contract)
    _verify_preregistration(preregistration)
    _verify_candidate_bank(bank)
    verified_resources = _verify_resources(root, preregistration)
    d1_contract = _load_yaml(
        root / verified_resources["d1_contract"]["path"]
    )
    d1_declared_closure = _verify_d1_declared_closure(root, d1_contract)
    _require(
        all(
            preregistration["resources"].get(label) == row
            for label, row in contract.get("frozen_inputs", {}).items()
        )
        and set(contract.get("frozen_inputs", {})).issubset(
            preregistration["resources"]
        ),
        "contract and preregistration frozen inputs disagree",
    )
    modules = _load_project_modules(root, verified_resources)
    _require(
        tuple(modules["integrity"].RISK_REPAIR_IDS) == EXPECTED_RISK_IDS,
        "loaded integrity module drifted",
    )

    semantic_review = _semantic_review(modules, bank)
    d1_review = _verify_d1_design_input(root)
    integrity_review = _integrity_review(root, modules, preregistration)
    forbidden = _forbidden_r6_artifacts(root)

    r5_after = _tree_snapshot(root, R5_ARTIFACT_RELATIVE)
    d1_after = _tree_snapshot(root, D1_ARTIFACT_RELATIVE)
    _require(r5_before == r5_after, "R6 review modified frozen R5 artifacts")
    _require(d1_before == d1_after, "R6 review modified frozen D1 artifacts")

    report = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": "fam_teb_v2_04g_r6_semantic_alignment_design_review_1",
        "status": "design_preregistration_review_pass_execution_not_authorized",
        "review_result": "pass",
        "design_only": True,
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "single_factor_review": semantic_review,
        "frozen_d1_design_input": d1_review,
        "integrity_repair_review": integrity_review,
        "resource_integrity": {
            "preregistered_resource_count": len(verified_resources),
            "all_preregistered_hashes_match": True,
            "resources": verified_resources,
            "d1_declared_dependency_closure": d1_declared_closure,
            "r5_artifact_tree": _summary(r5_before),
            "r5_expected_file_count": EXPECTED_R5_FILE_COUNT,
            "r5_expected_tree_sha256": EXPECTED_R5_TREE_SHA256,
            "r5_before_and_after_identical": True,
            "d1_artifact_tree": _summary(d1_before),
            "d1_before_and_after_identical": True,
        },
        "implementation": {
            "contract": {
                "path": str(CONTRACT_RELATIVE),
                "sha256": _sha256(contract_path),
            },
            "preregistration": {
                "path": str(PREREGISTRATION_RELATIVE),
                "sha256": _sha256(preregistration_path),
            },
            "candidate_bank": {
                "path": str(CANDIDATE_BANK_RELATIVE),
                "sha256": _sha256(candidate_bank_path),
            },
            "semantic_reference": {
                "path": str(SEMANTIC_REFERENCE_RELATIVE),
                "sha256": _sha256(root / SEMANTIC_REFERENCE_RELATIVE),
            },
            "integrity_protocol": {
                "path": str(INTEGRITY_PROTOCOL_RELATIVE),
                "sha256": _sha256(root / INTEGRITY_PROTOCOL_RELATIVE),
            },
            "reviewer": {
                "path": str(REVIEWER_RELATIVE),
                "sha256": _sha256(reviewer_path),
            },
            "unit_tests": {
                "path": str(TEST_RELATIVE),
                "sha256": _sha256(test_path),
            },
        },
        "execution_boundary": {
            "seed_schedule_present": False,
            "seed_values": [],
            "evidence_budget_authorized": 0,
            "seed_or_evidence_units_consumed": 0,
            "runtime_candidate_configs_persisted": 0,
            "scenes_or_compiled_scenes_created": 0,
            "ros_nodes_or_launch_files_created": 0,
            "episode_runners_or_batches_created": 0,
            "execution_dependency_manifest_persisted": False,
            "execution_authorization_artifact_count": 0,
            "forbidden_artifact_scan_counts": forbidden,
        },
        "authorization_after_review": dict(
            preregistration["authorization_after_review"]
        ),
        "claims": {
            "single_factor_machine_specified": True,
            "semantic_candidate_offline_identifiable_on_frozen_d1_input": True,
            "all_six_integrity_protocols_offline_verified": True,
            "runtime_or_evaluator_parity_execution_proven": False,
            "safety_or_performance_claimed": False,
            "winner_ranked_or_frozen": False,
        },
        "side_effects": {
            "ros_started": False,
            "gazebo_started": False,
            "move_base_started": False,
            "component_executions": 0,
            "navigation_executions": 0,
            "seeds_consumed": 0,
            "evidence_units_consumed": 0,
            "r5_remaining_units_consumed": 0,
            "held_out_seeds_accessed": 0,
            "scene_files_changed": 0,
            "evaluator_files_changed": 0,
            "r5_files_changed": 0,
            "d1_files_changed": 0,
            "training_started": False,
            "real_vehicle_connected": False,
            "real_vehicle_teb_parameter_writes": 0,
            "only_persistent_write_is_canonical_design_review_report": True,
        },
    }
    _require(
        all(value is False for value in report["authorization_after_review"].values())
        and report["execution_boundary"]["execution_authorization_artifact_count"] == 0
        and report["side_effects"]["seeds_consumed"] == 0,
        "R6 report violates the non-execution boundary",
    )
    return report


def review(workspace, contract_path=None, output_path=None):
    """Build and persist only the canonical deterministic design report."""

    root = Path(workspace).resolve()
    output = _canonical_path(
        root,
        OUTPUT_RELATIVE if output_path is None else output_path,
        OUTPUT_RELATIVE,
        "R6 design review output",
    )
    report = build_report(root, contract_path=contract_path)
    _atomic_yaml(output, report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    default_workspace = Path(__file__).resolve().parents[4]
    parser.add_argument(
        "--workspace",
        default=str(default_workspace),
        help="Thesis workspace root",
    )
    parser.add_argument("--contract", help="Canonical R6 design contract")
    parser.add_argument("--output", help="Canonical R6 design review output")
    arguments = parser.parse_args(argv)
    report = review(
        arguments.workspace,
        contract_path=arguments.contract,
        output_path=arguments.output,
    )
    print(
        "R6 design review: {} (execution_authorized={}, seeds_consumed={})".format(
            report["review_result"],
            report["execution_authorized"],
            report["side_effects"]["seeds_consumed"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
