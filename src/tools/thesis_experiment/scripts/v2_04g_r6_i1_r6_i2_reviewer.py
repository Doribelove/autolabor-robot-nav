#!/usr/bin/env python3
"""Pure-offline integration reviewer for the independent R6-I2 repair.

The reviewer performs only single-open file reads, parsing, hashing and
in-memory checks.  It has no ROS or process-launching imports, creates no
authorization and allocates no seed.  Its only optional persistent write is
an atomic replacement of the one canonical machine review report.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml


sys.dont_write_bytecode = True

STAGE = "V2-04G-R6-I2"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml"
)
TRANSITION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i2_stage_transition.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i2_repair_preregistration.yaml"
)
ARTIFACT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review"
)
CLOSURE_RELATIVE = ARTIFACT_ROOT_RELATIVE / "execution_dependency_closure.yaml"
COMPONENT_REVIEW_RELATIVE = (
    ARTIFACT_ROOT_RELATIVE
    / "v2_04g_r6_i2_authorization_assessment_review.yaml"
)
OUTPUT_RELATIVE = (
    ARTIFACT_ROOT_RELATIVE / "v2_04g_r6_i2_integration_review.yaml"
)
MAIN_LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_execution_integration.launch"
)
SPAWN_LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_spawn_m2.launch"
)
R5_ROOT_RELATIVE = Path("artifacts/v2/calibration/v2_04g_r5")

EXPECTED_R5_FILE_COUNT = 68
EXPECTED_R5_TREE_SHA256 = (
    "ecb1f33093dee469008c2ad2d783b3e8ffd1c0739db7903b5df273717e270984"
)
EXPECTED_XACRO_TARGET = "/opt/ros/noetic/lib/xacro/xacro"
EXPECTED_FACTOR_FIELD = "supervisor.dynamic.conflict_estimator_id"
EXPECTED_FACTOR_LEAF = "dynamic.conflict_estimator_id"
LEGACY_ESTIMATOR = "legacy_class_conditioned_geometry_v1"
ALIGNED_ESTIMATOR = "shared_circle_envelope_first_contact_v1"

I1_FROZEN_RESOURCES = {
    "i1_dependency_closure": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "execution_dependency_closure.yaml",
        "3f78ffd2ef1f022b97dcb03957b6472030fa0c86446e25bfb5724bbad19df69d",
    ),
    "i1_stage_report": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_stage_report.yaml",
        "7b1744474278f43d563e1e362ee02e64c9746db30a31bf0dfc26897a8018a50e",
    ),
    "i1_terminal_assessment": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_terminal_assessment.yaml",
        "8a13a9e7c284a21f0537d591b5bb0959a64c9ee9eb1525038fbd8fbc3f3c0e1d",
    ),
    "i1_authorization": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_bounded_simulation_authorization.yaml",
        "3eb157c0ea2ec4a6af2dea86f2756871512f06a7aee2eab24f6a96be03f68db3",
    ),
    "i1_bounded_runner": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_bounded_validation.py",
        "7c4ee80727569ffb5f8a670be1b6a571004f2d7d8b17136f7f40610c6990cd10",
    ),
    "i1_assessor": (
        "src/tools/thesis_experiment/scripts/assess_v2_04g_r6_i1.py",
        "35eee8ebab36e875525a9a08cc252e7043e9ac9ad3a9228651680602fe326d50",
    ),
}

EXPECTED_RUNTIME_BINDINGS = (
    "$(find gazebo_ros)/launch/empty_world.launch",
    "node:gazebo_ros:spawn_model",
    "node:move_base:move_base",
    "node:robot_state_publisher:robot_state_publisher",
    "package-executable:xacro:xacro",
)
EXPECTED_REPAIR_IDS = (
    "R6-I2-BOOTSTRAP-CLOCK",
    "R6-I2-EXTERNAL-HASH-CLOSURE",
    "R6-I2-AUTHORIZATION-CLOSED-SCHEMA",
    "R6-I2-SINGLE-OPEN-HASH-PARSE",
    "R6-I2-DETERMINISTIC-ASSESSOR",
    "R6-I2-CREDENTIAL-SAFE-LOGGING",
)
EXPECTED_REQUIRED_ORDER = (
    "base_spawn",
    "unpause_request",
    "successful_unpause_ack",
    "first_strictly_positive_post_ack_clock",
    "second_strictly_greater_positive_post_ack_clock",
    "release_move_base_and_teb_service_wait",
)
EXPECTED_RESOURCE_PATHS = {
    "r6_design_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_semantic_alignment_design_contract.yaml"
    ),
    "r6_design_preregistration": (
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_alignment_preregistration.yaml"
    ),
    "r6_design_candidate_bank": (
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_candidates.yaml"
    ),
    "r6_semantic_reference": (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "r6_relative_ttc_supervisor.py"
    ),
    "r6_design_integrity": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_integrity.py"
    ),
    "frozen_evaluator": (
        "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py"
    ),
    "i1_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_execution_integration_contract.yaml"
    ),
    "i1_dependency_closure": I1_FROZEN_RESOURCES[
        "i1_dependency_closure"
    ][0],
    "i1_integration_review": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_integration_review.yaml"
    ),
    "i1_authorization": I1_FROZEN_RESOURCES["i1_authorization"][0],
    "i1_stage_report": I1_FROZEN_RESOURCES["i1_stage_report"][0],
    "i1_terminal_assessment": I1_FROZEN_RESOURCES[
        "i1_terminal_assessment"
    ][0],
    "i1_bounded_runner": I1_FROZEN_RESOURCES["i1_bounded_runner"][0],
    "i1_assessor": I1_FROZEN_RESOURCES["i1_assessor"][0],
    "legacy_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "runtime_candidate_configs/r6_semantics_legacy_control/"
        "supervisor.yaml"
    ),
    "aligned_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "runtime_candidate_configs/r6_semantics_circle_contact/"
        "supervisor.yaml"
    ),
    "compiled_scene_index": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "compiled_scenes/compiled_scene_index.yaml"
    ),
    "i2_stage_transition": TRANSITION_RELATIVE.as_posix(),
    "i2_repair_preregistration": PREREGISTRATION_RELATIVE.as_posix(),
    "i2_bootstrap_module": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_bootstrap.py"
    ),
    "i2_authorization_module": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_authorization.py"
    ),
    "i2_dependency_module": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_dependency.py"
    ),
    "i2_assessor": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_assessor.py"
    ),
    "i2_dependency_generator": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_dependency_generator.py"
    ),
    "i2_repair_harness": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_repair_harness.py"
    ),
    "i2_reviewer": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_reviewer.py"
    ),
    "i2_main_launch": MAIN_LAUNCH_RELATIVE.as_posix(),
    "i2_spawn_launch": SPAWN_LAUNCH_RELATIVE.as_posix(),
    "i2_authorization_assessment_component_review": (
        COMPONENT_REVIEW_RELATIVE.as_posix()
    ),
    "i2_bootstrap_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i2_bootstrap.py"
    ),
    "i2_authorization_assessment_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i2_authorization_assessment.py"
    ),
    "i2_dependency_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i2_dependency.py"
    ),
    "i2_harness_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i2_repair_harness.py"
    ),
    "i2_reviewer_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i2_review.py"
    ),
}


class R6I2ReviewError(ValueError):
    """Raised when any repair-review invariant fails closed."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """YAML loader rejecting duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise R6I2ReviewError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require(condition, message):
    if not condition:
        raise R6I2ReviewError(message)


def _exact(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return (
            set(actual) == set(expected)
            and all(_exact(actual[key], expected[key]) for key in expected)
        )
    if isinstance(expected, list):
        return (
            len(actual) == len(expected)
            and all(_exact(left, right) for left, right in zip(actual, expected))
        )
    return actual == expected


def _require_exact(actual, expected, label):
    _require(_exact(actual, expected), "{} drifted".format(label))


def _closed(value, keys, label):
    _require(isinstance(value, dict), "{} must be a mapping".format(label))
    _require(
        set(value) == set(keys),
        "{} keys drifted; missing={} extra={}".format(
            label, sorted(set(keys) - set(value)), sorted(set(value) - set(keys))
        ),
    )
    return value


def _hex_digest(value, label):
    _require(
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value),
        "{} must be a lowercase SHA256".format(label),
    )
    return value


def _declared_parts(relative):
    _require(isinstance(relative, (str, Path)), "path must be text")
    candidate = Path(relative)
    _require(not candidate.is_absolute(), "path must be relative")
    _require(
        candidate.parts
        and all(part not in {"", ".", ".."} for part in candidate.parts),
        "path contains unsafe components",
    )
    return candidate.parts


def _read_bytes_once(workspace, relative):
    """Read one regular workspace file through a no-follow descriptor chain."""

    root = Path(workspace)
    _require(root.is_absolute() and root == root.resolve(), "unsafe workspace")
    parts = _declared_parts(relative)
    read_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = (
        read_flags
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors = []
    try:
        current = os.open(str(root), directory_flags)
        descriptors.append(current)
        for component in parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        descriptor = os.open(
            parts[-1],
            read_flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=current,
        )
        descriptors.append(descriptor)
        before = os.fstat(descriptor)
        _require(stat.S_ISREG(before.st_mode), "resource is not regular")
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        identity = lambda row: (
            row.st_dev,
            row.st_ino,
            row.st_mode,
            row.st_size,
            row.st_mtime_ns,
            row.st_ctime_ns,
        )
        payload = b"".join(chunks)
        _require(
            identity(before) == identity(after) and len(payload) == before.st_size,
            "resource changed during single-open read",
        )
        return payload
    except OSError as exc:
        raise R6I2ReviewError(
            "cannot safely read {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _snapshot(workspace, relative, yaml_document=False):
    payload = _read_bytes_once(workspace, relative)
    result = {
        "path": Path(relative).as_posix(),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
        "payload": payload,
    }
    if yaml_document:
        try:
            document = yaml.load(
                payload.decode("utf-8"), Loader=_UniqueKeyLoader
            )
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise R6I2ReviewError(
                "cannot strictly parse {}: {}".format(relative, exc)
            ) from exc
        _require(
            isinstance(document, dict),
            "{} must contain a mapping".format(relative),
        )
        result["document"] = document
    return result


def _verify_contract(document):
    _closed(
        document,
        {
            "schema_version",
            "architecture_generation",
            "stage",
            "contract_id",
            "status",
            "contract_date",
            "scope",
            "simulation_only",
            "formal_result",
            "runtime_ready",
            "execution_ready",
            "execution_authorized",
            "training_allowed",
            "real_vehicle_use_forbidden",
            "objective",
            "independent_stage_boundary",
            "frozen_experimental_boundary",
            "repair_scope",
            "future_bootstrap_adapter_contract",
            "dependency_closure_contract",
            "authorization_validation_contract",
            "static_integration_review_gates",
            "forbidden_actions",
            "resources",
            "review_output",
        },
        "contract",
    )
    _require_exact(
        {
            key: document[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "contract_id",
                "status",
                "contract_date",
                "scope",
                "simulation_only",
                "formal_result",
                "runtime_ready",
                "execution_ready",
                "execution_authorized",
                "training_allowed",
                "real_vehicle_use_forbidden",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "contract_id": (
                "fam_teb_v2_04g_r6_i2_bootstrap_integrity_repair_review_1"
            ),
            "status": (
                "repair_integration_review_contract_execution_not_authorized"
            ),
            "contract_date": "2026-07-19",
            "scope": (
                "offline_design_implementation_test_and_static_"
                "integration_review_only"
            ),
            "simulation_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "execution_ready": False,
            "execution_authorized": False,
            "training_allowed": False,
            "real_vehicle_use_forbidden": True,
        },
        "contract boundary",
    )
    _closed(
        document["independent_stage_boundary"],
        {
            "source_stage",
            "source_status",
            "source_identity_reused",
            "source_seed_reused",
            "source_budget_reused",
            "retry_or_resume_i1_allowed",
            "i1_forfeited_units_reusable",
            "current_stage_authorization_manifest_allowed",
            "current_stage_execution_artifact_allowed",
            "current_stage_seed_values",
            "current_stage_execution_schedule",
            "current_stage_evidence_budget_authorized",
            "current_stage_evidence_budget_consumed",
            "storage_namespace_note",
        },
        "independent stage boundary",
    )
    boundary = document["independent_stage_boundary"]
    _require_exact(
        {
            "source_stage": boundary["source_stage"],
            "source_status": boundary["source_status"],
            "source_identity_reused": boundary["source_identity_reused"],
            "source_seed_reused": boundary["source_seed_reused"],
            "source_budget_reused": boundary["source_budget_reused"],
            "retry_or_resume_i1_allowed": boundary[
                "retry_or_resume_i1_allowed"
            ],
            "i1_forfeited_units_reusable": boundary[
                "i1_forfeited_units_reusable"
            ],
            "current_stage_authorization_manifest_allowed": boundary[
                "current_stage_authorization_manifest_allowed"
            ],
            "current_stage_execution_artifact_allowed": boundary[
                "current_stage_execution_artifact_allowed"
            ],
            "current_stage_seed_values": boundary["current_stage_seed_values"],
            "current_stage_execution_schedule": boundary[
                "current_stage_execution_schedule"
            ],
            "current_stage_evidence_budget_authorized": boundary[
                "current_stage_evidence_budget_authorized"
            ],
            "current_stage_evidence_budget_consumed": boundary[
                "current_stage_evidence_budget_consumed"
            ],
        },
        {
            "source_stage": "V2-04G-R6-I1",
            "source_status": "terminal_failure",
            "source_identity_reused": False,
            "source_seed_reused": False,
            "source_budget_reused": False,
            "retry_or_resume_i1_allowed": False,
            "i1_forfeited_units_reusable": False,
            "current_stage_authorization_manifest_allowed": False,
            "current_stage_execution_artifact_allowed": False,
            "current_stage_seed_values": [],
            "current_stage_execution_schedule": [],
            "current_stage_evidence_budget_authorized": 0,
            "current_stage_evidence_budget_consumed": 0,
        },
        "contract zero-execution boundary",
    )
    frozen = _closed(
        document["frozen_experimental_boundary"],
        {
            "factor_count",
            "runtime_field",
            "control_value",
            "aligned_value",
            "only_runtime_leaf_allowed_to_differ",
            "thresholds",
            "factor_change_allowed",
            "threshold_change_allowed",
            "scene_change_allowed",
            "evaluator_change_allowed",
            "anchor_or_mechanism_change_allowed",
            "runtime_profile_write_allowed",
        },
        "frozen experimental boundary",
    )
    _require_exact(
        {
            key: frozen[key]
            for key in frozen
            if key != "thresholds"
        },
        {
            "factor_count": 1,
            "runtime_field": EXPECTED_FACTOR_FIELD,
            "control_value": LEGACY_ESTIMATOR,
            "aligned_value": ALIGNED_ESTIMATOR,
            "only_runtime_leaf_allowed_to_differ": EXPECTED_FACTOR_LEAF,
            "factor_change_allowed": False,
            "threshold_change_allowed": False,
            "scene_change_allowed": False,
            "evaluator_change_allowed": False,
            "anchor_or_mechanism_change_allowed": False,
            "runtime_profile_write_allowed": False,
        },
        "frozen factor boundary",
    )
    _require_exact(
        frozen["thresholds"],
        {
            "runtime_and_evaluator_horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "robot_circle_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "legacy_closest_approach_threshold_m": 1.35,
            "overlay_release_confirmation_s": 0.20,
            "world_model_classification_horizon_s": 2.0,
        },
        "frozen thresholds",
    )
    repair_scope = _closed(
        document["repair_scope"],
        {"exact_repair_count", "repairs"},
        "repair scope",
    )
    _require(
        type(repair_scope["exact_repair_count"]) is int
        and repair_scope["exact_repair_count"] == 6,
        "repair count drifted",
    )
    _require(
        tuple(repair_scope["repairs"]) == EXPECTED_REPAIR_IDS,
        "repair IDs or order drifted",
    )
    repair_schemas = {
        "R6-I2-BOOTSTRAP-CLOCK": {
            "source_finding",
            "required_order",
            "fail_closed_on",
        },
        "R6-I2-EXTERNAL-HASH-CLOSURE": {
            "source_finding",
            "inherited_python_binding_count",
            "runtime_binding_count",
            "canonical_path_and_sha256_required",
            "interpreter_or_builtin_provider_required",
            "unresolved_count_required",
        },
        "R6-I2-AUTHORIZATION-CLOSED-SCHEMA": {
            "source_finding",
            "closed_type_sensitive_schema_required",
            "exact_schedule_required",
            "every_resource_required",
            "independent_closure_digest_required",
            "every_scope_and_safety_flag_required",
        },
        "R6-I2-SINGLE-OPEN-HASH-PARSE": {
            "source_finding",
            "every_path_component_no_follow_required",
            "one_regular_file_descriptor_required",
            "one_byte_snapshot_for_hash_and_parse_required",
            "duplicate_yaml_keys_rejected",
            "file_identity_stability_required",
        },
        "R6-I2-DETERMINISTIC-ASSESSOR": {
            "source_finding",
            "all_source_values_explicit_parameters",
            "free_variable_stage_reference_allowed",
            "write_or_execution_side_effect_allowed",
            "repeated_build_must_match",
        },
        "R6-I2-CREDENTIAL-SAFE-LOGGING": {
            "source_finding",
            "exact_child_environment_allowlist_required",
            "loopback_ros_master_required",
            "attempt_confined_ros_home_and_log_dir_required",
            "credential_like_keys_and_values_persisted",
            "argv_credential_rejection_required",
            "pre_persistence_log_redaction_required",
        },
    }
    for repair_id, value in repair_scope["repairs"].items():
        _closed(value, repair_schemas[repair_id], repair_id)
    _require_exact(
        repair_scope["repairs"]["R6-I2-BOOTSTRAP-CLOCK"][
            "required_order"
        ],
        list(EXPECTED_REQUIRED_ORDER),
        "bootstrap required order",
    )
    _require_exact(
        {
            "inherited_python_binding_count": repair_scope["repairs"][
                "R6-I2-EXTERNAL-HASH-CLOSURE"
            ]["inherited_python_binding_count"],
            "runtime_binding_count": repair_scope["repairs"][
                "R6-I2-EXTERNAL-HASH-CLOSURE"
            ]["runtime_binding_count"],
            "unresolved_count_required": repair_scope["repairs"][
                "R6-I2-EXTERNAL-HASH-CLOSURE"
            ]["unresolved_count_required"],
        },
        {
            "inherited_python_binding_count": 39,
            "runtime_binding_count": 5,
            "unresolved_count_required": 0,
        },
        "external binding counts",
    )
    _require_exact(
        document["future_bootstrap_adapter_contract"],
        {
            "current_review_may_call_adapter": False,
            "current_review_may_import_ros": False,
            "current_review_may_spawn_subprocess": False,
            "current_review_may_connect_ros_master": False,
            "injected_observation_callbacks_only": True,
            "service_wait_callback_before_clock_release_allowed": False,
            "xacro_binding_id": "package-executable:xacro:xacro",
            "xacro_path_must_equal_dependency_closure_target": True,
        },
        "future bootstrap adapter boundary",
    )
    closure_contract = _closed(
        document["dependency_closure_contract"],
        {
            "inherited_external_python_names_source",
            "exact_runtime_binding_names",
            "external_file_records_are_absolute_canonical",
            "external_file_records_have_sha256_and_size",
            "external_file_rehash_required",
            "local_dependency_reproduction_required",
            "authorization_resource_allowed_in_closure",
            "unresolved_required",
        },
        "dependency closure contract",
    )
    _require_exact(
        closure_contract["inherited_external_python_names_source"],
        {
            "path": I1_FROZEN_RESOURCES["i1_dependency_closure"][0],
            "expected_count": 39,
        },
        "inherited Python name source",
    )
    _require_exact(
        closure_contract["exact_runtime_binding_names"],
        list(EXPECTED_RUNTIME_BINDINGS),
        "runtime binding names",
    )
    _require_exact(
        {
            key: closure_contract[key]
            for key in closure_contract
            if key
            not in {
                "inherited_external_python_names_source",
                "exact_runtime_binding_names",
            }
        },
        {
            "external_file_records_are_absolute_canonical": True,
            "external_file_records_have_sha256_and_size": True,
            "external_file_rehash_required": True,
            "local_dependency_reproduction_required": True,
            "authorization_resource_allowed_in_closure": False,
            "unresolved_required": [],
        },
        "dependency closure safety boundary",
    )
    _require_exact(
        document["authorization_validation_contract"],
        {
            "current_authorization_is_absent": True,
            "current_validation_is_synthetic_and_offline_only": True,
            "future_authorization_must_be_separate": True,
            "future_authorization_requires_explicit_user_instruction": True,
            "future_seed_and_budget_must_be_fresh": True,
            "future_identity_must_not_reuse_r6_i1": True,
            "validator_must_run_before_journal_or_subprocess": True,
        },
        "authorization validation boundary",
    )
    gates = _closed(
        document["static_integration_review_gates"],
        {
            "strict_yaml_and_duplicate_key_rejection",
            "contract_resource_hashes_match",
            "i1_frozen_hashes_match",
            "r5_artifact_file_count",
            "r5_artifact_tree_sha256",
            "semantic_profiles_have_one_exact_leaf_difference",
            "threshold_values_match",
            "compiled_scene_index_matches",
            "evaluator_matches",
            "launch_xml_parse_only",
            "launch_process_start_allowed",
            "dependency_closure_reproduces",
            "all_repair_tests_pass",
            "authorization_file_absent",
            "execution_artifact_root_absent",
            "seed_or_evidence_consumption_required",
        },
        "static review gates",
    )
    _require(
        all(
            value is True
            for key, value in gates.items()
            if key
            not in {
                "r5_artifact_file_count",
                "r5_artifact_tree_sha256",
                "launch_process_start_allowed",
                "seed_or_evidence_consumption_required",
            }
        ),
        "a required static review gate is disabled",
    )
    _require_exact(
        {
            "r5_artifact_file_count": gates["r5_artifact_file_count"],
            "r5_artifact_tree_sha256": gates["r5_artifact_tree_sha256"],
            "launch_process_start_allowed": gates[
                "launch_process_start_allowed"
            ],
            "seed_or_evidence_consumption_required": gates[
                "seed_or_evidence_consumption_required"
            ],
        },
        {
            "r5_artifact_file_count": EXPECTED_R5_FILE_COUNT,
            "r5_artifact_tree_sha256": EXPECTED_R5_TREE_SHA256,
            "launch_process_start_allowed": False,
            "seed_or_evidence_consumption_required": 0,
        },
        "static review fixed gates",
    )
    resources = document["resources"]
    _require(
        isinstance(resources, dict)
        and set(resources) == set(EXPECTED_RESOURCE_PATHS),
        "contract resource labels drifted",
    )
    for label, expected_path in EXPECTED_RESOURCE_PATHS.items():
        _closed(resources[label], {"path", "sha256"}, label)
        _require_exact(resources[label]["path"], expected_path, label + " path")
        _hex_digest(resources[label]["sha256"], label + " SHA256")
    _require_exact(
        document["review_output"],
        {
            "path": OUTPUT_RELATIVE.as_posix(),
            "deterministic": True,
            "atomic_write": True,
            "execution_authorization_created": False,
        },
        "review output contract",
    )
    _require_exact(
        document["forbidden_actions"],
        [
            "create_r6_i2_execution_authorization",
            "allocate_or_consume_seed",
            "start_ros_gazebo_move_base_or_teb",
            "retry_or_resume_r6_i1",
            "reuse_r6_i1_forfeited_units",
            "retry_or_resume_r5",
            "consume_r5_remaining_68_units",
            "access_held_out_5001_5010",
            "change_semantic_factor_threshold_scene_or_evaluator",
            "freeze_or_rank_winner",
            "start_v2_05_sac_or_training",
            "connect_real_vehicle",
            "write_real_vehicle_teb_parameters",
        ],
        "forbidden action boundary",
    )
    return document


def _verify_transition(document):
    _closed(
        document,
        {
            "schema_version",
            "architecture_generation",
            "transition_id",
            "source_stage",
            "source_status",
            "target_stage",
            "target_scope",
            "transition_date",
            "authorization_source",
            "independent_stage",
            "source_identity_reused",
            "source_budget_reused",
            "source_seed_reused",
            "execution_authorized",
            "ros_or_gazebo_start_authorized",
            "seed_allocation_authorized",
            "seed_values",
            "evidence_budget_authorized",
            "training_authorized",
            "real_vehicle_authorized",
            "factor_or_threshold_change_authorized",
            "scene_or_evaluator_change_authorized",
            "required_repairs",
            "source_bindings",
            "terminal_firewall",
        },
        "stage transition",
    )
    _require_exact(
        {
            key: document[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "source_stage",
                "source_status",
                "target_stage",
                "independent_stage",
                "source_identity_reused",
                "source_budget_reused",
                "source_seed_reused",
                "execution_authorized",
                "ros_or_gazebo_start_authorized",
                "seed_allocation_authorized",
                "seed_values",
                "evidence_budget_authorized",
                "training_authorized",
                "real_vehicle_authorized",
                "factor_or_threshold_change_authorized",
                "scene_or_evaluator_change_authorized",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "source_stage": "V2-04G-R6-I1",
            "source_status": "terminal_failure",
            "target_stage": STAGE,
            "independent_stage": True,
            "source_identity_reused": False,
            "source_budget_reused": False,
            "source_seed_reused": False,
            "execution_authorized": False,
            "ros_or_gazebo_start_authorized": False,
            "seed_allocation_authorized": False,
            "seed_values": [],
            "evidence_budget_authorized": 0,
            "training_authorized": False,
            "real_vehicle_authorized": False,
            "factor_or_threshold_change_authorized": False,
            "scene_or_evaluator_change_authorized": False,
        },
        "stage transition boundary",
    )
    _require(
        tuple(document["required_repairs"])
        == (
            "positive_clock_before_move_base_readiness",
            "canonical_path_and_sha_external_dependency_closure",
            "closed_authorization_schema_and_exact_schedule_enforcement",
            "single_open_no_follow_hash_and_parse",
            "deterministic_offline_assessor",
            "credential_safe_child_environment_and_log_redaction",
        ),
        "transition repair list drifted",
    )
    bindings = document["source_bindings"]
    _require(
        set(bindings)
        == {"i1_stage_report", "i1_terminal_assessment", "i1_authorization"},
        "transition I1 bindings drifted",
    )
    for label, row in bindings.items():
        _closed(row, {"path", "sha256"}, label)
        expected_path, expected_sha = I1_FROZEN_RESOURCES[label]
        _require_exact(row, {"path": expected_path, "sha256": expected_sha}, label)
    firewall = document["terminal_firewall"]
    _require_exact(
        firewall,
        {
            "retry_or_resume_i1_allowed": False,
            "i1_forfeited_units_reusable": False,
            "prior_identity_reuse_allowed": False,
            "held_out_5001_5010_accessed": False,
            "r5_remaining_units_consumed": 0,
        },
        "transition terminal firewall",
    )
    return document


def _verify_preregistration(document):
    _closed(
        document,
        {
            "schema_version",
            "architecture_generation",
            "stage",
            "preregistration_id",
            "status",
            "preregistration_date",
            "scope",
            "execution_authorized",
            "ros_or_gazebo_start_authorized",
            "seed_allocation_authorized",
            "seed_values",
            "execution_schedule",
            "evidence_budget_authorized",
            "retry_or_resume_i1_authorized",
            "i1_forfeited_units_reused",
            "prior_identity_reuse_allowed",
            "training_authorized",
            "real_vehicle_authorized",
            "formal_result",
            "runtime_ready",
            "frozen_experimental_boundary",
            "repair_items",
            "integration_review_gates",
            "future_execution_boundary",
        },
        "repair preregistration",
    )
    _require_exact(
        {
            key: document[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "status",
                "scope",
                "execution_authorized",
                "ros_or_gazebo_start_authorized",
                "seed_allocation_authorized",
                "seed_values",
                "execution_schedule",
                "evidence_budget_authorized",
                "retry_or_resume_i1_authorized",
                "i1_forfeited_units_reused",
                "prior_identity_reuse_allowed",
                "training_authorized",
                "real_vehicle_authorized",
                "formal_result",
                "runtime_ready",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "status": "repair_review_preregistered_execution_not_authorized",
            "scope": "offline_and_static_integration_repair_review_only",
            "execution_authorized": False,
            "ros_or_gazebo_start_authorized": False,
            "seed_allocation_authorized": False,
            "seed_values": [],
            "execution_schedule": [],
            "evidence_budget_authorized": 0,
            "retry_or_resume_i1_authorized": False,
            "i1_forfeited_units_reused": False,
            "prior_identity_reuse_allowed": False,
            "training_authorized": False,
            "real_vehicle_authorized": False,
            "formal_result": False,
            "runtime_ready": False,
        },
        "repair preregistration boundary",
    )
    frozen = document["frozen_experimental_boundary"]
    _closed(
        frozen,
        {
            "only_future_factor_field",
            "factor_levels",
            "thresholds",
            "factor_change_in_i2_allowed",
            "threshold_change_in_i2_allowed",
            "scene_change_in_i2_allowed",
            "evaluator_change_in_i2_allowed",
            "anchor_or_mechanism_change_in_i2_allowed",
            "runtime_profile_write_in_i2_allowed",
            "frozen_bindings",
        },
        "preregistered frozen boundary",
    )
    _require_exact(
        frozen["only_future_factor_field"],
        EXPECTED_FACTOR_FIELD,
        "factor field",
    )
    _require_exact(
        frozen["factor_levels"],
        {
            "r6_semantics_legacy_control": LEGACY_ESTIMATOR,
            "r6_semantics_circle_contact": ALIGNED_ESTIMATOR,
        },
        "factor levels",
    )
    _require_exact(
        frozen["thresholds"],
        {
            "ttc_horizon_s": 5.0,
            "minimum_track_confidence": 0.45,
            "robot_circle_radius_m": 0.62,
            "minimum_relative_speed_mps": 0.05,
            "legacy_closest_approach_threshold_m": 1.35,
            "overlay_release_confirmation_s": 0.20,
            "world_model_classification_horizon_s": 2.0,
        },
        "preregistered thresholds",
    )
    _require(
        all(
            frozen[key] is False
            for key in (
                "factor_change_in_i2_allowed",
                "threshold_change_in_i2_allowed",
                "scene_change_in_i2_allowed",
                "evaluator_change_in_i2_allowed",
                "anchor_or_mechanism_change_in_i2_allowed",
                "runtime_profile_write_in_i2_allowed",
            )
        ),
        "a frozen preregistration gate is permissive",
    )
    bindings = frozen["frozen_bindings"]
    _require(
        set(bindings)
        == {
            "legacy_supervisor",
            "aligned_supervisor",
            "compiled_scene_index",
            "evaluator",
        },
        "frozen binding labels drifted",
    )
    for label, row in bindings.items():
        _closed(row, {"path", "sha256"}, "frozen " + label)
        _hex_digest(row["sha256"], label)
    repairs = document["repair_items"]
    _require(
        isinstance(repairs, list)
        and [row.get("repair_id") for row in repairs]
        == list(EXPECTED_REPAIR_IDS),
        "preregistered repair items drifted",
    )
    for row in repairs:
        _closed(
            row,
            {"repair_id", "source_finding", "implementation_requirement", "verification"},
            "repair item",
        )
        _require(
            isinstance(row["implementation_requirement"], list)
            and row["implementation_requirement"],
            "repair implementation requirement is empty",
        )
    gates = document["integration_review_gates"]
    _closed(
        gates,
        {
            "exact_repair_item_count",
            "pure_python_or_static_tests_only",
            "launch_xml_parse_only",
            "launch_process_start_allowed",
            "authorization_file_must_be_absent",
            "execution_artifact_root_must_be_absent",
            "seed_or_evidence_consumption_must_equal",
            "semantic_boundary_hashes_must_match",
            "i1_frozen_hashes_must_match",
            "external_binding_unresolved_count_must_equal",
            "all_repair_tests_must_pass",
        },
        "preregistered review gates",
    )
    _require_exact(
        gates,
        {
            "exact_repair_item_count": 6,
            "pure_python_or_static_tests_only": True,
            "launch_xml_parse_only": True,
            "launch_process_start_allowed": False,
            "authorization_file_must_be_absent": True,
            "execution_artifact_root_must_be_absent": True,
            "seed_or_evidence_consumption_must_equal": 0,
            "semantic_boundary_hashes_must_match": True,
            "i1_frozen_hashes_must_match": True,
            "external_binding_unresolved_count_must_equal": 0,
            "all_repair_tests_must_pass": True,
        },
        "preregistered review gates",
    )
    future = document["future_execution_boundary"]
    _closed(
        future,
        {
            "current_review_may_create_authorization",
            "current_review_may_allocate_seed_or_budget",
            "current_review_may_start_ros_or_gazebo",
            "future_stage_must_be_independent",
            "future_seed_and_budget_must_be_fresh",
            "future_authorization_requires_new_explicit_user_instruction",
            "i1_retry_or_resume_remains_forbidden",
        },
        "future execution boundary",
    )
    _require(
        future["current_review_may_create_authorization"] is False
        and future["current_review_may_allocate_seed_or_budget"] is False
        and future["current_review_may_start_ros_or_gazebo"] is False
        and all(
            future[key] is True
            for key in (
                "future_stage_must_be_independent",
                "future_seed_and_budget_must_be_fresh",
                "future_authorization_requires_new_explicit_user_instruction",
                "i1_retry_or_resume_remains_forbidden",
            )
        ),
        "future execution boundary drifted",
    )
    return document


def _verify_resources(workspace, resources):
    records = {}
    for label in sorted(EXPECTED_RESOURCE_PATHS):
        row = resources[label]
        snapshot = _snapshot(workspace, row["path"])
        _require(
            snapshot["sha256"] == row["sha256"],
            "{} resource hash mismatched".format(label),
        )
        records[label] = {
            "path": row["path"],
            "sha256": row["sha256"],
            "size_bytes": snapshot["size_bytes"],
        }
    return records


def _verify_i1_frozen(workspace):
    result = {}
    for label, (path, expected_sha) in I1_FROZEN_RESOURCES.items():
        snapshot = _snapshot(workspace, path)
        _require(
            snapshot["sha256"] == expected_sha,
            "{} frozen hash mismatched".format(label),
        )
        result[label] = {"path": path, "sha256": expected_sha}
    return result


def _tree_snapshot(workspace, relative_root):
    root = Path(workspace) / relative_root
    _require(root.is_dir() and not root.is_symlink(), "R5 tree is missing")
    records = []
    for path in sorted(root.rglob("*")):
        _require(not path.is_symlink(), "R5 tree contains a symlink")
        if path.is_file():
            relative = path.relative_to(workspace).as_posix()
            snapshot = _snapshot(workspace, relative)
            records.append({"path": relative, "sha256": snapshot["sha256"]})
    canonical = "".join(
        "{} {}\n".format(row["path"], row["sha256"]) for row in records
    ).encode("utf-8")
    result = {
        "file_count": len(records),
        "tree_sha256": hashlib.sha256(canonical).hexdigest(),
    }
    _require_exact(
        result,
        {
            "file_count": EXPECTED_R5_FILE_COUNT,
            "tree_sha256": EXPECTED_R5_TREE_SHA256,
        },
        "R5 artifact tree",
    )
    return result


def _leaf_differences(left, right, prefix=""):
    if isinstance(left, dict) and isinstance(right, dict):
        differences = []
        for key in sorted(set(left) | set(right)):
            path = "{}.{}".format(prefix, key) if prefix else key
            if key not in left or key not in right:
                differences.append(path)
            else:
                differences.extend(
                    _leaf_differences(left[key], right[key], path)
                )
        return differences
    if not _exact(left, right):
        return [prefix]
    return []


def _verify_semantic_boundary(workspace, preregistration, contract):
    bindings = preregistration["frozen_experimental_boundary"][
        "frozen_bindings"
    ]
    documents = {}
    for label in ("legacy_supervisor", "aligned_supervisor"):
        row = bindings[label]
        snapshot = _snapshot(workspace, row["path"], yaml_document=True)
        _require(
            snapshot["sha256"] == row["sha256"],
            label + " preregistration hash mismatched",
        )
        documents[label] = snapshot["document"]
    differences = _leaf_differences(
        documents["legacy_supervisor"], documents["aligned_supervisor"]
    )
    _require_exact(differences, [EXPECTED_FACTOR_LEAF], "semantic leaf difference")
    legacy = documents["legacy_supervisor"]
    aligned = documents["aligned_supervisor"]
    _require_exact(
        legacy["dynamic"]["conflict_estimator_id"],
        LEGACY_ESTIMATOR,
        "legacy estimator",
    )
    _require_exact(
        aligned["dynamic"]["conflict_estimator_id"],
        ALIGNED_ESTIMATOR,
        "aligned estimator",
    )
    expected_profile_thresholds = {
        "dynamic.predicted_ttc_max_s": 5.0,
        "dynamic.minimum_track_confidence": 0.45,
        "dynamic.robot_radius_m": 0.62,
        "dynamic.minimum_relative_speed_mps": 0.05,
        "dynamic.closest_approach_max_m": 1.35,
        "transition.overlay_release_confirmation_s": 0.20,
    }
    actual = {
        "dynamic.predicted_ttc_max_s": legacy["dynamic"][
            "predicted_ttc_max_s"
        ],
        "dynamic.minimum_track_confidence": legacy["dynamic"][
            "minimum_track_confidence"
        ],
        "dynamic.robot_radius_m": legacy["dynamic"]["robot_radius_m"],
        "dynamic.minimum_relative_speed_mps": legacy["dynamic"][
            "minimum_relative_speed_mps"
        ],
        "dynamic.closest_approach_max_m": legacy["dynamic"][
            "closest_approach_max_m"
        ],
        "transition.overlay_release_confirmation_s": legacy["transition"][
            "overlay_release_confirmation_s"
        ],
    }
    _require_exact(actual, expected_profile_thresholds, "runtime thresholds")
    i1_contract_row = contract["resources"]["i1_contract"]
    i1_contract = _snapshot(
        workspace, i1_contract_row["path"], yaml_document=True
    )["document"]
    _require_exact(
        i1_contract["frozen_values"]["world_model_prediction_horizon_s"],
        2.0,
        "world-model classification horizon",
    )
    for label in ("compiled_scene_index", "evaluator"):
        row = bindings[label]
        snapshot = _snapshot(workspace, row["path"])
        _require(
            snapshot["sha256"] == row["sha256"],
            label + " frozen hash mismatched",
        )
    _require(
        bindings["compiled_scene_index"]
        == contract["resources"]["compiled_scene_index"],
        "compiled scene binding differs across contract and preregistration",
    )
    _require(
        bindings["evaluator"] == contract["resources"]["frozen_evaluator"],
        "evaluator binding differs across contract and preregistration",
    )
    return {
        "profile_count": 2,
        "leaf_difference_count": 1,
        "only_leaf_difference": EXPECTED_FACTOR_LEAF,
        "threshold_count": 7,
        "compiled_scene_index_sha256": bindings[
            "compiled_scene_index"
        ]["sha256"],
        "evaluator_sha256": bindings["evaluator"]["sha256"],
        "pass": True,
    }


def _arg_map(root):
    result = {}
    for element in root.findall("arg"):
        name = element.get("name")
        _require(name and name not in result, "launch argument drifted")
        result[name] = dict(element.attrib)
    return result


def _verify_launch_xml(main_payload, spawn_payload):
    try:
        main_root = ET.fromstring(main_payload)
        spawn_root = ET.fromstring(spawn_payload)
    except ET.ParseError as exc:
        raise R6I2ReviewError("launch XML parse failed: {}".format(exc)) from exc
    _require(
        main_root.tag == "launch" and spawn_root.tag == "launch",
        "launch XML root drifted",
    )
    main_args = _arg_map(main_root)
    spawn_args = _arg_map(spawn_root)
    required_main = {
        "world",
        "seed",
        "xacro_executable",
        "rule_supervisor_config",
        "rule_supervisor_config_sha256",
        "anchor_bank",
        "mechanism_config",
        "attempt_stage",
        "attempt_profile_id",
        "attempt_scene_id",
        "attempt_number",
    }
    required_spawn = {"world", "seed", "xacro_executable"}
    _require(
        all("default" not in main_args[name] for name in required_main),
        "required main launch argument has a default",
    )
    _require(
        all("default" not in spawn_args[name] for name in required_spawn),
        "required spawn launch argument has a default",
    )
    _require(
        main_args["paused"].get("default") == "true"
        and spawn_args["paused"].get("default") == "true",
        "launch must default to paused simulation",
    )
    for name in (
        "allow_simulation_teb_parameter_write",
        "allow_unfrozen_simulation_candidate",
        "start_typed_transaction",
    ):
        _require(
            main_args[name].get("default") == "false",
            "{} must default false".format(name),
        )
    includes = main_root.findall("include")
    expected_include = (
        "$(find m2_gazebo)/launch/"
        "m2_v2_04g_r6_i2_spawn_m2.launch"
    )
    matching = [
        element for element in includes if element.get("file") == expected_include
    ]
    _require(len(matching) == 1, "I2 spawn include is not exact")
    include_args = {
        element.get("name"): element.get("value")
        for element in matching[0].findall("arg")
    }
    _require_exact(
        {
            key: include_args.get(key)
            for key in ("world", "seed", "xacro_executable", "paused")
        },
        {
            "world": "$(arg world)",
            "seed": "$(arg seed)",
            "xacro_executable": "$(arg xacro_executable)",
            "paused": "$(arg paused)",
        },
        "I2 spawn argument forwarding",
    )
    descriptions = [
        element
        for element in spawn_root.findall("param")
        if element.get("name") == "robot_description"
    ]
    _require(len(descriptions) == 1, "robot description command drifted")
    command = descriptions[0].get("command", "")
    _require(
        command.startswith("$(arg xacro_executable) ")
        and "$(find xacro)" not in command,
        "xacro executable is not an explicit reviewed input",
    )
    return {
        "xml_parse_only": True,
        "main_required_argument_count": len(required_main),
        "spawn_required_argument_count": len(required_spawn),
        "paused_default": True,
        "permissive_gate_defaults": False,
        "xacro_executable_required": True,
        "process_started": False,
        "pass": True,
    }


def _canonical_json_sha(document):
    return hashlib.sha256(
        json.dumps(
            document, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("utf-8")
    ).hexdigest()


def _read_external_regular_file_once(path):
    """Read one already-canonical external regular file without following it."""

    source = Path(path)
    _require(
        source.is_absolute()
        and source.as_posix() == os.path.realpath(str(source)),
        "external dependency path is not canonical",
    )
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(str(source), flags)
    except OSError as exc:
        raise R6I2ReviewError(
            "cannot safely open external file {}: {}".format(source, exc)
        ) from exc
    try:
        before = os.fstat(descriptor)
        _require(
            stat.S_ISREG(before.st_mode),
            "external dependency is not a regular file",
        )
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        payload = b"".join(chunks)
        _require(
            (
                before.st_dev,
                before.st_ino,
                before.st_mode,
                before.st_size,
                before.st_mtime_ns,
                before.st_ctime_ns,
            )
            == (
                after.st_dev,
                after.st_ino,
                after.st_mode,
                after.st_size,
                after.st_mtime_ns,
                after.st_ctime_ns,
            )
            and len(payload) == before.st_size,
            "external dependency changed during single-open read",
        )
        return payload
    finally:
        os.close(descriptor)


def _verify_dependency_closure(workspace, document):
    _closed(
        document,
        {
            "schema_version",
            "stage",
            "review_scope",
            "execution_authorized",
            "seed_or_evidence_units_allocated",
            "seed_or_evidence_units_consumed",
            "authorization_resources",
            "generator",
            "local",
            "external",
            "unresolved",
            "closure_sha256",
        },
        "dependency closure",
    )
    _require_exact(
        {
            key: document[key]
            for key in (
                "schema_version",
                "stage",
                "review_scope",
                "execution_authorized",
                "seed_or_evidence_units_allocated",
                "seed_or_evidence_units_consumed",
                "authorization_resources",
                "unresolved",
            )
        },
        {
            "schema_version": "3.0",
            "stage": STAGE,
            "review_scope": "offline_execution_integration_repair_only",
            "execution_authorized": False,
            "seed_or_evidence_units_allocated": 0,
            "seed_or_evidence_units_consumed": 0,
            "authorization_resources": [],
            "unresolved": [],
        },
        "dependency closure boundary",
    )
    local = _closed(
        document["local"],
        {
            "entrypoints",
            "files",
            "edges",
            "external_python_names",
            "external_runtime_names",
            "required_paths",
        },
        "local dependency closure",
    )
    _require(
        isinstance(local["entrypoints"], list) and local["entrypoints"],
        "dependency closure entrypoints are empty",
    )
    _require(
        local["external_python_names"]
        == sorted(set(local["external_python_names"])),
        "external Python names are not unique and sorted",
    )
    inherited_document = _snapshot(
        workspace,
        I1_FROZEN_RESOURCES["i1_dependency_closure"][0],
        yaml_document=True,
    )["document"]
    inherited_python = inherited_document.get("external_python_modules")
    _require(
        isinstance(inherited_python, list)
        and inherited_python == sorted(set(inherited_python))
        and len(inherited_python) == 39
        and set(inherited_python).issubset(
            set(local["external_python_names"])
        ),
        "I1 inherited Python binding coverage is incomplete",
    )
    _require(
        local["external_runtime_names"]
        == sorted(set(local["external_runtime_names"]))
        and set(local["external_runtime_names"]).issubset(
            set(EXPECTED_RUNTIME_BINDINGS)
        )
        and set(local["external_runtime_names"])
        | {"package-executable:xacro:xacro"}
        == set(EXPECTED_RUNTIME_BINDINGS),
        "discovered plus explicitly declared runtime closure drifted",
    )
    paths = []
    for row in local["files"]:
        _closed(row, {"path", "sha256", "size_bytes"}, "local file record")
        _hex_digest(row["sha256"], "local file SHA256")
        snapshot = _snapshot(workspace, row["path"])
        _require_exact(
            {
                "sha256": snapshot["sha256"],
                "size_bytes": snapshot["size_bytes"],
            },
            {"sha256": row["sha256"], "size_bytes": row["size_bytes"]},
            "local dependency record",
        )
        paths.append(row["path"])
    _require(
        paths == sorted(set(paths))
        and local["required_paths"] == paths,
        "local dependency paths are not unique and sorted",
    )
    available = set(paths)
    _require(
        set(local["entrypoints"]).issubset(available),
        "closure entrypoint is not a local file",
    )
    for edge in local["edges"]:
        _closed(edge, {"from", "to", "kind"}, "local dependency edge")
        _require(
            edge["from"] in available and edge["to"] in available,
            "dependency edge leaves local closure",
        )
    _require(
        local["edges"]
        == sorted(
            local["edges"],
            key=lambda row: (row["from"], row["to"], row["kind"]),
        )
        and len(
            {
                (row["from"], row["to"], row["kind"])
                for row in local["edges"]
            }
        )
        == len(local["edges"]),
        "local dependency edges are not unique and sorted",
    )
    external = _closed(
        document["external"],
        {
            "python_interpreter",
            "python_bindings",
            "runtime_bindings",
            "files",
            "unresolved",
            "closure_sha256",
        },
        "external dependency closure",
    )
    _require(external["unresolved"] == [], "external closure is unresolved")
    _closed(
        external["python_interpreter"],
        {"canonical_path", "sha256", "size_bytes"},
        "Python interpreter record",
    )
    file_paths = []
    for row in external["files"]:
        _closed(
            row, {"canonical_path", "sha256", "size_bytes"}, "external file"
        )
        path = Path(row["canonical_path"])
        _require(
            path.is_absolute()
            and path.as_posix() == os.path.realpath(str(path)),
            "external dependency path is not canonical",
        )
        _hex_digest(row["sha256"], "external file SHA256")
        payload = _read_external_regular_file_once(path)
        _require(
            hashlib.sha256(payload).hexdigest() == row["sha256"]
            and len(payload) == row["size_bytes"],
            "external dependency file drifted: {}".format(path),
        )
        file_paths.append(path.as_posix())
    _require(
        file_paths == sorted(set(file_paths)),
        "external file records are not unique and sorted",
    )
    python_names = [row.get("binding") for row in external["python_bindings"]]
    runtime_names = [row.get("binding") for row in external["runtime_bindings"]]
    _require_exact(
        python_names,
        local["external_python_names"],
        "closed Python binding names",
    )
    _require_exact(
        runtime_names,
        sorted(EXPECTED_RUNTIME_BINDINGS),
        "closed runtime binding names",
    )
    for section in ("python_bindings", "runtime_bindings"):
        for row in external[section]:
            expected_keys = (
                {
                    "binding",
                    "resolution_kind",
                    "module_origin",
                    "canonical_paths",
                }
                if section == "python_bindings"
                else {
                    "binding",
                    "resolution_kind",
                    "package",
                    "package_root",
                    "target_canonical_path",
                    "canonical_paths",
                }
            )
            _closed(row, expected_keys, "external binding row")
            _require(
                isinstance(row, dict)
                and isinstance(row.get("canonical_paths"), list)
                and row["canonical_paths"] == sorted(set(row["canonical_paths"]))
                and set(row["canonical_paths"]).issubset(set(file_paths)),
                "external binding path closure drifted",
            )
    xacro = [
        row
        for row in external["runtime_bindings"]
        if row["binding"] == "package-executable:xacro:xacro"
    ]
    _require(
        len(xacro) == 1
        and xacro[0].get("target_canonical_path") == EXPECTED_XACRO_TARGET,
        "xacro runtime target drifted",
    )
    external_payload = {
        key: value for key, value in external.items() if key != "closure_sha256"
    }
    _require(
        _canonical_json_sha(external_payload) == external["closure_sha256"],
        "external logical closure digest drifted",
    )
    closure_payload = {
        key: value for key, value in document.items() if key != "closure_sha256"
    }
    _require(
        _canonical_json_sha(closure_payload) == document["closure_sha256"],
        "complete logical closure digest drifted",
    )
    return {
        "local_file_count": len(paths),
        "local_edge_count": len(local["edges"]),
        "external_file_count": len(file_paths),
        "external_python_binding_count": len(python_names),
        "inherited_python_binding_count": len(inherited_python),
        "inherited_python_binding_coverage_count": len(
            set(inherited_python) & set(python_names)
        ),
        "external_runtime_binding_count": len(runtime_names),
        "unresolved_count": 0,
        "xacro_target_canonical_path": EXPECTED_XACRO_TARGET,
        "closure_sha256": document["closure_sha256"],
        "mechanically_rehashed": True,
        "process_started": False,
        "pass": True,
    }


def _verify_component_review(document):
    _closed(
        document,
        {
            "schema_version",
            "architecture_generation",
            "stage",
            "review_id",
            "status",
            "review_result",
            "offline_only",
            "execution_authorized",
            "real_authorization_created",
            "seed_values",
            "evidence_budget_units",
            "ros_or_gazebo_started",
            "repairs",
            "negative_test_classes",
            "scope_limit",
        },
        "authorization/assessment component review",
    )
    _require_exact(
        {
            key: document[key]
            for key in (
                "schema_version",
                "architecture_generation",
                "stage",
                "status",
                "review_result",
                "offline_only",
                "execution_authorized",
                "real_authorization_created",
                "seed_values",
                "evidence_budget_units",
                "ros_or_gazebo_started",
                "scope_limit",
            )
        },
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "status": "repair_component_review_pass_execution_not_authorized",
            "review_result": "pass",
            "offline_only": True,
            "execution_authorized": False,
            "real_authorization_created": False,
            "seed_values": [],
            "evidence_budget_units": 0,
            "ros_or_gazebo_started": False,
            "scope_limit": (
                "implementation_and_offline_review_only_no_execution_authority"
            ),
        },
        "component review boundary",
    )
    expected_repairs = {
        "single_open_no_follow_hash_and_parse",
        "closed_authorization_top_level_schema",
        "closed_nested_authorization_schema",
        "exact_preregistration_schedule_binding",
        "type_sensitive_schedule_comparison",
        "all_bound_resources_single_read_verified",
        "assessor_source_digests_explicitly_scoped",
        "assessor_has_no_free_stage_document_reference",
        "assessor_is_deterministic_and_write_free",
    }
    _require(
        set(document["repairs"]) == expected_repairs
        and all(value is True for value in document["repairs"].values()),
        "component repair review drifted",
    )
    _require(
        isinstance(document["negative_test_classes"], list)
        and len(document["negative_test_classes"]) == 12
        and len(set(document["negative_test_classes"])) == 12,
        "component negative-test coverage drifted",
    )
    return {
        "status": document["status"],
        "repair_check_count": len(document["repairs"]),
        "negative_test_class_count": len(document["negative_test_classes"]),
        "execution_authorized": False,
        "seed_values": [],
        "evidence_budget_units": 0,
        "ros_or_gazebo_started": False,
        "pass": True,
    }


def _verify_no_execution_evidence(workspace):
    root = Path(workspace)
    manifest_root = root / "experiments/manifests/v2/integration"
    authorizations = sorted(
        path.relative_to(root).as_posix()
        for path in manifest_root.glob("*r6_i2*authorization*.yaml")
    )
    forbidden = [
        ARTIFACT_ROOT_RELATIVE / "execution",
        ARTIFACT_ROOT_RELATIVE / "journals",
        ARTIFACT_ROOT_RELATIVE / "attempts",
        ARTIFACT_ROOT_RELATIVE / "v2_04g_r6_i2_stage_report.yaml",
        ARTIFACT_ROOT_RELATIVE / "v2_04g_r6_i2_execution_receipt.yaml",
    ]
    existing = [path.as_posix() for path in forbidden if (root / path).exists()]
    _require(authorizations == [], "R6-I2 execution authorization exists")
    _require(existing == [], "R6-I2 execution evidence exists")
    return {
        "authorization_manifest_count": 0,
        "execution_evidence_path_count": 0,
        "seed_values": [],
        "execution_schedule": [],
        "evidence_budget_authorized": 0,
        "evidence_units_consumed": 0,
        "pass": True,
    }


def build_review(workspace):
    """Build the deterministic non-authorizing review mapping."""

    root = Path(workspace).resolve()
    contract_snapshot = _snapshot(
        root, CONTRACT_RELATIVE, yaml_document=True
    )
    contract = _verify_contract(contract_snapshot["document"])
    transition = _verify_transition(
        _snapshot(root, TRANSITION_RELATIVE, yaml_document=True)["document"]
    )
    preregistration = _verify_preregistration(
        _snapshot(root, PREREGISTRATION_RELATIVE, yaml_document=True)[
            "document"
        ]
    )
    resources = _verify_resources(root, contract["resources"])
    i1 = _verify_i1_frozen(root)
    r5 = _tree_snapshot(root, R5_ROOT_RELATIVE)
    semantics = _verify_semantic_boundary(
        root, preregistration, contract
    )
    launch = _verify_launch_xml(
        _snapshot(root, MAIN_LAUNCH_RELATIVE)["payload"],
        _snapshot(root, SPAWN_LAUNCH_RELATIVE)["payload"],
    )
    closure = _verify_dependency_closure(
        root,
        _snapshot(root, CLOSURE_RELATIVE, yaml_document=True)["document"],
    )
    component = _verify_component_review(
        _snapshot(root, COMPONENT_REVIEW_RELATIVE, yaml_document=True)[
            "document"
        ]
    )
    absence = _verify_no_execution_evidence(root)
    _require(
        transition["seed_values"] == preregistration["seed_values"] == [],
        "transition/preregistration seed boundary differs",
    )
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": (
            "fam_teb_v2_04g_r6_i2_bootstrap_integrity_repair_review_1"
        ),
        "status": "repair_integration_review_pass_execution_not_authorized",
        "review_result": "pass",
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "resource_integrity": {
            "contract": {
                "path": CONTRACT_RELATIVE.as_posix(),
                "sha256": contract_snapshot["sha256"],
            },
            "resource_count": len(resources),
            "all_hashes_match": True,
            "resources": resources,
        },
        "i1_frozen_integrity": {
            "resource_count": len(i1),
            "all_hashes_match": True,
            "resources": i1,
        },
        "r5_frozen_tree": r5,
        "semantic_boundary_review": semantics,
        "launch_static_review": launch,
        "dependency_closure_review": closure,
        "authorization_assessment_component_review": component,
        "execution_absence_review": absence,
        "repair_count": 6,
        "all_repair_gates_pass": True,
        "side_effects": {
            "authorization_created": False,
            "seed_values": [],
            "execution_schedule": [],
            "seed_or_evidence_units_allocated": 0,
            "seed_or_evidence_units_consumed": 0,
            "ros_started": False,
            "gazebo_started": False,
            "move_base_started": False,
            "training_started": False,
            "real_vehicle_used": False,
            "real_vehicle_teb_written": False,
        },
        "claim_limit": (
            "bootstrap_and_integrity_repair_integration_review_only_"
            "no_execution_authority_or_evidence"
        ),
    }


def _atomic_yaml(path, value):
    target = Path(path)
    _require(target.parent.is_dir(), "review output directory is missing")
    _require(not target.is_symlink(), "review output is a symlink")
    payload = yaml.safe_dump(
        value,
        sort_keys=False,
        allow_unicode=True,
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.",
        dir=str(target.parent),
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path("/home/robot/robot_ws_base_rl"),
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="validate and emit only a one-line status",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="atomically write the canonical machine review report",
    )
    args = parser.parse_args()
    if args.check_only and args.output is not None:
        parser.error("--check-only and --output are mutually exclusive")
    review = build_review(args.workspace)
    if args.output is not None:
        root = args.workspace.resolve()
        expected = (root / OUTPUT_RELATIVE).resolve()
        supplied = args.output
        if not supplied.is_absolute():
            supplied = root / supplied
        if supplied.resolve() != expected:
            parser.error("output must be the canonical R6-I2 review report")
        _atomic_yaml(expected, review)
        print(review["status"])
    elif args.check_only:
        print(review["status"])
    else:
        print(yaml.safe_dump(review, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
