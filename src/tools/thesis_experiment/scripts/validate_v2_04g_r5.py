#!/usr/bin/env python3
"""Fail-closed, ROS-free preregistration audit for FAM-TEB V2-04G-R5."""

import argparse
import copy
import hashlib
import importlib.util
import json
from collections import Counter
from pathlib import Path
import tempfile

import yaml


STAGE = "V2-04G-R5"
DEFAULT_WORKSPACE = Path("/home/robot/robot_ws_base_rl")
DEFAULT_PREREGISTRATION = Path(
    "experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml"
)
DEFAULT_AUDIT = Path(
    "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_dry_run_audit.yaml"
)

CANDIDATE_IDS = [
    "r5_ttc_control_h500",
    "r5_ttc_h450",
    "r5_ttc_h400",
]
ELIGIBLE_CANDIDATE_IDS = ["r5_ttc_h450", "r5_ttc_h400"]
HORIZONS = {
    "r5_ttc_control_h500": 5.0,
    "r5_ttc_h450": 4.5,
    "r5_ttc_h400": 4.0,
}
READINESS_PROBE_SEEDS = [5111, 5112, 5113]
READINESS_SUPPORT_SEEDS = [5114, 5115, 5116, 5117]
NAVIGATION_SEEDS = list(range(5121, 5136))
HELD_OUT_SEEDS = list(range(5001, 5011))
PREVIOUS_VALIDATION_SEEDS = (
    list(range(4601, 4611)) + list(range(4801, 4811))
)
PRIOR_V2_04G_SEEDS = sum(
    (
        list(range(first, last + 1))
        for first, last in (
            (4901, 4915),
            (4921, 4935),
            (4941, 4946),
            (4951, 4965),
            (4971, 4976),
            (4981, 4986),
            (4991, 4996),
            (5021, 5035),
            (5041, 5046),
            (5051, 5056),
            (5061, 5075),
            (5081, 5086),
            (5091, 5105),
        )
    ),
    [],
)
NAVIGATION_FAMILIES = {
    "CRUISE": 3,
    "DYNAMIC": 3,
    "STATIC_DENSE": 3,
    "CORRIDOR": 3,
    "MANEUVER": 3,
}
READINESS_FAMILIES = {
    "CRUISE": 1,
    "DYNAMIC": 3,
    "STATIC_DENSE": 1,
    "CORRIDOR": 1,
    "MANEUVER": 1,
}

EXPECTED_PATHS = {
    "contract": "config/thesis_experiments/v2/v2_04g_r5_ttc_robustness_contract.yaml",
    "candidate_bank":
        "experiments/manifests/v2/calibration/v2_04g_r5_ttc_timing_candidates.yaml",
    "scene_derivation":
        "experiments/manifests/v2/calibration/v2_04g_r5_scene_derivation.yaml",
    "readiness_scene_derivation": (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r5_ttc_readiness_scene_derivation.yaml"
    ),
    "scene_manifest":
        "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_calibration_scenes.yaml",
    "compiled_scene_index": (
        "artifacts/v2/calibration/v2_04g_r5/"
        "compiled_scenes/compiled_scene_index.yaml"
    ),
    "readiness_scene_manifest": (
        "artifacts/v2/calibration/v2_04g_r5/"
        "v2_04g_r5_ttc_readiness_scenes.yaml"
    ),
    "readiness_compiled_scene_index": (
        "artifacts/v2/calibration/v2_04g_r5/"
        "ttc_readiness_compiled_scenes/compiled_scene_index.yaml"
    ),
    "candidate_materializer": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r5_candidate_materializer.py"
    ),
}

EXPECTED_RESOURCE_PATHS = {
    "config/thesis_experiments/v2/v2_04g_r5_ttc_robustness_contract.yaml",
    "experiments/manifests/v2/calibration/v2_04g_r5_ttc_timing_candidates.yaml",
    "experiments/manifests/v2/calibration/v2_04g_r5_scene_derivation.yaml",
    "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_calibration_scenes.yaml",
    (
        "artifacts/v2/calibration/v2_04g_r5/"
        "compiled_scenes/compiled_scene_index.yaml"
    ),
    (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r5_ttc_readiness_scene_derivation.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r5/"
        "v2_04g_r5_ttc_readiness_scenes.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r5/"
        "ttc_readiness_compiled_scenes/compiled_scene_index.yaml"
    ),
    (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r5_candidate_materializer.py"
    ),
    "src/tools/thesis_experiment/scripts/validate_v2_04g_r5.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r5.py",
    "src/tools/thesis_experiment/scripts/derive_v2_04g_scenes.py",
    "src/tools/thesis_experiment/scripts/compile_v2_scenes.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
    "config/thesis_experiments/v2/simulation_contract.yaml",
    "src/simulation/m2_gazebo/config/simulation_candidates.yaml",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/supervisor.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/anchor_bank.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/mechanism.yaml"
    ),
    "src/application/teb_mode_manager/src/teb_mode_manager/rule_supervisor.py",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "mechanism_controller.py"
    ),
    "src/application/teb_mode_manager/src/teb_mode_manager/action_pipeline.py",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "world_model_input_join.py"
    ),
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "bounded_context_join.py"
    ),
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "idempotent_typed_teb_transaction.py"
    ),
    (
        "src/application/teb_mode_manager/scripts/"
        "rule_context_supervisor_node.py"
    ),
    "src/perception/nav_world_model/src/nav_world_model/core.py",
    "src/perception/nav_world_model/scripts/nav_world_model_node.py",
    "src/perception/nav_world_model/src/nav_world_model/risk_evidence.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py",
    "src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp",
    (
        "src/simulation/m2_gazebo/launch/"
        "m2_v2_04g_r2_mechanism_calibration.launch"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_stage_report.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_assessment.yaml"
    ),
    (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r4_r1_preregistration.yaml"
    ),
    (
        "config/thesis_experiments/v2/"
        "v2_04g_r4_r1_clearance_repair_contract.yaml"
    ),
    (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r4_r1_clearance_candidates.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_calibration_scenes.yaml"
    ),
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "compiled_scenes/compiled_scene_index.yaml"
    ),
}

EXPECTED_IMMUTABLE_RESOURCE_SHA256 = {
    "src/tools/thesis_experiment/scripts/derive_v2_04g_scenes.py":
        "c477448794e2c509c7a652e4731344761179e3c3e2ebea5de2ee9e3f5260e65b",
    "src/tools/thesis_experiment/scripts/compile_v2_scenes.py":
        "a6111cf0f6edc2d86264f69a6989533a1b6a021e1685bb2d67b6475a32a26bae",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py":
        "f056841ebc32c2c849bafa1b7670352f9510fd2cf2c120a448fd89093560f718",
    "config/thesis_experiments/v2/simulation_contract.yaml":
        "47139897bc6ea4e9da7392e2d2df7dd441b68b218745f443f2fc112199e6b1bb",
    "src/simulation/m2_gazebo/config/simulation_candidates.yaml":
        "63edc80a38b4d35c5491660ad4d6882d2ddaf95b4ff66fda820ba20eadbfd946",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/supervisor.yaml"
    ): "2e29d6c0f2f32360976a76dbeef5e2bd5739ba1e64da888265ca7232a45aae39",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/anchor_bank.yaml"
    ): "2f5d52a07f389a60eb03a92aaf5c082a0f446d0e493c8ee6ed2500dc6b23fb67",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/runtime_candidate_configs/"
        "r4r1_clearance_m030/mechanism.yaml"
    ): "d08cf4aa3093278844bff95529c206dc190a606bb3702ae1f7af38c3ccc68645",
    "src/application/teb_mode_manager/src/teb_mode_manager/rule_supervisor.py":
        "ed42d56a0be93cfafd15aa46c59d1237dfec19ad2a5e2f90d9f7cea8d7846d14",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "mechanism_controller.py"
    ): "bd48e206b181efc1da9359b6fd01fe94ebd661975ae15d4b6fe5131d80e7716a",
    "src/application/teb_mode_manager/src/teb_mode_manager/action_pipeline.py":
        "d13873814e27bd95b8f3a752d631faa7d1e0e79a63c763408e678e926885fdba",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "world_model_input_join.py"
    ): "b0b54475fee338e197a1ce726d0eb63932f7879d015834e97474574f28230b55",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "bounded_context_join.py"
    ): "485a5c57397c845982508f1f4c67739ad8c58ebf2f63c9336a08e2f0f3157442",
    (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "idempotent_typed_teb_transaction.py"
    ): "2ad255dea35715709384c02683836847a9089b1a729677f7739d2b9ea53aabf1",
    (
        "src/application/teb_mode_manager/scripts/"
        "rule_context_supervisor_node.py"
    ): "63800f33ef583c5b3fe337f86027e62ad70ec68ecfe210018cb176f377a79732",
    "src/perception/nav_world_model/src/nav_world_model/core.py":
        "160e1c09ab14903c1f75e40862a9645c55a0f690097e1755c82ed072ce2f89ad",
    "src/perception/nav_world_model/scripts/nav_world_model_node.py":
        "ed27e2b79060e294e0ec087200cb508c0254114ad48b9c40d97c300b4e5680ee",
    "src/perception/nav_world_model/src/nav_world_model/risk_evidence.py":
        "96f20f43e5f764d8356725ee5b9d1598a4c9265ce228740f69710fd085b7a0dc",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py":
        "55ad5a0abd2d6a8fe41ed9942a512405e42d7be6c41eff779df8a7335496e681",
    "src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp":
        "c77027785d268001ea70f71169e1f6f1250a6bf3a30c7b1fccda8e613b2e0a65",
    (
        "src/simulation/m2_gazebo/launch/"
        "m2_v2_04g_r2_mechanism_calibration.launch"
    ): "e23b3b95b7ca7ed1605fde160e5652e433357c9e192c0f318e95a9a7c0a073ac",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_stage_report.yaml"
    ): "e1ad0aeb7739e8c1abad0f17059f8dbe31c671dd03584d96637830033e5ab22a",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_assessment.yaml"
    ): "201d2f7a8a5c4b679a1500a9487c342434f73222f0c262c4a8014441efc6bc04",
    (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r4_r1_preregistration.yaml"
    ): "70b5d7c48b5a9f1cf290a361042d69836485e0a0b30ee29eff5885f34f3758b1",
    (
        "config/thesis_experiments/v2/"
        "v2_04g_r4_r1_clearance_repair_contract.yaml"
    ): "f0dd447f0ab883744703338c29076c5b51b2fec28bcf309487af33fc27f13eb0",
    (
        "experiments/manifests/v2/calibration/"
        "v2_04g_r4_r1_clearance_candidates.yaml"
    ): "eb296b1daf81c1d17fc3a3caa665addfae3cbb51f2b6221a5e44e1526da5ae28",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "v2_04g_r4_r1_calibration_scenes.yaml"
    ): "29527d9bac30e002124a215d540244c7a053062854a2af5f38a8b3b3577cb0f4",
    (
        "artifacts/v2/calibration/v2_04g_r4_r1/"
        "compiled_scenes/compiled_scene_index.yaml"
    ): "96d0257b492207041032d3f7b883baa3e6510cab0284f793cad8712a3fb9c977",
}

EXPECTED_AUTHORIZATION = {
    "contract_and_preregistration_edits": True,
    "scene_derivation_and_sdf_compilation_without_ros": True,
    "validator_and_unit_tests": True,
    "dry_run_audit_without_ros_or_gazebo": True,
    "bounded_gazebo_calibration_execution": False,
    "formal_gazebo_batch": False,
    "winner_freeze": False,
    "held_out_validation": False,
    "v2_05": False,
    "sac_training": False,
    "real_vehicle_motion": False,
    "real_vehicle_parameter_write": False,
}
EXPECTED_PREREGISTRATION_AUTHORIZATION = dict(EXPECTED_AUTHORIZATION)

EXPECTED_CONTRACT_SECTION_SHA256 = {
    "scope": "328ab160e914f13b02a70bd82a0ca9cb0f3fe2141d256c74d6832aa23205ef4d",
    "ttc_activation_coverage_readiness_gate":
        "51293e5365feed0419af16d98e4fe22c21c18c2f5850392e6506ecf59695a1fb",
    "ttc_coverage_gate":
        "8178312d060932bd3d3cff7e29f54c209d6856577435103113124e118c7e4a7f",
    "hard_gates":
        "d08b89b4a81df8f843737ed6f26ae5ef046958fc0d0f0b46ff7b70b52a99f508",
    "ranking_after_all_hard_gates":
        "be18de3777b9076d9f3e1e3c8706a646e2b553c12234e905ffcdd82f27179db8",
    "budget_and_retry":
        "6438b5d8a7aeef86e429fe98b1bc7248f7dcbdf63c79158c179e438a258fea5b",
    "execution_order":
        "fec7df90c13b31d71361ecf2ad940d7ff0a59a16582bbc3a23ea216c00a65f8f",
    "stopping_rules":
        "337826a6360abc68e2199112cfabdab89ff6965513df8153cfb35708e5a3987e",
}


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found an unhashable key",
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found duplicate key {!r}".format(key),
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_yaml(path):
    path = Path(path)
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError("cannot strictly load YAML {}: {}".format(path, exc)) from exc
    if not isinstance(value, dict):
        raise ValueError("YAML root must be a mapping: {}".format(path))
    return value


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_sha256(value):
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _inside(root, path, label):
    root = Path(root).resolve()
    path = Path(path)
    _require(not path.is_absolute(), "{} path must be workspace-relative".format(label))
    resolved = (root / path).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError("{} escapes the workspace".format(label)) from exc
    return resolved


def _resource_entries(value, prefix=""):
    """Yield every nested {path, sha256} resource declaration."""
    if isinstance(value, dict):
        if "path" in value or "sha256" in value:
            _require(
                set(value) == {"path", "sha256"},
                "{} resource must contain only path and sha256".format(prefix),
            )
            yield prefix, value
            return
        for key, child in value.items():
            child_prefix = "{}.{}".format(prefix, key) if prefix else str(key)
            yield from _resource_entries(child, child_prefix)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _resource_entries(child, "{}[{}]".format(prefix, index))


def _verify_declared_resources(root, preregistration):
    entries = list(_resource_entries({
        "resources": preregistration.get("resources"),
        "frozen_r4_r1_boundary": preregistration.get("frozen_r4_r1_boundary"),
    }))
    _require(entries, "preregistration declares no frozen resources")
    by_path = {}
    verified = {}
    for label, resource in entries:
        relative = resource.get("path")
        digest = resource.get("sha256")
        _require(
            isinstance(relative, str) and relative and not relative.startswith("/"),
            "{} has an invalid relative path".format(label),
        )
        _require(
            isinstance(digest, str)
            and len(digest) == 64
            and all(character in "0123456789abcdef" for character in digest),
            "{} has an invalid SHA-256".format(label),
        )
        path = _inside(root, relative, label)
        _require(path.is_file(), "{} resource is missing: {}".format(label, relative))
        actual = _sha256(path)
        _require(actual == digest, "{} resource hash drifted".format(label))
        if relative in by_path:
            _require(
                by_path[relative] == digest,
                "{} has conflicting resource hashes".format(relative),
            )
        by_path[relative] = digest
        verified[label] = {"path": relative, "sha256": digest}
    _require(
        len(entries) == len(by_path) == len(EXPECTED_RESOURCE_PATHS)
        and set(by_path) == EXPECTED_RESOURCE_PATHS,
        "R5 frozen resource closure drifted",
    )
    for relative, expected_digest in EXPECTED_IMMUTABLE_RESOURCE_SHA256.items():
        _require(
            by_path.get(relative) == expected_digest,
            "immutable R4-R1/runtime resource drifted: {}".format(relative),
        )
    for name, relative in EXPECTED_PATHS.items():
        _require(
            relative in by_path,
            "required R5 resource is not frozen: {} ({})".format(name, relative),
        )
    return by_path, verified


def _verify_safety_boundary(name, document, require_architecture=True):
    expected = {
        "schema_version": "2.0",
        "stage": STAGE,
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    if require_architecture:
        expected["architecture_generation"] = "v2"
    for key, value in expected.items():
        _require(
            document.get(key) == value,
            "{} safety boundary drifted at {}".format(name, key),
        )


def _verify_contract_frozen_sections(contract):
    _require(
        contract.get("status") == "preregistered_design_dry_run_only",
        "contract status drifted",
    )
    for name, expected_digest in EXPECTED_CONTRACT_SECTION_SHA256.items():
        _require(name in contract, "contract section is missing: {}".format(name))
        _require(
            _canonical_sha256(contract[name]) == expected_digest,
            "contract frozen section drifted: {}".format(name),
        )


def _verify_factor(contract, preregistration, bank):
    expected_values = [5.0, 4.5, 4.0]
    contract_factor = contract.get("single_changed_factor", {})
    _require(
        contract_factor.get("name") == "dynamic_conflict_prediction_horizon_s"
        and contract_factor.get("runtime_field")
        == "supervisor.dynamic.predicted_ttc_max_s"
        and contract_factor.get("preregistered_values_s") == expected_values
        and contract_factor.get("control_value_s") == 5.0
        and contract_factor.get("repair_candidate_values_s") == [4.5, 4.0]
        and contract_factor.get("overlay_release_confirmation_s_frozen") == 0.20
        and contract_factor.get("evaluator_relative_ttc_horizon_s_frozen") == 5.0,
        "contract single-factor boundary drifted",
    )
    prereg_factor = preregistration.get("single_changed_factor", {})
    _require(
        prereg_factor.get("name") == contract_factor["name"]
        and prereg_factor.get("runtime_field") == contract_factor["runtime_field"]
        and prereg_factor.get("candidate_values_s") == expected_values
        and prereg_factor.get("control_value_s") == 5.0
        and prereg_factor.get("repair_candidate_values_s") == [4.5, 4.0]
        and prereg_factor.get("evaluator_relative_ttc_horizon_s_frozen") == 5.0
        and prereg_factor.get("overlay_release_confirmation_s_frozen") == 0.20
        and prereg_factor.get("all_anchor_values_changed") is False
        and prereg_factor.get(
            "supervisor_fields_other_than_runtime_field_changed"
        ) is False
        and prereg_factor.get("mechanism_controller_changed") is False
        and prereg_factor.get("typed_transaction_or_join_changed") is False
        and prereg_factor.get(
            "evaluator_or_scene_label_semantics_changed"
        ) is False,
        "preregistration single-factor boundary drifted",
    )
    bank_factor = bank.get("single_changed_factor", {})
    _require(
        bank_factor.get("name") == contract_factor["name"]
        and bank_factor.get("runtime_field") == contract_factor["runtime_field"]
        and bank_factor.get("preregistered_values_s") == expected_values
        and bank_factor.get("evaluator_ttc_horizon_s_frozen") == 5.0
        and bank_factor.get("overlay_release_confirmation_s_frozen") == 0.20,
        "candidate bank single-factor boundary drifted",
    )
    frozen = contract.get("frozen_passed_mechanisms", {})
    _require(frozen and all(value is False for value in frozen.values()),
             "a passed mechanism was not frozen")
    contract_m030 = contract.get("frozen_m030_input", {})
    prereg_m030 = preregistration.get("frozen_m030_input", {})
    _require(
        contract_m030.get("classification")
        == "demonstrated_clearance_repair_non_ranking_input_not_system_winner"
        and contract_m030.get("maneuver_forward_min_obstacle_dist_m") == 0.30
        and contract_m030.get("maneuver_reverse_min_obstacle_dist_m") == 0.30
        and contract_m030.get("maneuver_inflation_dist_m") == 0.52,
        "contract m030 non-ranking input drifted",
    )
    _require(
        prereg_m030.get("classification")
        == contract_m030.get("classification")
        and prereg_m030.get("maneuver_forward_min_obstacle_dist_m") == 0.30
        and prereg_m030.get("maneuver_reverse_min_obstacle_dist_m") == 0.30
        and prereg_m030.get("maneuver_inflation_dist_m") == 0.52
        and prereg_m030.get("control_or_candidate_ranking_claim") is False
        and prereg_m030.get("system_winner_claim") is False,
        "preregistered m030 non-ranking input drifted",
    )


def _load_python_module(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    _require(spec is not None and spec.loader is not None,
             "cannot import frozen Python resource {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_materializer(path):
    return _load_python_module(
        path, "v2_04g_r5_candidate_materializer_audit"
    )


def _diff_paths(left, right, prefix=""):
    if type(left) is not type(right):
        return {prefix or "<root>"}
    if isinstance(left, dict):
        differences = set()
        for key in set(left) | set(right):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in left or key not in right:
                differences.add(child)
            else:
                differences.update(_diff_paths(left[key], right[key], child))
        return differences
    if isinstance(left, list):
        differences = set()
        if len(left) != len(right):
            differences.add(prefix)
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            differences.update(_diff_paths(
                left_item, right_item, "{}[{}]".format(prefix, index)
            ))
        return differences
    return set() if left == right else {prefix or "<root>"}


def _deep_update(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def _without_identity(document, field):
    result = copy.deepcopy(document)
    _require(field in result, "materialized document lacks identity field {}".format(field))
    result.pop(field)
    return result


def _verify_candidates(root, bank, candidate_path, materializer_path):
    rows = bank.get("candidates")
    _require(isinstance(rows, list) and len(rows) == 3,
             "candidate bank must contain exactly three candidates")
    _require([row.get("candidate_id") for row in rows] == CANDIDATE_IDS,
             "candidate identities or order drifted")
    _require(
        [row.get("candidate_id") for row in rows if row.get("winner_eligible") is True]
        == ELIGIBLE_CANDIDATE_IDS,
        "winner-eligible candidate set drifted",
    )
    _require(rows[0].get("winner_eligible") is False,
             "the timing control must remain winner-ineligible")
    for row in rows:
        candidate_id = row["candidate_id"]
        _require(
            set(row) == {
                "candidate_id",
                "role",
                "predicted_ttc_max_s",
                "winner_eligible",
            },
            "{} candidate declaration contains an extra factor".format(candidate_id),
        )
        _require(
            row.get("predicted_ttc_max_s") == HORIZONS[candidate_id],
            "{} horizon drifted".format(candidate_id),
        )
        expected_role = (
            "frozen_r4_r1_m030_timing_control_not_winner_eligible"
            if candidate_id == CANDIDATE_IDS[0]
            else "ttc_prediction_timing_repair_candidate"
        )
        _require(
            row.get("role") == expected_role,
            "{} candidate role drifted".format(candidate_id),
        )

    frozen = bank.get("frozen_m030_input", {})
    _require(
        frozen.get("status")
        == "demonstrated_clearance_repair_non_ranking_input_not_system_winner",
        "m030 was misclassified as a system winner",
    )
    _require(
        frozen.get("maneuver_min_obstacle_dist_m") == 0.30
        and frozen.get("maneuver_inflation_dist_m") == 0.52,
        "declared m030 clearance input drifted",
    )
    frozen_paths = {}
    for name in ("source_stage_report", "supervisor", "anchor_bank", "mechanism"):
        resource = frozen.get(name, {})
        _require(set(resource) == {"path", "sha256"},
                 "m030 {} resource declaration drifted".format(name))
        path = _inside(root, resource["path"], "m030.{}".format(name))
        _require(path.is_file() and _sha256(path) == resource["sha256"],
                 "m030 {} resource drifted".format(name))
        frozen_paths[name] = path

    base_supervisor = _load_yaml(frozen_paths["supervisor"])
    base_anchor = _load_yaml(frozen_paths["anchor_bank"])
    base_mechanism = _load_yaml(frozen_paths["mechanism"])
    for anchor_id in ("anchor_maneuver_forward", "anchor_maneuver_reverse"):
        values = base_anchor.get("anchors", {}).get(anchor_id, {}).get("values", {})
        _require(
            values.get("min_obstacle_dist") == 0.30
            and values.get("inflation_dist") == 0.52,
            "frozen {} m030 clearance values drifted".format(anchor_id),
        )
    _require(
        base_supervisor.get("dynamic", {}).get("predicted_ttc_max_s") == 5.0
        and base_supervisor.get("transition", {}).get(
            "overlay_release_confirmation_s"
        ) == 0.20,
        "frozen supervisor TTC/release boundary drifted",
    )

    materializer = _load_materializer(materializer_path)
    materializer.WORKSPACE = Path(root).resolve()
    with tempfile.TemporaryDirectory(prefix="v2_04g_r5_audit_") as directory:
        runtime = materializer.materialize_candidates(candidate_path, directory)
        _require(list(runtime) == CANDIDATE_IDS,
                 "materializer output identities or order drifted")
        behavior_differences = {}
        for candidate_id in CANDIDATE_IDS:
            paths = runtime[candidate_id]
            supervisor = _load_yaml(paths["supervisor"])
            anchor = _load_yaml(paths["anchor_bank"])
            mechanism = _load_yaml(paths["mechanism"])
            supervisor_diff = _diff_paths(
                _without_identity(base_supervisor, "profile_id"),
                _without_identity(supervisor, "profile_id"),
            )
            expected_diff = (
                set()
                if candidate_id == CANDIDATE_IDS[0]
                else {"dynamic.predicted_ttc_max_s"}
            )
            _require(
                supervisor_diff == expected_diff,
                "{} changed supervisor behavior outside the single factor: {}".format(
                    candidate_id, sorted(supervisor_diff)
                ),
            )
            _require(
                supervisor["dynamic"]["predicted_ttc_max_s"]
                == HORIZONS[candidate_id],
                "{} materialized the wrong horizon".format(candidate_id),
            )
            _require(
                not _diff_paths(
                    _without_identity(base_anchor, "bank_id"),
                    _without_identity(anchor, "bank_id"),
                ),
                "{} changed frozen Anchor behavior".format(candidate_id),
            )
            _require(
                not _diff_paths(
                    _without_identity(base_mechanism, "profile_id"),
                    _without_identity(mechanism, "profile_id"),
                ),
                "{} changed frozen mechanism behavior".format(candidate_id),
            )
            behavior_differences[candidate_id] = sorted(supervisor_diff)
    return behavior_differences


def _verify_derivation(root, spec, manifest, expected_path, expected_seeds):
    source_resource = spec.get("source_manifest", {})
    _require(set(source_resource) == {"path", "sha256"},
             "derivation source manifest declaration drifted")
    source_path = _inside(root, source_resource["path"], "derivation.source_manifest")
    _require(
        source_path.is_file() and _sha256(source_path) == source_resource["sha256"],
        "derivation source manifest drifted",
    )
    source = _load_yaml(source_path)
    source_scenes = {
        row.get("scene_id"): row for row in source.get("scenes", [])
        if isinstance(row, dict)
    }
    rows = spec.get("scene_derivations")
    _require(isinstance(rows, list) and len(rows) == len(expected_seeds),
             "scene derivation count drifted")
    _require([row.get("seed") for row in rows] == expected_seeds,
             "scene derivation seed order drifted")
    _require(spec.get("target_path") == expected_path,
             "scene derivation target path drifted")
    target_scenes = manifest.get("scenes")
    _require(isinstance(target_scenes, list) and len(target_scenes) == len(rows),
             "derived scene manifest count drifted")
    target_by_id = {row.get("scene_id"): row for row in target_scenes}
    _require(len(target_by_id) == len(rows), "duplicate derived scene id")
    for derivation in rows:
        allowed_keys = {
            "source_scene_id",
            "target_scene_id",
            "seed",
            "layout_variant",
            "evaluator_reason",
            "scene_patch",
        }
        if "execution_role" in derivation:
            allowed_keys.add("execution_role")
        _require(
            set(derivation).issubset(allowed_keys)
            and {
                "source_scene_id",
                "target_scene_id",
                "seed",
                "layout_variant",
                "evaluator_reason",
            }.issubset(derivation),
            "scene derivation row contains an undeclared transformation",
        )
        source_scene = source_scenes.get(derivation.get("source_scene_id"))
        target_scene = target_by_id.get(derivation.get("target_scene_id"))
        _require(source_scene is not None, "derivation source scene is missing")
        _require(target_scene is not None, "derivation target scene is missing")
        _require(
            target_scene.get("seed") == derivation.get("seed")
            and target_scene.get("family") == source_scene.get("family")
            and target_scene.get("split") == "calibration"
            and target_scene.get("layout", {}).get("variant")
            == derivation.get("layout_variant")
            and target_scene.get("evaluator_only", {}).get("reason")
            == derivation.get("evaluator_reason"),
            "derived scene content drifted: {}".format(
                derivation.get("target_scene_id")
            ),
        )
        expected_scene = copy.deepcopy(source_scene)
        expected_scene["scene_id"] = derivation["target_scene_id"]
        expected_scene["seed"] = derivation["seed"]
        expected_scene["split"] = "calibration"
        expected_scene["layout"]["variant"] = derivation["layout_variant"]
        expected_scene["evaluator_only"]["reason"] = derivation[
            "evaluator_reason"
        ]
        _deep_update(expected_scene, derivation.get("scene_patch", {}))
        _require(
            target_scene == expected_scene,
            "derived scene changed frozen geometry or semantics: {}".format(
                derivation["target_scene_id"]
            ),
        )


def _verify_compiled_index(
    root,
    index,
    manifest,
    manifest_path,
    expected_count,
    expected_directory,
    scene_contract,
):
    normalized_manifest = scene_contract.load_v2_scene_manifest(
        Path(root) / manifest_path, root
    )
    expected_instances = scene_contract.compile_v2_manifest(
        normalized_manifest, root
    )
    expected_by_id = {
        row["scene"]["scene_id"]: row for row in expected_instances
    }
    _require(
        index.get("schema_version") == "2.0"
        and index.get("formal_result") is False
        and index.get("runtime_ready") is False
        and index.get("scene_count") == expected_count
        and index.get("manifest_id") == manifest.get("manifest_id"),
        "compiled scene index boundary drifted",
    )
    _require(
        len(expected_by_id) == expected_count
        and index.get("families")
        == [row["scene"]["family"] for row in expected_instances],
        "compiled scene index family inventory drifted",
    )
    files = index.get("files")
    _require(isinstance(files, list) and len(files) == 2 * expected_count,
             "compiled scene file inventory is incomplete")
    scene_ids = [row["scene_id"] for row in manifest["scenes"]]
    seen = Counter()
    directory = (Path(root) / expected_directory).resolve()
    for item in files:
        _require(set(item) == {"path", "sha256"},
                 "compiled scene resource declaration drifted")
        path = _inside(root, item["path"], "compiled_scene_file")
        try:
            path.relative_to(directory)
        except ValueError as exc:
            raise ValueError("compiled scene file escaped its frozen directory") from exc
        _require(path.is_file() and _sha256(path) == item["sha256"],
                 "compiled scene file drifted: {}".format(item["path"]))
        suffix = (
            ".instance.yaml" if path.name.endswith(".instance.yaml")
            else ".world" if path.suffix == ".world"
            else None
        )
        _require(suffix is not None, "unexpected compiled scene file type")
        scene_id = path.name[:-len(suffix)]
        _require(scene_id in scene_ids, "compiled file has an unknown scene id")
        seen[(scene_id, suffix)] += 1
        if suffix == ".instance.yaml":
            instance = _load_yaml(path)
            scene = instance.get("scene", {})
            manifest_scene = next(
                row for row in manifest["scenes"] if row["scene_id"] == scene_id
            )
            _require(
                scene.get("scene_id") == scene_id
                and scene.get("seed") == manifest_scene.get("seed")
                and scene.get("family") == manifest_scene.get("family")
                and scene.get("split") == "calibration",
                "compiled instance identity drifted: {}".format(scene_id),
            )
            _require(
                instance == expected_by_id[scene_id],
                "compiled instance content drifted: {}".format(scene_id),
            )
        else:
            expected_world = scene_contract.render_v2_scene_sdf(
                expected_by_id[scene_id]
            )
            _require(
                path.read_text(encoding="utf-8") == expected_world,
                "compiled SDF content drifted: {}".format(scene_id),
            )
    _require(
        seen == Counter(
            (scene_id, suffix)
            for scene_id in scene_ids
            for suffix in (".instance.yaml", ".world")
        ),
        "compiled scene inventory contains duplicates or omissions",
    )


def _verify_scenes(
    root,
    preregistration,
    nav_spec,
    nav_manifest,
    nav_index,
    readiness_spec,
    readiness_manifest,
    readiness_index,
    scene_contract,
):
    for name, document in (
        ("navigation derivation", nav_spec),
        ("readiness derivation", readiness_spec),
    ):
        _verify_safety_boundary(name, document, require_architecture=False)
        _require(
            document.get("status") == "preregistered_design_not_executed",
            "{} status drifted".format(name),
        )
    _require(
        nav_spec.get("single_factor")
        == "fresh_calibration_evidence_for_dynamic_conflict_prediction_horizon"
        and readiness_spec.get("single_factor")
        == "fresh_scene_based_ttc_activation_and_coverage_readiness",
        "scene derivation purpose drifted",
    )
    for name, document in (
        ("navigation scene manifest", nav_manifest),
        ("readiness scene manifest", readiness_manifest),
    ):
        _require(
            document.get("schema_version") == "2.0"
            and document.get("simulation_only") is True
            and document.get("formal_experiment") is False
            and document.get("runtime_ready") is False
            and document.get("real_vehicle_use_forbidden") is True,
            "{} safety boundary drifted".format(name),
        )

    nav_scene_ids = [
        "v2-04g-r5-cruise-s5121",
        "v2-04g-r5-cruise-s5122",
        "v2-04g-r5-cruise-s5123",
        "v2-04g-r5-dynamic-conflict-s5124",
        "v2-04g-r5-dynamic-conflict-s5125",
        "v2-04g-r5-dynamic-clear-s5126",
        "v2-04g-r5-static-s5127",
        "v2-04g-r5-static-s5128",
        "v2-04g-r5-static-s5129",
        "v2-04g-r5-corridor-s5130",
        "v2-04g-r5-corridor-s5131",
        "v2-04g-r5-corridor-s5132",
        "v2-04g-r5-maneuver-s5133",
        "v2-04g-r5-maneuver-s5134",
        "v2-04g-r5-maneuver-s5135",
    ]
    readiness_probe_ids = [
        "v2-04g-r5-readiness-dynamic-conflict-s5111",
        "v2-04g-r5-readiness-dynamic-conflict-s5112",
        "v2-04g-r5-readiness-dynamic-clear-s5113",
    ]
    readiness_support_ids = [
        "v2-04g-r5-readiness-compile-support-cruise-s5114",
        "v2-04g-r5-readiness-compile-support-static-s5115",
        "v2-04g-r5-readiness-compile-support-corridor-s5116",
        "v2-04g-r5-readiness-compile-support-maneuver-s5117",
    ]
    _require(preregistration.get("scene_ids") == nav_scene_ids,
             "preregistered navigation scene identities or order drifted")
    _require(
        preregistration.get("ttc_activation_coverage_readiness", {}).get(
            "execution_scene_ids"
        ) == readiness_probe_ids,
        "preregistered readiness probe identities or order drifted",
    )
    _require(
        preregistration.get("readiness_compile_support_boundary", {}).get("scene_ids")
        == readiness_support_ids,
        "compile-support-only scene identities or order drifted",
    )
    _require(
        [row.get("scene_id") for row in nav_manifest.get("scenes", [])]
        == nav_scene_ids,
        "navigation manifest scene identities or order drifted",
    )
    _require(
        [row.get("scene_id") for row in readiness_manifest.get("scenes", [])]
        == readiness_probe_ids + readiness_support_ids,
        "readiness manifest scene identities or order drifted",
    )
    _require(
        Counter(row.get("family") for row in nav_manifest["scenes"])
        == Counter(NAVIGATION_FAMILIES),
        "navigation scene family distribution drifted",
    )
    _require(
        Counter(row.get("family") for row in readiness_manifest["scenes"])
        == Counter(READINESS_FAMILIES),
        "readiness compiler-support family distribution drifted",
    )
    _verify_derivation(
        root,
        nav_spec,
        nav_manifest,
        EXPECTED_PATHS["scene_manifest"],
        NAVIGATION_SEEDS,
    )
    _verify_derivation(
        root,
        readiness_spec,
        readiness_manifest,
        EXPECTED_PATHS["readiness_scene_manifest"],
        READINESS_PROBE_SEEDS + READINESS_SUPPORT_SEEDS,
    )
    _require(
        readiness_spec.get("execution_probe_seeds") == READINESS_PROBE_SEEDS
        and readiness_spec.get("compile_support_only_seeds")
        == READINESS_SUPPORT_SEEDS,
        "readiness execution/support seed boundary drifted",
    )
    readiness_rows = readiness_spec["scene_derivations"]
    _require(
        [row.get("execution_role") for row in readiness_rows[:3]]
        == ["readiness_probe"] * 3
        and [row.get("execution_role") for row in readiness_rows[3:]]
        == ["compile_support_only_never_execute"] * 4,
        "readiness execution roles drifted",
    )
    _verify_compiled_index(
        root,
        nav_index,
        nav_manifest,
        EXPECTED_PATHS["scene_manifest"],
        15,
        "artifacts/v2/calibration/v2_04g_r5/compiled_scenes",
        scene_contract,
    )
    _verify_compiled_index(
        root,
        readiness_index,
        readiness_manifest,
        EXPECTED_PATHS["readiness_scene_manifest"],
        7,
        "artifacts/v2/calibration/v2_04g_r5/ttc_readiness_compiled_scenes",
        scene_contract,
    )
    return nav_scene_ids, readiness_probe_ids, readiness_support_ids


def _verify_seeds(preregistration, nav_spec, readiness_spec):
    firewall = preregistration.get("seed_firewall", {})
    _require(
        set(firewall) == {
            "ttc_readiness_probe_only_seeds",
            "readiness_compile_support_only_seeds",
            "readiness_derived_seeds",
            "navigation_calibration_seeds",
            "reserved_future_held_out_seeds",
            "previous_validation_seeds_forbidden",
            "all_prior_v2_04g_calibration_and_probe_seeds_forbidden",
            "readiness_probe_support_navigation_and_held_out_sets_pairwise_disjoint",
            "readiness_probe_seeds_used_for_navigation_or_ranking",
            "readiness_compile_support_seeds_executed_or_counted_as_evidence",
            "navigation_seeds_used_before_preregistration",
            "held_out_data_used",
            "held_out_seed_consumption",
        },
        "seed firewall declaration keys drifted",
    )
    _require(
        firewall.get("ttc_readiness_probe_only_seeds") == READINESS_PROBE_SEEDS,
        "readiness probe seed firewall drifted",
    )
    _require(
        firewall.get("readiness_compile_support_only_seeds")
        == READINESS_SUPPORT_SEEDS,
        "readiness compiler-support seed firewall drifted",
    )
    _require(
        firewall.get("navigation_calibration_seeds") == NAVIGATION_SEEDS,
        "navigation calibration seed firewall drifted",
    )
    _require(
        firewall.get("readiness_derived_seeds")
        == READINESS_PROBE_SEEDS + READINESS_SUPPORT_SEEDS,
        "readiness derived-seed inventory drifted",
    )
    _require(
        firewall.get("reserved_future_held_out_seeds") == HELD_OUT_SEEDS,
        "held-out seed reservation drifted",
    )
    probe = set(READINESS_PROBE_SEEDS)
    support = set(READINESS_SUPPORT_SEEDS)
    navigation = set(NAVIGATION_SEEDS)
    held_out = set(HELD_OUT_SEEDS)
    _require(
        len(probe | support | navigation | held_out)
        == len(probe) + len(support) + len(navigation) + len(held_out),
        "fresh readiness/navigation/held-out seed groups overlap",
    )
    _require(
        firewall.get("previous_validation_seeds_forbidden")
        == PREVIOUS_VALIDATION_SEEDS
        and firewall.get(
            "all_prior_v2_04g_calibration_and_probe_seeds_forbidden"
        ) == PRIOR_V2_04G_SEEDS,
        "historical seed firewall drifted",
    )
    prior = set(PRIOR_V2_04G_SEEDS)
    validation = set(PREVIOUS_VALIDATION_SEEDS)
    _require(
        (probe | support | navigation | held_out).isdisjoint(prior | validation),
        "fresh or held-out seeds overlap historical evidence",
    )
    _require(
        firewall.get("held_out_seed_consumption") is False
        and firewall.get("held_out_data_used") is False
        and firewall.get(
            "readiness_compile_support_seeds_executed_or_counted_as_evidence"
        ) is False
        and firewall.get(
            "readiness_probe_seeds_used_for_navigation_or_ranking"
        ) is False
        and firewall.get("navigation_seeds_used_before_preregistration") is False
        and firewall.get(
            "readiness_probe_support_navigation_and_held_out_sets_pairwise_disjoint"
        ) is True,
        "held-out or validation evidence was consumed",
    )
    nav_forbidden = nav_spec.get("forbidden_seed_sets", {})
    readiness_forbidden = readiness_spec.get("forbidden_seed_sets", {})
    _require(
        nav_forbidden.get("prior_validation")
        == firewall.get("previous_validation_seeds_forbidden")
        and nav_forbidden.get("all_prior_v2_04g_calibration_and_probe")
        == firewall.get(
            "all_prior_v2_04g_calibration_and_probe_seeds_forbidden"
        )
        and set(nav_forbidden.get(
            "ttc_activation_coverage_readiness_derived_only", []
        ))
        == probe | support
        and nav_forbidden.get("reserved_future_held_out") == HELD_OUT_SEEDS,
        "navigation derivation seed firewall drifted",
    )
    _require(
        readiness_forbidden.get("prior_validation")
        == firewall.get("previous_validation_seeds_forbidden")
        and readiness_forbidden.get("all_prior_v2_04g_calibration_and_probe")
        == firewall.get(
            "all_prior_v2_04g_calibration_and_probe_seeds_forbidden"
        )
        and readiness_forbidden.get("navigation_calibration_only")
        == NAVIGATION_SEEDS
        and readiness_forbidden.get("reserved_future_held_out") == HELD_OUT_SEEDS,
        "readiness derivation seed firewall drifted",
    )
    return {
        "ttc_readiness_probe_only_seeds": READINESS_PROBE_SEEDS,
        "readiness_compile_support_only_seeds": READINESS_SUPPORT_SEEDS,
        "navigation_calibration_seeds": NAVIGATION_SEEDS,
        "reserved_future_held_out_seeds": HELD_OUT_SEEDS,
    }


def _verify_readiness_schedule(preregistration, readiness_probe_ids):
    block = preregistration.get("ttc_activation_coverage_readiness", {})
    _require(
        set(block) == {
            "required_before_component_ttc_and_navigation",
            "profile_ids",
            "execution_scene_ids",
            "shared_scene_seeds_per_profile",
            "expected_status_order_per_profile",
            "planned_probe_count",
            "attempts_per_identity_max",
            "warmup_timeout_s",
            "measurement_duration_s",
            "minimum_message_count_per_stream",
            "required_consecutive_stable_count",
            "minimum_transaction_activated_fraction",
            "minimum_transaction_valid_fraction",
            "minimum_transaction_join_valid_fraction",
            "maximum_expected_context_hold_count_per_probe",
            "maximum_world_model_sequence_mismatch_count",
            "maximum_world_model_input_join_fault_count",
            "maximum_backend_transaction_fault_count",
            "maximum_unknown_transaction_fault_count",
            "expected_observed_conflict_count_per_profile",
            "expected_no_conflict_count_per_profile",
            "tracker_invalid_count_max_per_profile",
            "all_six_probes_required",
            "schedule",
        },
        "readiness declaration keys drifted",
    )
    _require(
        block.get("required_before_component_ttc_and_navigation") is True
        and block.get("profile_ids") == ELIGIBLE_CANDIDATE_IDS
        and block.get("execution_scene_ids") == readiness_probe_ids
        and block.get("shared_scene_seeds_per_profile") == READINESS_PROBE_SEEDS
        and block.get("expected_status_order_per_profile")
        == [
            "OBSERVED_CONFLICT",
            "OBSERVED_CONFLICT",
            "NO_CONFLICT_IN_HORIZON",
        ]
        and block.get("planned_probe_count") == 6
        and block.get("attempts_per_identity_max") == 1
        and block.get("warmup_timeout_s") == 12.0
        and block.get("measurement_duration_s") == 6.0
        and block.get("minimum_message_count_per_stream") == 20
        and block.get("required_consecutive_stable_count") == 10
        and block.get("minimum_transaction_activated_fraction") == 0.95
        and block.get("minimum_transaction_valid_fraction") == 0.95
        and block.get("minimum_transaction_join_valid_fraction") == 0.95
        and block.get("maximum_expected_context_hold_count_per_probe") == 1
        and block.get("maximum_world_model_sequence_mismatch_count") == 0
        and block.get("maximum_world_model_input_join_fault_count") == 0
        and block.get("maximum_backend_transaction_fault_count") == 0
        and block.get("maximum_unknown_transaction_fault_count") == 0
        and block.get("expected_observed_conflict_count_per_profile") == 2
        and block.get("expected_no_conflict_count_per_profile") == 1
        and block.get("tracker_invalid_count_max_per_profile") == 0
        and block.get("all_six_probes_required") is True,
        "readiness profile, coverage, or attempt boundary drifted",
    )
    rows = block.get("schedule")
    _require(isinstance(rows, list) and len(rows) == 6,
             "readiness schedule must contain exactly six evidence identities")
    expected_statuses = [
        "OBSERVED_CONFLICT",
        "OBSERVED_CONFLICT",
        "NO_CONFLICT_IN_HORIZON",
    ]
    expected = []
    sequence = 1
    for profile_id in ELIGIBLE_CANDIDATE_IDS:
        for scene_id, seed, status in zip(
            readiness_probe_ids, READINESS_PROBE_SEEDS, expected_statuses
        ):
            expected.append({
                "sequence": sequence,
                "identity": "r5-readiness-{}-s{}".format(profile_id, seed),
                "profile_id": profile_id,
                "scene_id": scene_id,
                "seed": seed,
                "expected_status": status,
                "attempt_limit": 1,
            })
            sequence += 1
    _require(rows == expected, "readiness schedule identities or order drifted")
    _require(
        not ({row["seed"] for row in rows} & set(READINESS_SUPPORT_SEEDS)),
        "compile-support-only seed entered the readiness evidence schedule",
    )
    return rows


def _verify_component_schedule(preregistration):
    block = preregistration.get("ttc_component_probe", {})
    _require(
        set(block) == {
            "required_after_readiness_and_before_navigation",
            "implementation",
            "required_status_order",
            "planned_probe_count",
            "attempts_per_identity_max",
            "seed_consumption",
            "schedule",
        },
        "TTC component declaration keys drifted",
    )
    statuses = [
        "OBSERVED_CONFLICT",
        "NO_CONFLICT_IN_HORIZON",
        "TRACKER_INVALID",
    ]
    expected = [
        {
            "sequence": index,
            "identity": identity,
            "expected_status": status,
            "attempt_limit": 1,
        }
        for index, (identity, status) in enumerate(zip(
            (
                "r5-ttc-component-observed-conflict",
                "r5-ttc-component-no-conflict",
                "r5-ttc-component-tracker-invalid",
            ),
            statuses,
        ), start=1)
    ]
    _require(
        block.get("required_after_readiness_and_before_navigation") is True
        and block.get("implementation")
        == "deterministic_ros_free_component_fixture"
        and block.get("required_status_order") == statuses
        and block.get("planned_probe_count") == 3
        and block.get("attempts_per_identity_max") == 1
        and block.get("seed_consumption") == "none"
        and block.get("schedule") == expected,
        "TTC component schedule, status coverage, or seed boundary drifted",
    )
    return expected


def _verify_budget_and_stopping(preregistration, contract):
    budget = preregistration.get("budget", {})
    expected = {
        "activation_readiness_probe_count": 6,
        "attempts_per_readiness_identity_max": 1,
        "ttc_component_probe_count": 3,
        "attempts_per_ttc_component_identity_max": 1,
        "fixed_reference_episode_count": 15,
        "candidate_count": 3,
        "episode_count_per_candidate": 15,
        "planned_navigation_episode_count": 60,
        "attempts_per_navigation_identity_max": 1,
        "compile_support_scene_count_not_evidence": 4,
        "total_evidence_unit_budget": 69,
    }
    for key, value in expected.items():
        _require(
            budget.get(key) == value,
            "preregistered budget drifted at {}".format(key),
        )
    _require(
        budget
        == dict(
            expected,
            budget_arithmetic="6_readiness_plus_3_component_plus_60_navigation",
            terminal_failure_identity_retry_forbidden=True,
            resume_after_any_terminal_failure_forbidden=True,
            preserve_failure_evidence=True,
            budget_expansion_forbidden=True,
        ),
        "preregistered budget contains an undeclared field or value",
    )
    _require(6 + 3 + 60 == budget["total_evidence_unit_budget"],
             "total evidence budget arithmetic drifted")
    _require(
        budget.get("budget_arithmetic")
        == "6_readiness_plus_3_component_plus_60_navigation"
        and budget.get("terminal_failure_identity_retry_forbidden") is True
        and budget.get("resume_after_any_terminal_failure_forbidden") is True
        and budget.get("preserve_failure_evidence") is True
        and budget.get("budget_expansion_forbidden") is True,
        "preregistered retry/budget boundary drifted",
    )
    retry = contract.get("budget_and_retry", {})
    _require(
        retry.get("activation_readiness_attempts_per_identity_max") == 1
        and retry.get("ttc_component_attempts_per_identity_max") == 1
        and retry.get("navigation_attempts_per_identity_max") == 1
        and retry.get("terminal_failure_identity_retry_forbidden") is True
        and retry.get("resume_after_any_terminal_failure_forbidden") is True
        and retry.get("preserve_failure_evidence") is True
        and retry.get("budget_expansion_forbidden") is True,
        "contract retry/stop boundary drifted",
    )
    _require(
        retry.get("readiness_compile_support_scene_execution_forbidden") is True,
        "contract permits compiler-support scene execution",
    )
    stopping = preregistration.get("stopping_rules", {})
    _require(
        stopping.get("readiness_failure")
        == "stop_before_component_ttc_and_navigation_and_preserve"
        and stopping.get("ttc_component_failure")
        == "stop_before_navigation_and_preserve"
        and stopping.get("any_terminal_evidence_failure")
        == "stop_immediately_preserve_identity_and_forbid_resume"
        and stopping.get("failed_identity_retry") == "forbidden"
        and stopping.get("fixed_reference_collision_or_fewer_than_14_successes")
        == "invalidate_split"
        and stopping.get(
            "candidate_collision_incomplete_evidence_or_any_hard_gate_failure"
        ) == "disqualify_candidate"
        and stopping.get("no_candidate_passes_all_hard_gates")
        == "stop_without_winner_held_out_or_v2_05"
        and stopping.get("early_performance_selection") == "forbidden"
        and stopping.get("post_hoc_scene_relabel_or_threshold_change")
        == "forbidden"
        and stopping.get("failure_episode_deletion") == "forbidden"
        and stopping.get("adverse_seed_deletion") == "forbidden"
        and stopping.get("seed_replacement") == "forbidden"
        and stopping.get("budget_expansion") == "forbidden",
        "preregistered failure stopping rules drifted",
    )
    return expected


def _build_navigation_schedule(preregistration, nav_manifest):
    scenes = {row["scene_id"]: row for row in nav_manifest["scenes"]}
    rows = []
    profiles = [("fixed_teb", "fixed_reference")] + [
        ("rule_multi_anchor", candidate_id) for candidate_id in CANDIDATE_IDS
    ]
    for method, profile_id in profiles:
        for scene_id in preregistration["scene_ids"]:
            scene = scenes[scene_id]
            rows.append({
                "sequence": len(rows) + 1,
                "stage": STAGE,
                "split": "calibration",
                "method": method,
                "profile_id": profile_id,
                "scene_id": scene_id,
                "family": scene["family"],
                "seed": scene["seed"],
            })
    _require(len(rows) == 60, "navigation schedule does not contain 60 episodes")
    identities = {
        (row["profile_id"], row["method"], row["scene_id"]) for row in rows
    }
    _require(len(identities) == 60, "navigation schedule has duplicate identities")
    _require(
        [row["profile_id"] for row in rows]
        == sum(([profile_id] * 15 for _, profile_id in profiles), []),
        "navigation schedule block order drifted",
    )
    declared = preregistration.get("navigation_schedule", {})
    _require(
        set(declared) == {
            "method_order",
            "scene_order",
            "exact_cartesian_product_required",
            "planned_episode_count",
            "attempts_per_identity_max",
            "duplicate_identity_count_max",
            "schedule",
        },
        "navigation schedule declaration keys drifted",
    )
    _require(
        declared.get("method_order")
        == ["fixed_reference"] + CANDIDATE_IDS
        and declared.get("scene_order") == preregistration["scene_ids"]
        and declared.get("exact_cartesian_product_required") is True
        and declared.get("planned_episode_count") == 60
        and declared.get("attempts_per_identity_max") == 1
        and declared.get("duplicate_identity_count_max") == 0,
        "navigation schedule declaration drifted",
    )
    expected_declared_rows = [
        {
            "sequence": row["sequence"],
            "method": row["method"],
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "attempt_limit": 1,
        }
        for row in rows
    ]
    _require(
        declared.get("schedule") == expected_declared_rows,
        "exact 60-navigation-identity schedule drifted",
    )
    payload = json.dumps(rows, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return rows, hashlib.sha256(payload).hexdigest()


def _write_atomic_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def validate(workspace, preregistration, audit_output):
    """Validate the frozen R5 design and atomically write one dry-run audit."""
    root = Path(workspace).resolve()
    _require(root.is_dir(), "workspace does not exist")
    prereg_path = Path(preregistration)
    if not prereg_path.is_absolute():
        prereg_path = _inside(root, prereg_path, "preregistration")
    else:
        prereg_path = prereg_path.resolve()
        try:
            prereg_path.relative_to(root)
        except ValueError as exc:
            raise ValueError("preregistration escapes the workspace") from exc
    audit_path = Path(audit_output)
    if not audit_path.is_absolute():
        audit_path = _inside(root, audit_path, "audit_output")
    else:
        audit_path = audit_path.resolve()
        try:
            audit_path.relative_to(root)
        except ValueError as exc:
            raise ValueError("audit output escapes the workspace") from exc
    expected_audit_root = (
        root / "artifacts/v2/calibration/v2_04g_r5"
    ).resolve()
    try:
        audit_path.relative_to(expected_audit_root)
    except ValueError as exc:
        raise ValueError("audit output must stay inside the R5 artifact root") from exc

    prereg = _load_yaml(prereg_path)
    _verify_safety_boundary("preregistration", prereg)
    _require(
        prereg.get("split") == "calibration"
        and prereg.get("status")
        == "preregistered_design_dry_run_only_not_executed",
        "preregistration split or status drifted",
    )
    by_path, verified_resources = _verify_declared_resources(root, prereg)

    documents = {}
    for name, relative in EXPECTED_PATHS.items():
        if name == "candidate_materializer":
            continue
        documents[name] = _load_yaml(root / relative)
    contract = documents["contract"]
    bank = documents["candidate_bank"]
    nav_spec = documents["scene_derivation"]
    readiness_spec = documents["readiness_scene_derivation"]
    nav_manifest = documents["scene_manifest"]
    nav_index = documents["compiled_scene_index"]
    readiness_manifest = documents["readiness_scene_manifest"]
    readiness_index = documents["readiness_compiled_scene_index"]

    _verify_safety_boundary("contract", contract)
    _verify_safety_boundary("candidate bank", bank)
    _verify_contract_frozen_sections(contract)
    _require(
        bank.get("status")
        == "preregistered_calibration_candidates_not_executed",
        "candidate-bank status drifted",
    )
    _require(
        contract.get("current_design_authorization") == EXPECTED_AUTHORIZATION
        and prereg.get("current_design_authorization")
        == EXPECTED_PREREGISTRATION_AUTHORIZATION,
        "design-only authorization boundary drifted",
    )
    _require(
        prereg.get("hard_gates") == contract.get("hard_gates"),
        "preregistered hard gates drifted from the contract",
    )
    dry_run_boundary = prereg.get("dry_run_audit_boundary", {})
    _require(
        dry_run_boundary == {
            "output_path": DEFAULT_AUDIT.as_posix(),
            "ros_process_start_allowed": False,
            "gazebo_process_start_allowed": False,
            "episode_execution_allowed": False,
            "candidate_runtime_persistent_write_allowed": False,
            "progress_or_result_persistent_write_allowed": False,
            "audit_output_write_allowed": True,
        },
        "dry-run side-effect boundary drifted",
    )
    support_boundary = prereg.get("readiness_compile_support_boundary", {})
    _require(
        set(support_boundary) == {
            "purpose",
            "scene_ids",
            "covered_families",
            "dynamic_family_covered_by_execution_probe_scenes",
            "included_in_readiness_schedule",
            "included_in_navigation_schedule",
            "included_in_evidence_budget",
            "included_in_candidate_ranking",
            "execution_forbidden",
        }
        and support_boundary.get("purpose")
        == "satisfy_frozen_five_foundation_family_compiler_contract_only"
        and support_boundary.get("covered_families")
        == ["CRUISE", "STATIC_DENSE", "CORRIDOR", "MANEUVER"]
        and support_boundary.get(
            "dynamic_family_covered_by_execution_probe_scenes"
        ) is True
        and support_boundary.get("included_in_readiness_schedule") is False
        and support_boundary.get("included_in_navigation_schedule") is False
        and support_boundary.get("included_in_evidence_budget") is False
        and support_boundary.get("included_in_candidate_ranking") is False
        and support_boundary.get("execution_forbidden") is True,
        "readiness compiler-support execution boundary drifted",
    )
    _verify_factor(contract, prereg, bank)
    _require(prereg.get("candidate_ids") == CANDIDATE_IDS,
             "preregistered candidate identities or order drifted")
    _require(
        prereg.get("winner_eligible_candidate_ids") == ELIGIBLE_CANDIDATE_IDS,
        "preregistered winner-eligible set drifted",
    )
    expected_candidate_roles = {
        candidate_id: {
            "predicted_ttc_max_s": HORIZONS[candidate_id],
            "role": (
                "frozen_r4_r1_m030_timing_control_not_winner_eligible"
                if candidate_id == CANDIDATE_IDS[0]
                else "ttc_prediction_timing_repair_candidate"
            ),
            "winner_eligible": candidate_id in ELIGIBLE_CANDIDATE_IDS,
        }
        for candidate_id in CANDIDATE_IDS
    }
    _require(
        prereg.get("candidate_roles") == expected_candidate_roles,
        "preregistered candidate roles drifted",
    )
    _require(
        bank.get("selection_boundary") == {
            "select_only_after_complete_fresh_calibration": True,
            "control_candidate_can_be_frozen": False,
            "ttc_readiness_and_navigation_gates_both_required": True,
            "candidate_budget_expansion_forbidden": True,
            "post_hoc_horizon_or_threshold_change_forbidden": True,
            "held_out_validation_used": False,
            "winner_freeze_requires_separate_user_instruction": True,
        },
        "candidate selection/freeze boundary drifted",
    )
    _require(
        prereg.get("post_stage_boundaries") == {
            "use_readiness_probe_or_compile_support_seeds_for_navigation_or_validation":
                False,
            "use_navigation_calibration_seeds_for_future_held_out_validation": False,
            "consume_reserved_held_out_seeds_5001_5010": False,
            "freeze_winner_without_separate_user_instruction": False,
            "start_v2_05": False,
            "start_sac_training": False,
            "connect_or_command_real_vehicle": False,
        },
        "post-stage authorization boundary drifted",
    )
    behavior_differences = _verify_candidates(
        root,
        bank,
        root / EXPECTED_PATHS["candidate_bank"],
        root / EXPECTED_PATHS["candidate_materializer"],
    )
    scene_contract = _load_python_module(
        root / "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
        "v2_04g_r5_scene_contract_audit",
    )
    nav_scene_ids, readiness_probe_ids, support_ids = _verify_scenes(
        root,
        prereg,
        nav_spec,
        nav_manifest,
        nav_index,
        readiness_spec,
        readiness_manifest,
        readiness_index,
        scene_contract,
    )
    seed_firewall = _verify_seeds(prereg, nav_spec, readiness_spec)
    readiness_schedule = _verify_readiness_schedule(prereg, readiness_probe_ids)
    component_schedule = _verify_component_schedule(prereg)
    budget = _verify_budget_and_stopping(prereg, contract)
    schedule, schedule_sha256 = _build_navigation_schedule(prereg, nav_manifest)

    audit = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "dry_run_audit_pass",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "preregistration": {
            "path": prereg_path.relative_to(root).as_posix(),
            "sha256": _sha256(prereg_path),
        },
        "single_changed_factor": {
            "runtime_field": "supervisor.dynamic.predicted_ttc_max_s",
            "candidate_values_s": [5.0, 4.5, 4.0],
            "overlay_release_confirmation_s_frozen": 0.20,
            "evaluator_relative_ttc_horizon_s_frozen": 5.0,
            "materialized_behavior_differences": behavior_differences,
        },
        "frozen_m030_input": {
            "classification": "non_ranking_input_not_system_winner",
            "maneuver_min_obstacle_dist_m": 0.30,
            "maneuver_inflation_dist_m": 0.52,
        },
        "resource_audit": {
            "declared_resource_count": len(verified_resources),
            "all_declared_hashes_match": True,
            "required_resource_paths": {
                name: {
                    "path": relative,
                    "sha256": by_path[relative],
                }
                for name, relative in EXPECTED_PATHS.items()
            },
        },
        "seed_firewall": seed_firewall,
        "readiness_plan": {
            "execution_probe_scene_ids": readiness_probe_ids,
            "compile_support_only_scene_ids_never_execute": support_ids,
            "schedule_identity_count": len(readiness_schedule),
            "schedule_contains_compile_support_only_scene": False,
        },
        "ttc_component_plan": {
            "schedule_identity_count": len(component_schedule),
            "status_order": [
                row["expected_status"] for row in component_schedule
            ],
            "seed_consumption": "none",
        },
        "navigation_plan": {
            "scene_ids": nav_scene_ids,
            "scene_count": 15,
            "scene_count_per_family": NAVIGATION_FAMILIES,
            "schedule_episode_count": len(schedule),
            "duplicate_identity_count": 0,
            "schedule_sha256": schedule_sha256,
        },
        "budget": budget,
        "failure_policy": {
            "attempts_per_identity_max": 1,
            "terminal_failure_identity_retry_forbidden": True,
            "resume_after_any_terminal_failure_forbidden": True,
            "failure_evidence_preserved": True,
        },
        "side_effects": {
            "ros_started": False,
            "gazebo_started": False,
            "navigation_episodes_started": 0,
            "ttc_probe_episodes_started": 0,
            "sac_training_started": False,
            "runtime_configs_persisted": 0,
            "progress_files_persisted": 0,
            "only_persistent_write_is_this_audit": True,
        },
        "authorization_after_audit": {
            "bounded_gazebo_calibration_execution": False,
            "formal_gazebo_batch": False,
            "winner_freeze": False,
            "held_out_validation": False,
            "v2_05": False,
            "sac_training": False,
            "real_vehicle_motion": False,
            "real_vehicle_parameter_write": False,
        },
    }
    _write_atomic_yaml(audit_path, audit)
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument(
        "--preregistration", type=Path, default=DEFAULT_PREREGISTRATION
    )
    parser.add_argument("--audit-output", type=Path, default=DEFAULT_AUDIT)
    args = parser.parse_args()
    audit = validate(args.workspace, args.preregistration, args.audit_output)
    print(
        "{}: {} ({} navigation identities, ROS/Gazebo not started)".format(
            audit["stage"],
            audit["status"],
            audit["navigation_plan"]["schedule_episode_count"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
