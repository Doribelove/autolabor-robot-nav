"""Preregistered V2-04C qualification, refinement, and freeze helpers."""

import copy
import hashlib
import math
from pathlib import Path
from statistics import median
from typing import Any, Dict, Mapping, Sequence

import yaml

from teb_mode_manager.action_pipeline import AnchorBank, FeasibleActionDecoder

from .v2_04b_calibration import apply_candidate_overlay
from .v2_scene import canonical_sha256, load_v2_scene_manifest


ANCHORS = (
    "anchor_balanced", "anchor_cruise", "anchor_static_dense",
    "anchor_corridor", "anchor_maneuver_forward", "anchor_maneuver_reverse",
)
FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
TTC_STATUSES = (
    "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID",
)


class V204CError(ValueError):
    """Raised when a V2-04C preregistration or evidence gate drifts."""


def _sha256(path: Any) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _exact(value: Mapping[str, Any], keys: Sequence[str], context: str) -> None:
    if not isinstance(value, dict):
        raise V204CError("{} must be a mapping".format(context))
    missing = sorted(set(keys) - set(value))
    extra = sorted(set(value) - set(keys))
    if missing or extra:
        raise V204CError("{} keys differ; missing={}, extra={}".format(
            context, missing, extra))


def validate_v2_04c_contract(
    contract: Mapping[str, Any], workspace: Any, verify_resources: bool = True,
) -> Mapping[str, Any]:
    _exact(contract, (
        "schema_version", "architecture_generation", "contract_id", "status",
        "formal_experiment", "simulation_only", "runtime_ready", "training_allowed",
        "real_vehicle_use_forbidden", "frozen_inputs", "split_boundary",
        "ttc_semantics", "refinement_design", "hard_gates", "aggregation",
        "freeze_gate", "claims",
    ), "v2_04c_contract")
    if (
        str(contract["schema_version"]) != "2.0"
        or contract["architecture_generation"] != "v2"
        or contract["contract_id"] != "fam_teb_v2_04c_refinement_freeze_1"
        or contract["status"] != "preregistered_simulation_only"
    ):
        raise V204CError("V2-04C identity drifted")
    boundaries = {
        "formal_experiment": False, "simulation_only": True,
        "runtime_ready": False, "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    for key, expected in boundaries.items():
        if contract[key] is not expected:
            raise V204CError("V2-04C boundary {} drifted".format(key))

    root = Path(workspace).resolve()
    inputs = contract["frozen_inputs"]
    expected_inputs = (
        "v2_04b_evidence", "candidate_anchor_bank", "v2_04b_candidate_plan",
        "v2_04b_freeze_assessment", "qualification_scenes", "refinement_scenes",
    )
    _exact(inputs, expected_inputs, "frozen_inputs")
    for name in expected_inputs:
        item = inputs[name]
        _exact(item, ("path", "sha256"), "frozen_inputs." + name)
        path = root / item["path"]
        if verify_resources and (not path.is_file() or _sha256(path) != item["sha256"]):
            raise V204CError("V2-04C resource {} hash drifted".format(name))

    split = contract["split_boundary"]
    _exact(split, (
        "selection_split", "permitted_splits", "forbidden_splits",
        "runtime_scene_labels_allowed", "evaluator_output_is_policy_input",
    ), "split_boundary")
    if (
        split["selection_split"] != "calibration"
        or split["permitted_splits"] != ["calibration"]
        or split["forbidden_splits"] != ["validation", "test_id", "test_ood"]
        or split["runtime_scene_labels_allowed"] is not False
        or split["evaluator_output_is_policy_input"] is not False
    ):
        raise V204CError("V2-04C split boundary drifted")

    ttc = contract["ttc_semantics"]
    _exact(ttc, (
        "interaction_radius_m", "prediction_horizon_s", "statuses",
        "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID",
        "qualification",
    ), "ttc_semantics")
    if tuple(ttc["statuses"]) != TTC_STATUSES or ttc["prediction_horizon_s"] != 5.0:
        raise V204CError("TTC status/horizon drifted")
    qualification = ttc["qualification"]
    _exact(qualification, (
        "candidate_id", "dynamic_overlay", "scene_ids", "episode_count",
        "observed_conflict_fraction_min", "tracker_health_valid_fraction_min_per_episode",
        "collision_count_max", "interface_failure_count_max",
        "successful_episode_count_min", "failure_action",
    ), "ttc_semantics.qualification")
    if (
        qualification["episode_count"] != 5
        or len(qualification["scene_ids"]) != 5
        or qualification["observed_conflict_fraction_min"] != 0.80
        or qualification["failure_action"]
        != "stop_before_refinement_and_preregister_new_contract"
    ):
        raise V204CError("TTC qualification gate drifted")

    design = contract["refinement_design"]
    _exact(design, (
        "method", "generator_relation", "level_order",
        "coordinate_step_fraction_of_domain", "incumbent_plus_joint_candidates_per_anchor",
        "anchor_count", "generated_candidate_count", "replicate_count_per_family",
        "planned_navigation_episode_count", "common_random_numbers",
        "early_stopping_allowed", "incomplete_candidate_selection_allowed",
        "deterministic_derived_feasibility_parameters",
        "incumbents", "factor_coordinates", "anchor_scene_families", "scene_seeds",
        "dynamic_overlay_by_family",
    ), "refinement_design")
    if (
        design["method"] != "resolution_iv_half_fraction_four_factor"
        or design["generator_relation"] != "D_equals_A_times_B_times_C"
        or len(design["level_order"]) != 8
        or any(len(row) != 4 or row[3] != row[0] * row[1] * row[2]
               for row in design["level_order"])
        or design["incumbent_plus_joint_candidates_per_anchor"] != 9
        or design["generated_candidate_count"] != 54
        or design["planned_navigation_episode_count"] != 180
        or design["early_stopping_allowed"] is not False
        or design["incomplete_candidate_selection_allowed"] is not False
        or design["deterministic_derived_feasibility_parameters"] != [
            "max_vel_theta", "inflation_dist", "dynamic_obstacle_inflation_dist"
        ]
    ):
        raise V204CError("V2-04C refinement design drifted")
    for key in ("incumbents", "factor_coordinates", "anchor_scene_families"):
        if tuple(design[key]) != ANCHORS:
            raise V204CError("refinement {} anchor order drifted".format(key))
    if tuple(design["scene_seeds"]) != FAMILIES:
        raise V204CError("refinement family seed order drifted")
    if any(len(design["factor_coordinates"][anchor]) != 4 for anchor in ANCHORS):
        raise V204CError("every Anchor requires four refinement factors")
    if any(len(design["scene_seeds"][family]) != 2 for family in FAMILIES):
        raise V204CError("every family requires two new seeds")

    gates = contract["hard_gates"]
    _exact(gates, (
        "collision_count_max_per_candidate", "interface_failure_count_max_per_candidate",
        "minimum_clearance_m_min", "every_episode_success_required",
        "tracker_invalid_count_max_for_dynamic", "complete_replicates_required",
    ), "hard_gates")
    if gates != {
        "collision_count_max_per_candidate": 0,
        "interface_failure_count_max_per_candidate": 0,
        "minimum_clearance_m_min": 0.25,
        "every_episode_success_required": True,
        "tracker_invalid_count_max_for_dynamic": 0,
        "complete_replicates_required": True,
    }:
        raise V204CError("V2-04C hard gates drifted")

    aggregation = contract["aggregation"]
    _exact(aggregation, (
        "replicate_aggregator", "reference", "metric_regret", "family_regret",
        "balanced_selection_tuple", "single_family_selection_tuple",
        "equality_tolerance", "objective_scales",
    ), "aggregation")
    if (
        aggregation["replicate_aggregator"] != "median"
        or aggregation["family_regret"] != "maximum_metric_regret"
        or tuple(aggregation["objective_scales"]) != FAMILIES
    ):
        raise V204CError("V2-04C aggregation drifted")
    for family, metrics in aggregation["objective_scales"].items():
        for name, spec in metrics.items():
            _exact(spec, ("direction", "scale"), "objective_scales.{}.{}".format(family, name))
            if spec["direction"] not in ("minimize", "maximize") or spec["scale"] <= 0.0:
                raise V204CError("objective direction/scale is invalid")

    freeze = contract["freeze_gate"]
    _exact(freeze, (
        "qualification_must_pass", "completed_navigation_episode_count_required",
        "all_candidate_hashes_required", "all_evaluation_and_trace_hashes_required",
        "all_six_anchor_winners_required", "winner_hard_gate_pass_required",
        "output_bank_path", "output_bank_status", "output_runtime_ready",
        "output_training_allowed", "output_real_vehicle_use_forbidden",
    ), "freeze_gate")
    if (
        freeze["completed_navigation_episode_count_required"] != 180
        or freeze["output_bank_status"] != "calibrated_simulation_frozen"
        or freeze["output_runtime_ready"] is not False
        or freeze["output_training_allowed"] is not False
        or freeze["output_real_vehicle_use_forbidden"] is not True
    ):
        raise V204CError("V2-04C freeze gate drifted")
    return contract


def validate_v2_04c_r2_amendment(
    amendment: Mapping[str, Any], workspace: Any, verify_resources: bool = True,
) -> Mapping[str, Any]:
    """Fail closed on the timeout-only R2 qualification amendment."""

    _exact(amendment, (
        "schema_version", "architecture_generation", "amendment_id", "stage",
        "status", "formal_experiment", "simulation_only", "runtime_ready",
        "training_allowed", "real_vehicle_use_forbidden", "failed_r1_evidence",
        "single_changed_factor", "invariants", "qualification_gate",
        "conditional_refinement_authorization", "claims",
    ), "v2_04c_r2_amendment")
    if (
        str(amendment["schema_version"]) != "2.0"
        or amendment["architecture_generation"] != "v2"
        or amendment["amendment_id"]
        != "fam_teb_v2_04c_ttc_qualification_r2_timeout_only_1"
        or amendment["stage"] != "V2-04C-Q-R2"
        or amendment["status"] != "preregistered_before_retry"
    ):
        raise V204CError("V2-04C R2 identity drifted")
    boundaries = {
        "formal_experiment": False, "simulation_only": True,
        "runtime_ready": False, "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    for key, expected in boundaries.items():
        if amendment[key] is not expected:
            raise V204CError("V2-04C R2 boundary {} drifted".format(key))

    root = Path(workspace).resolve()
    evidence = amendment["failed_r1_evidence"]
    evidence_names = (
        "preregistration", "qualification_plan", "qualification_progress",
        "qualification_assessment", "compiled_scene_index",
    )
    _exact(evidence, evidence_names, "failed_r1_evidence")
    for name in evidence_names:
        _exact(evidence[name], ("path", "sha256"), "failed_r1_evidence." + name)
        path = root / evidence[name]["path"]
        if verify_resources and (not path.is_file() or _sha256(path) != evidence[name]["sha256"]):
            raise V204CError("V2-04C R2 frozen R1 resource {} drifted".format(name))

    factor = amendment["single_changed_factor"]
    _exact(factor, (
        "name", "scope", "r1_value_s", "r2_value_s", "rationale",
    ), "single_changed_factor")
    if (
        factor["name"] != "dynamic_episode_timeout_s"
        or factor["scope"] != "experiment_manager_wall_clock_only"
        or factor["r1_value_s"] != 48.0
        or factor["r2_value_s"] != 80.0
    ):
        raise V204CError("V2-04C R2 is not the preregistered timeout-only retry")

    invariants = amendment["invariants"]
    _exact(invariants, (
        "base_contract", "scene_geometry_unchanged", "actor_timing_unchanged",
        "scene_seeds_unchanged", "balanced_profile_values_unchanged",
        "dynamic_overlay_unchanged", "ttc_semantics_unchanged",
        "qualification_thresholds_unchanged", "common_random_numbers_with_r1",
        "runtime_scene_labels_available_to_policy",
    ), "invariants")
    _exact(invariants["base_contract"], ("path", "sha256"), "invariants.base_contract")
    contract_path = root / invariants["base_contract"]["path"]
    if verify_resources and (
        not contract_path.is_file()
        or _sha256(contract_path) != invariants["base_contract"]["sha256"]
    ):
        raise V204CError("V2-04C R2 base contract drifted")
    invariant_flags = tuple(key for key in invariants if key != "base_contract")
    if any(
        invariants[key] is not (False if key == "runtime_scene_labels_available_to_policy" else True)
        for key in invariant_flags
    ):
        raise V204CError("V2-04C R2 invariant drifted")

    gate = amendment["qualification_gate"]
    _exact(gate, (
        "planned_episode_count", "observed_conflict_fraction_min",
        "tracker_health_valid_fraction_min_per_episode", "tracker_invalid_count_max",
        "successful_episode_count_min", "collision_count_max",
        "interface_failure_count_max", "failure_action",
    ), "qualification_gate")
    if gate != {
        "planned_episode_count": 5,
        "observed_conflict_fraction_min": 0.80,
        "tracker_health_valid_fraction_min_per_episode": 0.95,
        "tracker_invalid_count_max": 0,
        "successful_episode_count_min": 5,
        "collision_count_max": 0,
        "interface_failure_count_max": 0,
        "failure_action": "stop_before_refinement_and_preregister_new_contract",
    }:
        raise V204CError("V2-04C R2 qualification gate drifted")

    authorization = amendment["conditional_refinement_authorization"]
    _exact(authorization, (
        "qualification_must_pass", "refinement_plan", "dynamic_episode_timeout_s",
        "non_dynamic_scene_timeout_unchanged", "planned_navigation_episode_count",
        "early_stopping_allowed", "incomplete_candidate_selection_allowed",
    ), "conditional_refinement_authorization")
    _exact(authorization["refinement_plan"], ("path", "sha256"),
           "conditional_refinement_authorization.refinement_plan")
    refinement_path = root / authorization["refinement_plan"]["path"]
    if verify_resources and (
        not refinement_path.is_file()
        or _sha256(refinement_path) != authorization["refinement_plan"]["sha256"]
    ):
        raise V204CError("V2-04C R2 refinement plan drifted")
    if (
        authorization["qualification_must_pass"] is not True
        or authorization["dynamic_episode_timeout_s"] != 80.0
        or authorization["non_dynamic_scene_timeout_unchanged"] is not True
        or authorization["planned_navigation_episode_count"] != 180
        or authorization["early_stopping_allowed"] is not False
        or authorization["incomplete_candidate_selection_allowed"] is not False
    ):
        raise V204CError("V2-04C R2 refinement authorization drifted")
    return amendment


def build_v2_04c_r2_plan(amendment_path: Any, workspace: Any) -> Dict[str, Any]:
    """Clone the frozen R1 plan while changing only retry identity and timeout authority."""

    root = Path(workspace).resolve()
    source = Path(amendment_path).resolve()
    amendment = yaml.safe_load(source.read_text(encoding="utf-8"))
    validate_v2_04c_r2_amendment(amendment, root, True)
    r1_path = root / amendment["failed_r1_evidence"]["qualification_plan"]["path"]
    plan = yaml.safe_load(r1_path.read_text(encoding="utf-8"))
    if plan["candidate_count"] != 1 or plan["planned_episode_count"] != 5:
        raise V204CError("V2-04C R1 qualification plan drifted")
    retry = copy.deepcopy(plan)
    retry.update({
        "stage": "V2-04C-Q-R2",
        "plan_id": "fam_teb_v2_04c_ttc_qualification_r2_plan_1",
        "contract": {
            "path": source.relative_to(root).as_posix(),
            "sha256": _sha256(source),
        },
    })
    candidate = retry["candidates"][0]
    candidate["candidate_id"] = "v2_04c-q-r2-balanced-center"
    candidate["retry_provenance"] = {
        "r1_candidate_id": plan["candidates"][0]["candidate_id"],
        "single_changed_factor": "dynamic_episode_timeout_s",
        "episode_timeout_s": 80.0,
    }
    retry["claims"] = {
        "qualification_only": True,
        "timeout_only_retry": True,
        "common_random_numbers_with_r1": True,
        "refinement_selection_used": False,
    }
    return retry


def validate_v2_04c_r3_amendment(
    amendment: Mapping[str, Any], workspace: Any, verify_resources: bool = True,
) -> Mapping[str, Any]:
    """Fail closed on the tracker-to-TEB bridge R3 qualification amendment."""

    _exact(amendment, (
        "schema_version", "architecture_generation", "amendment_id", "stage",
        "status", "formal_experiment", "simulation_only", "runtime_ready",
        "training_allowed", "real_vehicle_use_forbidden", "failed_r2_evidence",
        "single_changed_factor", "bridge_resources", "invariants",
        "qualification_gate", "conditional_refinement_authorization", "claims",
    ), "v2_04c_r3_amendment")
    if (
        str(amendment["schema_version"]) != "2.0"
        or amendment["architecture_generation"] != "v2"
        or amendment["amendment_id"]
        != "fam_teb_v2_04c_ttc_qualification_r3_tracker_teb_bridge_1"
        or amendment["stage"] != "V2-04C-Q-R3"
        or amendment["status"] != "preregistered_before_retry"
    ):
        raise V204CError("V2-04C R3 identity drifted")
    for key, expected in {
        "formal_experiment": False, "simulation_only": True,
        "runtime_ready": False, "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }.items():
        if amendment[key] is not expected:
            raise V204CError("V2-04C R3 boundary {} drifted".format(key))

    root = Path(workspace).resolve()
    resource_groups = {
        "failed_r2_evidence": (
            "preregistration", "qualification_plan", "qualification_progress",
            "qualification_assessment",
        ),
        "bridge_resources": (
            "conversion_core", "ros_adapter", "world_model_launch",
            "simulation_launch", "episode_runner", "batch_runner",
        ),
    }
    for group, names in resource_groups.items():
        _exact(amendment[group], names, group)
        for name in names:
            item = amendment[group][name]
            _exact(item, ("path", "sha256"), "{}.{}".format(group, name))
            path = root / item["path"]
            if verify_resources and (not path.is_file() or _sha256(path) != item["sha256"]):
                raise V204CError("V2-04C R3 resource {}.{} drifted".format(group, name))

    factor = amendment["single_changed_factor"]
    _exact(factor, (
        "name", "r2_value", "r3_value", "input", "health_gate", "output",
        "fixed_frame", "invalid_health_behavior", "selected_motion_classes",
        "minimum_track_confidence",
    ), "single_changed_factor")
    if (
        factor["name"] != "healthy_tracker_velocity_bridge_to_teb_custom_obstacles"
        or factor["r2_value"] != "disabled" or factor["r3_value"] != "enabled"
        or factor["fixed_frame"] != "odom"
        or factor["minimum_track_confidence"] != 0.55
        or factor["selected_motion_classes"]
        != ["CROSSING", "HEAD_ON", "FOLLOWING", "DEPARTING"]
    ):
        raise V204CError("V2-04C R3 tracker bridge factor drifted")

    invariants = amendment["invariants"]
    _exact(invariants, (
        "base_contract", "scene_geometry_unchanged", "actor_timing_unchanged",
        "scene_seeds_unchanged", "balanced_profile_values_unchanged",
        "dynamic_overlay_unchanged", "dynamic_episode_timeout_s",
        "ttc_semantics_unchanged", "qualification_thresholds_unchanged",
        "common_random_numbers_with_r1_r2", "runtime_scene_labels_available_to_policy",
    ), "invariants")
    _exact(invariants["base_contract"], ("path", "sha256"), "invariants.base_contract")
    contract_path = root / invariants["base_contract"]["path"]
    if verify_resources and (
        not contract_path.is_file()
        or _sha256(contract_path) != invariants["base_contract"]["sha256"]
    ):
        raise V204CError("V2-04C R3 base contract drifted")
    for key in (
        "scene_geometry_unchanged", "actor_timing_unchanged", "scene_seeds_unchanged",
        "balanced_profile_values_unchanged", "dynamic_overlay_unchanged",
        "ttc_semantics_unchanged", "qualification_thresholds_unchanged",
        "common_random_numbers_with_r1_r2",
    ):
        if invariants[key] is not True:
            raise V204CError("V2-04C R3 invariant {} drifted".format(key))
    if (
        invariants["dynamic_episode_timeout_s"] != 80.0
        or invariants["runtime_scene_labels_available_to_policy"] is not False
    ):
        raise V204CError("V2-04C R3 timeout/policy boundary drifted")

    expected_gate = {
        "planned_episode_count": 5,
        "observed_conflict_fraction_min": 0.80,
        "tracker_health_valid_fraction_min_per_episode": 0.95,
        "tracker_invalid_count_max": 0,
        "successful_episode_count_min": 5,
        "collision_count_max": 0,
        "interface_failure_count_max": 0,
        "failure_action": "stop_before_refinement_and_preregister_new_contract",
    }
    if amendment["qualification_gate"] != expected_gate:
        raise V204CError("V2-04C R3 qualification gate drifted")
    authorization = amendment["conditional_refinement_authorization"]
    _exact(authorization, (
        "qualification_must_pass", "refinement_plan", "tracker_velocity_bridge_enabled",
        "dynamic_episode_timeout_s", "non_dynamic_scene_timeout_unchanged",
        "planned_navigation_episode_count", "early_stopping_allowed",
        "incomplete_candidate_selection_allowed",
    ), "conditional_refinement_authorization")
    plan_item = authorization["refinement_plan"]
    _exact(plan_item, ("path", "sha256"), "conditional_refinement_authorization.refinement_plan")
    plan_path = root / plan_item["path"]
    if verify_resources and (not plan_path.is_file() or _sha256(plan_path) != plan_item["sha256"]):
        raise V204CError("V2-04C R3 refinement plan drifted")
    if (
        authorization["qualification_must_pass"] is not True
        or authorization["tracker_velocity_bridge_enabled"] is not True
        or authorization["dynamic_episode_timeout_s"] != 80.0
        or authorization["non_dynamic_scene_timeout_unchanged"] is not True
        or authorization["planned_navigation_episode_count"] != 180
        or authorization["early_stopping_allowed"] is not False
        or authorization["incomplete_candidate_selection_allowed"] is not False
    ):
        raise V204CError("V2-04C R3 refinement authorization drifted")
    return amendment


def build_v2_04c_r3_plan(amendment_path: Any, workspace: Any) -> Dict[str, Any]:
    root = Path(workspace).resolve()
    source = Path(amendment_path).resolve()
    amendment = yaml.safe_load(source.read_text(encoding="utf-8"))
    validate_v2_04c_r3_amendment(amendment, root, True)
    r2_path = root / amendment["failed_r2_evidence"]["qualification_plan"]["path"]
    plan = yaml.safe_load(r2_path.read_text(encoding="utf-8"))
    retry = copy.deepcopy(plan)
    retry.update({
        "stage": "V2-04C-Q-R3",
        "plan_id": "fam_teb_v2_04c_ttc_qualification_r3_plan_1",
        "contract": {
            "path": source.relative_to(root).as_posix(), "sha256": _sha256(source),
        },
    })
    candidate = retry["candidates"][0]
    candidate["candidate_id"] = "v2_04c-q-r3-balanced-center"
    candidate["retry_provenance"] = {
        "r2_candidate_id": plan["candidates"][0]["candidate_id"],
        "single_changed_factor": "healthy_tracker_velocity_bridge_to_teb_custom_obstacles",
        "episode_timeout_s": 80.0,
    }
    retry["claims"] = {
        "qualification_only": True,
        "tracker_to_teb_bridge_only_retry": True,
        "common_random_numbers_with_r1_r2": True,
        "refinement_selection_used": False,
    }
    return retry


def _bounded_levels(base: float, lower: float, upper: float, fraction: float) -> Dict[int, float]:
    step = fraction * (upper - lower)
    low = max(lower, base - step)
    high = min(upper, base + step)
    if low == base:
        low = min(upper, base + 2.0 * step)
    if high == base:
        high = max(lower, base - 2.0 * step)
    values = sorted((float(low), float(high)))
    if len(set(values + [float(base)])) != 3:
        raise V204CError("cannot construct distinct refinement levels")
    return {-1: values[0], 1: values[1]}


def _candidate_lookup(plan: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    return {row["candidate_id"]: row for row in plan["candidates"]}


def build_v2_04c_plans(contract_path: Any, workspace: Any) -> Dict[str, Dict[str, Any]]:
    root = Path(workspace).resolve()
    source = Path(contract_path).resolve()
    contract = yaml.safe_load(source.read_text(encoding="utf-8"))
    validate_v2_04c_contract(contract, root, True)
    inputs = contract["frozen_inputs"]
    bank_path = root / inputs["candidate_anchor_bank"]["path"]
    bank = AnchorBank.from_file(bank_path)
    previous = yaml.safe_load((root / inputs["v2_04b_candidate_plan"]["path"]).read_text())
    prior = _candidate_lookup(previous)
    design = contract["refinement_design"]

    qualification_manifest = load_v2_scene_manifest(
        root / inputs["qualification_scenes"]["path"], root)
    q_scenes = {row["scene_id"]: row for row in qualification_manifest["scenes"]}
    q_base = prior[contract["ttc_semantics"]["qualification"]["candidate_id"]]
    q_values = bank.validate_values(q_base["values"], "V2-04C qualification")
    q_evaluations = []
    for scene_id in contract["ttc_semantics"]["qualification"]["scene_ids"]:
        scene = q_scenes[scene_id]
        effective = apply_candidate_overlay(bank, q_values, "CROSSING")
        q_evaluations.append({
            "scene_id": scene_id, "split": "calibration", "seed": scene["seed"],
            "family": "DYNAMIC", "dynamic_overlay": "CROSSING",
            "effective_profile_sha256": canonical_sha256(effective),
        })
    qualification_plan = {
        "schema_version": "2.0", "stage": "V2-04C-Q",
        "plan_id": "fam_teb_v2_04c_ttc_qualification_plan_1",
        "status": "preregistered", "formal_result": False,
        "simulation_only": True, "runtime_ready": False, "training_started": False,
        "test_or_validation_selection_used": False,
        "contract": {"path": source.relative_to(root).as_posix(), "sha256": _sha256(source)},
        "anchor_bank": dict(inputs["candidate_anchor_bank"]),
        "scene_manifest": dict(inputs["qualification_scenes"]),
        "candidate_count": 1, "planned_episode_count": len(q_evaluations),
        "completed_navigation_episode_count": 0,
        "candidates": [{
            "candidate_id": "v2_04c-q-balanced-center",
            "anchor_id": "anchor_balanced", "base_profile_id": q_base["base_profile_id"],
            "screen_coordinate": None, "screen_level": 0,
            "values": q_values, "profile_sha256": canonical_sha256(q_values),
            "evaluations": q_evaluations,
        }],
        "claims": {"qualification_only": True, "refinement_selection_used": False},
    }

    refinement_manifest = load_v2_scene_manifest(
        root / inputs["refinement_scenes"]["path"], root)
    scenes_by_family = {family: [] for family in FAMILIES}
    for scene in refinement_manifest["scenes"]:
        scenes_by_family[scene["family"]].append(scene)
    for family in FAMILIES:
        expected_seeds = design["scene_seeds"][family]
        if [row["seed"] for row in scenes_by_family[family]] != expected_seeds:
            raise V204CError("refinement scene seeds drifted for {}".format(family))

    decoder = FeasibleActionDecoder(bank)
    candidates = []
    planned = 0
    for anchor_id in ANCHORS:
        incumbent_source = prior[design["incumbents"][anchor_id]]
        incumbent = bank.validate_values(incumbent_source["values"], "V2-04C incumbent")
        coordinates = design["factor_coordinates"][anchor_id]
        levels = {}
        for name in coordinates:
            definition = bank.definitions[name]
            levels[name] = _bounded_levels(
                float(incumbent[name]), definition.lower, definition.upper,
                float(design["coordinate_step_fraction_of_domain"]),
            )
        rows = [("incumbent", None, dict(incumbent))]
        for index, pattern in enumerate(design["level_order"], 1):
            values = dict(incumbent)
            for name, level in zip(coordinates, pattern):
                values[name] = levels[name][level]
            feasible, reason = decoder._intrinsic_feasible(values, None)
            if reason != 0:
                raise V204CError("joint candidate requires projection")
            label = "hfff-{}".format("".join("p" if value > 0 else "m" for value in pattern))
            rows.append((label, pattern, feasible))
        for local_index, (label, pattern, values) in enumerate(rows):
            validated = bank.validate_values(values, "V2-04C candidate")
            evaluations = []
            for family in design["anchor_scene_families"][anchor_id]:
                overlay = design["dynamic_overlay_by_family"][family]
                effective = apply_candidate_overlay(bank, validated, overlay)
                for scene in scenes_by_family[family]:
                    evaluations.append({
                        "scene_id": scene["scene_id"], "split": "calibration",
                        "seed": scene["seed"], "family": family,
                        "dynamic_overlay": overlay,
                        "effective_profile_sha256": canonical_sha256(effective),
                    })
            candidate_id = "{}-rc{:02d}-{}".format(anchor_id, local_index, label)
            candidates.append({
                "candidate_id": candidate_id, "anchor_id": anchor_id,
                "base_profile_id": incumbent_source["base_profile_id"],
                "design_role": "incumbent" if local_index == 0 else "joint_fractional_factor",
                "screen_coordinate": None if local_index == 0 else "joint_fractional_factor",
                "screen_level": 0 if local_index == 0 else 1,
                "factor_pattern": pattern,
                "derived_parameter_changes": [
                    name for name in validated
                    if validated[name] != incumbent[name] and name not in coordinates
                ],
                "values": validated,
                "profile_sha256": canonical_sha256(validated), "evaluations": evaluations,
            })
            planned += len(evaluations)
    if len(candidates) != 54 or planned != 180:
        raise V204CError("V2-04C candidate/episode budget drifted")
    refinement_plan = {
        "schema_version": "2.0", "stage": "V2-04C",
        "plan_id": "fam_teb_v2_04c_refinement_plan_1", "status": "preregistered",
        "formal_result": False, "simulation_only": True, "runtime_ready": False,
        "training_started": False, "test_or_validation_selection_used": False,
        "contract": {"path": source.relative_to(root).as_posix(), "sha256": _sha256(source)},
        "anchor_bank": dict(inputs["candidate_anchor_bank"]),
        "scene_manifest": dict(inputs["refinement_scenes"]),
        "strategy": design["method"], "candidate_count": len(candidates),
        "planned_episode_count": planned, "completed_navigation_episode_count": 0,
        "candidates": candidates,
        "claims": {
            "joint_refinement_preregistered": True, "qualification_must_pass": True,
            "anchor_values_frozen": False, "validation_or_test_selection_used": False,
        },
    }
    return {"qualification": qualification_plan, "refinement": refinement_plan}


def write_plan(plan: Mapping[str, Any], path: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(dict(plan), sort_keys=False), encoding="utf-8")


def assess_ttc_qualification(
    contract: Mapping[str, Any], progress: Mapping[str, Any], progress_path: Any,
) -> Dict[str, Any]:
    qualification = contract["ttc_semantics"]["qualification"]
    if progress["valid_evidence_episode_count"] != qualification["episode_count"]:
        raise V204CError("TTC qualification evidence is incomplete")
    statuses = []
    episodes = []
    for row in progress["episodes"]:
        path = Path(row["evaluation"])
        if _sha256(path) != row["evaluation_sha256"]:
            raise V204CError("TTC qualification evaluation hash drifted")
        evaluation = yaml.safe_load(path.read_text(encoding="utf-8"))
        status = evaluation.get("ttc_status", "TRACKER_INVALID")
        if status not in TTC_STATUSES:
            raise V204CError("unknown TTC status")
        statuses.append(status)
        episodes.append({
            "scene_id": row["scene_id"], "status": status,
            "tracker_health_valid_fraction": evaluation.get("tracker_health_valid_fraction", 0.0),
            "finite_ttc_sample_count": evaluation.get("finite_ttc_sample_count", 0),
            "minimum_predicted_ttc_s": evaluation["metrics"]["family"]["minimum_predicted_ttc_s"],
            "success": evaluation["metrics"]["common"]["success"],
            "collision": evaluation["metrics"]["common"]["collision"],
        })
    observed = statuses.count("OBSERVED_CONFLICT")
    tracker_invalid = statuses.count("TRACKER_INVALID")
    coverage_ok = all(
        item["tracker_health_valid_fraction"]
        >= qualification["tracker_health_valid_fraction_min_per_episode"]
        for item in episodes
    )
    passed = bool(
        observed / len(statuses) >= qualification["observed_conflict_fraction_min"]
        and tracker_invalid == 0 and coverage_ok
        and sum(item["collision"] for item in episodes) <= qualification["collision_count_max"]
        and sum(item["success"] for item in episodes)
        >= qualification["successful_episode_count_min"]
        and progress["interface_failure_count"] <= qualification["interface_failure_count_max"]
    )
    return {
        "schema_version": "2.0", "stage": "V2-04C-Q",
        "status": "qualification_passed" if passed else "qualification_failed",
        "formal_result": False, "simulation_only": True, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "progress_sha256": _sha256(progress_path), "episode_count": len(episodes),
        "observed_conflict_count": observed,
        "observed_conflict_fraction": observed / len(episodes),
        "tracker_invalid_count": tracker_invalid,
        "tracker_health_coverage_gate_passed": coverage_ok,
        "episodes": episodes,
        "decision": {
            "start_refinement": passed,
            "failure_action": None if passed else qualification["failure_action"],
        },
        "claims": {"anchor_values_frozen": False, "sac_training_started": False},
    }


def _metric(evaluation: Mapping[str, Any], name: str, horizon: float) -> float:
    if name == "ttc_objective_value_s":
        status = evaluation.get("ttc_status", "TRACKER_INVALID")
        if status == "TRACKER_INVALID":
            raise V204CError("tracker-invalid episode cannot be scored")
        if status == "NO_CONFLICT_IN_HORIZON":
            return horizon
        value = evaluation["metrics"]["family"]["minimum_predicted_ttc_s"]
        if value is None:
            raise V204CError("observed-conflict episode lacks finite TTC")
        return float(value)
    for section in ("common", "family"):
        if name in evaluation["metrics"][section]:
            return float(evaluation["metrics"][section][name])
    raise V204CError("objective metric {} is missing".format(name))


def freeze_v2_04c(
    contract: Mapping[str, Any], plan: Mapping[str, Any], progress: Mapping[str, Any],
    qualification_report: Mapping[str, Any], source_bank_path: Any,
) -> Dict[str, Any]:
    if qualification_report["decision"]["start_refinement"] is not True:
        raise V204CError("qualification did not authorize refinement freeze")
    if (
        progress["valid_evidence_episode_count"] != 180
        or progress["interface_failure_count"] != 0
        or len(progress["episodes"]) != 180
    ):
        raise V204CError("refinement evidence is incomplete or has interface failures")
    candidate_map = {row["candidate_id"]: row for row in plan["candidates"]}
    evidence = {}
    for row in progress["episodes"]:
        path = Path(row["evaluation"])
        if _sha256(path) != row["evaluation_sha256"]:
            raise V204CError("refinement evaluation hash drifted")
        evaluation = yaml.safe_load(path.read_text(encoding="utf-8"))
        evidence[(row["candidate_id"], row["scene_id"])] = evaluation

    gates = contract["hard_gates"]
    scales = contract["aggregation"]["objective_scales"]
    horizon = float(contract["ttc_semantics"]["prediction_horizon_s"])
    summaries = {}
    for candidate_id, candidate in candidate_map.items():
        family_evaluations = {}
        hard_pass = True
        for expected in candidate["evaluations"]:
            evaluation = evidence.get((candidate_id, expected["scene_id"]))
            if evaluation is None:
                hard_pass = False
                continue
            common = evaluation["metrics"]["common"]
            if (
                not common["success"] or common["collision"]
                or common["minimum_clearance_m"] < gates["minimum_clearance_m_min"]
                or evaluation.get("runner_fault_reason", "")
                or evaluation.get("typed_startup_snapshot_restored") is not True
                or (expected["family"] == "DYNAMIC"
                    and evaluation.get("ttc_status") == "TRACKER_INVALID")
            ):
                hard_pass = False
            family_evaluations.setdefault(expected["family"], []).append(evaluation)
        aggregate = {}
        for family, rows in family_evaluations.items():
            if len(rows) != 2:
                hard_pass = False
                continue
            aggregate[family] = {
                name: median(_metric(row, name, horizon) for row in rows)
                for name in scales[family]
            }
        summaries[candidate_id] = {
            "candidate": candidate, "hard_gate_pass": hard_pass,
            "aggregate": aggregate,
        }

    design = contract["refinement_design"]
    references = {}
    for anchor_id in ANCHORS:
        incumbent = next(
            item for item in summaries.values()
            if item["candidate"]["anchor_id"] == anchor_id
            and item["candidate"]["design_role"] == "incumbent"
        )
        if not incumbent["hard_gate_pass"]:
            raise V204CError("incumbent failed hard gate for {}".format(anchor_id))
        references[anchor_id] = incumbent["aggregate"]

    winners = {}
    ranking = {}
    for anchor_id in ANCHORS:
        candidates = []
        for candidate_id, summary in summaries.items():
            candidate = summary["candidate"]
            if candidate["anchor_id"] != anchor_id or not summary["hard_gate_pass"]:
                continue
            family_regrets = {}
            original = []
            navigation_sum = 0.0
            for family, metrics in summary["aggregate"].items():
                regrets = []
                for name, spec in scales[family].items():
                    value = metrics[name]
                    reference = references[anchor_id][family][name]
                    regret = ((value - reference) / spec["scale"]
                              if spec["direction"] == "minimize"
                              else (reference - value) / spec["scale"])
                    regrets.append(regret)
                    original.append(value if spec["direction"] == "minimize" else -value)
                family_regrets[family] = max(regrets)
                navigation_sum += metrics.get("navigation_time_s", 0.0)
            if anchor_id == "anchor_balanced":
                score = (
                    max(family_regrets.values()),
                    sum(family_regrets.values()) / len(family_regrets),
                    navigation_sum,
                    candidate_id,
                )
            else:
                score = (next(iter(family_regrets.values())), tuple(original), candidate_id)
            candidates.append((score, candidate_id, family_regrets, summary["aggregate"]))
        if not candidates:
            raise V204CError("no hard-gate candidate for {}".format(anchor_id))
        candidates.sort(key=lambda item: item[0])
        score, winner_id, family_regrets, aggregate = candidates[0]
        winners[anchor_id] = candidate_map[winner_id]
        ranking[anchor_id] = {
            "winner_candidate_id": winner_id,
            "winner_profile_sha256": candidate_map[winner_id]["profile_sha256"],
            "selection_score": list(score[:-1]),
            "family_regrets": family_regrets, "aggregate_metrics": aggregate,
            "hard_gate_candidate_count": len(candidates),
        }

    source_path = Path(source_bank_path)
    bank_data = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    bank_data["bank_id"] = "fam_teb_v2_04c_anchor_bank_frozen_1"
    bank_data["status"] = "calibrated_simulation_frozen"
    bank_data["source_provenance"] = {
        "baseline": "v2_04b_screen_plus_v2_04c_independent_refinement",
        "mode_deltas": "preregistered_resolution_iv_half_fraction_and_minimax_regret",
        "formal_test_scenes_used": False,
    }
    for anchor_id, winner in winners.items():
        bank_data["anchors"][anchor_id]["values"] = dict(winner["values"])
        bank_data["anchors"][anchor_id]["profile_id"] = (
            "profile_{}_v2_04c_frozen".format(anchor_id[len("anchor_"):])
        )
    # Construction must still satisfy every typed and intrinsic feasibility rule.
    frozen_for_validation = copy.deepcopy(bank_data)
    frozen_for_validation["status"] = "uncalibrated_simulation_candidate"
    AnchorBank(frozen_for_validation)
    return {
        "bank": bank_data,
        "report": {
            "schema_version": "2.0", "stage": "V2-04C",
            "status": "anchor_bank_frozen", "formal_result": False,
            "simulation_only": True, "runtime_ready": False,
            "training_started": False, "real_vehicle_used": False,
            "planned_navigation_episode_count": 180,
            "valid_evidence_episode_count": 180,
            "successful_episode_count": progress["successful_episode_count"],
            "hard_gate_pass_episode_count": progress["hard_gate_pass_episode_count"],
            "interface_failure_count": progress["interface_failure_count"],
            "winners": ranking,
            "claims": {
                "qualification_passed": True, "anchor_values_frozen": True,
                "validation_or_test_selection_used": False,
                "sac_training_started": False, "real_vehicle_validated": False,
            },
        },
    }


def write_frozen_outputs(result: Mapping[str, Any], bank_path: Any, report_path: Any) -> None:
    bank_destination = Path(bank_path)
    report_destination = Path(report_path)
    bank_destination.parent.mkdir(parents=True, exist_ok=True)
    report_destination.parent.mkdir(parents=True, exist_ok=True)
    bank_text = yaml.safe_dump(result["bank"], sort_keys=False)
    bank_destination.write_text(bank_text, encoding="utf-8")
    report = copy.deepcopy(result["report"])
    report["frozen_anchor_bank"] = {
        "path": str(bank_destination),
        "sha256": hashlib.sha256(bank_text.encode("utf-8")).hexdigest(),
    }
    report_destination.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
