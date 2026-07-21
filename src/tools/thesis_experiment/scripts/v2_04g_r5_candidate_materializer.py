#!/usr/bin/env python3
"""Materialize the R5 one-factor Dynamic TTC prediction-horizon sweep."""

import copy
import hashlib
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def _normalized_identity(document, identity_fields):
    normalized = copy.deepcopy(document)
    for field, value in identity_fields.items():
        normalized[field] = value
    return normalized


def materialize_candidates(candidate_path, output_dir):
    bank = _load(candidate_path)
    if not (
        bank.get("schema_version") == "2.0"
        and bank.get("architecture_generation") == "v2"
        and bank.get("stage") == "V2-04G-R5"
        and bank.get("simulation_only") is True
        and bank.get("calibration_only") is True
        and bank.get("formal_result") is False
        and bank.get("runtime_ready") is False
        and bank.get("training_allowed") is False
        and bank.get("real_vehicle_use_forbidden") is True
    ):
        raise ValueError("R5 candidate-bank safety boundary drifted")
    factor = bank.get("single_changed_factor", {})
    if factor != {
        "name": "dynamic_conflict_prediction_horizon_s",
        "runtime_field": "supervisor.dynamic.predicted_ttc_max_s",
        "preregistered_values_s": [5.0, 4.5, 4.0],
        "evaluator_ttc_horizon_s_frozen": 5.0,
        "overlay_release_confirmation_s_frozen": 0.20,
        "all_anchor_values_changed": False,
        "maneuver_clearance_changed": False,
        "mechanism_controller_changed": False,
        "supervisor_fields_other_than_runtime_field_changed": False,
        "transaction_or_join_changed": False,
        "evaluator_or_scene_label_semantics_changed": False,
    }:
        raise ValueError("R5 single-factor declaration drifted")
    frozen = bank.get("frozen_m030_input", {})
    if frozen.get("status") != (
        "demonstrated_clearance_repair_non_ranking_input_not_system_winner"
    ):
        raise ValueError("R5 m030 input was misclassified as a winner")
    for key in ("source_stage_report", "supervisor", "anchor_bank", "mechanism"):
        resource = frozen[key]
        path = WORKSPACE / resource["path"]
        if not path.is_file() or _sha256(path) != resource["sha256"]:
            raise ValueError("R5 frozen m030 resource drifted: {}".format(key))
    base_supervisor = _load(WORKSPACE / frozen["supervisor"]["path"])
    base_anchor = _load(WORKSPACE / frozen["anchor_bank"]["path"])
    base_mechanism = _load(WORKSPACE / frozen["mechanism"]["path"])
    if not (
        base_supervisor["dynamic"]["predicted_ttc_max_s"] == 5.0
        and base_supervisor["transition"]["overlay_release_confirmation_s"] == 0.20
        and base_anchor["anchors"]["anchor_maneuver_forward"]["values"][
            "min_obstacle_dist"] == 0.30
        and base_anchor["anchors"]["anchor_maneuver_reverse"]["values"][
            "min_obstacle_dist"] == 0.30
    ):
        raise ValueError("R5 frozen m030 numeric boundary drifted")

    output = Path(output_dir)
    materialized = {}
    seen = set()
    for row in bank.get("candidates", []):
        candidate_id = row["candidate_id"]
        if candidate_id in seen:
            raise ValueError("duplicate R5 candidate id")
        seen.add(candidate_id)
        horizon = float(row["predicted_ttc_max_s"])
        if horizon not in factor["preregistered_values_s"]:
            raise ValueError("R5 prediction horizon is outside preregistered values")
        supervisor = copy.deepcopy(base_supervisor)
        anchor = copy.deepcopy(base_anchor)
        mechanism = copy.deepcopy(base_mechanism)
        supervisor["profile_id"] = (
            "fam_teb_v2_04g_r5_{}_supervisor".format(candidate_id)
        )
        supervisor["dynamic"]["predicted_ttc_max_s"] = horizon
        anchor["bank_id"] = "fam_teb_v2_04g_r5_{}_anchor_input".format(candidate_id)
        mechanism["profile_id"] = (
            "fam_teb_v2_04g_r5_{}_mechanism".format(candidate_id)
        )

        supervisor_check = _normalized_identity(
            supervisor, {"profile_id": base_supervisor["profile_id"]}
        )
        supervisor_check["dynamic"]["predicted_ttc_max_s"] = 5.0
        if supervisor_check != base_supervisor:
            raise ValueError("R5 materialization changed another supervisor field")
        anchor_check = _normalized_identity(
            anchor, {"bank_id": base_anchor["bank_id"]}
        )
        if anchor_check != base_anchor:
            raise ValueError("R5 materialization changed an Anchor value")
        mechanism_check = _normalized_identity(
            mechanism, {"profile_id": base_mechanism["profile_id"]}
        )
        if mechanism_check != base_mechanism:
            raise ValueError("R5 materialization changed mechanism behavior")

        target = output / candidate_id
        supervisor_path = target / "supervisor.yaml"
        anchor_path = target / "anchor_bank.yaml"
        mechanism_path = target / "mechanism.yaml"
        _write(supervisor_path, supervisor)
        _write(anchor_path, anchor)
        _write(mechanism_path, mechanism)
        materialized[candidate_id] = {
            "supervisor": supervisor_path.resolve(),
            "anchor_bank": anchor_path.resolve(),
            "mechanism": mechanism_path.resolve(),
            "predicted_ttc_max_s": horizon,
            "winner_eligible": bool(row["winner_eligible"]),
        }
    expected = {row["candidate_id"] for row in bank.get("candidates", [])}
    if set(materialized) != expected or len(expected) != 3:
        raise ValueError("R5 candidate materialization incomplete")
    return materialized


if __name__ == "__main__":
    raise SystemExit(
        "Use validate_v2_04g_r5.py for the authorized no-ROS dry-run audit"
    )
