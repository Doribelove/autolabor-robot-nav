#!/usr/bin/env python3
"""Materialize the R4-R1 one-factor Maneuver clearance sweep."""

import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R2_BATCH = Path(__file__).with_name("v2_04g_r2_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r2_frozen_materializer_for_r4_r1", R2_BATCH
)
_R2 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R2)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _write(path, value):
    _R2._R1._write_yaml(Path(path), value)


def _assert_single_factor(base_anchor, candidate_anchor, value):
    expected = copy.deepcopy(base_anchor)
    expected["status"] = candidate_anchor["status"]
    expected["bank_id"] = candidate_anchor["bank_id"]
    expected["source_provenance"] = candidate_anchor["source_provenance"]
    for anchor_id in ("anchor_maneuver_forward", "anchor_maneuver_reverse"):
        expected["anchors"][anchor_id]["values"]["min_obstacle_dist"] = value
    if candidate_anchor != expected:
        raise ValueError("R4-R1 materialization changed more than Maneuver min_obstacle_dist")


def materialize_candidates(candidate_path, output_dir):
    bank = _load(candidate_path)
    if not (
        bank.get("stage") == "V2-04G-R4-R1"
        and bank.get("simulation_only") is True
        and bank.get("runtime_ready") is False
        and bank.get("training_allowed") is False
        and bank.get("real_vehicle_use_forbidden") is True
    ):
        raise ValueError("R4-R1 candidate bank boundary drifted")
    factor = bank["single_changed_factor"]
    if not (
        factor.get("name") == "maneuver_anchor_min_obstacle_dist_m"
        and factor.get("frozen_inflation_dist_m") == 0.52
        and factor.get("minimum_inflation_gap_m") == 0.20
        and factor.get("maneuver_speed_or_time_values_changed") is False
        and factor.get("maneuver_reverse_state_machine_changed") is False
        and factor.get("supervisor_or_transition_values_changed") is False
    ):
        raise ValueError("R4-R1 single-factor declaration drifted")
    base_spec = bank["base_aggressive_candidate"]
    base_bank_path = WORKSPACE / base_spec["candidate_bank_path"]
    if _sha256(base_bank_path) != base_spec["candidate_bank_sha256"]:
        raise ValueError("R4-R1 frozen R2 bank hash drifted")
    diagnosis = bank["frozen_r4_diagnosis"]
    diagnosis_path = WORKSPACE / diagnosis["stage_report_path"]
    if _sha256(diagnosis_path) != diagnosis["stage_report_sha256"]:
        raise ValueError("R4-R1 frozen R4 diagnosis drifted")
    with tempfile.TemporaryDirectory() as directory:
        base_runtime = _R2.materialize_candidates(base_bank_path, directory)[
            base_spec["candidate_id"]
        ]
        base_supervisor = _load(base_runtime["supervisor"])
        base_anchor = _load(base_runtime["anchor_bank"])
        base_mechanism = _load(base_runtime["mechanism"])
    for anchor_id in ("anchor_maneuver_forward", "anchor_maneuver_reverse"):
        values = base_anchor["anchors"][anchor_id]["values"]
        if not (
            values["min_obstacle_dist"] == base_spec["maneuver_min_obstacle_dist_m"]
            and values["inflation_dist"] == base_spec["maneuver_inflation_dist_m"]
        ):
            raise ValueError("R4-R1 aggressive base Maneuver envelope drifted")
    output = Path(output_dir)
    materialized = {}
    seen = set()
    for row in bank["candidates"]:
        candidate_id = row["candidate_id"]
        if candidate_id in seen:
            raise ValueError("duplicate R4-R1 candidate id")
        seen.add(candidate_id)
        value = float(row["maneuver_min_obstacle_dist_m"])
        if value not in (0.28, 0.30, 0.32):
            raise ValueError("R4-R1 clearance value is outside preregistered sweep")
        if base_spec["maneuver_inflation_dist_m"] - value < (
            factor["minimum_inflation_gap_m"] - 1e-12
        ):
            raise ValueError("R4-R1 Maneuver inflation gap is infeasible")
        supervisor = copy.deepcopy(base_supervisor)
        anchor = copy.deepcopy(base_anchor)
        mechanism = copy.deepcopy(base_mechanism)
        supervisor["profile_id"] = "fam_teb_v2_04g_r4_r1_{}_supervisor".format(candidate_id)
        anchor["status"] = "uncalibrated_simulation_candidate"
        anchor["bank_id"] = "fam_teb_v2_04g_r4_r1_{}_anchor_candidate".format(candidate_id)
        anchor["source_provenance"]["mode_deltas"] = (
            "v2_04g_r4_r1_single_factor_maneuver_clearance"
        )
        for anchor_id in ("anchor_maneuver_forward", "anchor_maneuver_reverse"):
            anchor["anchors"][anchor_id]["values"]["min_obstacle_dist"] = value
        mechanism["profile_id"] = "fam_teb_v2_04g_r4_r1_{}_mechanism".format(candidate_id)
        _assert_single_factor(base_anchor, anchor, value)
        # Supervisor and mechanism identity metadata may change; behavior must not.
        supervisor_check = copy.deepcopy(supervisor)
        supervisor_check["profile_id"] = base_supervisor["profile_id"]
        if supervisor_check != base_supervisor:
            raise ValueError("R4-R1 supervisor behavior drifted")
        mechanism_check = copy.deepcopy(mechanism)
        mechanism_check["profile_id"] = base_mechanism["profile_id"]
        if mechanism_check != base_mechanism:
            raise ValueError("R4-R1 maneuver state machine drifted")
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
            "mechanism": str(mechanism_path.resolve()),
        }
    if set(materialized) != {row["candidate_id"] for row in bank["candidates"]}:
        raise ValueError("R4-R1 candidate materialization incomplete")
    return materialized
