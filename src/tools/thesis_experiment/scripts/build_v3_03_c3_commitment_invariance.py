#!/usr/bin/env python3
"""Build V3-03-C3 pure-offline commitment-invariance evidence."""

import argparse
import copy
import hashlib
import math
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_03_c3_commitment_invariance_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_03_c3_commitment_invariance_preregistration.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_03_c3")
EFFECTIVE_CONTRACT = ROOT / "v3_03_c3_effective_planner_contract.yaml"
REPORT = ROOT / "v3_03_c3_offline_report.yaml"
CLOSURE = ROOT / "v3_03_c3_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_03_c3_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_03_c3_independent_review.yaml"

SOURCE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path(
        "artifacts/v3/planner/v3_03_c2/"
        "v3_03_c2_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c2/"
        "v3_03_c2_terminal_failure_record.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c0/"
        "v3_03_c0_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/core.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c3_commitment_invariance.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c3_commitment_invariance.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_03_c3_commitment_invariance.py"
    ),
)


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    value = yaml.safe_load(read(path))
    core.require(isinstance(value, dict), "YAML root is not mapping")
    return value


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def receipt(path):
    payload = read(path)
    return {
        "path": path.as_posix(),
        "sha256": sha(payload),
        "size_bytes": len(payload),
    }


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        core.require(
            absolute.read_bytes() == payload,
            "refusing overwrite: {}".format(path),
        )
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def effective_contract(contract):
    value = copy.deepcopy(
        load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    )
    value["stage"] = "V3-03-C3"
    value["stage_purpose"] = contract["stage_purpose"]
    value["contract_id"] = "v3_scope_v3_03_c3_effective_planner_1"
    value["status"] = "pure_offline_commitment_invariance_candidate"
    value["hard_verifier"]["control_commitment_horizon_s"] = float(
        contract["single_factor_repair"]["control_commitment_horizon_s"]
    )
    value["authority"] = copy.deepcopy(contract["authority"])
    value["claim_if_pass"] = contract["claim_if_pass"]
    value["forbidden_claims"] = copy.deepcopy(contract["forbidden_claims"])
    return value


def snapshot(scene_id, ego_x, ego_speed, actor_coordinate, actor_speed, angle):
    return {
        "scene_snapshot_id": scene_id,
        "world_model_seq": 1,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {
            "x_m": ego_x,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "speed_mps": ego_speed,
        },
        "actor": {
            "coordinate_m": actor_coordinate,
            "speed_mps": actor_speed,
            "path_angle_deg": angle,
            "monotone_coordinate_certified": True,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


def selected(result):
    selected_id = result["decision"]["selected_candidate_id"]
    evaluation = next(
        item
        for item in result["evaluations"]
        if item["candidate_id"] == selected_id
    )
    candidate = next(
        item
        for item in result["candidate_set"]["candidates"]
        if item["candidate_id"] == selected_id
    )
    return candidate, evaluation


def validate(contract, preregistration):
    core.require(contract["stage"] == "V3-03-C3", "stage differs")
    core.require(
        contract["status"]
        == "pure_offline_preregistered_before_materialization",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-03-C3"
        and preregistration["status"]
        == "frozen_before_offline_commitment_invariance_materialization",
        "preregistration differs",
    )
    factor = contract["single_factor_repair"]
    core.require(
        factor["factor_count"] == 1
        and factor["factor_id"]
        == "control_commitment_braking_forward_invariance"
        and float(factor["control_commitment_horizon_s"]) == 0.25
        and float(factor["minimum_braking_margin_m"]) == 0.05
        and factor["predictor_or_mode_gate_changed"] is False
        and factor["cleared_certificate_changed"] is False
        and factor["candidate_generator_changed"] is False
        and factor["ranker_weights_changed"] is False
        and factor["footprint_or_margin_changed"] is False
        and factor["integration_gates_changed"] is False,
        "single-factor boundary differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    core.require(
        sha(read(Path(contract["upstream"]["planner_contract_path"])))
        == contract["upstream"]["planner_contract_sha256"]
        and sha(read(Path(contract["upstream"]["predictor_path"])))
        == contract["upstream"]["predictor_sha256"],
        "upstream hash differs",
    )


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    planner = effective_contract(contract)
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    failure_snapshot = snapshot(
        "c2_pre_failure_decision_6",
        -1.70,
        0.515,
        -2.1609,
        0.50,
        72.0,
    )
    failure_result = core.plan(failure_snapshot, model, planner)
    failure_candidate, failure_evaluation = selected(failure_result)
    failure_fixture_pass = (
        failure_candidate["candidate_id"] == "certified_stop_v000"
        and failure_evaluation["accepted"] is True
    )
    cases = []
    nonfinite = 0
    for angle in (72.0, 90.0, 108.0):
        for actor_coordinate, actor_speed in (
            (-2.8, 0.20),
            (-2.5, 0.325),
            (-2.2, 1.20),
        ):
            for ego_x_index in range(15):
                ego_x = -3.0 + 0.1 * ego_x_index
                for ego_speed_index in range(7):
                    ego_speed = 0.2 * ego_speed_index
                    value = snapshot(
                        "grid_{}_{}_{}_{}_{}".format(
                            angle,
                            actor_coordinate,
                            actor_speed,
                            ego_x_index,
                            ego_speed_index,
                        ),
                        ego_x,
                        ego_speed,
                        actor_coordinate,
                        actor_speed,
                        angle,
                    )
                    stop = core.generate_candidate_set(
                        value, planner
                    )["candidates"][0]
                    _, current_margin = core._backup_invariant(
                        value, stop, planner
                    )
                    if current_margin < 0.05 - 1e-12:
                        continue
                    first = core.plan(value, model, planner)
                    second = core.plan(value, model, planner)
                    candidate, evaluation = selected(first)
                    endpoint = next(
                        point
                        for point in candidate["points"]
                        if abs(
                            float(point["time_from_start_s"]) - 0.25
                        )
                        <= 1e-12
                    )
                    next_value = copy.deepcopy(value)
                    next_value["ego"]["x_m"] = endpoint["x_m"]
                    next_value["ego"]["speed_mps"] = endpoint[
                        "longitudinal_velocity_mps"
                    ]
                    _, next_margin = core._backup_invariant(
                        next_value, candidate, planner
                    )
                    invariant = (
                        evaluation["accepted"] is True
                        and evaluation["commitment_braking_margin_m"]
                        >= 0.05 - 1e-12
                        and next_margin >= 0.05 - 1e-12
                    )
                    numbers = (
                        evaluation["minimum_clearance_m"],
                        evaluation["braking_margin_m"],
                        evaluation["commitment_braking_margin_m"],
                        evaluation["analytic_score"],
                    )
                    nonfinite += sum(not math.isfinite(float(x)) for x in numbers)
                    cases.append(
                        {
                            "selected_accepted": evaluation["accepted"],
                            "commitment_invariant": invariant,
                            "deterministic": core.canonical_bytes(first)
                            == core.canonical_bytes(second),
                        }
                    )
    cleared = []
    for angle in (72.0, 90.0, 108.0):
        value = snapshot(
            "cleared_{}".format(angle), -2.2, 0.0, 1.35, 0.9, angle
        )
        result = core.plan(value, model, planner)
        candidate, evaluation = selected(result)
        cleared.append(
            candidate["candidate_id"] != "certified_stop_v000"
            and evaluation["accepted"] is True
        )
    metrics = {
        "c2_failure_state_selected_candidate_id": failure_candidate[
            "candidate_id"
        ],
        "c2_failure_state_selected_accepted": failure_evaluation["accepted"],
        "property_case_count": len(cases),
        "safe_state_selected_accepted_rate": sum(
            row["selected_accepted"] for row in cases
        )
        / len(cases),
        "selected_commitment_invariant_rate": sum(
            row["commitment_invariant"] for row in cases
        )
        / len(cases),
        "deterministic_replay_rate": sum(
            row["deterministic"] for row in cases
        )
        / len(cases),
        "cleared_motion_selection_rate": sum(cleared) / len(cleared),
        "nonfinite_output_count": nonfinite,
    }
    thresholds = contract["offline_gates"]
    gates = {
        "c2_failure_state_repair_pass": failure_fixture_pass,
        "safe_state_selected_accepted_pass": metrics[
            "safe_state_selected_accepted_rate"
        ]
        >= float(thresholds["safe_state_selected_accepted_rate_min"]),
        "selected_commitment_invariant_pass": metrics[
            "selected_commitment_invariant_rate"
        ]
        >= float(thresholds["selected_commitment_invariant_rate_min"]),
        "deterministic_replay_pass": metrics["deterministic_replay_rate"]
        >= float(thresholds["deterministic_replay_rate_min"]),
        "cleared_motion_selection_pass": metrics[
            "cleared_motion_selection_rate"
        ]
        >= float(thresholds["cleared_motion_selection_rate_min"]),
        "finite_output_pass": metrics["nonfinite_output_count"]
        <= int(thresholds["nonfinite_output_count_max"]),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C3",
        "report_id": "v3_03_c3_offline_report_1",
        "status": (
            "v3_03_c3_commitment_invariance_offline_pass"
            if all(gates.values())
            else "v3_03_c3_commitment_invariance_offline_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "gates": gates,
        "overall_pass": all(gates.values()),
        "claim": {
            "offline_commitment_invariance_pass": all(gates.values()),
            "segmented_gazebo_integration": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    closure = {
        "schema_version": "1.0",
        "stage": "V3-03-C3",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in SOURCE_RESOURCES],
    }
    outputs = {
        EFFECTIVE_CONTRACT: yaml_bytes(planner),
        REPORT: yaml_bytes(report),
        CLOSURE: yaml_bytes(closure),
    }
    outputs[EVIDENCE_FREEZE] = (
        "\n".join(
            "{}  {}".format(sha(payload), path.as_posix())
            for path, payload in outputs.items()
        )
        + "\n"
    ).encode("utf-8")
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build()
    for path, payload in outputs.items():
        if args.check:
            core.require(
                (W / path).is_file() and read(path) == payload,
                "persisted output differs: {}".format(path),
            )
        else:
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} overall_pass={} report_sha256={}".format(
            report["status"], report["overall_pass"], sha(outputs[REPORT])
        )
    )


if __name__ == "__main__":
    main()
