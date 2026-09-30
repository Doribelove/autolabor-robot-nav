#!/usr/bin/env python3
"""Build V3-03-C0 pure-offline closed-loop recovery evidence."""

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
    "v3_03_c0_closed_loop_recovery_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_03_c0_closed_loop_recovery_preregistration.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_03_c0")
EFFECTIVE_CONTRACT = ROOT / "v3_03_c0_effective_planner_contract.yaml"
REPORT = ROOT / "v3_03_c0_offline_report.yaml"
CLOSURE = ROOT / "v3_03_c0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_03_c0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_03_c0_independent_review.yaml"

SOURCE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path(
        "artifacts/v3/planner/v3_03_b0/"
        "v3_03_b0_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_b0/"
        "v3_03_b0_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_e1/"
        "v3_02_e1_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"
    ),
    Path("src/simulation/m2_gazebo/CMakeLists.txt"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c0_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c0_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_03_c0_closed_loop.py"
    ),
)
BINARY_RESOURCES = (
    Path("devel/lib/libv2_trajectory_actor_plugin.so"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("devel/lib/libv3_episode_recorder_plugin.so"),
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
        load_yaml(
            Path(contract["upstream"]["b0_effective_planner_contract_path"])
        )
    )
    value["stage"] = "V3-03-C0"
    value["stage_purpose"] = contract["stage_purpose"]
    value["contract_id"] = "v3_scope_v3_03_c0_effective_planner_1"
    value["status"] = "pure_offline_closed_loop_recovery_candidate"
    value["predictor_runtime"]["cleared_actor_bypass"] = copy.deepcopy(
        contract["cleared_actor_bypass"]
    )
    value["authority"] = copy.deepcopy(contract["authority"])
    value["claim_if_pass"] = contract["claim_if_pass"]
    value["forbidden_claims"] = copy.deepcopy(contract["forbidden_claims"])
    return value


def snapshot(
    scene_id,
    actor_coordinate,
    actor_speed,
    angle,
    certified,
    ego_x=-2.2,
    ego_speed=0.0,
):
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
            "monotone_coordinate_certified": certified,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


def validate(contract, preregistration):
    core.require(contract["stage"] == "V3-03-C0", "stage differs")
    core.require(
        contract["status"]
        == "pure_offline_preregistered_before_materialization",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-03-C0"
        and preregistration["status"]
        == "frozen_before_offline_closed_loop_recovery_materialization",
        "preregistration differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    parent = read(
        Path(contract["upstream"]["b0_effective_planner_contract_path"])
    )
    core.require(
        sha(parent)
        == contract["upstream"]["b0_effective_planner_contract_sha256"],
        "B0 planner contract changed",
    )
    model = read(Path(contract["upstream"]["predictor_path"]))
    core.require(
        sha(model) == contract["upstream"]["predictor_sha256"],
        "predictor changed",
    )
    factor = contract["single_factor_extension"]
    core.require(
        factor["predictor_model_changed"] is False
        and factor["observed_mode_gate_changed"] is False
        and factor["hard_verifier_footprints_or_margins_changed"] is False
        and factor["analytic_ranker_weights_changed"] is False
        and factor["candidate_speed_roster_changed"] is False,
        "single-factor boundary differs",
    )


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    planner = effective_contract(contract)
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    radius = core._combined_radius(planner)
    minimum_angle = min(
        abs(math.sin(math.radians(float(value))))
        for value in contract["cleared_actor_bypass"][
            "path_angle_support_deg"
        ]
    )
    geometric_margin = (
        float(contract["cleared_actor_bypass"]["minimum_coordinate_m"])
        * minimum_angle
        - radius
    )
    cleared_rows = []
    for angle in (70.0, 90.0, 110.0):
        for ego_x, ego_speed in ((-2.2, 0.0), (-0.5, 0.8), (0.2, 1.2)):
            value = snapshot(
                "cleared_{}_{}_{}".format(angle, ego_x, ego_speed),
                1.35,
                0.9,
                angle,
                True,
                ego_x,
                ego_speed,
            )
            first = core.plan(value, model, planner)
            second = core.plan(value, model, planner)
            selected = first["decision"]["selected_candidate_id"]
            cleared_rows.append(
                {
                    "angle_deg": angle,
                    "ego_x_m": ego_x,
                    "ego_speed_mps": ego_speed,
                    "selected_candidate_id": selected,
                    "motion_selected": selected != "certified_stop_v000",
                    "deterministic": core.canonical_bytes(first)
                    == core.canonical_bytes(second),
                }
            )
    missing_certificate = []
    for angle in (70.0, 90.0, 110.0):
        value = snapshot(
            "missing_{}".format(angle), 1.35, 0.9, angle, False
        )
        result = core.plan(value, model, planner)
        missing_certificate.append(
            result["decision"]["selected_candidate_id"]
            == "certified_stop_v000"
        )
    preclear = []
    for speed in (0.20, 0.325, 1.20):
        value = snapshot(
            "preclear_{}".format(speed), -2.6, speed, 90.0, True
        )
        result = core.plan(value, model, planner)
        preclear.append(
            result["decision"]["selected_candidate_id"]
            == "certified_stop_v000"
        )
    stop_validation = []
    for speed in (0.0, 0.4, 0.8, 1.2):
        value = snapshot(
            "stop_{}".format(speed),
            -2.6,
            0.2,
            90.0,
            True,
            -2.6,
            speed,
        )
        candidates = core.generate_candidate_set(value, planner)["candidates"]
        stop = next(item for item in candidates if item["certified_stop"])
        stop_validation.append(not core.validate_candidate(stop, value, planner))
    metrics = {
        "combined_footprint_and_margin_radius_m": radius,
        "minimum_cleared_geometric_margin_m": geometric_margin,
        "cleared_fixture_count": len(cleared_rows),
        "cleared_motion_selection_rate": sum(
            row["motion_selected"] for row in cleared_rows
        )
        / len(cleared_rows),
        "missing_certificate_stop_rate": sum(missing_certificate)
        / len(missing_certificate),
        "preclear_conflict_stop_rate": sum(preclear) / len(preclear),
        "variable_initial_speed_stop_valid_rate": sum(stop_validation)
        / len(stop_validation),
        "deterministic_replay_rate": sum(
            row["deterministic"] for row in cleared_rows
        )
        / len(cleared_rows),
    }
    thresholds = contract["offline_gates"]
    gates = {
        "cleared_geometry_pass": geometric_margin
        >= float(thresholds["minimum_cleared_geometric_margin_m"]),
        "cleared_motion_selection_pass": metrics[
            "cleared_motion_selection_rate"
        ]
        >= float(thresholds["cleared_motion_selection_rate_min"]),
        "missing_certificate_stop_pass": metrics[
            "missing_certificate_stop_rate"
        ]
        >= float(thresholds["missing_certificate_stop_rate_min"]),
        "preclear_conflict_stop_pass": metrics[
            "preclear_conflict_stop_rate"
        ]
        >= float(thresholds["preclear_conflict_stop_rate_min"]),
        "variable_initial_speed_stop_valid_pass": metrics[
            "variable_initial_speed_stop_valid_rate"
        ]
        >= float(
            thresholds["variable_initial_speed_stop_valid_rate_min"]
        ),
        "deterministic_replay_pass": metrics["deterministic_replay_rate"]
        == 1.0,
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C0",
        "report_id": "v3_03_c0_offline_report_1",
        "status": (
            "v3_03_c0_closed_loop_recovery_offline_pass"
            if all(gates.values())
            else "v3_03_c0_closed_loop_recovery_offline_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "gates": gates,
        "overall_pass": all(gates.values()),
        "cleared_fixtures": cleared_rows,
        "claim": {
            "offline_yield_and_recovery_logic_pass": all(gates.values()),
            "segmented_gazebo_integration": False,
            "navigation_safety": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    closure = {
        "schema_version": "1.0",
        "stage": "V3-03-C0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "source_resources": [receipt(path) for path in SOURCE_RESOURCES],
        "binary_resources": [receipt(path) for path in BINARY_RESOURCES],
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
