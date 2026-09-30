#!/usr/bin/env python3
"""Build V3-03-C5 pure-offline reachable-occupancy evidence."""

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
    "v3_03_c5_post_conflict_reachable_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_03_c5_post_conflict_reachable_preregistration.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_03_c5")
EFFECTIVE_CONTRACT = ROOT / "v3_03_c5_effective_planner_contract.yaml"
REPORT = ROOT / "v3_03_c5_offline_report.yaml"
CLOSURE = ROOT / "v3_03_c5_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_03_c5_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_03_c5_independent_review.yaml"
C4_EPISODES = Path("artifacts/v3/planner/v3_03_c4/execution/episodes")

BASE_SOURCE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path("artifacts/v3/planner/v3_03_c4/v3_03_c4_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/planner/v3_03_c4/execution/"
        "v3_03_c4_independent_failure_review.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c4/execution/"
        "v3_03_c4_closed_loop_assessment.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c3/"
        "v3_03_c3_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c5_post_conflict_reachable.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c5_post_conflict_reachable.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_03_c5_post_conflict_reachable.py"
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


def episode_paths():
    return tuple(
        path.relative_to(W)
        for path in sorted((W / C4_EPISODES).glob("*/episode_receipt.yaml"))
    )


def effective_contract(contract):
    value = copy.deepcopy(
        load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    )
    value["stage"] = "V3-03-C5"
    value["stage_purpose"] = contract["stage_purpose"]
    value["contract_id"] = "v3_scope_v3_03_c5_effective_planner_1"
    value["status"] = "pure_offline_post_conflict_reachable_candidate"
    value["predictor_runtime"]["post_conflict_reachable_bypass"] = (
        copy.deepcopy(contract["single_factor_repair"])
    )
    value["authority"] = copy.deepcopy(contract["authority"])
    value["claim_if_pass"] = contract["claim_if_pass"]
    value["forbidden_claims"] = copy.deepcopy(contract["forbidden_claims"])
    return value


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
    core.require(contract["stage"] == "V3-03-C5", "stage differs")
    core.require(
        contract["status"]
        == "pure_offline_preregistered_before_materialization",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-03-C5"
        and preregistration["status"]
        == "frozen_before_offline_post_conflict_reachable_materialization",
        "preregistration differs",
    )
    factor = contract["single_factor_repair"]
    core.require(
        factor["factor_count"] == 1
        and factor["factor_id"]
        == "certified_post_conflict_reachable_occupancy"
        and float(factor["minimum_coordinate_m"]) == 0.0
        and float(factor["certified_minimum_speed_mps"]) == 0.12
        and float(factor["certified_maximum_deceleration_mps2"]) == 2.4
        and float(factor["certified_path_coordinate_upper_m"]) == 3.5
        and factor["learned_predictor_used_in_bypass"] is False
        and factor["time_indexed_geometric_verification_retained"] is True
        and factor["commitment_invariance_changed"] is False
        and factor["predictor_model_or_mode_gate_changed"] is False
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
    for section, path_key, sha_key in (
        ("upstream", "planner_contract_path", "planner_contract_sha256"),
        ("upstream", "predictor_path", "predictor_sha256"),
        ("parent", "terminal_freeze_path", "terminal_freeze_sha256"),
        ("parent", "negative_review_path", "negative_review_sha256"),
    ):
        core.require(
            sha(read(Path(contract[section][path_key])))
            == contract[section][sha_key],
            "{} hash differs".format(path_key),
        )


def actual_minimum_progress(
    speed, elapsed, minimum_speed, maximum_deceleration
):
    if speed <= minimum_speed:
        return minimum_speed * elapsed
    deceleration_time = (speed - minimum_speed) / maximum_deceleration
    active = min(elapsed, deceleration_time)
    return (
        speed * active
        - 0.5 * maximum_deceleration * active * active
        + minimum_speed * max(0.0, elapsed - deceleration_time)
    )


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    planner = effective_contract(contract)
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    episodes = [load_yaml(path) for path in episode_paths()]
    core.require(len(episodes) == 9, "C4 episode count differs")
    selected_total = 0
    selected_accepted = 0
    commitment_total = 0
    commitment_invariant = 0
    deterministic = 0
    nonfinite = 0
    activations = []
    advances = []
    for episode in episodes:
        decisions = episode["decisions"]
        old_clear = next(
            int(row["decision_index"])
            for row in decisions
            if row["actor_cleared_certificate_active"]
        )
        first_certified_motion = None
        for row in decisions:
            snapshot = row["snapshot_runtime_input_without_policy"]
            first = core.plan(snapshot, model, planner)
            second = core.plan(snapshot, model, planner)
            candidate, evaluation = selected(first)
            selected_total += 1
            selected_accepted += evaluation["accepted"] is True
            deterministic += core.canonical_bytes(first) == core.canonical_bytes(
                second
            )
            numbers = (
                evaluation["minimum_clearance_m"],
                evaluation["braking_margin_m"],
                evaluation["commitment_braking_margin_m"],
                evaluation["analytic_score"],
            )
            nonfinite += sum(
                not math.isfinite(float(value)) for value in numbers
            )
            if not core.actor_cleared(snapshot, planner):
                commitment_total += 1
                commitment_invariant += (
                    evaluation["accepted"] is True
                    and float(evaluation["commitment_braking_margin_m"])
                    >= float(
                        planner["hard_verifier"][
                            "minimum_braking_margin_m"
                        ]
                    )
                    - 1e-12
                )
            if (
                core.actor_post_conflict_reachable(snapshot, planner)
                and candidate["candidate_id"] != "certified_stop_v000"
                and evaluation["accepted"] is True
                and first_certified_motion is None
            ):
                tube = core.predict_occupancy_tube(
                    snapshot, candidate, model, planner
                )
                core.require(
                    tube["safety_certified_post_conflict_reachable"] is True,
                    "reachable certificate absent",
                )
                first_certified_motion = int(row["decision_index"])
        activations.append(first_certified_motion is not None)
        advance = (
            (old_clear - first_certified_motion) * 0.25
            if first_certified_motion is not None
            else 0.0
        )
        advances.append(advance)

    factor = contract["single_factor_repair"]
    reachable_cases = 0
    reachable_conservative = 0
    for coordinate in (0.0, 0.4, 1.0):
        for speed in (0.12, 0.3, 0.8, 1.4):
            for elapsed_index in range(81):
                elapsed = elapsed_index * 0.05
                certified = core._post_conflict_reachable_lower_coordinate(
                    coordinate, speed, elapsed, factor
                )
                for actual_floor in (0.12, 0.2, 0.5):
                    for actual_deceleration in (0.4, 1.2, 2.4):
                        actual = coordinate + actual_minimum_progress(
                            speed,
                            elapsed,
                            actual_floor,
                            actual_deceleration,
                        )
                        reachable_cases += 1
                        reachable_conservative += certified <= actual + 1e-12

    negative_cases = []
    for angle, certified in ((90.0, False), (69.0, True), (111.0, True)):
        snapshot = {
            "scene_snapshot_id": "negative_{}_{}".format(angle, certified),
            "world_model_seq": 1,
            "snapshot_age_s": 0.0,
            "predictor_latency_ms": 0.0,
            "verifier_latency_ms": 0.0,
            "ego": {
                "x_m": -2.2,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "speed_mps": 0.0,
            },
            "actor": {
                "coordinate_m": 0.7,
                "speed_mps": 1.0,
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
        result = core.plan(snapshot, model, planner)
        candidate, evaluation = selected(result)
        negative_cases.append(
            candidate["candidate_id"] == "certified_stop_v000"
            and evaluation["accepted"] is True
            and not core.actor_post_conflict_reachable(snapshot, planner)
        )

    metrics = {
        "c4_recorded_episode_count": len(episodes),
        "c4_recorded_decision_count": selected_total,
        "c4_post_conflict_preclear_episode_activation_rate": (
            sum(activations) / len(activations)
        ),
        "c4_counterfactual_motion_advance_episode_rate": (
            sum(value >= 0.50 - 1e-12 for value in advances)
            / len(advances)
        ),
        "c4_counterfactual_advance_s_min": min(advances),
        "c4_counterfactual_advance_s_max": max(advances),
        "selected_candidate_accepted_rate": (
            selected_accepted / selected_total
        ),
        "selected_commitment_invariant_rate": (
            commitment_invariant / commitment_total
        ),
        "reachable_bound_case_count": reachable_cases,
        "reachable_lower_bound_conservative_rate": (
            reachable_conservative / reachable_cases
        ),
        "missing_or_out_of_support_stop_rate": (
            sum(negative_cases) / len(negative_cases)
        ),
        "deterministic_replay_rate": deterministic / selected_total,
        "nonfinite_output_count": nonfinite,
    }
    thresholds = contract["offline_gates"]
    gates = {
        "c4_evidence_count_pass": (
            metrics["c4_recorded_episode_count"]
            == int(thresholds["c4_recorded_episode_count"])
            and metrics["c4_recorded_decision_count"]
            == int(thresholds["c4_recorded_decision_count"])
        ),
        "post_conflict_activation_pass": metrics[
            "c4_post_conflict_preclear_episode_activation_rate"
        ]
        >= float(
            thresholds[
                "c4_post_conflict_preclear_episode_activation_rate_min"
            ]
        ),
        "counterfactual_motion_advance_pass": metrics[
            "c4_counterfactual_motion_advance_episode_rate"
        ]
        >= float(
            thresholds["c4_counterfactual_motion_advance_episode_rate_min"]
        )
        and metrics["c4_counterfactual_advance_s_min"]
        >= float(thresholds["c4_counterfactual_advance_s_min"]),
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        >= float(thresholds["selected_candidate_accepted_rate_min"]),
        "selected_commitment_invariant_pass": metrics[
            "selected_commitment_invariant_rate"
        ]
        >= float(thresholds["selected_commitment_invariant_rate_min"]),
        "reachable_lower_bound_conservative_pass": metrics[
            "reachable_lower_bound_conservative_rate"
        ]
        >= float(
            thresholds["reachable_lower_bound_conservative_rate_min"]
        ),
        "missing_certificate_stop_pass": metrics[
            "missing_or_out_of_support_stop_rate"
        ]
        >= float(thresholds["missing_certificate_stop_rate_min"]),
        "out_of_support_stop_pass": metrics[
            "missing_or_out_of_support_stop_rate"
        ]
        >= float(thresholds["out_of_support_stop_rate_min"]),
        "deterministic_replay_pass": metrics[
            "deterministic_replay_rate"
        ]
        >= float(thresholds["deterministic_replay_rate_min"]),
        "finite_output_pass": metrics["nonfinite_output_count"]
        <= int(thresholds["nonfinite_output_count_max"]),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C5",
        "report_id": "v3_03_c5_offline_report_1",
        "status": (
            "v3_03_c5_post_conflict_reachable_offline_pass"
            if all(gates.values())
            else "v3_03_c5_post_conflict_reachable_offline_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "gates": gates,
        "overall_pass": all(gates.values()),
        "claim": {
            "offline_post_conflict_reachable_certificate_pass": all(
                gates.values()
            ),
            "fresh_gazebo_goal_recovery": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    sources = BASE_SOURCE_RESOURCES + episode_paths()
    closure = {
        "schema_version": "1.0",
        "stage": "V3-03-C5",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in sources],
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
