#!/usr/bin/env python3
"""Build the pure-offline V3-03-B0 observed-history mode-gate closure."""

import argparse
import copy
import hashlib
import importlib.util
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


CONTRACT = Path(
    "config/thesis_experiments/v3/v3_03_b0_observed_mode_gate_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_03_b0_observed_mode_gate_preregistration.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_03_b0")
EFFECTIVE_CONTRACT = ROOT / "v3_03_b0_effective_planner_contract.yaml"
REPORT = ROOT / "v3_03_b0_mode_gate_report.yaml"
CLOSURE = ROOT / "v3_03_b0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_03_b0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_03_b0_independent_review.yaml"

D1_MODULE = (
    W
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"
)
FITTER_MODULE = (
    W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"
)

SOURCE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path("config/thesis_experiments/v3/v3_03_a0_safe_planner_contract.yaml"),
    Path(
        "artifacts/v3/planner/v3_03_a0/"
        "v3_03_a0_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_model_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d2/"
        "v3_02_d2_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/offline_readiness/"
        "v3_02_d1_exact_schedule.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_a3/execution/"
        "v3_03_a3_integration_assessment.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_a3/execution/"
        "v3_03_a3_evidence_freeze.sha256"
    ),
    Path(
        "artifacts/v3/planner/v3_03_a3/execution/"
        "v3_03_a3_independent_result_review.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/core.py"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"
    ),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_03_b0_mode_gate.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_03_b0_mode_gate.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_03_b0_mode_gate.py"),
)

HORIZONS = ("t1_5", "t2_5", "t3_5", "t4_5")
POLICIES = ("cautious", "assertive", "hesitant")


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    value = yaml.safe_load(read(path))
    core.require(isinstance(value, dict), "YAML root is not a mapping")
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
        return
    absolute.parent.mkdir(parents=True, exist_ok=True)
    absolute.write_bytes(payload)


def mode_configuration(contract):
    return {
        "mode_inference_id": contract["mode_inference"][
            "mode_inference_id"
        ],
        "observable_feature": "actor_speed_mps",
        "hidden_policy_runtime_input": False,
        "cautious_hesitant_threshold_mps": float(
            contract["mode_inference"][
                "cautious_hesitant_threshold_mps"
            ]
        ),
        "hesitant_assertive_threshold_mps": float(
            contract["mode_inference"][
                "hesitant_assertive_threshold_mps"
            ]
        ),
        "ambiguity_half_width_mps": float(
            contract["mode_inference"]["ambiguity_half_width_mps"]
        ),
        "ambiguous_or_nonfinite_behavior": (
            "fail_closed_to_certified_stop"
        ),
    }


def effective_contract(contract):
    value = copy.deepcopy(
        load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    )
    value["stage"] = "V3-03-B0"
    value["stage_purpose"] = contract["stage_purpose"]
    value["contract_id"] = "v3_scope_v3_03_b0_effective_planner_1"
    value["status"] = "pure_offline_mode_gate_candidate"
    value["upstream"]["a0_contract_path"] = contract["upstream"][
        "planner_contract_path"
    ]
    value["predictor_runtime"]["mode_inference"] = mode_configuration(contract)
    value["predictor_runtime"]["maximum_full_tube_width_m"] = float(
        contract["effective_predictor_runtime"][
            "maximum_full_tube_width_m"
        ]
    )
    value["authority"] = copy.deepcopy(contract["authority"])
    value["claim_if_pass"] = contract["claim_if_pass"]
    value["forbidden_claims"] = copy.deepcopy(contract["forbidden_claims"])
    return value


def validate(contract, preregistration):
    core.require(contract["stage"] == "V3-03-B0", "stage differs")
    core.require(
        contract["status"]
        == "pure_offline_preregistered_before_materialization",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-03-B0"
        and preregistration["status"]
        == "frozen_before_offline_mode_gate_materialization",
        "preregistration differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    core.require(
        contract["mode_inference"]["hidden_policy_runtime_input"] is False
        and contract["effective_predictor_runtime"]["model_refit_allowed"]
        is False
        and contract["effective_predictor_runtime"][
            "recalibration_allowed"
        ]
        is False,
        "runtime firewall differs",
    )
    for path_key, sha_key in (
        ("predictor_path", "predictor_sha256"),
        ("d1_schedule_path", "d1_schedule_sha256"),
        ("a3_assessment_path", "a3_assessment_sha256"),
    ):
        core.require(
            sha(read(Path(contract["upstream"][path_key])))
            == contract["upstream"][sha_key],
            "{} hash differs".format(path_key),
        )


def d1_episodes(d1):
    schedule = load_yaml(d1.SCHEDULE_RELATIVE)
    episodes = []
    resources = []
    for row in schedule["rows"]:
        root = d1.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        raw_path = root / "raw_pose_trace.csv"
        receipt_path = root / "attempt_receipt.yaml"
        attempt = load_yaml(receipt_path)
        raw = read(raw_path)
        core.require(
            attempt["raw_csv_sha256"] == sha(raw),
            "D1 raw receipt differs: {}".format(row["unit_id"]),
        )
        metrics = d1.trace_metrics(d1.load_trace(W / raw_path), row)
        core.require(
            metrics == attempt["metrics_preview"],
            "D1 metric replay differs: {}".format(row["unit_id"]),
        )
        episodes.append(
            {
                "row": row,
                "metrics": metrics,
                "features": d1.model_features(metrics),
            }
        )
        resources.extend((raw_path, receipt_path))
    return schedule, episodes, resources


def derive_support(episodes):
    support = {}
    for policy in POLICIES:
        values = [
            float(item["features"][2])
            for item in episodes
            if item["row"]["actor_policy_evaluator_only"] == policy
        ]
        support[policy] = {
            "count": len(values),
            "minimum_mps": min(values),
            "maximum_mps": max(values),
        }
    return support


def threshold_midpoints(support):
    return {
        "cautious_hesitant_threshold_mps": 0.5
        * (
            support["cautious"]["maximum_mps"]
            + support["hesitant"]["minimum_mps"]
        ),
        "hesitant_assertive_threshold_mps": 0.5
        * (
            support["hesitant"]["maximum_mps"]
            + support["assertive"]["minimum_mps"]
        ),
    }


def evaluate_split(episodes, role, model, mode_gate, fitter, margin):
    selected = [
        item
        for item in episodes
        if item["row"]["role_evaluator_only"] == role
    ]
    correct = 0
    ambiguous = 0
    hits = []
    per_horizon = {horizon: [] for horizon in HORIZONS}
    errors = []
    for item in selected:
        label = item["row"]["actor_policy_evaluator_only"]
        try:
            inferred = core.infer_observed_mode(
                float(item["features"][2]), mode_gate
            )
        except core.PlannerError:
            inferred = ""
            ambiguous += 1
        correct += inferred == label
        if not inferred:
            hits.extend([False] * len(HORIZONS))
            for horizon in HORIZONS:
                per_horizon[horizon].append(False)
            continue
        truth = item["metrics"]["target_actor_conflict_coordinates_m"]
        for index, horizon in enumerate(HORIZONS):
            prediction = fitter.prediction(
                model, inferred, index, item["features"]
            )
            error = abs(prediction - float(truth[index]))
            hit = error <= float(model["calibration"]["radius_m"]) + 1e-12
            errors.append(error)
            hits.append(hit)
            per_horizon[horizon].append(hit)
    full_width = (
        2.0 * float(model["calibration"]["radius_m"]) + 2.0 * margin
    )
    return {
        "episode_count": len(selected),
        "classifier_correct_count": correct,
        "classifier_accuracy": correct / len(selected),
        "ambiguous_count": ambiguous,
        "prediction_point_count": len(hits),
        "marginal_coverage": sum(hits) / len(hits),
        "per_horizon_coverage": {
            horizon: sum(values) / len(values)
            for horizon, values in per_horizon.items()
        },
        "actual_mode_mae_m": sum(errors) / len(errors),
        "mean_full_tube_width_m": full_width,
    }


def fault_checks(planner_contract, model):
    snapshot = {
        "scene_snapshot_id": "v3_03_b0_fault",
        "world_model_seq": 1,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {
            "x_m": -2.6,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "speed_mps": 0.8,
        },
        "actor": {
            "coordinate_m": -2.6,
            "speed_mps": 0.0,
            "path_angle_deg": 90.0,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }
    stop_id = planner_contract["candidate_generator"][
        "certified_stop_candidate_id"
    ]
    gate = planner_contract["predictor_runtime"]["mode_inference"]
    ambiguous_values = [
        float(gate["cautious_hesitant_threshold_mps"]),
        float(gate["hesitant_assertive_threshold_mps"]),
    ]
    ambiguous_stop = 0
    for value in ambiguous_values:
        case = copy.deepcopy(snapshot)
        case["actor"]["speed_mps"] = value
        result = core.plan(case, model, planner_contract)
        ambiguous_stop += (
            result["decision"]["selected_candidate_id"] == stop_id
        )
    case = copy.deepcopy(snapshot)
    case["actor"]["speed_mps"] = float("nan")
    result = core.plan(case, model, planner_contract)
    nonfinite_stop = result["decision"]["selected_candidate_id"] == stop_id
    return {
        "ambiguous_case_count": len(ambiguous_values),
        "ambiguous_stop_rate": ambiguous_stop / len(ambiguous_values),
        "nonfinite_case_count": 1,
        "nonfinite_stop_rate": 1.0 if nonfinite_stop else 0.0,
    }


def replay_a3(planner_contract, model):
    root = Path("artifacts/v3/planner/v3_03_a3/execution/decisions")
    rows = []
    for path in sorted((W / root).glob("*/decision_receipt.yaml")):
        frozen = yaml.safe_load(path.read_bytes())
        snapshot = frozen["snapshot_runtime_input_without_policy"]
        result = core.plan(snapshot, model, planner_contract)
        inferred = core.infer_observed_mode(
            float(snapshot["actor"]["speed_mps"]),
            planner_contract["predictor_runtime"]["mode_inference"],
        )
        selected = result["decision"]["selected_candidate_id"]
        rows.append(
            {
                "decision_id": frozen["decision_id"],
                "evaluator_policy": frozen[
                    "actor_policy_evaluator_only"
                ],
                "observed_speed_mps": snapshot["actor"]["speed_mps"],
                "inferred_mode": inferred,
                "selected_candidate_id": selected,
                "non_stop_selected": selected
                != planner_contract["candidate_generator"][
                    "certified_stop_candidate_id"
                ],
            }
        )
    return {
        "decision_count": len(rows),
        "classifier_accuracy": sum(
            row["inferred_mode"] == row["evaluator_policy"] for row in rows
        )
        / len(rows),
        "non_stop_selection_count": sum(
            row["non_stop_selected"] for row in rows
        ),
        "non_stop_selection_rate": sum(
            row["non_stop_selected"] for row in rows
        )
        / len(rows),
        "interpretation": (
            "observed_mode_gate_correct_but_first_decision_continuous_motion_"
            "candidates_remain_unsafe;_closed_loop_yield_then_replan_required"
        ),
        "used_for_gate_or_threshold": False,
        "rows": rows,
    }


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    d1 = load_module(D1_MODULE, "v3_03_b0_d1")
    fitter = load_module(FITTER_MODULE, "v3_03_b0_fitter")
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    schedule, episodes, episode_resources = d1_episodes(d1)
    core.require(
        len(episodes) == 135
        and schedule["training_rows"] == 90
        and schedule["calibration_rows"] == 45,
        "D1 episode roster differs",
    )
    support = derive_support(episodes)
    derived = threshold_midpoints(support)
    for name, value in derived.items():
        core.require(
            abs(value - float(contract["mode_inference"][name])) <= 1e-12,
            "{} is not the support-gap midpoint".format(name),
        )
    planner_contract = effective_contract(contract)
    mode_gate = planner_contract["predictor_runtime"]["mode_inference"]
    margin = float(
        planner_contract["predictor_runtime"][
            "additional_coordinate_margin_m"
        ]
    )
    training = evaluate_split(
        episodes, "training", model, mode_gate, fitter, margin
    )
    calibration = evaluate_split(
        episodes, "calibration", model, mode_gate, fitter, margin
    )
    faults = fault_checks(planner_contract, model)
    a3 = replay_a3(planner_contract, model)
    thresholds = contract["offline_gates"]
    gates = {
        "training_classifier_accuracy_pass": training[
            "classifier_accuracy"
        ]
        >= float(thresholds["training_classifier_accuracy_min"]),
        "calibration_classifier_accuracy_pass": calibration[
            "classifier_accuracy"
        ]
        >= float(thresholds["calibration_classifier_accuracy_min"]),
        "training_calibration_ambiguity_pass": (
            training["ambiguous_count"] + calibration["ambiguous_count"]
        )
        <= int(thresholds["ambiguous_training_calibration_count_max"]),
        "calibration_marginal_coverage_pass": calibration[
            "marginal_coverage"
        ]
        >= float(thresholds["calibration_marginal_coverage_min"]),
        "calibration_per_horizon_coverage_pass": min(
            calibration["per_horizon_coverage"].values()
        )
        >= float(thresholds["calibration_per_horizon_coverage_min"]),
        "calibration_width_pass": calibration[
            "mean_full_tube_width_m"
        ]
        <= float(thresholds["calibration_mean_full_tube_width_max_m"]),
        "calibration_mae_pass": calibration["actual_mode_mae_m"]
        <= float(thresholds["calibration_actual_mode_mae_max_m"]),
        "ambiguous_fault_stop_pass": faults["ambiguous_stop_rate"]
        >= float(thresholds["ambiguous_fault_stop_rate_min"]),
        "nonfinite_fault_stop_pass": faults["nonfinite_stop_rate"]
        >= float(thresholds["nonfinite_fault_stop_rate_min"]),
        "runtime_hidden_policy_firewall_pass": (
            mode_gate["hidden_policy_runtime_input"] is False
        ),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-B0",
        "report_id": "v3_03_b0_mode_gate_report_1",
        "status": (
            "v3_03_b0_observed_mode_gate_offline_pass"
            if all(gates.values())
            else "v3_03_b0_observed_mode_gate_offline_fail"
        ),
        "formal_result": False,
        "d1_support": support,
        "derived_thresholds": derived,
        "training": training,
        "calibration": calibration,
        "fault_injection": faults,
        "retrospective_a3_descriptive_replay": a3,
        "gates": gates,
        "overall_pass": all(gates.values()),
        "claim": {
            "observed_mode_gate_calibration_qualified": all(gates.values()),
            "a3_first_decision_motion_restored": (
                a3["non_stop_selection_count"] > 0
            ),
            "navigation_safety": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "next_required_stage": (
            "fresh_heldout_mode_gate_prediction_validation_before_"
            "closed_loop_yield_then_replan_integration"
        ),
        "authority": contract["authority"],
    }
    closure_resources = list(SOURCE_RESOURCES) + episode_resources
    closure = {
        "schema_version": "1.0",
        "stage": "V3-03-B0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in closure_resources],
    }
    outputs = {
        EFFECTIVE_CONTRACT: yaml_bytes(planner_contract),
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
