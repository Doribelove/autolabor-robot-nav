#!/usr/bin/env python3
"""Build the V3-06-C0 low-speed candidate offline foundation."""

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
    "v3_06_c0_low_speed_candidate_foundation_contract.yaml"
)
DESIGN = Path(
    "experiments/manifests/v3/v3_06_c0_low_speed_candidate_design.yaml"
)
ROOT = Path("artifacts/v3/development/v3_06_c0")
PLANNER = ROOT / "v3_06_c0_effective_planner_contract.yaml"
REPORT = ROOT / "v3_06_c0_offline_activity_report.yaml"
CLOSURE = ROOT / "v3_06_c0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_06_c0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_06_c0_independent_review.yaml"
TERMINAL_FREEZE = ROOT / "v3_06_c0_terminal_freeze.sha256"
B0_SCHEDULE = Path(
    "artifacts/v3/development/v3_06_b0/offline_readiness/"
    "v3_06_b0_exact_schedule.yaml"
)

BASE_RESOURCES = (
    CONTRACT,
    DESIGN,
    Path("artifacts/v3/analysis/v3_06_a0/v3_06_a0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/development/v3_06_b0/"
        "v3_06_b0_terminal_freeze.sha256"
    ),
    B0_SCHEDULE,
    Path("artifacts/v3/development/v3_06_b0/execution/v3_06_b0_calibration_assessment.yaml"),
    Path("artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml"),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_06_c0_low_speed_candidate.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_c0_low_speed_candidate.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_06_c0_low_speed_candidate.py"
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


def episode_paths(schedule):
    return tuple(
        Path("artifacts/v3/development/v3_06_b0/execution/episodes")
        / row["unit_id"]
        / "episode_receipt.yaml"
        for row in schedule["rows"]
        if row["method"] == "v3_greedy_control"
    )


def validate(contract, design):
    core.require(contract["stage"] == "V3-06-C0", "stage differs")
    core.require(
        contract["status"] == "pure_offline_single_factor_design",
        "status differs",
    )
    factor = contract["single_factor"]
    core.require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "low_speed_candidate_density"
        and factor["original_target_speeds_mps"]
        == [0.0, 0.6, 0.8, 1.0, 1.2]
        and factor["proposed_target_speeds_mps"]
        == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2]
        and all(
            factor[key] is False
            for key in (
                "hard_verifier_changed",
                "predictor_changed",
                "certified_stop_changed",
                "ranker_changed",
                "tracking_or_scene_changed",
            )
        ),
        "single factor differs",
    )
    core.require(
        design["status"] == "frozen_before_effective_planner_materialization"
        and design["selection_firewall"]["no_performance_selection"] is True,
        "design firewall differs",
    )
    for path_key, sha_key in (
        ("planner_contract_path", "planner_contract_sha256"),
        ("predictor_path", "predictor_sha256"),
    ):
        core.require(
            sha(read(Path(contract["upstream"][path_key])))
            == contract["upstream"][sha_key],
            "{} hash differs".format(path_key),
        )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )


def build():
    contract = load_yaml(CONTRACT)
    design = load_yaml(DESIGN)
    validate(contract, design)
    original = load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    proposed = copy.deepcopy(original)
    proposed["stage"] = "V3-06-C0"
    proposed["stage_purpose"] = contract["stage_purpose"]
    proposed["contract_id"] = contract["contract_id"]
    proposed["status"] = "pure_offline_low_speed_candidate_foundation"
    proposed["parent_planner"] = {
        "path": contract["upstream"]["planner_contract_path"],
        "sha256": contract["upstream"]["planner_contract_sha256"],
    }
    proposed["candidate_generator"]["target_speeds_mps"] = [
        0.0,
        0.2,
        0.4,
        0.6,
        0.8,
        1.0,
        1.2,
    ]
    proposed["candidate_generator"]["low_speed_candidate_amendment"] = {
        "factor_count": 1,
        "factor_id": "low_speed_candidate_density",
        "new_candidate_ids": ["feasible_v020", "feasible_v040"],
        "hard_verifier_changed": False,
        "predictor_changed": False,
        "certified_stop_changed": False,
        "ranker_changed": False,
    }
    proposed["authority"] = contract["authority"]
    proposed["claim_if_pass"] = contract["claim_if_pass"]
    proposed["forbidden_claims"] = contract["forbidden_claims"]

    schedule = load_yaml(B0_SCHEDULE)
    paths = episode_paths(schedule)
    core.require(len(paths) == 12, "control episode roster differs")
    shared_ids = {
        "certified_stop_v000",
        "feasible_v060",
        "feasible_v080",
        "feasible_v100",
        "feasible_v120",
        "teb_fallback_v080",
    }
    decision_count = 0
    shared_exact = 0
    selected_accepted = 0
    stop_present = 0
    deterministic = 0
    newly_selected = 0
    old_stop_replaced = 0
    new_candidate_accepted = 0
    selected_counts = {}
    nonfinite = 0
    for path in paths:
        episode = load_yaml(path)
        for decision in episode["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            old = core.plan(snapshot, model, original)
            first = core.plan(snapshot, model, proposed)
            second = core.plan(snapshot, model, proposed)
            old_evaluations = {
                item["candidate_id"]: item for item in old["evaluations"]
            }
            new_evaluations = {
                item["candidate_id"]: item for item in first["evaluations"]
            }
            shared_exact += all(
                old_evaluations[candidate] == new_evaluations[candidate]
                for candidate in shared_ids
            )
            selected_id = first["decision"]["selected_candidate_id"]
            selected = new_evaluations[selected_id]
            selected_counts[selected_id] = (
                selected_counts.get(selected_id, 0) + 1
            )
            decision_count += 1
            selected_accepted += selected["accepted"] is True
            stop_present += (
                sum(
                    item["candidate_id"] == "certified_stop_v000"
                    for item in first["evaluations"]
                )
                == 1
            )
            deterministic += (
                core.canonical_bytes(first) == core.canonical_bytes(second)
            )
            newly_selected += selected_id in ("feasible_v020", "feasible_v040")
            old_stop_replaced += (
                old["decision"]["selected_candidate_id"]
                == "certified_stop_v000"
                and selected_id in ("feasible_v020", "feasible_v040")
            )
            new_candidate_accepted += sum(
                new_evaluations[candidate]["accepted"] is True
                for candidate in ("feasible_v020", "feasible_v040")
            )
            nonfinite += sum(
                not math.isfinite(float(item["minimum_clearance_m"]))
                for item in first["evaluations"]
            )

    metrics = {
        "episode_count": len(paths),
        "decision_count": decision_count,
        "shared_candidate_evaluation_exact_rate": (
            shared_exact / decision_count
        ),
        "selected_candidate_accepted_rate": (
            selected_accepted / decision_count
        ),
        "certified_stop_present_rate": stop_present / decision_count,
        "deterministic_replay_rate": deterministic / decision_count,
        "new_candidate_accepted_evaluation_count": new_candidate_accepted,
        "newly_selected_nonstop_decision_count": newly_selected,
        "original_stop_replaced_by_new_candidate_count": old_stop_replaced,
        "selected_candidate_counts": dict(sorted(selected_counts.items())),
        "nonfinite_output_count": nonfinite,
        "simulation_units": 0,
        "training_steps": 0,
    }
    gates_spec = contract["offline_gates"]
    gates = {
        "episode_roster_pass": metrics["episode_count"] == 12,
        "decision_roster_pass": metrics["decision_count"] == 406,
        "shared_candidate_evaluation_exact_pass": metrics[
            "shared_candidate_evaluation_exact_rate"
        ]
        >= float(gates_spec["shared_candidate_evaluation_exact_rate_min"]),
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        >= float(gates_spec["selected_candidate_accepted_rate_min"]),
        "certified_stop_present_pass": metrics[
            "certified_stop_present_rate"
        ]
        >= float(gates_spec["certified_stop_present_rate_min"]),
        "deterministic_replay_pass": metrics["deterministic_replay_rate"]
        >= float(gates_spec["deterministic_replay_rate_min"]),
        "behavioral_activity_pass": metrics[
            "newly_selected_nonstop_decision_count"
        ]
        >= int(gates_spec["newly_admitted_nonstop_decision_count_min"]),
        "finite_output_pass": metrics["nonfinite_output_count"]
        <= int(gates_spec["nonfinite_output_count_max"]),
        "zero_authority_pass": all(
            value is False for value in contract["authority"].values()
        ),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C0",
        "report_id": "v3_06_c0_offline_activity_report_1",
        "status": (
            "v3_06_c0_low_speed_candidate_foundation_pass"
            if all(gates.values())
            else "v3_06_c0_low_speed_candidate_foundation_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "offline_gates": gates,
        "overall_pass": all(gates.values()),
        "disposition": {
            "fresh_development_simulation_justified": all(gates.values()),
            "performance_improvement_proven": False,
            "learning_activated": False,
        },
        "claim": {
            "offline_candidate_foundation_complete": all(gates.values()),
            "simulation_integration": False,
            "performance_improvement": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    outputs = {
        PLANNER: yaml_bytes(proposed),
        REPORT: yaml_bytes(report),
    }
    resources = BASE_RESOURCES + paths
    closure = {
        "schema_version": "1.0",
        "stage": "V3-06-C0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in resources],
    }
    outputs[CLOSURE] = yaml_bytes(closure)
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
    if args.check:
        for path, payload in outputs.items():
            core.require(
                (W / path).is_file() and (W / path).read_bytes() == payload,
                "output differs: {}".format(path),
            )
    else:
        for path, payload in outputs.items():
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} active={} planner_sha256={}".format(
            report["status"],
            report["metrics"]["newly_selected_nonstop_decision_count"],
            sha(outputs[PLANNER]),
        )
    )


if __name__ == "__main__":
    main()
