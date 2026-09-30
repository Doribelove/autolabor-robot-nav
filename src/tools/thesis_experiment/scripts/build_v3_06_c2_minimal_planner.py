#!/usr/bin/env python3
"""Build the V3-06-C2 minimal-density planner."""

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
    "config/thesis_experiments/v3/v3_06_c2_minimal_density_planner_contract.yaml"
)
ROOT = Path("artifacts/v3/development/v3_06_c2")
PLANNER = ROOT / "v3_06_c2_effective_planner_contract.yaml"
REPORT = ROOT / "v3_06_c2_offline_replay_report.yaml"
REVIEW = ROOT / "v3_06_c2_independent_review.yaml"
TERMINAL = ROOT / "v3_06_c2_terminal_freeze.sha256"
C1_MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    return yaml.safe_load((W / path).read_bytes())


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def load_c1():
    spec = importlib.util.spec_from_file_location("v3_06_c2_c1", C1_MODULE)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def build():
    contract = load_yaml(CONTRACT)
    dense_path = Path(contract["upstream"]["dense_planner_path"])
    dense_bytes = (W / dense_path).read_bytes()
    if sha(dense_bytes) != contract["upstream"]["dense_planner_sha256"]:
        raise RuntimeError("dense planner hash differs")
    if any(contract["authority"].values()):
        raise RuntimeError("authority must remain zero")
    dense = yaml.safe_load(dense_bytes)
    minimal = copy.deepcopy(dense)
    minimal["stage"] = "V3-06-C2"
    minimal["stage_purpose"] = contract["stage_purpose"]
    minimal["contract_id"] = contract["contract_id"]
    minimal["status"] = "pure_offline_minimal_density_latency_candidate"
    minimal["candidate_generator"]["target_speeds_mps"] = [
        0.0,
        0.4,
        0.6,
        0.8,
        1.0,
        1.2,
    ]
    minimal["candidate_generator"]["low_speed_candidate_amendment"] = {
        "factor_count": 1,
        "factor_id": "remove_v020_candidate_for_latency",
        "retained_new_candidate_id": "feasible_v040",
        "removed_candidate_id": "feasible_v020",
        "hard_verifier_changed": False,
        "predictor_changed": False,
        "certified_stop_changed": False,
        "ranker_changed": False,
    }
    minimal["authority"] = contract["authority"]
    model = load_yaml(
        Path(
            "artifacts/v3/prediction/v3_02_d1/model/"
            "v3_02_d1_candidate_predictor.yaml"
        )
    )
    c1 = load_c1()
    schedule = c1.load_yaml_relative(W, c1.SCHEDULE_RELATIVE)
    rows = [row for row in schedule["rows"] if row["method"] == "add_v040"]
    exact_selected = 0
    exact_shared = 0
    accepted = 0
    decisions = 0
    for row in rows:
        episode = c1.load_yaml_relative(
            W,
            c1.EXECUTION_RELATIVE
            / "episodes"
            / row["unit_id"]
            / "episode_receipt.yaml",
        )
        for decision in episode["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            dense_result = c1.apply_selection_method(
                core.plan(snapshot, model, dense), row, dense
            )
            minimal_result = core.plan(snapshot, model, minimal)
            dense_evaluations = {
                item["candidate_id"]: item
                for item in dense_result["evaluations"]
                if item["candidate_id"] != "feasible_v020"
            }
            minimal_evaluations = {
                item["candidate_id"]: item
                for item in minimal_result["evaluations"]
            }
            selected_id = minimal_result["decision"]["selected_candidate_id"]
            selected = minimal_evaluations[selected_id]
            decisions += 1
            exact_selected += (
                selected_id
                == dense_result["decision"]["selected_candidate_id"]
            )
            exact_shared += dense_evaluations == minimal_evaluations
            accepted += selected["accepted"] is True
    metrics = {
        "episode_count": len(rows),
        "decision_count": decisions,
        "dense_candidate_count": len(dense["candidate_generator"]["target_speeds_mps"]) + 1,
        "minimal_candidate_count": len(minimal["candidate_generator"]["target_speeds_mps"]) + 1,
        "selected_candidate_exact_replay_rate": exact_selected / decisions,
        "shared_evaluation_exact_rate": exact_shared / decisions,
        "selected_candidate_accepted_rate": accepted / decisions,
        "simulation_units": 0,
    }
    gates = {
        "episode_count_pass": metrics["episode_count"] == 12,
        "decision_count_pass": metrics["decision_count"] == 399,
        "selected_candidate_exact_replay_pass": metrics[
            "selected_candidate_exact_replay_rate"
        ]
        == 1.0,
        "shared_evaluation_exact_pass": metrics[
            "shared_evaluation_exact_rate"
        ]
        == 1.0,
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        == 1.0,
        "candidate_count_reduction_pass": metrics["minimal_candidate_count"]
        == metrics["dense_candidate_count"] - 1,
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C2",
        "status": (
            "v3_06_c2_minimal_density_offline_replay_pass"
            if all(gates.values())
            else "v3_06_c2_minimal_density_offline_replay_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "offline_gates": gates,
        "overall_pass": all(gates.values()),
        "fresh_latency_qualification_required": True,
        "performance_improvement": False,
        "authority": contract["authority"],
    }
    return {PLANNER: yaml_bytes(minimal), REPORT: yaml_bytes(report)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build()
    if args.check:
        for path, payload in outputs.items():
            if (W / path).read_bytes() != payload:
                raise RuntimeError("C2 output differs")
    else:
        for path, payload in outputs.items():
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} planner_sha256={}".format(
            report["status"], sha(outputs[PLANNER])
        )
    )


if __name__ == "__main__":
    main()
