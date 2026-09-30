#!/usr/bin/env python3
"""Build the V3-06-C4 v0.8-deduplicated dense planner."""

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

CONTRACT = Path("config/thesis_experiments/v3/v3_06_c4_v080_dedup_planner_contract.yaml")
ROOT = Path("artifacts/v3/development/v3_06_c4")
PLANNER = ROOT / "v3_06_c4_effective_planner_contract.yaml"
REPORT = ROOT / "v3_06_c4_offline_equivalence_report.yaml"
REVIEW = ROOT / "v3_06_c4_independent_review.yaml"
TERMINAL = ROOT / "v3_06_c4_terminal_freeze.sha256"
C1_MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    return yaml.safe_load((W / path).read_bytes())


def yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True, width=1000000).encode("utf-8")


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def load_c1():
    spec = importlib.util.spec_from_file_location("v3_06_c4_c1", C1_MODULE)
    value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value)
    return value


def build():
    contract = load_yaml(CONTRACT)
    dense_path = Path(contract["upstream"]["dense_planner_path"])
    dense_bytes = (W / dense_path).read_bytes()
    if sha(dense_bytes) != contract["upstream"]["dense_planner_sha256"]:
        raise RuntimeError("dense planner hash differs")
    dense = yaml.safe_load(dense_bytes)
    dedup = copy.deepcopy(dense)
    dedup["stage"] = "V3-06-C4"
    dedup["stage_purpose"] = contract["stage_purpose"]
    dedup["contract_id"] = contract["contract_id"]
    dedup["status"] = "pure_offline_v080_deduplicated_dense_candidate"
    dedup["candidate_generator"]["target_speeds_mps"] = [
        0.0, 0.2, 0.4, 0.6, 1.0, 1.2
    ]
    dedup["candidate_generator"]["low_speed_candidate_amendment"] = {
        "factor_count": 1,
        "factor_id": "eliminate_duplicate_feasible_v080",
        "removed_candidate_id": "feasible_v080",
        "retained_equivalent_candidate_id": "teb_fallback_v080",
        "retained_new_candidate_ids": ["feasible_v020", "feasible_v040"],
        "hard_verifier_changed": False,
        "predictor_changed": False,
        "certified_stop_changed": False,
        "ranker_changed": False,
    }
    dedup["authority"] = contract["authority"]
    model = load_yaml(Path(
        "artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml"
    ))
    c1 = load_c1()
    schedule = c1.load_yaml_relative(W, c1.SCHEDULE_RELATIVE)
    rows = [row for row in schedule["rows"] if row["method"] == "add_v020_v040"]
    decisions = 0
    removed_selected = 0
    trajectory_exact = 0
    accepted = 0
    for row in rows:
        episode = c1.load_yaml_relative(
            W, c1.EXECUTION_RELATIVE / "episodes" / row["unit_id"] / "episode_receipt.yaml"
        )
        for decision in episode["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            old = core.plan(snapshot, model, dense)
            new = core.plan(snapshot, model, dedup)
            old_id = old["decision"]["selected_candidate_id"]
            new_id = new["decision"]["selected_candidate_id"]
            old_candidate = next(
                item for item in old["candidate_set"]["candidates"]
                if item["candidate_id"] == old_id
            )
            new_candidate = next(
                item for item in new["candidate_set"]["candidates"]
                if item["candidate_id"] == new_id
            )
            new_eval = next(
                item for item in new["evaluations"]
                if item["candidate_id"] == new_id
            )
            decisions += 1
            removed_selected += old_id == "feasible_v080"
            trajectory_exact += (
                old_candidate["target_speed_mps"] == new_candidate["target_speed_mps"]
                and old_candidate["points"] == new_candidate["points"]
            )
            accepted += new_eval["accepted"] is True
    metrics = {
        "episode_count": len(rows),
        "decision_count": decisions,
        "dense_candidate_count": len(dense["candidate_generator"]["target_speeds_mps"]) + 1,
        "deduplicated_candidate_count": len(dedup["candidate_generator"]["target_speeds_mps"]) + 1,
        "removed_candidate_selected_count": removed_selected,
        "selected_trajectory_exact_rate": trajectory_exact / decisions,
        "selected_candidate_accepted_rate": accepted / decisions,
        "simulation_units": 0,
    }
    gates = {
        "episode_count_pass": len(rows) == 12,
        "decision_count_pass": decisions == 395,
        "removed_candidate_unselected_pass": removed_selected == 0,
        "selected_trajectory_exact_pass": metrics["selected_trajectory_exact_rate"] == 1.0,
        "selected_candidate_accepted_pass": metrics["selected_candidate_accepted_rate"] == 1.0,
        "candidate_count_reduction_pass": metrics["deduplicated_candidate_count"]
        == metrics["dense_candidate_count"] - 1,
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-06-C4",
        "status": (
            "v3_06_c4_v080_dedup_offline_equivalence_pass"
            if all(gates.values()) else "v3_06_c4_v080_dedup_offline_equivalence_fail"
        ),
        "formal_result": False, "metrics": metrics, "offline_gates": gates,
        "overall_pass": all(gates.values()),
        "fresh_latency_and_effect_qualification_required": True,
        "performance_improvement": False, "authority": contract["authority"],
    }
    return {PLANNER: yaml_bytes(dedup), REPORT: yaml_bytes(report)}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args(); outputs = build()
    if args.check:
        for path, payload in outputs.items():
            if (W / path).read_bytes() != payload:
                raise RuntimeError("C4 output differs")
    else:
        for path, payload in outputs.items():
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print("status={} planner_sha256={}".format(report["status"], sha(outputs[PLANNER])))


if __name__ == "__main__":
    main()
