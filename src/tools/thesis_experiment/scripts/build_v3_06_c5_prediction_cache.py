#!/usr/bin/env python3
"""Build the V3-06-C5 duplicate-prediction cache foundation."""

import argparse
import hashlib
import importlib.util
from pathlib import Path
import sys

import yaml

W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
THESIS_SRC = W / "src/tools/thesis_experiment/src"
for path in (PACKAGE_SRC, THESIS_SRC):
    if str(path) not in sys.path: sys.path.insert(0, str(path))
from v3_scope_planner import core
_CACHE_PATH = (
    W
    / "src/tools/thesis_experiment/src/thesis_experiment/"
    "v3_06_cached_core.py"
)
_CACHE_SPEC = importlib.util.spec_from_file_location(
    "v3_06_cached_core_builder", _CACHE_PATH
)
cached_core = importlib.util.module_from_spec(_CACHE_SPEC)
_CACHE_SPEC.loader.exec_module(cached_core)

CONTRACT = Path("config/thesis_experiments/v3/v3_06_c5_duplicate_prediction_cache_contract.yaml")
ROOT = Path("artifacts/v3/development/v3_06_c5")
REPORT = ROOT / "v3_06_c5_offline_equivalence_report.yaml"
REVIEW = ROOT / "v3_06_c5_independent_review.yaml"
TERMINAL = ROOT / "v3_06_c5_terminal_freeze.sha256"
C1_MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"


def sha(payload): return hashlib.sha256(payload).hexdigest()
def load_yaml(path): return yaml.safe_load((W / path).read_bytes())
def yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True, width=1000000).encode("utf-8")


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload: raise RuntimeError("refusing overwrite")
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True); absolute.write_bytes(payload)


def load_c1():
    spec = importlib.util.spec_from_file_location("v3_06_c5_c1", C1_MODULE)
    value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value)
    return value


def build():
    contract = load_yaml(CONTRACT)
    planner_path = Path(contract["upstream"]["dense_planner_path"])
    planner_bytes = (W / planner_path).read_bytes()
    if sha(planner_bytes) != contract["upstream"]["dense_planner_sha256"]:
        raise RuntimeError("planner hash differs")
    planner = yaml.safe_load(planner_bytes)
    model = load_yaml(Path(
        "artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml"
    ))
    c1 = load_c1(); schedule = c1.load_yaml_relative(W, c1.SCHEDULE_RELATIVE)
    rows = [row for row in schedule["rows"] if row["method"] == "add_v020_v040"]
    decisions = exact = accepted = computed = reused = 0
    for row in rows:
        episode = c1.load_yaml_relative(
            W, c1.EXECUTION_RELATIVE / "episodes" / row["unit_id"] / "episode_receipt.yaml"
        )
        for decision in episode["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            reference = core.plan(snapshot, model, planner)
            cached, calls = cached_core.plan_with_diagnostics(snapshot, model, planner)
            selected_id = cached["decision"]["selected_candidate_id"]
            selected = next(
                item for item in cached["evaluations"] if item["candidate_id"] == selected_id
            )
            decisions += 1
            exact += core.canonical_bytes(reference) == core.canonical_bytes(cached)
            accepted += selected["accepted"] is True
            computed += calls["computed"]; reused += calls["reused"]
    metrics = {
        "episode_count": len(rows), "decision_count": decisions,
        "exact_plan_output_rate": exact / decisions,
        "selected_candidate_accepted_rate": accepted / decisions,
        "predictor_calls_computed": computed, "predictor_calls_reused": reused,
        "saved_predictor_call_count": reused, "saved_calls_per_decision": reused / decisions,
        "simulation_units": 0,
    }
    gates = {
        "episode_count_pass": len(rows) == 12,
        "decision_count_pass": decisions == 395,
        "exact_output_pass": metrics["exact_plan_output_rate"] == 1.0,
        "accepted_pass": metrics["selected_candidate_accepted_rate"] == 1.0,
        "saved_call_pass": reused >= 395,
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-06-C5",
        "status": (
            "v3_06_c5_prediction_cache_equivalence_pass"
            if all(gates.values()) else "v3_06_c5_prediction_cache_equivalence_fail"
        ),
        "formal_result": False, "metrics": metrics, "offline_gates": gates,
        "overall_pass": all(gates.values()),
        "fresh_latency_and_effect_qualification_required": True,
        "performance_improvement": False, "authority": contract["authority"],
    }
    return {REPORT: yaml_bytes(report)}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args(); outputs = build()
    if args.check:
        for path, payload in outputs.items():
            if (W / path).read_bytes() != payload: raise RuntimeError("C5 differs")
    else:
        for path, payload in outputs.items(): write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print("status={} saved_calls={}".format(
        report["status"], report["metrics"]["saved_predictor_call_count"]
    ))


if __name__ == "__main__": main()
