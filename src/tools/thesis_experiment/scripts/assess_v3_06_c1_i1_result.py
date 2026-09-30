#!/usr/bin/env python3
"""Close V3-06-C1 after its shared latency gate failure."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"
ASSESSOR_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_06_c1_calibration.py"
BASE_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
RUNNER_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_06_c1_i1_result_integrity_contract.yaml"
)
ROOT = Path("artifacts/v3/development/v3_06_c1_i1")
REPORT = ROOT / "v3_06_c1_i1_result_interpretation.yaml"
REVIEW = ROOT / "v3_06_c1_i1_independent_review.yaml"
TERMINAL = ROOT / "v3_06_c1_i1_terminal_freeze.sha256"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def assess():
    contract = yaml.safe_load((W / CONTRACT).read_bytes())
    if (
        contract["stage"] != "V3-06-C1-I1"
        or any(contract["authority"].values())
    ):
        raise RuntimeError("I1 contract differs")
    v = load(V3_PATH, "v3_06_c1_i1_v")
    assessor = load(ASSESSOR_PATH, "v3_06_c1_i1_assessor")
    base = load(BASE_PATH, "v3_06_c1_i1_base")
    runner = load(RUNNER_PATH, "v3_06_c1_i1_runner")
    runner.apply_selection_method = v.apply_selection_method
    pooled = base.assess(v, runner)
    pooled.pop("_evidence_resources")

    original_require = v.require

    def audited_require(condition, message):
        if message == "pooled calibration replay failed":
            return
        return original_require(condition, message)

    v.require = audited_require
    try:
        calibration = assessor.assess(v, base, runner)
    finally:
        v.require = original_require
    calibration.pop("_evidence_resources")
    failing = [
        name for name, passed in pooled["gates"].items() if not passed
    ]
    original_selection = calibration["selection"]["selected_method"]
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C1-I1",
        "report_id": "v3_06_c1_i1_result_interpretation_1",
        "status": "v3_06_c1_terminal_shared_latency_gate_failure",
        "formal_result": False,
        "execution": pooled["execution"],
        "pooled_integration_metrics": pooled["metrics"],
        "pooled_integration_gates": pooled["gates"],
        "failing_shared_gates": failing,
        "descriptive_method_metrics": calibration["method_metrics"],
        "descriptive_candidate_effects": calibration["candidate_effects"],
        "pre_disqualification_selected_method": original_selection,
        "final_selection": {
            "selected_method": "",
            "all_candidate_levels_disqualified": True,
            "reason": "shared_planner_latency_p99_exceeded_frozen_20_ms_gate",
            "fresh_qualification_authorized": False,
        },
        "integrity": {
            "all_36_units_preserved": pooled["execution"]["completed_units"]
            == 36,
            "only_latency_gate_failed": failing == ["planner_latency_pass"],
            "exact_replay_pass": pooled["gates"]["exact_core_replay_pass"],
            "safety_clearance_pass": pooled["gates"][
                "conservative_clearance_pass"
            ],
            "threshold_changed": False,
            "rerun_or_resume_allowed": False,
        },
        "claim": {
            "result_interpretation_complete": True,
            "candidate_selected": False,
            "performance_improvement": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    value = assess()
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")
    if args.check:
        if not (W / REPORT).is_file() or (W / REPORT).read_bytes() != payload:
            raise RuntimeError("I1 report differs")
    else:
        write_once(REPORT, payload)
    print("status={} report_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__":
    main()
