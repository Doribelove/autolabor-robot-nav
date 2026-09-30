#!/usr/bin/env python3
"""Replay the C5 audit with equivalent exception caching."""

import argparse
import importlib.util
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
BASE = W / "src/tools/thesis_experiment/scripts/build_v3_06_c5_prediction_cache.py"
CACHE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_cached_core_r1.py"
ROOT = Path("artifacts/v3/development/v3_06_c5_r1")
REPORT = ROOT / "v3_06_c5_r1_offline_equivalence_report.yaml"
REVIEW = ROOT / "v3_06_c5_r1_independent_review.yaml"
TERMINAL = ROOT / "v3_06_c5_r1_terminal_freeze.sha256"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload: raise RuntimeError("refusing overwrite")
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True); absolute.write_bytes(payload)


def build():
    base = load(BASE, "v3_06_c5_r1_base")
    cache = load(CACHE, "v3_06_c5_r1_cache")
    base.cached_core = cache
    base.REPORT = REPORT
    raw = base.build()[REPORT]
    report = yaml.safe_load(raw)
    report["stage"] = "V3-06-C5-R1"
    report["status"] = (
        "v3_06_c5_r1_exception_cache_equivalence_pass"
        if report["overall_pass"]
        else "v3_06_c5_r1_exception_cache_equivalence_fail"
    )
    report["repair"] = {
        "factor_id": "cache_equivalent_prediction_exceptions",
        "parent_saved_call_count": 302,
        "plan_output_semantics_changed": False,
    }
    return yaml.safe_dump(
        report, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args(); payload = build()
    if args.check:
        if (W / REPORT).read_bytes() != payload: raise RuntimeError("R1 differs")
    else: write_once(REPORT, payload)
    report = yaml.safe_load(payload)
    print("status={} saved_calls={}".format(
        report["status"], report["metrics"]["saved_predictor_call_count"]
    ))


if __name__ == "__main__": main()
