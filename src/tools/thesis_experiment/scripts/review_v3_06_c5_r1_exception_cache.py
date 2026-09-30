#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_06_c5_r1_exception_cache.py"
spec = importlib.util.spec_from_file_location("v3_06_c5_r1_review", BUILDER)
b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
sha = lambda payload: hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args(); report_bytes = b.build()
    if (W / b.REPORT).read_bytes() != report_bytes: raise RuntimeError("replay differs")
    report = yaml.safe_load(report_bytes)
    if not (
        report["overall_pass"] and report["metrics"]["exact_plan_output_rate"] == 1.0
        and report["metrics"]["saved_predictor_call_count"] >= 395
    ): raise RuntimeError("R1 gates failed")
    review = {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-06-C5-R1", "status": "v3_06_c5_r1_exception_cache_review_pass",
        "report_sha256": sha(report_bytes),
        "saved_predictor_call_count": report["metrics"]["saved_predictor_call_count"],
        "fresh_qualification_required": True, "simulation_authorized": False,
        "training_authorized": False, "real_vehicle_authorized": False,
    }
    payload = yaml.safe_dump(
        review, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")
    freeze = "{}  {}\n{}  {}\n".format(
        sha(report_bytes), b.REPORT.as_posix(), sha(payload), b.REVIEW.as_posix()
    ).encode("utf-8")
    if args.check:
        if (W / b.REVIEW).read_bytes() != payload or (W / b.TERMINAL).read_bytes() != freeze:
            raise RuntimeError("review differs")
    else:
        write_once(W / b.REVIEW, payload); write_once(W / b.TERMINAL, freeze)
    print("status={} review_sha256={}".format(review["status"], sha(payload)))


if __name__ == "__main__": main()
