#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_06_c4_v080_dedup_planner.py"
spec = importlib.util.spec_from_file_location("v3_06_c4_review", BUILDER)
b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
sha = lambda payload: hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args(); outputs = b.build()
    for path, payload in outputs.items():
        if (W / path).read_bytes() != payload: raise RuntimeError("builder replay differs")
    report = b.yaml.safe_load(outputs[b.REPORT])
    if not report["overall_pass"]: raise RuntimeError("C4 failed")
    review = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-06-C4",
        "status": "v3_06_c4_v080_dedup_review_pass",
        "planner_sha256": sha(outputs[b.PLANNER]), "report_sha256": sha(outputs[b.REPORT]),
        "fresh_qualification_required": True, "simulation_authorized": False,
        "training_authorized": False, "real_vehicle_authorized": False,
    }
    payload = b.yaml_bytes(review); output = W / b.REVIEW
    freeze = (
        "{}  {}\n{}  {}\n{}  {}\n".format(
            sha(outputs[b.PLANNER]), b.PLANNER.as_posix(),
            sha(outputs[b.REPORT]), b.REPORT.as_posix(),
            sha(payload), b.REVIEW.as_posix(),
        )
    ).encode("utf-8")
    if args.check:
        if output.read_bytes() != payload or (W / b.TERMINAL).read_bytes() != freeze:
            raise RuntimeError("review differs")
    else:
        write_once(output, payload); write_once(W / b.TERMINAL, freeze)
    print("status={} review_sha256={}".format(review["status"], sha(payload)))


if __name__ == "__main__": main()
