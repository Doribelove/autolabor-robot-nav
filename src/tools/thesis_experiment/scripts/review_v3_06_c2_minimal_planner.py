#!/usr/bin/env python3
"""Independent review for V3-06-C2."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = W / "src/tools/thesis_experiment/scripts/build_v3_06_c2_minimal_planner.py"


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_06_c2_review", BUILDER_PATH)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    builder = load_builder()
    outputs = builder.build()
    for path, payload in outputs.items():
        if (W / path).read_bytes() != payload:
            raise RuntimeError("C2 builder replay differs")
    report = yaml.safe_load(outputs[builder.REPORT])
    if not report["overall_pass"]:
        raise RuntimeError("C2 gates failed")
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C2",
        "status": "v3_06_c2_minimal_density_review_pass",
        "planner_sha256": sha(outputs[builder.PLANNER]),
        "report_sha256": sha(outputs[builder.REPORT]),
        "fresh_latency_qualification_required": True,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = builder.yaml_bytes(review)
    output = W / builder.REVIEW
    freeze = (
        "{}  {}\n{}  {}\n{}  {}\n".format(
            sha(outputs[builder.PLANNER]),
            builder.PLANNER.as_posix(),
            sha(outputs[builder.REPORT]),
            builder.REPORT.as_posix(),
            sha(payload),
            builder.REVIEW.as_posix(),
        )
    ).encode("utf-8")
    if args.check:
        if output.read_bytes() != payload or (W / builder.TERMINAL).read_bytes() != freeze:
            raise RuntimeError("C2 review differs")
    else:
        write_once(output, payload)
        write_once(W / builder.TERMINAL, freeze)
    print("status={} review_sha256={}".format(review["status"], sha(payload)))


if __name__ == "__main__":
    main()
