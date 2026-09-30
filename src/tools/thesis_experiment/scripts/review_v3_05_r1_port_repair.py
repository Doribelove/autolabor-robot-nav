#!/usr/bin/env python3
"""Independent replay review for V3-05-R1."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_05_r1_port_repair.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_05_r1_review_builder", BUILDER
    )
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


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
            raise RuntimeError("builder replay differs: {}".format(path))
    report = yaml.safe_load(outputs[builder.REPORT])
    closure = yaml.safe_load(outputs[builder.CLOSURE])
    if not report["overall_pass"] or not all(report["gates"].values()):
        raise RuntimeError("R1 gates failed")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-05-R1",
        "status": "v3_05_r1_master_port_repair_review_pass",
        "report_sha256": builder.sha(outputs[builder.REPORT]),
        "evidence_freeze_sha256": builder.sha(
            outputs[builder.EVIDENCE_FREEZE]
        ),
        "dependency_resource_count": len(closure["resources"]),
        "all_gates_pass": True,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = builder.yaml_bytes(value)
    output = W / builder.INDEPENDENT_REVIEW
    if args.check:
        if not output.is_file() or output.read_bytes() != payload:
            raise RuntimeError("review differs")
    else:
        write_once(output, payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
