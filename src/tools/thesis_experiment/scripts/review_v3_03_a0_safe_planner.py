#!/usr/bin/env python3
"""Independent deterministic replay review for V3-03-A0."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_03_a0_safe_planner.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_03_a0_builder", BUILDER_PATH)
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


def review(builder):
    outputs = builder.build()
    for path, payload in outputs.items():
        builder.core.require(
            (W / path).is_file() and (W / path).read_bytes() == payload,
            "builder replay differs: {}".format(path),
        )
    summary = yaml.safe_load(outputs[builder.SUMMARY])
    deterministic = yaml.safe_load(outputs[builder.DETERMINISTIC_REPORT])
    properties = yaml.safe_load(outputs[builder.PROPERTY_REPORT])
    closure = yaml.safe_load(outputs[builder.CLOSURE])
    for item in closure["resources"]:
        payload = (W / item["path"]).read_bytes()
        builder.core.require(
            sha(payload) == item["sha256"]
            and len(payload) == item["size_bytes"],
            "resource receipt differs: {}".format(item["path"]),
        )
    builder.core.require(
        summary["overall_pass"] is True
        and deterministic["overall_pass"] is True
        and properties["overall_pass"] is True
        and all(summary["gates"].values()),
        "offline gate failed",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-A0",
        "review_id": "v3_03_a0_independent_review_1",
        "status": "v3_03_a0_pure_offline_safe_planner_review_pass",
        "summary_sha256": sha(outputs[builder.SUMMARY]),
        "evidence_freeze_sha256": sha(outputs[builder.EVIDENCE_FREEZE]),
        "dependency_resource_count": len(closure["resources"]),
        "deterministic_scenes_replayed": deterministic["scene_count"],
        "property_scenes_replayed": properties["scene_count"],
        "digest_tamper_cases_replayed": properties["digest_tamper_count"],
        "all_gates_pass": True,
        "simulation_authorized": False,
        "training_authorized": False,
        "navigation_safety_claim_authorized": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    builder = load_builder()
    value = review(builder)
    payload = builder.yaml_bytes(value)
    path = W / builder.INDEPENDENT_REVIEW
    if args.check:
        builder.core.require(
            path.is_file() and path.read_bytes() == payload,
            "review differs",
        )
    else:
        write_once(path, payload)
    print("status={} review_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__":
    main()
