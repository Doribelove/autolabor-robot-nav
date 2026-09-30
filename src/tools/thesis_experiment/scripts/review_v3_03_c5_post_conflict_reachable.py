#!/usr/bin/env python3
"""Independent replay review for V3-03-C5."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_03_c5_post_conflict_reachable.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_03_c5_builder_review", BUILDER
    )
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
    report = yaml.safe_load(outputs[builder.REPORT])
    planner = yaml.safe_load(outputs[builder.EFFECTIVE_CONTRACT])
    closure = yaml.safe_load(outputs[builder.CLOSURE])
    for item in closure["resources"]:
        payload = (W / item["path"]).read_bytes()
        builder.core.require(
            sha(payload) == item["sha256"]
            and len(payload) == item["size_bytes"],
            "resource receipt differs: {}".format(item["path"]),
        )
    metrics = report["metrics"]
    builder.core.require(
        report["overall_pass"] is True
        and all(report["gates"].values())
        and planner["predictor_runtime"][
            "post_conflict_reachable_bypass"
        ]["learned_predictor_used_in_bypass"]
        is False
        and metrics["c4_recorded_episode_count"] == 9
        and metrics["c4_recorded_decision_count"] == 330
        and metrics["reachable_lower_bound_conservative_rate"] == 1.0,
        "C5 offline gate failed",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C5",
        "review_id": "v3_03_c5_independent_review_1",
        "status": "v3_03_c5_post_conflict_reachable_review_pass",
        "report_sha256": sha(outputs[builder.REPORT]),
        "effective_planner_contract_sha256": sha(
            outputs[builder.EFFECTIVE_CONTRACT]
        ),
        "evidence_freeze_sha256": sha(outputs[builder.EVIDENCE_FREEZE]),
        "c4_episode_count_replayed": metrics[
            "c4_recorded_episode_count"
        ],
        "c4_decision_count_replayed": metrics[
            "c4_recorded_decision_count"
        ],
        "reachable_bound_case_count": metrics[
            "reachable_bound_case_count"
        ],
        "dependency_resource_count": len(closure["resources"]),
        "all_gates_pass": True,
        "simulation_authorized": False,
        "training_authorized": False,
        "performance_claim_authorized": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    builder = load_builder()
    value = review(builder)
    payload = builder.yaml_bytes(value)
    output = W / builder.INDEPENDENT_REVIEW
    if args.check:
        builder.core.require(
            output.is_file() and output.read_bytes() == payload,
            "review differs",
        )
    else:
        write_once(output, payload)
    print("status={} review_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__":
    main()
