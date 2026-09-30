#!/usr/bin/env python3
"""Independent replay review for V3-04-A0."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_04_a0_learning_audit.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_04_a0_builder_review", BUILDER
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
    closure = yaml.safe_load(outputs[builder.CLOSURE])
    for item in closure["resources"]:
        payload = (W / item["path"]).read_bytes()
        builder.core.require(
            sha(payload) == item["sha256"]
            and len(payload) == item["size_bytes"],
            "resource receipt differs: {}".format(item["path"]),
        )
    metrics = report["metrics"]
    observations = report["learning_activation_observations"]
    builder.core.require(
        report["overall_pass"] is True
        and all(report["integrity_gates"].values())
        and observations["all_learning_activation_requirements_pass"]
        is False
        and metrics["training_steps"] == 0
        and metrics["learned_model_artifact_count"] == 0
        and report["disposition"]["analytic_ranker_retained"] is True,
        "V3-04-A0 disposition differs",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-04-A0",
        "review_id": "v3_04_a0_independent_review_1",
        "status": "v3_04_a0_learning_not_justified_review_pass",
        "report_sha256": sha(outputs[builder.REPORT]),
        "evidence_freeze_sha256": sha(outputs[builder.EVIDENCE_FREEZE]),
        "c6_decision_count_replayed": metrics["c6_decision_count"],
        "multi_safe_action_decision_count": metrics[
            "multi_safe_action_decision_count"
        ],
        "dependency_resource_count": len(closure["resources"]),
        "learning_activated": False,
        "training_steps": 0,
        "analytic_ranker_retained": True,
        "all_integrity_gates_pass": True,
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
    terminal_resources = [
        builder.REPORT,
        builder.CLOSURE,
        builder.EVIDENCE_FREEZE,
        builder.INDEPENDENT_REVIEW,
    ]
    if args.check:
        builder.core.require(
            output.is_file() and output.read_bytes() == payload,
            "review differs",
        )
        expected = (
            "\n".join(
                "{}  {}".format(
                    sha((W / item).read_bytes()), item.as_posix()
                )
                for item in terminal_resources
            )
            + "\n"
        ).encode("utf-8")
        builder.core.require(
            (W / builder.TERMINAL_FREEZE).read_bytes() == expected,
            "terminal freeze differs",
        )
    else:
        write_once(output, payload)
        freeze = (
            "\n".join(
                "{}  {}".format(
                    sha((W / item).read_bytes()), item.as_posix()
                )
                for item in terminal_resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / builder.TERMINAL_FREEZE, freeze)
    print("status={} review_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__":
    main()
