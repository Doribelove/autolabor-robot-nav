#!/usr/bin/env python3
"""Independent replay of the V3-13-Q1 preflight failure freeze."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "freeze_v3_13_q1_preflight_failure.py"
)
FREEZE = W / (
    "artifacts/v3/multiactor/v3_13_q1_preflight/"
    "terminal_failure_freeze.yaml"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_preflight/"
    "terminal_failure_independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_failure_builder", BUILDER
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    expected = builder.build()
    if FREEZE.read_bytes() != expected:
        raise RuntimeError("failure freeze replay differs")
    freeze = yaml.safe_load(expected)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-PREFLIGHT",
        "status": "terminal_failure_independent_review_pass",
        "terminal_failure_freeze_sha256": hashlib.sha256(
            expected
        ).hexdigest(),
        "semantic_execution_reached": False,
        "consumed_units": freeze["consumed_units"],
        "forfeited_units": freeze["forfeited_units"],
        "same_identity_retry_or_resume_authorized": False,
        "new_overlay_only_repair_stage_required": True,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    if REVIEW.exists():
        raise RuntimeError("failure review exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
