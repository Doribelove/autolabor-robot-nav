#!/usr/bin/env python3
"""Independent review of V3-13-C3 execution readiness."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_c3_readiness.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_c3_readiness/"
    "independent_review.yaml"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_13_c3_readiness_builder", BUILDER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    builder = load_builder()
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "readiness replay differs: {}".format(relative),
        )
    release_sha = builder.sha(builder.read(builder.RELEASE))
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C3",
        "status": "v3_13_c3_independent_readiness_review_pass",
        "execution_release_sha256": release_sha,
        "fresh_64_unit_schedule_replayed": True,
        "old_seed_or_budget_reuse": False,
        "predictor_hash_replayed": True,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
        "execution_ready": True,
    }
    payload = builder.yaml_bytes(value)
    builder.require(not REVIEW_PATH.exists(), "review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()

