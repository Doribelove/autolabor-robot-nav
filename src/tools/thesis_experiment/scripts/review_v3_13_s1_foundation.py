#!/usr/bin/env python3
"""Independent replay of the zero-authority V3-13-S1 foundation."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s1_foundation.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s1_foundation/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s1_builder", BUILDER_PATH
    )
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    outputs = builder.build_outputs()
    for relative, payload in outputs.items():
        builder.require(
            builder.read(relative) == payload,
            "S1 foundation replay differs: {}".format(relative),
        )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "v3_13_s1_independent_offline_foundation_review_pass",
        "foundation_manifest_sha256": builder.sha(
            builder.read(builder.MANIFEST)
        ),
        "future_32_unit_schedule_replayed": True,
        "early_and_late_assertive_wait_states_replayed": True,
        "matched_post_stop_support_still_absent": True,
        "simulation_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
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

