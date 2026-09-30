#!/usr/bin/env python3
"""Freeze the formal analysis implementation before V3-11-F1 execution."""

import hashlib
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT = (
    WORKSPACE
    / "artifacts/v3/actual_teb/v3_11_f1_release/"
    "formal_analysis_release.yaml"
)
FORMAL_RELEASE = (
    WORKSPACE
    / "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_formal_release.yaml"
)
EXPECTED_FORMAL_RELEASE_SHA256 = (
    "b21835d5d55a55fd130aae836c609fd117481b9d4cd54155ba8fc78b57dc9ea6"
)
ROSTER = [
    "src/tools/thesis_experiment/scripts/assess_v3_11_f1_actual_teb_formal.py",
    "src/tools/thesis_experiment/scripts/review_v3_11_f1_actual_teb_formal.py",
    "config/thesis_experiments/v3/v3_11_f1_actual_teb_formal_contract.yaml",
    "experiments/manifests/v3/v3_11_f1_actual_teb_formal_release.yaml",
    "experiments/manifests/v3/v3_11_f1_actual_teb_simulation_authorization.yaml",
    "artifacts/v3/actual_teb/v3_11_f1_release/exact_schedule.yaml",
    "artifacts/v3/actual_teb/v3_11_f1_release/dependency_freeze.yaml",
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    require(path.is_file(), "required file absent: {}".format(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    require(not OUTPUT.exists(), "immutable analysis release already exists")
    require(
        digest(FORMAL_RELEASE) == EXPECTED_FORMAL_RELEASE_SHA256,
        "formal execution release changed",
    )
    resources = []
    for relative in ROSTER:
        path = WORKSPACE / relative
        resources.append(
            {
                "path": relative,
                "sha256": digest(path),
                "size_bytes": path.stat().st_size,
            }
        )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1",
        "status": "immutable_preexecution_analysis_release",
        "formal_execution_release_sha256": EXPECTED_FORMAL_RELEASE_SHA256,
        "analysis_method": (
            "raw paired replay; Python random.Random(31100) Mersenne Twister; "
            "200000 block-bootstrap replicates; lower order statistic at "
            "floor(0.025*200000)"
        ),
        "post_execution_analysis_change_authorized": False,
        "selective_exclusion_authorized": False,
        "resources": resources,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    temporary = OUTPUT.with_suffix(OUTPUT.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(OUTPUT))
    print(
        "status={} resources={} analysis_release_sha256={}".format(
            value["status"],
            len(resources),
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
