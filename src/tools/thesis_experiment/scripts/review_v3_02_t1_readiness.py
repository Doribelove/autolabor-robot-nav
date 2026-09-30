#!/usr/bin/env python3
"""Independent V3-02-T1 frozen-model release review."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_02_t1", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def review(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    prereg = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, prereg)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    closure = v.load_yaml_relative(W, v.CLOSURE_RELATIVE)
    bindings = v.load_yaml_relative(W, v.BINDINGS_RELATIVE)
    release = v.load_yaml_relative(W, v.RELEASE_RELATIVE)
    v.check_receipts(W, closure["source_resources"])
    v.check_receipts(W, closure["binary_resources"])
    expected_seeds = {
        row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]
    }
    v.require(
        {row["seed"] for row in schedule["rows"]} == expected_seeds
        and len(schedule["rows"]) == int(schedule["row_count"]),
        "held-out schedule differs",
    )
    v.require(
        release["frozen_model_sha256"] == contract["frozen_model"]["sha256"]
        and release["model_write_allowed"] is False,
        "model firewall differs",
    )
    v.require(
        release["schedule"]["sha256"]
        == v.sha256_bytes(v.yaml_bytes(schedule))
        and release["runtime_bindings"]["sha256"]
        == v.sha256_bytes(v.yaml_bytes(bindings))
        and release["dependency_closure"]["sha256"]
        == v.sha256_bytes(v.yaml_bytes(closure)),
        "release binding differs",
    )
    value = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "review_id": "{}_independent_release_review_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": "{}_frozen_model_release_review_pass".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "release_sha256": v.sha256_bytes(v.read_file_once(W / v.RELEASE_RELATIVE)),
        "frozen_model_sha256": contract["frozen_model"]["sha256"],
        "test_row_count": schedule["row_count"],
        "test_seeds": sorted(expected_seeds),
        "refit_or_recalibration_allowed": False,
        "unresolved_count": 0,
        "execution_ready": True,
        "execution_authorized": False,
        "real_vehicle_authorized": False,
    }
    review_bytes = v.yaml_bytes(value)
    resources = [
        v.SCHEDULE_RELATIVE,
        v.BINDINGS_RELATIVE,
        v.CLOSURE_RELATIVE,
        v.RELEASE_RELATIVE,
    ]
    lines = [
        "{}  {}".format(
            v.sha256_bytes(v.read_file_once(W / item)), item.as_posix()
        )
        for item in resources
    ]
    lines.append(
        "{}  {}".format(v.sha256_bytes(review_bytes), v.REVIEW_RELATIVE.as_posix())
    )
    return review_bytes, ("\n".join(lines) + "\n").encode("utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    review_bytes, freeze = review(v)
    for relative, payload in {
        v.REVIEW_RELATIVE: review_bytes,
        v.READINESS_FREEZE_RELATIVE: freeze,
    }.items():
        path = W / relative
        if args.check:
            v.require(path.is_file() and path.read_bytes() == payload, "review differs")
        else:
            write_once(path, payload)
    print("review_sha256={}".format(v.sha256_bytes(review_bytes)))


if __name__ == "__main__":
    main()
