#!/usr/bin/env python3
"""Independent V3-02-A1 release review."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_02_predictor", M)
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
    reserved = set(
        contract["authority_boundary"]["held_out_test_seeds_reserved"]
    )
    v.require(
        all(row["seed"] not in reserved for row in schedule["rows"])
        and schedule["held_out_test_rows"] == 0,
        "held-out firewall failed",
    )
    bindings = v.load_yaml_relative(W, v.BINDINGS_RELATIVE)
    closure = v.load_yaml_relative(W, v.CLOSURE_RELATIVE)
    release = v.load_yaml_relative(W, v.RELEASE_RELATIVE)
    v.check_receipts(W, closure["source_resources"])
    v.check_receipts(W, closure["binary_resources"])
    for item in closure["executable_resources"]:
        payload = v.read_file_once(Path(item["path"]), allow_hard_links=True)
        v.require(v.sha256_bytes(payload) == item["sha256"], "executable differs")
    v.require(
        bindings["gazebo_arguments"] == ["--verbose", "WORLD"],
        "Gazebo command differs",
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
        "status": "{}_data_release_review_pass".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "release_sha256": v.sha256_bytes(v.read_file_once(W / v.RELEASE_RELATIVE)),
        "schedule_row_count": schedule["row_count"],
        "training_row_count": schedule["training_rows"],
        "calibration_row_count": schedule["calibration_rows"],
        "held_out_test_row_count": 0,
        "reserved_test_seeds_absent": sorted(reserved),
        "hidden_policy_evaluator_only": True,
        "unresolved_count": 0,
        "execution_ready": True,
        "execution_authorized": False,
        "held_out_test_authorized": False,
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
