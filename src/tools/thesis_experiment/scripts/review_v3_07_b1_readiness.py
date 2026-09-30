#!/usr/bin/env python3
"""Independent review of the V3-07-B1 held-out release."""

import argparse
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_b1.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
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
    preregistration = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, preregistration)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    v.require(
        sorted({int(row["seed"]) for row in schedule["rows"]})
        == [6661, 6662, 6663, 6664]
        and all(row["role_evaluator_only"] == "held_out_test" for row in schedule["rows"])
        and schedule["held_out_test_rows"] == 20,
        "held-out roster differs",
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
        bindings["gazebo_arguments"] == ["--verbose", "WORLD"]
        and bindings["ros_master_required"] is False
        and bindings["hardware_device_access_required"] is False,
        "runtime binding differs",
    )
    v.require(
        release["schedule"]["sha256"] == v.sha256_bytes(v.yaml_bytes(schedule))
        and release["runtime_bindings"]["sha256"]
        == v.sha256_bytes(v.yaml_bytes(bindings))
        and release["dependency_closure"]["sha256"]
        == v.sha256_bytes(v.yaml_bytes(closure))
        and release["frozen_model"]["sha256"]
        == v.sha256_bytes(v.read_file_once(W / v.MODEL_RELATIVE)),
        "release binding differs",
    )
    value = {
        "schema_version": "1.0",
        "stage": "V3-07-B1",
        "review_id": "v3_07_b1_independent_release_review_1",
        "status": "v3_07_b1_heldout_release_review_pass",
        "release_sha256": v.sha256_bytes(v.read_file_once(W / v.RELEASE_RELATIVE)),
        "schedule_row_count": 20,
        "held_out_test_row_count": 20,
        "fresh_held_out_seeds": [6661, 6662, 6663, 6664],
        "frozen_model_sha256": release["frozen_model"]["sha256"],
        "hidden_policy_evaluator_only": True,
        "unresolved_count": 0,
        "execution_ready": True,
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    review_bytes = v.yaml_bytes(value)
    resources = [
        v.SCHEDULE_RELATIVE, v.BINDINGS_RELATIVE, v.CLOSURE_RELATIVE,
        v.RELEASE_RELATIVE,
    ]
    lines = [
        "{}  {}".format(v.sha256_bytes(v.read_file_once(W / item)), item)
        for item in resources
    ]
    lines.append(
        "{}  {}".format(v.sha256_bytes(review_bytes), v.REVIEW_RELATIVE)
    )
    return review_bytes, ("\n".join(lines) + "\n").encode()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(M, "v3_07_b1_review")
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
