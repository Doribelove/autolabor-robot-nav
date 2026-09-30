#!/usr/bin/env python3
"""Independent V3-01-R1 single-factor release review."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_01_r1", M)
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


def build_review(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    prereg = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, prereg)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
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
        and "-u" not in bindings["gazebo_arguments"],
        "pause flag repair differs",
    )
    if (
        contract["single_factor_repair"]["factor_id"]
        == "actor_response_margin_seed_jitter_fraction"
    ):
        actor_source = v.read_file_once(
            W
            / "src/simulation/m2_gazebo/include/m2_gazebo/"
            "v3_reactive_actor.h"
        ).decode("utf-8")
        v.require(
            "const double jitter = 0.0 * v3SignedSeedJitter" in actor_source,
            "R2 jitter repair is absent",
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
    review = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "review_id": "{}_independent_release_review_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": "{}_single_factor_release_review_pass".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "release_sha256": v.sha256_bytes(
            v.read_file_once(W / v.RELEASE_RELATIVE)
        ),
        "single_factor_count": 1,
        "pause_flag_removed": True,
        "scene_actor_recorder_thresholds_unchanged": True,
        "schedule_row_count": 36,
        "fresh_seeds": contract["experimental_unit"]["fresh_seeds"],
        "unresolved_count": 0,
        "execution_ready": True,
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    review_bytes = v.yaml_bytes(review)
    frozen = [
        v.SCHEDULE_RELATIVE,
        v.BINDINGS_RELATIVE,
        v.CLOSURE_RELATIVE,
        v.RELEASE_RELATIVE,
    ]
    lines = [
        "{}  {}".format(
            v.sha256_bytes(v.read_file_once(W / item)), item.as_posix()
        )
        for item in frozen
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
    review, freeze = build_review(v)
    for relative, payload in {
        v.REVIEW_RELATIVE: review,
        v.READINESS_FREEZE_RELATIVE: freeze,
    }.items():
        path = W / relative
        if args.check:
            v.require(path.is_file() and path.read_bytes() == payload, "review differs")
        else:
            write_once(path, payload)
    print("review_sha256={}".format(v.sha256_bytes(review)))


if __name__ == "__main__":
    main()
