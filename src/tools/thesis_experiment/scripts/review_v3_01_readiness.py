#!/usr/bin/env python3
"""Independent deterministic review of the V3-01 release candidate."""

import argparse
import importlib.util
from pathlib import Path
import sys


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("v3_01_qualification", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_once(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing to overwrite different file: {}".format(path))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def review(module):
    contract = module.load_yaml_relative(WORKSPACE, module.CONTRACT_RELATIVE)
    preregistration = module.load_yaml_relative(
        WORKSPACE, module.PREREGISTRATION_RELATIVE
    )
    module.validate_contract(contract, preregistration)
    schedule = module.load_yaml_relative(WORKSPACE, module.SCHEDULE_RELATIVE)
    expected_schedule = module.exact_schedule(contract)
    module.require(schedule == expected_schedule, "schedule reproduction differs")
    bindings = module.load_yaml_relative(WORKSPACE, module.BINDINGS_RELATIVE)
    closure = module.load_yaml_relative(WORKSPACE, module.CLOSURE_RELATIVE)
    release = module.load_yaml_relative(WORKSPACE, module.RELEASE_RELATIVE)
    module.check_receipts(WORKSPACE, closure["source_resources"])
    module.check_receipts(WORKSPACE, closure["binary_resources"])
    for item in closure["executable_resources"]:
        payload = module.read_file_once(
            Path(item["path"]), allow_hard_links=True
        )
        module.require(
            module.sha256_bytes(payload) == item["sha256"],
            "executable binding differs",
        )
    module.require(closure["unresolved_count"] == 0, "closure unresolved")
    module.require(
        release["schedule"]["sha256"]
        == module.sha256_bytes(module.yaml_bytes(schedule)),
        "release schedule binding differs",
    )
    module.require(
        release["runtime_bindings"]["sha256"]
        == module.sha256_bytes(module.yaml_bytes(bindings)),
        "release bindings differ",
    )
    module.require(
        release["dependency_closure"]["sha256"]
        == module.sha256_bytes(module.yaml_bytes(closure)),
        "release closure differs",
    )
    module.require(
        release["execution_ready"] is False
        and release["execution_authorized"] is False
        and release["training_authorized"] is False
        and release["real_vehicle_authorized"] is False,
        "release authority differs",
    )
    review_value = {
        "schema_version": "1.0",
        "stage": "V3-01",
        "review_id": "v3_scope_v3_01_independent_release_review_1",
        "status": "v3_01_release_readiness_review_pass_authorization_absent",
        "release_sha256": module.sha256_bytes(
            module.read_file_once(WORKSPACE / module.RELEASE_RELATIVE)
        ),
        "schedule_digest": schedule["schedule_digest"],
        "schedule_row_count": len(schedule["rows"]),
        "unique_unit_count": len({row["unit_id"] for row in schedule["rows"]}),
        "fresh_seed_roster": sorted({row["seed"] for row in schedule["rows"]}),
        "policy_roster": sorted(
            {row["actor_policy_evaluator_only"] for row in schedule["rows"]}
        ),
        "candidate_roster": sorted(
            {row["candidate_id_evaluator_only"] for row in schedule["rows"]}
        ),
        "source_receipts_rehashed": len(closure["source_resources"]),
        "binary_receipts_rehashed": len(closure["binary_resources"]),
        "executable_receipts_rehashed": len(closure["executable_resources"]),
        "unresolved_count": 0,
        "execution_ready": True,
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
        "claim_scope": "crossing_reactive_actor_qualification_only",
    }
    review_payload = module.yaml_bytes(review_value)
    freeze_resources = [
        module.SCHEDULE_RELATIVE,
        module.BINDINGS_RELATIVE,
        module.CLOSURE_RELATIVE,
        module.RELEASE_RELATIVE,
    ]
    freeze_lines = [
        "{}  {}".format(
            module.sha256_bytes(module.read_file_once(WORKSPACE / relative)),
            relative.as_posix(),
        )
        for relative in freeze_resources
    ]
    freeze_lines.append(
        "{}  {}".format(
            module.sha256_bytes(review_payload), module.REVIEW_RELATIVE.as_posix()
        )
    )
    freeze_payload = ("\n".join(freeze_lines) + "\n").encode("utf-8")
    return review_payload, freeze_payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    module = load_module()
    review_payload, freeze_payload = review(module)
    outputs = {
        module.REVIEW_RELATIVE: review_payload,
        module.READINESS_FREEZE_RELATIVE: freeze_payload,
    }
    for relative, payload in outputs.items():
        path = WORKSPACE / relative
        if args.check:
            if not path.is_file() or path.read_bytes() != payload:
                raise module.V301Error("review output differs: {}".format(relative))
        else:
            write_once(path, payload)
    print(
        module.yaml_bytes(
            {
                "status": "v3_01_release_readiness_review_pass",
                "review_sha256": module.sha256_bytes(review_payload),
                "execution_ready": True,
                "execution_authorized": False,
            }
        ).decode("utf-8").strip()
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("V3-01 independent review failed closed: {}".format(exc), file=sys.stderr)
        raise
