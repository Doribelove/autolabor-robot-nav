#!/usr/bin/env python3
"""Build deterministic zero-authority V3-13-D0 offline fixtures."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
CORE = Path(
    "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core.py"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/build_v3_13_d0_foundation.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/review_v3_13_d0_foundation.py"
)
TEST = Path("src/tools/thesis_experiment/tests/test_v3_13_d0.py")
ROOT = Path("artifacts/v3/multiactor/v3_13_d0_foundation")
CANDIDATES = ROOT / "candidate_bank.yaml"
FIXTURES = ROOT / "offline_fixture_acceptance.yaml"
CLONES = ROOT / "cloned_intervention_schema_receipt.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_core():
    spec = importlib.util.spec_from_file_location("v3_13_core", W / CORE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        require(path.read_bytes() == payload, "refusing overwrite: {}".format(relative))
    else:
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))


def base_snapshot(scene_id="dual_crossing_staggered_open"):
    return {
        "scene_snapshot_id": "v3_13_d0_" + scene_id,
        "world_model_seq": 1,
        "ego": {
            "x_m": -2.6,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "speed_mps": 0.8,
        },
        "actors": [
            {
                "track_id": 1,
                "coordinate_m": -2.9,
                "coordinate_speed_mps": 0.12,
                "path_angle_deg": 90.0,
                "path_origin_x_m": 0.0,
                "path_origin_y_m": 0.0,
                "age_s": 0.01,
            },
            {
                "track_id": 2,
                "coordinate_m": -2.7,
                "coordinate_speed_mps": 0.32,
                "path_angle_deg": 110.0,
                "path_origin_x_m": 2.2,
                "path_origin_y_m": 0.0,
                "age_s": 0.01,
            },
        ],
        "scene": {
            "scene_id": scene_id,
            "corridor_half_width_m": 2.0,
            "static_rectangles": [],
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


def fixture_snapshots():
    nominal = base_snapshot()
    left_blocked = base_snapshot("left_blocked_fixture")
    for actor in left_blocked["actors"]:
        actor["path_origin_y_m"] = 20.0
    left_blocked["scene"]["static_rectangles"] = [
        {
            "x_min_m": 0.5,
            "x_max_m": 1.5,
            "y_min_m": 0.4,
            "y_max_m": 1.4,
        }
    ]
    right_blocked = base_snapshot("right_blocked_fixture")
    for actor in right_blocked["actors"]:
        actor["path_origin_y_m"] = 20.0
    right_blocked["scene"]["static_rectangles"] = [
        {
            "x_min_m": 0.5,
            "x_max_m": 1.5,
            "y_min_m": -1.4,
            "y_max_m": -0.4,
        }
    ]
    narrow = base_snapshot("narrow_corridor_fixture")
    for actor in narrow["actors"]:
        actor["path_origin_y_m"] = 20.0
    narrow["scene"]["corridor_half_width_m"] = 1.35
    joint = base_snapshot("joint_actor_aggregation_fixture")
    joint["actors"][0].update(
        {
            "coordinate_m": -2.7,
            "coordinate_speed_mps": 0.32,
            "path_angle_deg": -110.0,
            "path_origin_x_m": 2.2,
            "path_origin_y_m": 0.0,
        }
    )
    stale = base_snapshot("stale_track_fixture")
    stale["actors"][1]["age_s"] = 0.25
    return [nominal, left_blocked, right_blocked, narrow, joint, stale]


def build_outputs():
    contract = load_yaml(CONTRACT)
    core = load_core()
    authority = contract["authority"]
    require(authority["ros_or_gazebo_execution_authorized"] is False, "execution authority differs")
    require(authority["evidence_seed_consumption_authorized"] is False, "seed authority differs")
    require(authority["training_authorized"] is False, "training authority differs")
    snapshot = base_snapshot()
    candidate_set = core.generate_candidate_set(snapshot, contract)
    candidate_validation = {
        candidate["candidate_id"]: list(
            core.validate_candidate(candidate, contract)
        )
        for candidate in candidate_set["candidates"]
    }
    candidate_bank = {
        "schema_version": "1.0",
        "stage": "V3-13-D0",
        "status": "offline_candidate_bank_fixture_pass",
        "candidate_set": candidate_set,
        "candidate_validation_failures": candidate_validation,
        "all_candidates_valid": all(
            not failures for failures in candidate_validation.values()
        ),
        "training_steps": 0,
    }
    fixture_rows = []
    for fixture in fixture_snapshots():
        try:
            result = core.plan(
                fixture, contract["offline_fixture_model"], contract
            )
            fixture_rows.append(
                {
                    "scene_snapshot_id": fixture["scene_snapshot_id"],
                    "status": "PLAN_COMPLETE",
                    "selected_candidate_id": result["decision"][
                        "selected_candidate_id"
                    ],
                    "safety_fallback_used": result["decision"][
                        "safety_fallback_used"
                    ],
                    "accepted_candidate_ids": [
                        item["candidate_id"]
                        for item in result["evaluations"]
                        if item["accepted"]
                    ],
                    "rejected_candidates": {
                        item["candidate_id"]: item["rejection_codes"]
                        for item in result["evaluations"]
                        if not item["accepted"]
                    },
                    "result_digest": core.digest(result),
                }
            )
        except core.MultiActorPlannerError as error:
            fixture_rows.append(
                {
                    "scene_snapshot_id": fixture["scene_snapshot_id"],
                    "status": "FAIL_CLOSED",
                    "error": str(error),
                }
            )
    order_snapshot = base_snapshot("actor_order_invariance")
    forward = core.plan(
        order_snapshot, contract["offline_fixture_model"], contract
    )
    reverse_snapshot = dict(order_snapshot)
    reverse_snapshot["actors"] = list(reversed(order_snapshot["actors"]))
    reverse = core.plan(
        reverse_snapshot, contract["offline_fixture_model"], contract
    )
    order_invariant = (
        forward["decision"]["selected_candidate_id"]
        == reverse["decision"]["selected_candidate_id"]
        and [
            (item["candidate_id"], item["accepted"], item["rejection_codes"])
            for item in forward["evaluations"]
        ]
        == [
            (item["candidate_id"], item["accepted"], item["rejection_codes"])
            for item in reverse["evaluations"]
        ]
    )
    fixtures = {
        "schema_version": "1.0",
        "stage": "V3-13-D0",
        "status": "offline_multiactor_fixture_acceptance_pass",
        "fixture_count": len(fixture_rows),
        "fixtures": fixture_rows,
        "actor_order_invariant": order_invariant,
        "stale_track_fails_closed": any(
            row["scene_snapshot_id"].endswith("stale_track_fixture")
            and row["status"] == "FAIL_CLOSED"
            for row in fixture_rows
        ),
        "differential_action_expectations": {
            "left_blocked_fixture": "spatial_right_v120",
            "right_blocked_fixture": "spatial_left_v120",
            "narrow_corridor_fixture": "straight_v120",
            "joint_actor_aggregation_fixture": "certified_stop_v000",
        },
        "hidden_policy_runtime_input": False,
        "predictor_qualified": False,
        "performance_claim_authorized": False,
        "training_steps": 0,
    }
    candidate_ids = [
        spec["candidate_id"]
        for spec in contract["candidate_generator"]["candidates"]
    ]
    initial_digest = core.digest(base_snapshot())
    clone_rows = [
        {
            "candidate_id": candidate_id,
            "initial_state_digest": initial_digest,
            "behavior_propensity": 1.0 / len(candidate_ids),
            "evaluator_hidden_type": "cautious_hesitant",
            "hidden_policy_runtime_input": False,
        }
        for candidate_id in candidate_ids
    ]
    clone_receipt = core.validate_cloned_intervention_block(
        clone_rows, candidate_ids
    )
    clones = {
        "schema_version": "1.0",
        "stage": "V3-13-D0",
        "status": "offline_clone_schema_fixture_pass",
        "rows": clone_rows,
        "receipt": clone_receipt,
        "simulation_data_collected": False,
        "learning_support_qualified": False,
        "training_authorized": False,
    }
    require(candidate_bank["all_candidates_valid"], "candidate bank invalid")
    require(order_invariant, "actor order changes plan")
    require(fixtures["stale_track_fails_closed"], "stale track did not fail closed")
    fixture_by_name = {
        row["scene_snapshot_id"].replace("v3_13_d0_", ""): row
        for row in fixture_rows
    }
    for fixture_name, expected_candidate in fixtures[
        "differential_action_expectations"
    ].items():
        require(
            fixture_by_name[fixture_name]["status"] == "PLAN_COMPLETE"
            and fixture_by_name[fixture_name]["selected_candidate_id"]
            == expected_candidate,
            "{} did not select {}".format(
                fixture_name, expected_candidate
            ),
        )
    require(
        fixture_by_name["joint_actor_aggregation_fixture"][
            "safety_fallback_used"
        ],
        "joint actor aggregation did not trigger certified stop",
    )
    require(clone_receipt["full_action_support"], "clone support fixture failed")
    return {
        CANDIDATES: yaml_bytes(candidate_bank),
        FIXTURES: yaml_bytes(fixtures),
        CLONES: yaml_bytes(clones),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    resources = [CONTRACT, CORE, BUILDER, REVIEWER, TEST] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path) for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_d0_foundation_{} manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
