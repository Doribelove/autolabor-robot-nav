#!/usr/bin/env python3
"""Build the zero-authority V3-13-S1 sequential support foundation."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s1_sequential_clone_contract.yaml"
)
D0_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
RUNTIME = Path(
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_multiactor_sequential_clone_episode.py"
)
LAUNCH = Path(
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_sequential_clone_episode.launch"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/build_v3_13_s1_foundation.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/review_v3_13_s1_foundation.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/test_v3_13_s1_design.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_s1_foundation")
SCHEDULE = ROOT / "future_exact_schedule.yaml"
READINESS = ROOT / "offline_wait_state_readiness.yaml"
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


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def load_core():
    path = (
        W
        / "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core.py"
    )
    spec = importlib.util.spec_from_file_location(
        "v3_13_s1_foundation_core", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def schedule_rows(contract):
    rows = []
    sequence = 0
    block_index = 0
    actions = contract["population"]["branch_actions"]
    seeds = contract["population"]["seeds"]
    for stratum in contract["population"]["scene_policy_strata"]:
        for wait_depth_s in contract["population"][
            "pre_action_wait_depths_s"
        ]:
            seed = int(seeds[block_index])
            block_index += 1
            order = list(actions)
            if block_index % 2 == 0:
                order.reverse()
            for candidate_id in order:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "block_index": block_index,
                        "seed": seed,
                        "scene_id": stratum["scene_id"],
                        "actor_policies_evaluator_only": list(
                            stratum["actor_policies_evaluator_only"]
                        ),
                        "pre_action_wait_s": float(wait_depth_s),
                        "forced_candidate_id": candidate_id,
                        "behavior_propensity": 0.25,
                        "attempt": 1,
                    }
                )
    return rows


def c3_stop_reports():
    reports = []
    roots = sorted(
        (
            W
            / "artifacts/v3/multiactor/v3_13_c3_execution/episodes"
        ).glob("*certified_stop_v000")
    )
    for root in roots:
        report = yaml.safe_load(
            (root / "episode_report.yaml").read_bytes()
        )
        key = (
            report["scene_id"],
            tuple(report["actor_policies_evaluator_only"]),
        )
        if key in {
            (
                item["scene_id"],
                tuple(item["actor_policies_evaluator_only"]),
            )
            for item in load_yaml(CONTRACT)["population"][
                "scene_policy_strata"
            ]
        }:
            reports.append(report)
    require(len(reports) == 4, "risk stop report count")
    return reports


def build_outputs():
    contract = load_yaml(CONTRACT)
    authority = contract["authority"]
    require(not authority["simulation_execution_authorized"], "execution authority")
    require(authority["authorized_units"] == 0, "unit authority")
    require(not authority["training_authorized"], "training authority")
    rows = schedule_rows(contract)
    require(len(rows) == 32, "future schedule row count")
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "future_schedule_frozen_without_execution_authority",
        "row_count": len(rows),
        "rows": rows,
    }
    core = load_core()
    d0 = load_yaml(D0_CONTRACT)
    wait_rows = []
    for report in c3_stop_reports():
        for wait_depth_s in contract["population"][
            "pre_action_wait_depths_s"
        ]:
            sample = min(
                report["result"]["trace"],
                key=lambda item: abs(
                    float(item["elapsed_s"]) - float(wait_depth_s)
                ),
            )
            modes = [
                core.infer_observable_mode(
                    float(actor["coordinate_speed_mps"]), d0
                )
                for actor in sample["actors"]
            ]
            wait_rows.append(
                {
                    "source_c3_block_index": report["block_index"],
                    "scene_id": report["scene_id"],
                    "actor_policies_evaluator_only": report[
                        "actor_policies_evaluator_only"
                    ],
                    "wait_depth_s": float(wait_depth_s),
                    "observed_sample_time_s": float(
                        sample["elapsed_s"]
                    ),
                    "ego_x_m": float(sample["ego"]["x_m"]),
                    "ego_speed_mps": float(sample["ego"]["speed_mps"]),
                    "observable_modes": modes,
                    "all_assertive": modes == ["assertive", "assertive"],
                }
            )
    readiness = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "offline_sequential_wait_state_foundation_pass",
        "source": "fresh C3 certified-stop traces; readiness only",
        "wait_state_count": len(wait_rows),
        "wait_states": wait_rows,
        "all_wait_states_assertive": all(
            row["all_assertive"] for row in wait_rows
        ),
        "all_wait_states_stopped": all(
            abs(row["ego_speed_mps"])
            <= float(
                contract["common_prefix"]["decision_speed_mps_max"]
            )
            for row in wait_rows
        ),
        "all_wait_states_in_ego_x_range": all(
            float(
                contract["common_prefix"]["decision_ego_x_range_m"][0]
            )
            <= row["ego_x_m"]
            <= float(
                contract["common_prefix"]["decision_ego_x_range_m"][1]
            )
            for row in wait_rows
        ),
        "matched_post_stop_alternative_actions_collected": False,
        "simulation_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    require(readiness["all_wait_states_assertive"], "assertive wait state absent")
    require(readiness["all_wait_states_stopped"], "wait state not stopped")
    require(
        readiness["all_wait_states_in_ego_x_range"],
        "wait state x outside range",
    )
    return {
        SCHEDULE: yaml_bytes(schedule),
        READINESS: yaml_bytes(readiness),
    }


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    resources = [
        CONTRACT,
        D0_CONTRACT,
        RUNTIME,
        LAUNCH,
        BUILDER,
        REVIEWER,
        TEST,
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path) for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_s1_foundation_{} manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
