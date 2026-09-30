#!/usr/bin/env python3
"""Fresh-seed feasibility and leakage gate for the V6 PCT training target."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import torch

from v5_e2e_scope_rl.asymmetric_ppo import POLICY_OBSERVATION_KEYS, AsymmetricActorCritic
from v5_e2e_scope_rl.privileged_distillation import PrivilegedRankSensorEnv
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(seed, family):
    environment = PrivilegedRankSensorEnv({"family": family})
    observation, _ = environment.reset(seed=seed)
    target_count = 0
    while True:
        target = environment.privileged_rank_target()
        action = environment.analytic_action() if target < 0 else target
        target_count += int(target >= 0)
        observation, _, terminated, truncated, info = environment.step(action)
        if terminated or truncated:
            break
    return {
        "seed": seed,
        "family": family,
        "goal_reached": bool(info["goal_reached"]),
        "contact_count": int(info["contact_count"]),
        "mask_violation_count": int(info["mask_violation_count"]),
        "restricted_time_s": float(
            info["restricted_elapsed_s"] if info["goal_reached"] else 20.0
        ),
        "privileged_target_decisions": target_count,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    rows = []
    for family, seed_start in ((FAMILY_NAMES[2], 951200), (FAMILY_NAMES[3], 951300)):
        rows.extend(run(seed, family) for seed in range(seed_start, seed_start + 100))
    episodes_path = arguments.output_directory / "teacher_component_episodes.csv"
    with episodes_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    partial = [row for row in rows if row["family"] == FAMILY_NAMES[2]]
    narrow = [row for row in rows if row["family"] == FAMILY_NAMES[3]]
    environment = PrivilegedRankSensorEnv()
    observation, _ = environment.reset(seed=951399)
    model = AsymmetricActorCritic()
    actor_checkpoint = model.actor.actor_checkpoint()
    leakage = {
        "policy_observation_keys_unchanged": tuple(observation) == POLICY_OBSERVATION_KEYS,
        "privileged_target_absent_from_policy_observation": "privileged_rank_target"
        not in observation,
        "actor_checkpoint_contains_privileged_modules": actor_checkpoint[
            "contains_privileged_modules"
        ],
        "privileged_parameter_names": [
            name for name in actor_checkpoint["state_dict"] if "privileg" in name.lower()
        ],
    }
    gates = {
        "partial_teacher_goal_fraction_minimum_0_80": sum(
            row["goal_reached"] for row in partial
        )
        / len(partial)
        >= 0.80,
        "narrow_teacher_goal_fraction_minimum_0_50": sum(
            row["goal_reached"] for row in narrow
        )
        / len(narrow)
        >= 0.50,
        "zero_contact": sum(row["contact_count"] for row in rows) == 0,
        "zero_mask_violation": sum(row["mask_violation_count"] for row in rows) == 0,
        "observation_schema_unchanged": leakage["policy_observation_keys_unchanged"],
        "target_not_in_actor_input": leakage[
            "privileged_target_absent_from_policy_observation"
        ],
        "actor_checkpoint_privilege_free": (
            leakage["actor_checkpoint_contains_privileged_modules"] is False
            and not leakage["privileged_parameter_names"]
        ),
    }
    report = {
        "status": (
            "v6_00_pct_component_gate_pass"
            if all(gates.values())
            else "v6_00_pct_component_gate_fail"
        ),
        "preflight_seed_range_retained_without_episode_rows": [951000, 951199],
        "preflight_aggregate": {
            "partial_goal_count": 87,
            "narrow_goal_count": 72,
            "episodes_per_family": 100,
            "contact_count": 0,
        },
        "audit_seed_range": [951200, 951399],
        "partial_goal_count": sum(row["goal_reached"] for row in partial),
        "narrow_goal_count": sum(row["goal_reached"] for row in narrow),
        "episodes_per_family": 100,
        "contact_count": sum(row["contact_count"] for row in rows),
        "mask_violation_count": sum(row["mask_violation_count"] for row in rows),
        "leakage": leakage,
        "gates": gates,
        "pass": all(gates.values()),
        "performance_claim_authorized": False,
    }
    report_path = arguments.output_directory / "component_audit.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "teacher_component_episodes.csv": sha256(episodes_path),
        "component_audit.json": sha256(report_path),
    }
    (arguments.output_directory / "sha256_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()

