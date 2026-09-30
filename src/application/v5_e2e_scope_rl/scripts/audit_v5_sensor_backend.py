#!/usr/bin/env python3
"""Run the frozen V5-01 sensor/shield/learnability component audit."""

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
import torch

from v5_e2e_scope_rl.asymmetric_ppo import (
    AsymmetricActorCritic,
    SensorActor,
    observations_to_torch,
    stack_observations,
)
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES, SensorCandidateEnv


def run_policy(seed, family, method):
    environment = SensorCandidateEnv({"family": family})
    observation, _ = environment.reset(seed=seed)
    initial_side_y = (
        environment.static_circles[0]["y"]
        if environment.static_circles
        else environment.actors[0]["y"]
    )
    action_counts = Counter()
    safe_state_count = 0
    decision_count = 0
    while True:
        safe_actions = np.flatnonzero(observation["action_mask"])
        safe_state_count += int(len(safe_actions) >= 2)
        decision_count += 1
        if method == "analytic":
            action = environment.analytic_action()
        elif method == "masked_random":
            action = int(environment.np_random.choice(safe_actions))
        elif method == "evaluator_side_oracle":
            spatial = [7, 5] if initial_side_y > 0.0 else [6, 4]
            available = [candidate for candidate in spatial if candidate in safe_actions]
            if available:
                action = available[0]
            else:
                action = next(
                    (candidate for candidate in (3, 2, 1, 0) if candidate in safe_actions),
                    0,
                )
        else:
            raise ValueError("UNKNOWN_AUDIT_METHOD")
        action_counts[action] += 1
        observation, _, terminated, truncated, info = environment.step(action)
        if terminated or truncated:
            break
    restricted = (
        info["restricted_elapsed_s"]
        if info["goal_reached"]
        else environment.max_steps * environment.dt
    )
    return {
        "seed": int(seed),
        "family": family,
        "method": method,
        "goal_reached": bool(info["goal_reached"]),
        "contact_count": int(info["contact_count"]),
        "mask_violation_count": int(info["mask_violation_count"]),
        "restricted_time_s": float(restricted),
        "minimum_physical_clearance_m": float(info["minimum_physical_clearance_m"]),
        "safe_multiaction_state_fraction": float(safe_state_count / decision_count),
        "decision_count": int(decision_count),
        "action_counts": json.dumps(dict(sorted(action_counts.items())), sort_keys=True),
    }


def ridge_direction_accuracy(train_count=160, test_count=120):
    features = []
    current_features = []
    labels = []
    for index in range(train_count + test_count):
        side = -1.0 if index % 2 == 0 else 1.0
        environment = SensorCandidateEnv({"family": FAMILY_NAMES[0]})
        observation, _ = environment.reset(
            seed=881000 + index,
            options={"actor_side": side},
        )
        scans = observation["scans"].astype(np.float64)
        # The fit sees only normalized laser history.  Labels are evaluator-only
        # motion directions and are never appended to the input.
        features.append(scans.reshape(-1))
        current_features.append(scans[-1])
        labels.append(-side)
    labels = np.asarray(labels, dtype=np.float64)

    def fit_and_score(values):
        values = np.asarray(values, dtype=np.float64)
        train = values[:train_count]
        test = values[train_count:]
        mean = train.mean(axis=0)
        scale = train.std(axis=0)
        scale[scale < 1.0e-5] = 1.0
        train = (train - mean) / scale
        test = (test - mean) / scale
        design = np.concatenate((train, np.ones((train_count, 1))), axis=1)
        regularizer = 0.08
        gram = design @ design.T + regularizer * np.eye(train_count)
        dual = np.linalg.solve(gram, labels[:train_count])
        weights = design.T @ dual
        test_design = np.concatenate((test, np.ones((test_count, 1))), axis=1)
        predictions = np.where(test_design @ weights >= 0.0, 1.0, -1.0)
        return float(np.mean(predictions == labels[train_count:]))

    return fit_and_score(features), fit_and_score(current_features)


def actor_leakage_audit(output_directory):
    environment = SensorCandidateEnv()
    observation, _ = environment.reset(seed=882001)
    batch = observations_to_torch(
        stack_observations([observation]), torch.device("cpu")
    )
    model = AsymmetricActorCritic(asymmetric_critic=True)
    checkpoint_path = output_directory / "untrained_actor_leakage_fixture.pt"
    torch.save(model.actor.actor_checkpoint(), checkpoint_path)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    actor = SensorActor.from_actor_checkpoint(checkpoint)
    actor.eval()
    with torch.no_grad():
        actor.choose_actions(batch, advantage_gate_margin=0.10)
    privileged_parameter_names = [
        name for name in checkpoint["state_dict"] if "privileg" in name.lower()
    ]
    extra_rejected = False
    bad_batch = dict(batch)
    bad_batch["privileged"] = torch.as_tensor(environment.privileged_observation())[None, :]
    try:
        actor(bad_batch)
    except ValueError as error:
        extra_rejected = str(error) == "ACTOR_OBSERVATION_SCHEMA_VIOLATION"
    return {
        "actor_checkpoint_loads_without_privileged_schema": True,
        "privileged_parameter_names": privileged_parameter_names,
        "extra_privileged_key_rejected": extra_rejected,
        "pass": not privileged_parameter_names and extra_rejected,
    }


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--episodes-per-family-method", type=int, default=25)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)

    history_accuracy, current_accuracy = ridge_direction_accuracy()
    rows = []
    count = arguments.episodes_per_family_method
    for family_index, family in enumerate(FAMILY_NAMES):
        for method_index, method in enumerate(("analytic", "masked_random")):
            for offset in range(count):
                seed = 883000 + family_index * 1000 + method_index * 100 + offset
                rows.append(run_policy(seed, family, method))
        if family in (FAMILY_NAMES[2], FAMILY_NAMES[3]):
            for offset in range(count):
                seed = 883000 + family_index * 1000 + offset
                rows.append(run_policy(seed, family, "evaluator_side_oracle"))

    episodes_path = arguments.output_directory / "component_episodes.csv"
    with episodes_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    safety_rows = [row for row in rows if row["method"] != "evaluator_side_oracle"]
    paired_regrets = []
    for family in (FAMILY_NAMES[2], FAMILY_NAMES[3]):
        analytic = {
            row["seed"]: row
            for row in rows
            if row["family"] == family and row["method"] == "analytic"
        }
        oracle = {
            row["seed"]: row
            for row in rows
            if row["family"] == family and row["method"] == "evaluator_side_oracle"
        }
        paired_regrets.extend(
            oracle[seed]["restricted_time_s"] + 0.25 < analytic[seed]["restricted_time_s"]
            for seed in sorted(set(analytic) & set(oracle))
        )
    leakage = actor_leakage_audit(arguments.output_directory)
    report = {
        "status": "v5_01_sensor_backend_component_audit_pass",
        "scope": "development_component_evidence_not_formal_performance",
        "history_direction_accuracy": history_accuracy,
        "current_scan_direction_accuracy": current_accuracy,
        "safe_multiaction_state_fraction": float(
            np.mean([row["safe_multiaction_state_fraction"] for row in safety_rows])
        ),
        "analytic_sequential_regret_block_fraction": float(np.mean(paired_regrets)),
        "safety_episode_count": len(safety_rows),
        "safety_contact_count": int(sum(row["contact_count"] for row in safety_rows)),
        "safety_mask_violation_count": int(
            sum(row["mask_violation_count"] for row in safety_rows)
        ),
        "analytic_goal_count": int(
            sum(row["goal_reached"] for row in rows if row["method"] == "analytic")
        ),
        "oracle_goal_count_on_two_challenge_families": int(
            sum(
                row["goal_reached"]
                for row in rows
                if row["method"] == "evaluator_side_oracle"
            )
        ),
        "minimum_physical_clearance_m": float(
            min(row["minimum_physical_clearance_m"] for row in safety_rows)
        ),
        "actor_leakage_audit": leakage,
        "gates": {
            "history_direction_accuracy_minimum_0_80": history_accuracy >= 0.80,
            "safe_multiaction_fraction_minimum_0_25": float(
                np.mean([row["safe_multiaction_state_fraction"] for row in safety_rows])
            )
            >= 0.25,
            "analytic_regret_fraction_minimum_0_20": float(np.mean(paired_regrets))
            >= 0.20,
            "zero_contact": sum(row["contact_count"] for row in safety_rows) == 0,
            "zero_mask_violation": sum(
                row["mask_violation_count"] for row in safety_rows
            )
            == 0,
            "actor_leakage_audit": leakage["pass"],
        },
    }
    report["pass"] = all(report["gates"].values())
    if not report["pass"]:
        report["status"] = "v5_01_sensor_backend_component_audit_fail"
    report_path = arguments.output_directory / "component_audit.json"
    with report_path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    manifest = {
        "component_episodes.csv": sha256(episodes_path),
        "component_audit.json": sha256(report_path),
        "untrained_actor_leakage_fixture.pt": sha256(
            arguments.output_directory / "untrained_actor_leakage_fixture.pt"
        ),
    }
    with (arguments.output_directory / "sha256_manifest.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()

