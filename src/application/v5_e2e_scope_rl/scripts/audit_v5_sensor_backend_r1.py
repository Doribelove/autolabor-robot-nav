#!/usr/bin/env python3
"""Independent fresh-seed R1 repair of the V5 direction probe only."""

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES, SensorCandidateEnv


def load_v5_01_module():
    path = Path(__file__).with_name("audit_v5_sensor_backend.py")
    spec = importlib.util.spec_from_file_location("frozen_v5_01_audit", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fresh_ridge_direction_accuracy(train_count=160, test_count=120):
    features = []
    latest_features = []
    labels = []
    for index in range(train_count + test_count):
        side = -1.0 if index % 2 == 0 else 1.0
        environment = SensorCandidateEnv({"family": FAMILY_NAMES[0]})
        observation, _ = environment.reset(
            seed=887000 + index,
            options={"actor_side": side},
        )
        scans = observation["scans"].astype(np.float64)
        features.append(scans.reshape(-1))
        latest_features.append(scans[-1])
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
        gram = design @ design.T + 0.08 * np.eye(train_count)
        weights = design.T @ np.linalg.solve(gram, labels[:train_count])
        test_design = np.concatenate((test, np.ones((test_count, 1))), axis=1)
        predictions = np.where(test_design @ weights >= 0.0, 1.0, -1.0)
        return float(np.mean(predictions == labels[train_count:]))

    return fit_and_score(features), fit_and_score(latest_features)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--predecessor-directory", type=Path, required=True)
    parser.add_argument("--episodes-per-family-method", type=int, default=25)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    frozen = load_v5_01_module()

    flattened_accuracy, latest_frame_accuracy = fresh_ridge_direction_accuracy()
    rows = []
    count = arguments.episodes_per_family_method
    for family_index, family in enumerate(FAMILY_NAMES):
        for method_index, method in enumerate(("analytic", "masked_random")):
            for offset in range(count):
                seed = 888000 + family_index * 1000 + method_index * 100 + offset
                rows.append(frozen.run_policy(seed, family, method))
        if family in (FAMILY_NAMES[2], FAMILY_NAMES[3]):
            for offset in range(count):
                seed = 888000 + family_index * 1000 + offset
                rows.append(frozen.run_policy(seed, family, "evaluator_side_oracle"))

    episodes_path = arguments.output_directory / "component_episodes.csv"
    with episodes_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    safety_rows = [row for row in rows if row["method"] != "evaluator_side_oracle"]
    regrets = []
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
        regrets.extend(
            oracle[seed]["restricted_time_s"] + 0.25 < analytic[seed]["restricted_time_s"]
            for seed in sorted(set(analytic) & set(oracle))
        )
    leakage = frozen.actor_leakage_audit(arguments.output_directory)
    multiaction_fraction = float(
        np.mean([row["safe_multiaction_state_fraction"] for row in safety_rows])
    )
    regret_fraction = float(np.mean(regrets))
    gates = {
        "latest_frame_direction_accuracy_minimum_0_80": latest_frame_accuracy >= 0.80,
        "safe_multiaction_fraction_minimum_0_25": multiaction_fraction >= 0.25,
        "analytic_regret_fraction_minimum_0_20": regret_fraction >= 0.20,
        "zero_contact": sum(row["contact_count"] for row in safety_rows) == 0,
        "zero_mask_violation": sum(
            row["mask_violation_count"] for row in safety_rows
        )
        == 0,
        "actor_leakage_audit": leakage["pass"],
    }
    predecessor_bindings = {
        name: sha256(arguments.predecessor_directory / name)
        for name in (
            "component_episodes.csv",
            "component_audit.json",
            "sha256_manifest.json",
        )
    }
    report = {
        "status": (
            "v5_01_r1_sensor_backend_component_audit_pass"
            if all(gates.values())
            else "v5_01_r1_sensor_backend_component_audit_fail"
        ),
        "scope": "development_component_evidence_not_formal_performance",
        "single_repair": "history_compatible_latest_frame_direction_probe",
        "latest_frame_direction_accuracy": latest_frame_accuracy,
        "diagnostic_flattened_history_accuracy": flattened_accuracy,
        "history_improvement_claim_authorized": False,
        "safe_multiaction_state_fraction": multiaction_fraction,
        "analytic_sequential_regret_block_fraction": regret_fraction,
        "safety_episode_count": len(safety_rows),
        "safety_contact_count": int(sum(row["contact_count"] for row in safety_rows)),
        "safety_mask_violation_count": int(
            sum(row["mask_violation_count"] for row in safety_rows)
        ),
        "minimum_physical_clearance_m": float(
            min(row["minimum_physical_clearance_m"] for row in safety_rows)
        ),
        "actor_leakage_audit": leakage,
        "predecessor_failure_bindings": predecessor_bindings,
        "gates": gates,
        "pass": all(gates.values()),
    }
    report_path = arguments.output_directory / "component_audit_r1.json"
    with report_path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    manifest = {
        "component_episodes.csv": sha256(episodes_path),
        "component_audit_r1.json": sha256(report_path),
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
