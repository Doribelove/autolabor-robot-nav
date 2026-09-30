#!/usr/bin/env python3
"""Run the fresh Latin-square-style V6 PCT formal evaluation."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.evaluation import run_episode
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES


METHODS = (
    "sensor_analytic_prior",
    "frozen_v5_full",
    "pct_scope_rl_v6",
    "pct_scope_rl_v6_no_history",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_actor(path, expected_sha):
    if sha256(path) != expected_sha:
        raise RuntimeError("FORMAL_ACTOR_HASH_MISMATCH:" + str(path))
    actor = SensorActor.from_actor_checkpoint(
        torch.load(path, map_location="cpu", weights_only=False)
    )
    actor.eval()
    return actor


def summarize(rows):
    return {
        "episodes": len(rows),
        "goal_count": int(sum(row["goal_reached"] for row in rows)),
        "contact_count": int(sum(row["contact_count"] for row in rows)),
        "mask_violation_count": int(sum(row["mask_violation_count"] for row in rows)),
        "mean_failure_inclusive_restricted_time_s": float(
            np.mean([row["failure_inclusive_restricted_time_s"] for row in rows])
        ),
        "minimum_physical_clearance_m": float(
            min(row["minimum_physical_clearance_m"] for row in rows)
        ),
        "mean_inference_latency_ms": float(
            np.mean([row["mean_inference_latency_ms"] for row in rows])
        ),
        "p99_episode_inference_latency_ms": float(
            np.percentile([row["p99_inference_latency_ms"] for row in rows], 99.0)
        ),
    }


def bootstrap_effect(treatment, baseline, seed):
    random = np.random.default_rng(seed)
    distribution = np.empty(20000, dtype=np.float64)
    for index in range(len(distribution)):
        selected = random.integers(0, len(baseline), size=len(baseline))
        distribution[index] = 1.0 - treatment[selected].mean() / baseline[selected].mean()
    return distribution


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selection-report", type=Path, required=True)
    parser.add_argument("--training-release", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    selection = json.loads(arguments.selection_report.read_text(encoding="utf-8"))
    if not selection.get("pass") or not selection.get("fresh_formal_seed_access_authorized"):
        raise RuntimeError("V6_FRESH_FORMAL_ENTRY_GATE_NOT_PASSED")
    release = json.loads(arguments.training_release.read_text(encoding="utf-8"))
    selected = {row["identity"]: row for row in selection["selected_checkpoints"]}
    initialization = {row["v6_identity"]: row for row in release["initializations"]}
    identities = ("pct_seed9501", "pct_seed9502", "pct_seed9503", "pct_seed9504")
    v6_actors = {
        identity: load_actor(
            Path(selected[identity]["checkpoint_path"]),
            selected[identity]["checkpoint_sha256"],
        )
        for identity in identities
    }
    v5_actors = {
        identity: load_actor(
            Path(initialization[identity]["actor_path"]),
            initialization[identity]["actor_sha256"],
        )
        for identity in identities
    }
    rows = []
    for block_index, seed in enumerate(range(953100, 953180)):
        family = FAMILY_NAMES[block_index % 4]
        identity = identities[(block_index // 4) % 4]
        for method in METHODS:
            actor = None
            no_history = False
            actor_sha = "none"
            if method == "frozen_v5_full":
                actor = v5_actors[identity]
                actor_sha = initialization[identity]["actor_sha256"]
            elif method in ("pct_scope_rl_v6", "pct_scope_rl_v6_no_history"):
                actor = v6_actors[identity]
                actor_sha = selected[identity]["checkpoint_sha256"]
                no_history = method.endswith("no_history")
            row = run_episode(
                seed=seed,
                family=family,
                method=method,
                actor=actor,
                advantage_gate_margin=0.10 if actor is not None else None,
                no_scan_history=no_history,
            )
            row.update(
                {
                    "block_index": block_index,
                    "policy_identity": identity if actor is not None else "none",
                    "actor_sha256": actor_sha,
                }
            )
            rows.append(row)
    episodes_path = arguments.output_directory / "formal_episodes.csv"
    with episodes_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    by_method = {
        method: [row for row in rows if row["method"] == method] for method in METHODS
    }
    summaries = {method: summarize(values) for method, values in by_method.items()}
    arrays = {
        method: np.asarray(
            [row["failure_inclusive_restricted_time_s"] for row in by_method[method]],
            dtype=np.float64,
        )
        for method in METHODS
    }
    primary_effect = float(
        1.0 - arrays["pct_scope_rl_v6"].mean() / arrays["sensor_analytic_prior"].mean()
    )
    mechanism_effect = float(
        1.0 - arrays["pct_scope_rl_v6"].mean() / arrays["frozen_v5_full"].mean()
    )
    primary_bootstrap = bootstrap_effect(
        arrays["pct_scope_rl_v6"], arrays["sensor_analytic_prior"], 60421
    )
    mechanism_bootstrap = bootstrap_effect(
        arrays["pct_scope_rl_v6"], arrays["frozen_v5_full"], 60422
    )
    primary_interval = np.percentile(primary_bootstrap, [1.25, 98.75])
    mechanism_interval = np.percentile(mechanism_bootstrap, [2.5, 97.5])
    family_rows = []
    for family in FAMILY_NAMES:
        for method in METHODS:
            subset = [row for row in by_method[method] if row["family"] == family]
            family_rows.append(
                {
                    "family": family,
                    "method": method,
                    **summarize(subset),
                }
            )
    family_path = arguments.output_directory / "family_summary.csv"
    with family_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(family_rows[0]))
        writer.writeheader()
        writer.writerows(family_rows)
    gates = {
        "relative_reduction_minimum_0_08": primary_effect >= 0.08,
        "97_5_percent_interval_lower_strictly_positive": float(primary_interval[0]) > 0.0,
        "goal_noninferior": summaries["pct_scope_rl_v6"]["goal_count"]
        >= summaries["sensor_analytic_prior"]["goal_count"],
        "zero_contact": summaries["pct_scope_rl_v6"]["contact_count"] == 0,
        "zero_mask_violation": summaries["pct_scope_rl_v6"]["mask_violation_count"] == 0,
    }
    report = {
        "status": (
            "v6_02_f1_fresh_formal_positive"
            if all(gates.values())
            else "v6_02_f1_fresh_formal_negative"
        ),
        "formal_result": True,
        "method_effectiveness_demonstrated": all(gates.values()),
        "primary_relative_reduction_vs_analytic": primary_effect,
        "primary_two_sided_97_5_percent_interval": [
            float(primary_interval[0]),
            float(primary_interval[1]),
        ],
        "secondary_relative_reduction_vs_frozen_v5": mechanism_effect,
        "secondary_two_sided_95_percent_interval": [
            float(mechanism_interval[0]),
            float(mechanism_interval[1]),
        ],
        "summaries": summaries,
        "gates": gates,
        "latin_assignment": {
            "family": "block_index_modulo_4",
            "policy_identity": "floor_block_index_div_4_modulo_4",
            "episodes_per_identity_family_method": 5,
        },
        "gazebo_entry_authorized": all(gates.values()),
        "real_vehicle_authorized": False,
        "selection_report_sha256": sha256(arguments.selection_report),
        "training_release_sha256": sha256(arguments.training_release),
    }
    report_path = arguments.output_directory / "formal_assessment.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    bootstrap_path = arguments.output_directory / "bootstrap_distributions.csv"
    with bootstrap_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("replicate", "primary_vs_analytic", "secondary_vs_frozen_v5"))
        writer.writerows(
            (index, primary_bootstrap[index], mechanism_bootstrap[index])
            for index in range(20000)
        )
    manifest = {
        path.name: sha256(path)
        for path in sorted(arguments.output_directory.iterdir())
        if path.is_file()
    }
    (arguments.output_directory / "sha256_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if all(gates.values()) else 2)


if __name__ == "__main__":
    main()

