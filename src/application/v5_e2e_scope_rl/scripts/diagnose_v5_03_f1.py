#!/usr/bin/env python3
"""Pure-offline attribution of the terminal V5-03-F1 mixed/negative result."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal-directory", type=Path, required=True)
    parser.add_argument("--component-r1-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    episodes_path = arguments.formal_directory / "formal_episodes.csv"
    assessment_path = arguments.formal_directory / "formal_assessment.json"
    rows = list(csv.DictReader(episodes_path.open(encoding="utf-8")))
    assessment = json.loads(assessment_path.read_text(encoding="utf-8"))
    if assessment["status"] != "v5_03_f1_fast_formal_negative":
        raise RuntimeError("UNEXPECTED_V5_03_STATUS")
    methods = sorted({row["method"] for row in rows})
    families = sorted({row["family"] for row in rows})
    family_rows = []
    for family in families:
        for method in methods:
            subset = [
                row for row in rows if row["family"] == family and row["method"] == method
            ]
            family_rows.append(
                {
                    "family": family,
                    "method": method,
                    "episodes": len(subset),
                    "goal_count": sum(row["goal_reached"] == "True" for row in subset),
                    "mean_failure_inclusive_restricted_time_s": float(
                        np.mean(
                            [
                                float(row["failure_inclusive_restricted_time_s"])
                                for row in subset
                            ]
                        )
                    ),
                    "learned_difference_count": sum(
                        int(row["learned_difference_count"]) for row in subset
                    ),
                    "gate_intervention_count": sum(
                        int(row["advantage_gate_intervention_count"]) for row in subset
                    ),
                    "contact_count": sum(int(row["contact_count"]) for row in subset),
                }
            )
    family_path = arguments.output_directory / "family_attribution.csv"
    with family_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(family_rows[0]))
        writer.writeheader()
        writer.writerows(family_rows)
    component_rows = list(
        csv.DictReader(
            (arguments.component_r1_directory / "component_episodes.csv").open(
                encoding="utf-8"
            )
        )
    )
    narrow_oracle = [
        row
        for row in component_rows
        if row["family"] == "narrow_corridor_yield"
        and row["method"] == "evaluator_side_oracle"
    ]
    full = [row for row in rows if row["method"] == "full_advantage_gated_v5"]
    no_history = [row for row in rows if row["method"] == "no_scan_history"]
    ungated = [
        row for row in rows if row["method"] == "asymmetric_residual_masked_ppo"
    ]
    report = {
        "status": "v5_04_d1_offline_failure_attribution_complete",
        "source_formal_status": assessment["status"],
        "source_formal_assessment_sha256": sha256(assessment_path),
        "source_formal_episodes_sha256": sha256(episodes_path),
        "no_episode_rerun": True,
        "formal_result_unchanged": True,
        "primary_effect": assessment["primary_relative_time_reduction"],
        "primary_interval": assessment["paired_bootstrap_95_percent_interval"],
        "frozen_practical_gate": 0.08,
        "family_conclusions": {
            "bidirectional_reactive_crossing": "ceiling; all primary methods 15/15",
            "sequential_two_actor_crossing": "ceiling; all primary methods 15/15",
            "partially_occluded_crossing": "learning benefit; full 15/15 versus analytic 9/15",
            "narrow_corridor_yield": "unlearned commitment; full and analytic both 0/15",
        },
        "history_ablation_time_delta_s": float(
            np.mean([float(row["failure_inclusive_restricted_time_s"]) for row in no_history])
            - np.mean([float(row["failure_inclusive_restricted_time_s"]) for row in full])
        ),
        "ungated_time_delta_s": float(
            np.mean([float(row["failure_inclusive_restricted_time_s"]) for row in ungated])
            - np.mean([float(row["failure_inclusive_restricted_time_s"]) for row in full])
        ),
        "narrow_component_side_oracle_goal_count": sum(
            row["goal_reached"] == "True" for row in narrow_oracle
        ),
        "narrow_component_side_oracle_episodes": len(narrow_oracle),
        "formal_training_seed_family_confound": True,
        "confound_description": (
            "The frozen block_index modulo-four rule maps both family and asymmetric "
            "training identity, so each selected training seed saw only one family in F1. "
            "The aggregate mixed-policy estimand remains valid, but per-seed and per-family "
            "effects cannot be separated."
        ),
        "authorized_single_successor_factor": "privileged_counterfactual_candidate_rank_distillation",
        "why_this_factor": (
            "The hard action set contains feasible lateral commitments and the sensor input "
            "identifies hazard side, but PPO plus a privileged value baseline did not discover "
            "the delayed narrow-corridor commitment. A training-only privileged candidate-rank "
            "target changes credit assignment while keeping observations, actions, reward, "
            "shield, analytic prior and deployment actor interface fixed."
        ),
        "prohibited_repairs": [
            "rerun_v5_formal_seeds",
            "lower_eight_percent_gate",
            "select_only_positive_families",
            "select_only_positive_training_seeds",
            "authorize_v5_gazebo",
        ],
    }
    report_path = arguments.output_directory / "failure_attribution.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "family_attribution.csv": sha256(family_path),
        "failure_attribution.json": sha256(report_path),
    }
    (arguments.output_directory / "sha256_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
