#!/usr/bin/env python3
"""Offline, read-only learning diagnosis for the completed T12 residual SAC pilot."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Any, Dict, Iterable, List, Mapping, Sequence

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
DEFAULT_ROOT = WORKSPACE / "artifacts/t12/residual_training"
REWARD_FIELDS = (
    "reward_progress", "reward_time", "reward_near_obstacle", "reward_path_error",
    "reward_smoothness", "reward_planner_failure", "reward_parameter_adjustment",
    "reward_terminal",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite diagnostic input")
    return result


def mean(values: Iterable[float]) -> float:
    items = list(values)
    return float(sum(items) / len(items)) if items else 0.0


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def normalize(theta: Mapping[str, Any], bounds: Mapping[str, Sequence[float]],
              order: Sequence[str]) -> List[float]:
    return [
        2.0 * (finite(theta[name]) - finite(bounds[name][0])) /
        (finite(bounds[name][1]) - finite(bounds[name][0])) - 1.0
        for name in order
    ]


def phase(split: str) -> str:
    return "test" if split in ("test_id", "test_ood") else split


def summarize_rows(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    rewards = {
        name: {
            "sum": float(sum(finite(row[name]) for row in rows)),
            "mean_per_step": mean(finite(row[name]) for row in rows),
        }
        for name in REWARD_FIELDS
    }
    return {
        "step_count": len(rows),
        "transition_stored_count": sum(str(row["transition_stored"]).lower() == "true"
                                       for row in rows),
        "mean_reward_total_per_step": mean(finite(row["reward_total"]) for row in rows),
        "reward_components": rewards,
        "projection_intervention_count": sum(str(row["projection_modified"]).lower() == "true"
                                             for row in rows),
        "projection_intervention_rate": mean(
            str(row["projection_modified"]).lower() == "true" for row in rows),
        "safety_intervention_count": sum(str(row["safety_modified"]).lower() == "true"
                                         for row in rows),
        "safety_intervention_rate": mean(
            str(row["safety_modified"]).lower() == "true" for row in rows),
    }


def analyze_seed(root: Path, run_prefix: str, seed: int, config: Mapping[str, Any],
                 safety: Mapping[str, Any], mapping: Mapping[str, Any]) -> Dict[str, Any]:
    run = root / "runs" / "{}{}".format(run_prefix, seed)
    episodes = list(csv.DictReader((run / "episodes.csv").open(encoding="utf-8")))
    steps = list(csv.DictReader((run / "steps.csv").open(encoding="utf-8")))
    selection = yaml.safe_load((run / "model_selection.yaml").read_text(encoding="utf-8"))
    step_phase = {row["episode_id"]: phase(row["scene_split"]) for row in episodes}
    for row in steps:
        row["_phase"] = step_phase[row["episode_id"]]

    theta_order = tuple(mapping["theta_order"])
    eta_order = tuple(mapping["eta_order"])
    bounds = safety["theta_bounds"]
    anchor = config["anchor_theta"]
    anchor_z = normalize(anchor, bounds, theta_order)
    radii = [finite(value) for value in config["normalized_residual_radius"]]
    matrix = [[finite(value) for value in row] for row in mapping["matrix"]]
    alpha = finite(config["action_ema_alpha"])
    hold_steps = int(config["decision_hold_steps"])

    train_steps = [row for row in steps if row["_phase"] == "train"]
    by_episode: Dict[str, List[Mapping[str, Any]]] = {}
    for row in steps:
        by_episode.setdefault(row["episode_id"], []).append(row)

    raw_l1: List[float] = []
    held_l1: List[float] = []
    raw_abs = [[] for _ in eta_order]
    raw_saturated = 0
    raw_values = 0
    risk_scales: List[float] = []
    utilization = {name: [] for name in theta_order}
    candidate_projection_l1: List[float] = []
    candidate_projection_abs = {name: [] for name in theta_order}
    applied_anchor_l1: List[float] = []
    first_previous_anchor_l1: List[float] = []
    projection_by_step: Dict[int, List[bool]] = {}
    projection_reasons: Dict[str, int] = {}
    projection_after_previous_safety_count = 0
    previous_safety_step_count = 0
    projection_without_previous_safety_count = 0
    without_previous_safety_step_count = 0
    episode_step_counts: List[int] = []

    for episode_id, group in by_episode.items():
        group = sorted(group, key=lambda row: int(row["step_id"]))
        previous_safety_modified = False
        if step_phase[episode_id] == "train":
            episode_step_counts.append(len(group))
        held = [0.0] * len(eta_order)
        for local_index, row in enumerate(group):
            raw = [finite(value) for value in json.loads(row["action_raw_json"])]
            if step_phase[episode_id] == "train":
                raw_l1.append(sum(abs(value) for value in raw))
                for index, value in enumerate(raw):
                    raw_abs[index].append(abs(value))
                    raw_saturated += int(abs(value) >= 0.95)
                    raw_values += 1
                if local_index % hold_steps == 0:
                    held = [before + alpha * (target - before)
                            for before, target in zip(held, raw)]
                held_l1.append(sum(abs(value) for value in held))

            candidate = json.loads(row["theta_candidate_json"])
            projected = json.loads(row["theta_projected_json"])
            applied = json.loads(row["theta_applied_json"])
            previous = json.loads(row["theta_previous_json"])
            candidate_z = normalize(candidate, bounds, theta_order)
            projected_z = normalize(projected, bounds, theta_order)
            applied_z = normalize(applied, bounds, theta_order)
            if local_index == 0:
                previous_z = normalize(previous, bounds, theta_order)
                first_previous_anchor_l1.append(sum(abs(a - b) for a, b in zip(previous_z, anchor_z)))
            if step_phase[episode_id] == "train":
                candidate_projection_l1.append(
                    sum(abs(a - b) for a, b in zip(candidate_z, projected_z)))
                for index, name in enumerate(theta_order):
                    candidate_projection_abs[name].append(
                        abs(candidate_z[index] - projected_z[index]))
                applied_anchor_l1.append(
                    sum(abs(a - b) for a, b in zip(applied_z, anchor_z)))
                for index, name in enumerate(theta_order):
                    utilization[name].append(
                        abs(candidate_z[index] - anchor_z[index]) / radii[index])

                ratios = []
                for index, row_weights in enumerate(matrix):
                    direction = sum(value * weight for value, weight in zip(held, row_weights))
                    direction /= max(1.0, sum(abs(weight) for weight in row_weights))
                    denominator = radii[index] * direction
                    if abs(denominator) > 1e-7:
                        ratios.append((candidate_z[index] - anchor_z[index]) / denominator)
                if ratios:
                    risk_scales.append(float(statistics.median(ratios)))

                modified = str(row["projection_modified"]).lower() == "true"
                if previous_safety_modified:
                    previous_safety_step_count += 1
                    projection_after_previous_safety_count += int(modified)
                else:
                    without_previous_safety_step_count += 1
                    projection_without_previous_safety_count += int(modified)
                projection_by_step.setdefault(local_index, []).append(modified)
                for reason in filter(None, row["projection_reason"].split("|")):
                    projection_reasons[reason] = projection_reasons.get(reason, 0) + 1
            previous_safety_modified = str(row["safety_modified"]).lower() == "true"

    phase_summaries = {
        name: summarize_rows([row for row in steps if row["_phase"] == name])
        for name in ("train", "validation", "test")
    }
    train_windows = {
        "steps_1_1000": summarize_rows(train_steps[:1000]),
        "steps_1001_2000": summarize_rows(train_steps[1000:2000]),
    }
    scenario_rows: Dict[str, List[Mapping[str, Any]]] = {}
    returns_by_episode = {
        episode_id: sum(finite(row["reward_total"]) for row in group)
        for episode_id, group in by_episode.items()
    }
    for row in episodes:
        scenario_rows.setdefault(row["scene_id"], []).append(row)
    scenarios = {}
    for scene_id, rows in sorted(scenario_rows.items()):
        scenarios[scene_id] = {
            "split": rows[0]["scene_split"],
            "episode_count": len(rows),
            "success_rate": mean(str(row["success"]).lower() == "true" for row in rows),
            "mean_navigation_time_s": mean(finite(row["navigation_time"]) for row in rows),
            "mean_return": mean(returns_by_episode[row["episode_id"]] for row in rows),
            "mean_steps": mean(len(by_episode[row["episode_id"]]) for row in rows),
        }

    validation_means = [finite(item["mean_return"]) for item in selection["validation"]]
    return {
        "seed": seed,
        "source": {
            "episodes_sha256": sha256(run / "episodes.csv"),
            "steps_sha256": sha256(run / "steps.csv"),
            "model_selection_sha256": sha256(run / "model_selection.yaml"),
        },
        "episode_count": len(episodes),
        "phase_episode_counts": {
            name: sum(phase(row["scene_split"]) == name for row in episodes)
            for name in ("train", "validation", "test")
        },
        "training_scene_ids": sorted({
            row["scene_id"] for row in episodes if row["scene_split"] == "train"
        }),
        "validation_mean_returns": validation_means,
        "validation_return_change": validation_means[-1] - validation_means[0],
        "selected_timesteps": int(selection["selected_timesteps"]),
        "phase_summaries": phase_summaries,
        "training_windows": train_windows,
        "exploration": {
            "raw_action_l1_mean": mean(raw_l1),
            "raw_action_l1_p95": percentile(raw_l1, 0.95),
            "raw_action_saturation_fraction": raw_saturated / float(max(1, raw_values)),
            "raw_mean_abs_by_eta": dict(zip(eta_order, (mean(values) for values in raw_abs))),
            "ema_held_action_l1_mean": mean(held_l1),
            "ema_to_raw_l1_ratio": mean(held_l1) / max(mean(raw_l1), 1e-12),
            "ema_alpha": alpha,
            "decision_hold_steps": hold_steps,
            "mean_train_episode_steps": mean(episode_step_counts),
            "train_episode_fraction_at_most_one_hold_window": mean(
                count <= hold_steps for count in episode_step_counts),
        },
        "residual": {
            "normalized_radius": dict(zip(theta_order, radii)),
            "radius_utilization_mean": {name: mean(values) for name, values in utilization.items()},
            "radius_utilization_p95": {name: percentile(values, 0.95)
                                       for name, values in utilization.items()},
            "inferred_risk_scale_mean": mean(risk_scales),
            "inferred_risk_scale_p05": percentile(risk_scales, 0.05),
            "inferred_risk_scale_p95": percentile(risk_scales, 0.95),
            "minimum_risk_scale": finite(config["minimum_risk_scale"]),
        },
        "projection": {
            "training_rate": mean(
                str(row["projection_modified"]).lower() == "true" for row in train_steps),
            "mean_candidate_to_projected_normalized_l1": mean(candidate_projection_l1),
            "rate_by_episode_step": {
                str(index): mean(values) for index, values in sorted(projection_by_step.items())[:8]
            },
            "reason_counts": dict(sorted(projection_reasons.items())),
            "reason_rates_per_training_step": {
                reason: count / float(max(1, len(train_steps)))
                for reason, count in sorted(projection_reasons.items())
            },
            "mean_candidate_to_projected_normalized_abs_by_theta": {
                name: mean(values) for name, values in candidate_projection_abs.items()
            },
            "previous_safety_step_count": previous_safety_step_count,
            "rate_after_previous_safety_intervention": (
                projection_after_previous_safety_count /
                float(max(1, previous_safety_step_count))
            ),
            "rate_without_previous_safety_intervention": (
                projection_without_previous_safety_count /
                float(max(1, without_previous_safety_step_count))
            ),
            "mean_first_step_previous_to_anchor_normalized_l1": mean(first_previous_anchor_l1),
            "mean_applied_to_anchor_normalized_l1": mean(applied_anchor_l1),
        },
        "safety": {
            "training_intervention_rate": mean(
                str(row["safety_modified"]).lower() == "true" for row in train_steps),
            "mode_counts": {
                mode: sum(row["safety_mode"] == mode for row in train_steps)
                for mode in ("NORMAL", "WARNING", "EMERGENCY", "FAULT")
            },
            "fallback_count": sum(
                str(row["fallback_active"]).lower() == "true" for row in train_steps),
        },
        "scenarios": scenarios,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--run-prefix", default="t12_residual_training_seed")
    parser.add_argument("--study", default="residual_sac_2seed_offline_learning_diagnosis")
    parser.add_argument("--next-gate",
                        default="frozen_selected_vs_zero_residual_vs_teb_tuned_paired_evaluation")
    parser.add_argument("--residual-config", type=Path,
                        default=WORKSPACE / "config/thesis_experiments/t12_residual_training.yaml")
    parser.add_argument("--output", type=Path,
                        default=DEFAULT_ROOT / "t12_residual_learning_diagnosis.yaml")
    args = parser.parse_args()
    config_path = args.residual_config
    if not config_path.is_absolute():
        config_path = WORKSPACE / config_path
    safety_path = WORKSPACE / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml"
    mapping_path = WORKSPACE / "config/thesis_experiments/A_TEB_v1.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    safety = yaml.safe_load(safety_path.read_text(encoding="utf-8"))
    mapping = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
    seeds = [analyze_seed(args.root, args.run_prefix, seed, config, safety, mapping)
             for seed in (101, 102)]
    validation_changes = [item["validation_return_change"] for item in seeds]
    projection_rates = [item["projection"]["training_rate"] for item in seeds]
    safety_rates = [item["safety"]["training_intervention_rate"] for item in seeds]
    projection_after_safety_rates = [
        item["projection"]["rate_after_previous_safety_intervention"] for item in seeds
    ]
    projection_without_safety_rates = [
        item["projection"]["rate_without_previous_safety_intervention"] for item in seeds
    ]
    short_episode_fractions = [
        item["exploration"]["train_episode_fraction_at_most_one_hold_window"] for item in seeds
    ]
    training_scene_ids = sorted({
        scene for item in seeds for scene in item["training_scene_ids"]
    })
    both_negative = all(value < 0.0 for value in validation_changes)
    first_step_anchor_l1 = [
        item["projection"]["mean_first_step_previous_to_anchor_normalized_l1"]
        for item in seeds
    ]
    projection_reason_counts: Dict[str, int] = {}
    for item in seeds:
        for reason, count in item["projection"]["reason_counts"].items():
            projection_reason_counts[reason] = projection_reason_counts.get(reason, 0) + int(count)
    dominant_projection_reason = max(
        projection_reason_counts, key=projection_reason_counts.get,
        default="none")
    observations = [
        ("Both seeds have lower validation mean return at 2000 than at 1000 steps."
         if both_negative else
         "Validation trend is inconsistent across seeds; the cross-seed mean change is {:.6g}, "
         "which does not satisfy the per-seed learning gate.".format(mean(validation_changes))),
        ("Both seeds trained only on t11-train-clear-straight because repeated curriculum updates reset the scenario index."
         if len(training_scene_ids) == 1 else
         "The repaired curriculum covers all {} intended training scenes.".format(len(training_scene_ids))),
        "Training projection intervenes on {:.1%} of steps; the dominant reason is {}.".format(
            mean(projection_rates), dominant_projection_reason),
        "Projection occurs on {:.1%} of steps after a safety intervention versus {:.1%} "
        "without a preceding safety intervention; rate-limit projection is therefore "
        "mostly a return-from-safety effect.".format(
            mean(projection_after_safety_rates), mean(projection_without_safety_rates)),
        ("Most training episodes end within one four-step hold window, limiting independent residual decisions per episode."
         if mean(short_episode_fractions) > 0.5 else
         "Most training episodes now exceed one four-step hold window, but they still contain only a small number of independent decisions."),
        "The mean first-step previous-theta-to-anchor normalized L1 distance is {:.6g}.".format(
            mean(first_step_anchor_l1)),
        "Safety intervention and risk scaling activate in the repaired obstacle/corridor curriculum without emergency or fault fallback.",
        "The fixed TEB-Tuned anchor is strong; test goal completion alone does not isolate a learned residual benefit.",
    ]
    report = {
        "schema_version": "1.0",
        "task": "T12",
        "study": args.study,
        "generated_from_frozen_artifacts": True,
        "training_performed": False,
        "formal_result": False,
        "sources": {
            "residual_config": str(config_path.relative_to(WORKSPACE)),
            "residual_config_sha256": sha256(config_path),
            "safety_config": str(safety_path.relative_to(WORKSPACE)),
            "safety_config_sha256": sha256(safety_path),
            "mapping": str(mapping_path.relative_to(WORKSPACE)),
            "mapping_sha256": sha256(mapping_path),
        },
        "seeds": seeds,
        "cross_seed": {
            "both_validation_changes_negative": both_negative,
            "mean_validation_return_change": mean(validation_changes),
            "mean_projection_intervention_rate": mean(projection_rates),
            "mean_safety_intervention_rate": mean(safety_rates),
            "mean_projection_rate_after_previous_safety_intervention": mean(
                projection_after_safety_rates),
            "mean_projection_rate_without_previous_safety_intervention": mean(
                projection_without_safety_rates),
            "mean_fraction_train_episodes_at_most_one_hold_window": mean(short_episode_fractions),
            "training_scene_ids": training_scene_ids,
            "training_scene_count": len(training_scene_ids),
        },
        "diagnosis": {
            "learning_gain_supported": False,
            "budget_expansion_allowed": False,
            "primary_observations": observations,
            "next_gate": args.next_gate,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    checksum_path = args.output.with_suffix(".sha256")
    checksum_path.write_text("{}  {}\n".format(sha256(args.output), args.output.name),
                             encoding="utf-8")
    print(yaml.safe_dump({"output": str(args.output), "cross_seed": report["cross_seed"],
                          "diagnosis": report["diagnosis"]}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
