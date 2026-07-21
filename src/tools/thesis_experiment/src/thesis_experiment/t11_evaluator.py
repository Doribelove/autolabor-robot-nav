"""Strict complete-matrix evaluator for the frozen T11 Gazebo study."""

import csv
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple

import yaml

from .run_artifacts import RunValidator
from .t11_contract import validate_t11_contract


class T11EvaluationError(ValueError):
    pass


GROUPS = (
    "RL-TEB-Semantic-Eta", "RL-TEB-Direct-Theta",
    "RL-TEB-Eta-ProjectionOnly", "RL-TEB-Eta-NoSafety",
    "RL-TEB-Eta-NoFallback",
)


def _bool(value: Any) -> bool:
    if value in (True, "true", "True", "1", 1):
        return True
    if value in (False, "false", "False", "0", 0):
        return False
    raise T11EvaluationError("invalid bool {}".format(value))


def _float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        raise T11EvaluationError("invalid numeric value {}".format(value))


def _read_csv(path: Path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    success = [_bool(row["success"]) for row in rows]
    collision = [_bool(row["collision"]) for row in rows]
    metrics = (
        "navigation_time", "path_length", "min_obstacle_distance",
        "near_collision_time_ratio", "parameter_total_variation",
        "projection_intervention_count", "safety_filter_intervention_count",
        "safety_fallback_count",
    )
    return {
        "episode_count": len(rows), "success_count": sum(success),
        "success_rate": sum(success) / float(len(rows)),
        "collision_count": sum(collision),
        "collision_rate": sum(collision) / float(len(rows)),
        "termination_reasons": {
            reason: sum(row["termination_reason"] == reason for row in rows)
            for reason in sorted(set(row["termination_reason"] for row in rows))
        },
        "metrics": {
            name: {
                "mean": sum(_float(row[name]) for row in rows) / len(rows),
                "median": median(_float(row[name]) for row in rows),
            } for name in metrics
        },
    }


def _paired(
    indexed: Mapping[Tuple[str, int, str, int], Mapping[str, Any]],
    left: str, right: str,
) -> Dict[str, Any]:
    left_keys = sorted(key for key in indexed if key[0] == left)
    pairs = []
    for key in left_keys:
        match = (right,) + key[1:]
        if match not in indexed:
            raise T11EvaluationError("missing paired key {}".format(match))
        lrow, rrow = indexed[key], indexed[match]
        pairs.append({
            "training_seed": key[1], "scene_id": key[2], "evaluation_seed": key[3],
            "success_delta": int(_bool(lrow["success"])) - int(_bool(rrow["success"])),
            "collision_delta": int(_bool(lrow["collision"])) - int(_bool(rrow["collision"])),
            "navigation_time_delta_s": _float(lrow["navigation_time"]) - _float(rrow["navigation_time"]),
            "clearance_delta_m": _float(lrow["min_obstacle_distance"]) - _float(rrow["min_obstacle_distance"]),
        })
    return {"left": left, "right": right, "pair_count": len(pairs), "pairs": pairs}


def evaluate_t11(
    workspace: Any,
    training_seeds: Sequence[int] = (),
    amendment_path: Any = None,
) -> Dict[str, Any]:
    root = Path(workspace)
    config_path = root / "config/thesis_experiments/t11_formal.yaml"
    contract = validate_t11_contract(config_path, root)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    scene_data = yaml.safe_load(
        (root / config["scene_manifest"]).read_text(encoding="utf-8"))
    frozen_seeds = tuple(int(value) for value in config["training"]["seeds"])
    seeds = tuple(int(value) for value in training_seeds) or frozen_seeds
    if not seeds or len(set(seeds)) != len(seeds):
        raise T11EvaluationError("training seed subset must be non-empty and unique")
    if not set(seeds).issubset(frozen_seeds):
        raise T11EvaluationError("training seed subset is outside the frozen contract")
    reduced = seeds != frozen_seeds
    if reduced and amendment_path is None:
        raise T11EvaluationError("reduced T11 evaluation requires an amendment")
    amendment = None
    if amendment_path is not None:
        amendment_file = Path(amendment_path)
        if not amendment_file.is_absolute():
            amendment_file = root / amendment_file
        amendment = yaml.safe_load(amendment_file.read_text(encoding="utf-8"))
        declared = tuple(int(value) for value in amendment["primary_training_seeds"])
        if declared != seeds:
            raise T11EvaluationError("amendment seed subset does not match evaluator")
    eval_seeds = config["evaluation"]["evaluation_seeds"]
    test_scenes = [item for item in scene_data["scenes"]
                   if item["split"] in config["evaluation"]["splits"]]
    expected = {
        (group, int(seed), scene["scene_id"], int(eval_seed))
        for group in GROUPS for seed in seeds for scene in test_scenes for eval_seed in eval_seeds
    }
    run_root = root / "artifacts/t11/runs"
    all_manifests = sorted(run_root.glob("*/run_manifest.yaml"))
    manifests, supplementary_manifests = [], []
    for manifest in all_manifests:
        data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        destination = manifests if int(data["training_seed"]) in seeds else supplementary_manifests
        destination.append(manifest)
    expected_run_count = len(seeds) * 5
    if len(manifests) != expected_run_count:
        raise T11EvaluationError(
            "T11 run count {} != {}".format(len(manifests), expected_run_count))
    validator = RunValidator(
        root / "docs/thesis_experiment/schemas/episode_metrics_schema.csv",
        root / "docs/thesis_experiment/schemas/step_metrics_schema.csv")
    validations, all_rows, learning = [], [], {}
    for manifest in manifests:
        validation = validator.validate(manifest)
        data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        if data["completion"].get("excluded_from_formal_results") is not False:
            raise T11EvaluationError("formal run is marked excluded: {}".format(manifest))
        validations.append({"manifest": str(manifest), "validation": validation})
        rows = _read_csv(manifest.parent / data["artifacts"]["episode_csv"])
        all_rows.extend(row for row in rows if row["scene_split"] in ("test_id", "test_ood"))
        selection_path = manifest.parent / "model_selection.yaml"
        selection = yaml.safe_load(selection_path.read_text(encoding="utf-8"))
        if selection.get("test_results_used_for_selection") is not False:
            raise T11EvaluationError("test leakage flag in {}".format(selection_path))
        if "validation" in selection:
            points = sorted(selection["validation"], key=lambda item: item["timesteps"])
            auc = sum(
                0.5 * (right["timesteps"] - left["timesteps"]) *
                (left["mean_return"] + right["mean_return"])
                for left, right in zip(points, points[1:]))
            threshold = config["training"]["validation_selection"]["validation_return_threshold"]
            reached = next((item["timesteps"] for item in points
                            if item["mean_return"] >= threshold), None)
            learning[(selection["algorithm"], int(selection["training_seed"]))] = {
                "validation_return_auc": auc, "time_to_threshold_steps": reached,
                "threshold_reached": reached is not None,
                "selected_timesteps": selection["selected_timesteps"],
                "selected_mean_validation_return": selection["selected_mean_validation_return"],
                "curve": points,
            }
    indexed = {}
    for row in all_rows:
        key = (row["algorithm"], int(row["training_seed"]),
               row["scene_id"], int(row["seed"]))
        if key not in expected:
            raise T11EvaluationError("unexpected T11 evaluation row {}".format(key))
        if key in indexed:
            raise T11EvaluationError("duplicate T11 evaluation row {}".format(key))
        indexed[key] = row
    missing = sorted(expected - set(indexed))
    if missing:
        raise T11EvaluationError("incomplete T11 matrix; {} rows missing".format(len(missing)))
    by_group = {}
    for group in GROUPS:
        selected = [indexed[key] for key in sorted(indexed) if key[0] == group]
        by_group[group] = {
            "all": _summary(selected),
            "test_id": _summary([row for row in selected if row["scene_split"] == "test_id"]),
            "test_ood": _summary([row for row in selected if row["scene_split"] == "test_ood"]),
        }
    return {
        "schema_version": "1.0", "task": "T11", "status": "passed", "passed": True,
        "formal_experiment": not reduced, "amended_reduced_experiment": reduced,
        "simulation_only": True,
        "contract": contract, "complete_matrix": True,
        "primary_training_seeds": list(seeds),
        "frozen_training_seeds": list(frozen_seeds),
        "amendment": amendment,
        "failure_rows_retained": True, "test_checkpoint_selection_forbidden": True,
        "run_count": len(manifests), "evaluation_episode_count": len(indexed),
        "expected_evaluation_episode_count": len(expected),
        "supplementary_run_count": len(supplementary_manifests),
        "supplementary_manifests": [str(path) for path in supplementary_manifests],
        "run_validations": validations, "learning": {
            "{}:seed{}".format(key[0], key[1]): value for key, value in sorted(learning.items())
        },
        "groups": by_group,
        "paired": {
            "semantic_vs_direct": _paired(indexed, "RL-TEB-Semantic-Eta", "RL-TEB-Direct-Theta"),
            "full_vs_projection": _paired(indexed, "RL-TEB-Semantic-Eta", "RL-TEB-Eta-ProjectionOnly"),
            "full_vs_no_safety": _paired(indexed, "RL-TEB-Semantic-Eta", "RL-TEB-Eta-NoSafety"),
            "full_vs_no_fallback": _paired(indexed, "RL-TEB-Semantic-Eta", "RL-TEB-Eta-NoFallback"),
        },
        "limitations": [
            "gazebo_kinematic_model_is_not_real_vehicle_validation",
            "five_hundred_steps_is_a_frozen_initial_budget_and_nonconvergence_is_reported",
            "inferential_statistics_and_holm_correction_are_deferred_to_T14",
        ] + (["primary_matrix_reduced_post_preregistration_due_to_experiment_budget"]
             if reduced else []),
    }
