"""Fail-closed validation for the frozen T11 Gazebo study contract."""

import hashlib
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


class T11ContractError(ValueError):
    pass


SPLITS = ("train", "validation", "test_id", "test_ood")
TRAINING_ALGORITHMS = ("RL-TEB-Semantic-Eta", "RL-TEB-Direct-Theta")
ABLATIONS = ("FullSafety", "ProjectionOnly", "NoSafety", "NoFallback")


def _load(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise T11ContractError("YAML root must be a mapping: {}".format(path))
    return value


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_t11_contract(config_path: Any, workspace: Any) -> Dict[str, Any]:
    root, source = Path(workspace), Path(config_path)
    config = _load(source)
    if (config.get("schema_version") != "1.0" or
            config.get("status") != "frozen_gazebo_t11"):
        raise T11ContractError("T11 config must be frozen_gazebo_t11 schema 1.0")
    if config.get("simulation_only") is not True or config.get("formal_experiment") is not True:
        raise T11ContractError("T11 must be a formal simulation-only contract")
    if config.get("real_vehicle_use_forbidden") is not True:
        raise T11ContractError("T11 must forbid real-vehicle use")
    algorithms = config.get("algorithms", {})
    if tuple(algorithms.get("training", ())) != TRAINING_ALGORITHMS:
        raise T11ContractError("T11 training algorithm order drifted")
    if tuple(algorithms.get("runtime_safety_ablations", ())) != ABLATIONS:
        raise T11ContractError("T11 safety ablation order drifted")
    training = config.get("training", {})
    seeds = training.get("seeds", [])
    if len(seeds) < 3 or len(seeds) != len(set(seeds)):
        raise T11ContractError("T11 requires at least three unique training seeds")
    evaluation = config.get("evaluation", {})
    eval_seeds = evaluation.get("evaluation_seeds", [])
    if len(eval_seeds) < 10 or len(eval_seeds) != len(set(eval_seeds)):
        raise T11ContractError("T11 requires at least ten paired evaluation seeds")
    if evaluation.get("no_test_checkpoint_selection") is not True:
        raise T11ContractError("test checkpoint selection must be forbidden")
    weights = config.get("reward", {}).get("weights", {})
    required_weights = (
        "progress", "elapsed_time", "near_obstacle", "path_error", "smoothness",
        "angular_acceleration", "planner_failure", "parameter_adjustment",
        "goal_terminal", "collision_terminal",
    )
    if tuple(weights.keys()) != required_weights or any(float(weights[k]) < 0 for k in weights):
        raise T11ContractError("T11 reward weights are missing, reordered or negative")
    scene_path = root / config["scene_manifest"]
    scenes = _load(scene_path)
    if (scenes.get("status") != "frozen_gazebo_t11" or
            scenes.get("formal_experiment") is not True or
            scenes.get("real_vehicle_use_forbidden") is not True):
        raise T11ContractError("T11 scene manifest boundary is invalid")
    indexed, counts = {}, {split: 0 for split in SPLITS}
    for scene in scenes.get("scenes", []):
        required = ("scene_id", "split", "layout", "start", "goal", "timeout_s")
        if not isinstance(scene, dict) or any(key not in scene for key in required):
            raise T11ContractError("T11 scene is incomplete")
        if scene["scene_id"] in indexed or scene["split"] not in counts:
            raise T11ContractError("T11 scene ID/split is invalid")
        if scene["layout"] not in ("clear", "obstacle", "corridor"):
            raise T11ContractError("T11 scene layout is unsupported")
        if len(scene["start"]) != 3 or len(scene["goal"]) != 3:
            raise T11ContractError("T11 start/goal must be x,y,yaw triples")
        indexed[scene["scene_id"]] = scene
        counts[scene["split"]] += 1
    if any(counts[split] < 1 for split in SPLITS):
        raise T11ContractError("every T11 split must contain a scene")
    prereg_path = scene_path.parent / "preregistration.yaml"
    prereg = _load(prereg_path)
    if prereg.get("frozen") is not True:
        raise T11ContractError("T11 preregistration must be frozen")
    if prereg.get("training_seeds") != seeds or prereg.get("evaluation_seeds") != eval_seeds:
        raise T11ContractError("T11 preregistration seed drift")
    return {
        "status": "valid", "study_version": config["study_version"],
        "config_sha256": _sha(source), "scene_manifest_sha256": _sha(scene_path),
        "preregistration_sha256": _sha(prereg_path), "scene_counts": counts,
        "training_seed_count": len(seeds), "evaluation_seed_count": len(eval_seeds),
    }
