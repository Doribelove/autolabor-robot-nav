"""Deterministic simulation-scene contracts for calibration and training.

The module is deliberately ROS-free.  A scenario manifest describes immutable
episode inputs; :func:`build_perturbation_plan` expands them into paired
baseline/single-parameter pilot runs using common random numbers.
"""

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import yaml


THETA_ORDER = (
    "max_vel_x", "max_vel_theta", "acc_lim_x", "acc_lim_theta",
    "min_obstacle_dist", "inflation_dist", "weight_obstacle",
    "weight_viapoint", "weight_optimaltime",
)
SCENE_SPLITS = frozenset(
    ("train", "validation", "test_id", "test_ood", "test_disturbance", "real")
)
T02_WORLD_ROOT = Path("src/simulation/m2_gazebo/worlds")
T02_WORLDS = frozenset(("empty.world", "obstacle_test.world", "regression.world"))


class ScenarioContractError(ValueError):
    """Raised when a manifest or expanded run plan is unsafe or ambiguous."""


def canonical_sha256(value: Any) -> str:
    """Hash JSON-compatible data independent of mapping insertion order."""

    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioContractError("{} must be a number".format(label))
    result = float(value)
    if not math.isfinite(result):
        raise ScenarioContractError("{} must be finite".format(label))
    return result


def _pose(value: Any, label: str) -> Dict[str, float]:
    if not isinstance(value, dict) or set(value) != {"x_m", "y_m", "yaw_rad"}:
        raise ScenarioContractError("{} must contain exactly x_m, y_m, yaw_rad".format(label))
    return {name: _finite_number(value[name], "{}.{}".format(label, name))
            for name in ("x_m", "y_m", "yaw_rad")}


def _resolve_world(world: Any, workspace_root: Path) -> Tuple[str, Path]:
    if not isinstance(world, str) or not world:
        raise ScenarioContractError("scene.world must be a non-empty relative path")
    relative = Path(world)
    if relative.is_absolute() or ".." in relative.parts:
        raise ScenarioContractError("scene.world must be a workspace-relative T02 path")
    if relative.parent != T02_WORLD_ROOT or relative.name not in T02_WORLDS:
        raise ScenarioContractError("scene.world must name an existing T02 world")
    resolved_root = workspace_root.resolve()
    resolved = (resolved_root / relative).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError:
        raise ScenarioContractError("scene.world escapes the workspace")
    if not resolved.is_file():
        raise ScenarioContractError("scene.world does not exist: {}".format(relative))
    return relative.as_posix(), resolved


def _required_bool(data: Mapping[str, Any], key: str, expected: bool) -> None:
    if data.get(key) is not expected:
        raise ScenarioContractError("{} must be {}".format(key, str(expected).lower()))


def validate_scenario_manifest(
    manifest: Mapping[str, Any], workspace_root: Any,
) -> Dict[str, Any]:
    """Validate and normalize a T07 simulation calibration manifest.

    Scene identifiers may not repeat, and the same world/start/goal tuple may
    not occur in two splits.  This catches accidental train/test leakage before
    a run is launched.
    """

    if not isinstance(manifest, dict):
        raise ScenarioContractError("scenario manifest must be a mapping")
    result = copy.deepcopy(dict(manifest))
    if str(result.get("schema_version")) != "1.0":
        raise ScenarioContractError("schema_version must be '1.0'")
    if result.get("purpose") != "calibration_pilot":
        raise ScenarioContractError("purpose must be calibration_pilot")
    if result.get("data_classification") != "pipeline_calibration_pilot_non_formal":
        raise ScenarioContractError("data_classification must mark non-formal pilot data")
    _required_bool(result, "simulation_only", True)
    _required_bool(result, "formal_experiment", False)
    _required_bool(result, "real_vehicle_use_forbidden", True)
    _required_bool(result, "paired_common_random_numbers", True)
    sources = result.get("sources")
    if sources != {
        "worlds": "src/simulation/m2_gazebo/worlds",
        "theta_candidates": "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml",
    }:
        raise ScenarioContractError("sources must pin the T02 worlds and current simulation theta candidates")

    theta = result.get("theta")
    if not isinstance(theta, dict):
        raise ScenarioContractError("theta must be a mapping")
    if tuple(theta.get("order", ())) != THETA_ORDER:
        raise ScenarioContractError("theta.order must be the frozen nine-parameter order")
    baseline = theta.get("baseline")
    deltas = theta.get("pilot_delta")
    bounds = theta.get("simulation_candidate_bounds")
    if not isinstance(baseline, dict) or set(baseline) != set(THETA_ORDER):
        raise ScenarioContractError("theta.baseline must contain exactly the nine theta candidates")
    if not isinstance(deltas, dict) or set(deltas) != set(THETA_ORDER):
        raise ScenarioContractError("theta.pilot_delta must contain exactly the nine theta candidates")
    if not isinstance(bounds, dict) or set(bounds) != set(THETA_ORDER):
        raise ScenarioContractError("theta.simulation_candidate_bounds must contain exactly nine candidates")
    theta["baseline"] = {name: _finite_number(baseline[name], "baseline.{}".format(name))
                         for name in THETA_ORDER}
    theta["pilot_delta"] = {name: _finite_number(deltas[name], "pilot_delta.{}".format(name))
                            for name in THETA_ORDER}
    if any(value <= 0.0 for value in theta["pilot_delta"].values()):
        raise ScenarioContractError("every pilot delta must be positive")
    normalized_bounds = {}
    for name in THETA_ORDER:
        pair = bounds[name]
        if not isinstance(pair, list) or len(pair) != 2:
            raise ScenarioContractError("simulation bound for {} must be [min, max]".format(name))
        lower = _finite_number(pair[0], "bounds.{}.min".format(name))
        upper = _finite_number(pair[1], "bounds.{}.max".format(name))
        if lower >= upper:
            raise ScenarioContractError("simulation bound for {} is empty".format(name))
        if not (lower <= theta["baseline"][name] - theta["pilot_delta"][name] and
                theta["baseline"][name] + theta["pilot_delta"][name] <= upper):
            raise ScenarioContractError("baseline +/- pilot delta for {} exceeds simulation candidates".format(name))
        normalized_bounds[name] = [lower, upper]
    theta["simulation_candidate_bounds"] = normalized_bounds
    candidate_path = Path(workspace_root) / sources["theta_candidates"]
    try:
        candidates = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioContractError("cannot load simulation theta candidates: {}".format(exc))
    if not isinstance(candidates, dict):
        raise ScenarioContractError("simulation theta candidates must be a mapping")
    source_bounds = candidates.get("theta_bounds")
    source_deltas = candidates.get("max_delta_per_step")
    if (source_bounds != bounds or not isinstance(source_deltas, dict) or
            set(source_deltas) != set(THETA_ORDER)):
        raise ScenarioContractError("manifest theta candidates drift from t05_simulation_safety.yaml")
    for name in THETA_ORDER:
        source_delta = _finite_number(source_deltas[name], "source_delta.{}".format(name))
        if theta["pilot_delta"][name] > source_delta:
            raise ScenarioContractError("pilot delta for {} exceeds the simulation rate candidate".format(name))

    scenes = result.get("scenes")
    if not isinstance(scenes, list) or not scenes:
        raise ScenarioContractError("scenes must be a non-empty list")
    root = Path(workspace_root)
    ids = set()
    fingerprints: Dict[str, str] = {}
    normalized_scenes = []
    for index, raw in enumerate(scenes):
        label = "scenes[{}]".format(index)
        if not isinstance(raw, dict):
            raise ScenarioContractError("{} must be a mapping".format(label))
        scene = copy.deepcopy(raw)
        scene_id = scene.get("scene_id")
        if not isinstance(scene_id, str) or not scene_id.strip():
            raise ScenarioContractError("{}.scene_id must be non-empty".format(label))
        if scene_id in ids:
            raise ScenarioContractError("duplicate scene_id: {}".format(scene_id))
        ids.add(scene_id)
        split = scene.get("split")
        if split not in SCENE_SPLITS or split == "real":
            raise ScenarioContractError("{}.split must be a simulation split".format(label))
        world, _ = _resolve_world(scene.get("world"), root)
        layout = scene.get("layout")
        if layout not in ("clear", "obstacle", "corridor"):
            raise ScenarioContractError("{}.layout must be clear, obstacle, or corridor".format(label))
        start, goal = _pose(scene.get("start"), label + ".start"), _pose(scene.get("goal"), label + ".goal")
        timeout_s = _finite_number(scene.get("timeout_s"), label + ".timeout_s")
        if timeout_s <= 0.0:
            raise ScenarioContractError("{}.timeout_s must be positive".format(label))
        seeds = scene.get("seeds")
        if (not isinstance(seeds, list) or not seeds or
                any(isinstance(seed, bool) or not isinstance(seed, int) or seed < 0 for seed in seeds)):
            raise ScenarioContractError("{}.seeds must be non-negative integers".format(label))
        if len(seeds) != len(set(seeds)):
            raise ScenarioContractError("{}.seeds contains duplicates".format(label))
        collision = scene.get("collision")
        if not isinstance(collision, dict) or collision.get("source") not in ("gazebo_contacts", "scan_clearance"):
            raise ScenarioContractError("{}.collision must define a supported deterministic source".format(label))
        if _finite_number(collision.get("threshold_m"), label + ".collision.threshold_m") <= 0.0:
            raise ScenarioContractError("{}.collision.threshold_m must be positive".format(label))
        randomization = scene.get("randomization")
        if not isinstance(randomization, dict):
            raise ScenarioContractError("{}.randomization must be a mapping".format(label))
        expected_spec_hash = canonical_sha256(randomization)
        if scene.get("randomization_spec_hash") != expected_spec_hash:
            raise ScenarioContractError("{}.randomization_spec_hash mismatch".format(label))
        fingerprint = canonical_sha256({"world": world, "layout": layout,
                                        "start": start, "goal": goal,
                                        "randomization": randomization})
        previous_split = fingerprints.get(fingerprint)
        if previous_split is not None and previous_split != split:
            raise ScenarioContractError("scene leakage across splits for world/start/goal/randomization")
        fingerprints[fingerprint] = split
        scene.update(world=world, start=start, goal=goal, timeout_s=timeout_s, seeds=list(seeds))
        normalized_scenes.append(scene)
    result["scenes"] = normalized_scenes
    return result


def build_perturbation_plan(manifest: Mapping[str, Any], workspace_root: Any) -> List[Dict[str, Any]]:
    """Expand a manifest to one baseline and +/- each theta per scene/seed."""

    data = validate_scenario_manifest(manifest, workspace_root)
    baseline = data["theta"]["baseline"]
    deltas = data["theta"]["pilot_delta"]
    runs = []
    for scene in data["scenes"]:
        for seed in scene["seeds"]:
            randomization_hash = canonical_sha256(
                {"scene_id": scene["scene_id"], "seed": seed, "spec": scene["randomization"]}
            )
            conditions = [("baseline", None, 0, baseline)]
            for parameter in THETA_ORDER:
                for sign, suffix in ((-1, "minus"), (1, "plus")):
                    values = dict(baseline)
                    values[parameter] += sign * deltas[parameter]
                    conditions.append(("{}_{}".format(parameter, suffix), parameter, sign, values))
            for condition, parameter, sign, values in conditions:
                identity = {"scene_id": scene["scene_id"], "split": scene["split"],
                            "seed": seed, "condition": condition,
                            "randomization_hash": randomization_hash}
                runs.append({
                    "run_id": "t07-" + canonical_sha256(identity)[:16],
                    "scene_id": scene["scene_id"], "split": scene["split"],
                    "world": scene["world"], "layout": scene["layout"],
                    "start": copy.deepcopy(scene["start"]),
                    "goal": copy.deepcopy(scene["goal"]), "seed": seed,
                    "timeout_s": scene["timeout_s"], "collision": copy.deepcopy(scene["collision"]),
                    "randomization": copy.deepcopy(scene["randomization"]),
                    "randomization_hash": randomization_hash, "condition": condition,
                    "perturbed_parameter": parameter, "perturbation_sign": sign,
                    "theta": values,
                })
    validate_perturbation_plan(runs, data)
    return runs


def validate_perturbation_plan(runs: Sequence[Mapping[str, Any]], manifest: Mapping[str, Any]) -> None:
    """Check IDs, baseline coverage, +/- pairing, and common-random-number hashes."""

    if len({run.get("run_id") for run in runs}) != len(runs):
        raise ScenarioContractError("perturbation plan has duplicate run_id")
    expected_conditions = {"baseline"}
    expected_conditions.update(name + suffix for name in THETA_ORDER for suffix in ("_minus", "_plus"))
    grouped: Dict[Tuple[str, int], List[Mapping[str, Any]]] = {}
    for run in runs:
        grouped.setdefault((run["scene_id"], run["seed"]), []).append(run)
    expected_pairs = {(scene["scene_id"], seed) for scene in manifest["scenes"] for seed in scene["seeds"]}
    if set(grouped) != expected_pairs:
        raise ScenarioContractError("perturbation plan scene/seed coverage mismatch")
    scenes = {scene["scene_id"]: scene for scene in manifest["scenes"]}
    baseline = manifest["theta"]["baseline"]
    deltas = manifest["theta"]["pilot_delta"]
    for key, group in grouped.items():
        conditions = [run.get("condition") for run in group]
        if set(conditions) != expected_conditions or len(conditions) != len(expected_conditions):
            raise ScenarioContractError("{} must have one baseline and paired +/- theta runs".format(key))
        scene, seed = scenes[key[0]], key[1]
        expected_hash = canonical_sha256(
            {"scene_id": scene["scene_id"], "seed": seed, "spec": scene["randomization"]}
        )
        hashes = {run.get("randomization_hash") for run in group}
        if hashes != {expected_hash}:
            raise ScenarioContractError("{} violates paired common random numbers".format(key))
        for run in group:
            if any(run.get(name) != scene[name] for name in
                   ("split", "world", "layout", "start", "goal", "timeout_s", "collision", "randomization")):
                raise ScenarioContractError("{} run metadata drifted from its scene".format(key))
            parameter, sign = run.get("perturbed_parameter"), run.get("perturbation_sign")
            expected_theta = dict(baseline)
            if run["condition"] == "baseline":
                if parameter is not None or sign != 0:
                    raise ScenarioContractError("{} baseline metadata is invalid".format(key))
            else:
                if parameter not in THETA_ORDER or sign not in (-1, 1):
                    raise ScenarioContractError("{} perturbation metadata is invalid".format(key))
                expected_theta[parameter] += sign * deltas[parameter]
                suffix = "minus" if sign < 0 else "plus"
                if run["condition"] != "{}_{}".format(parameter, suffix):
                    raise ScenarioContractError("{} perturbation condition is inconsistent".format(key))
            if run.get("theta") != expected_theta:
                raise ScenarioContractError("{} theta does not match baseline/perturbation".format(key))
            identity = {"scene_id": scene["scene_id"], "split": scene["split"],
                        "seed": seed, "condition": run["condition"],
                        "randomization_hash": expected_hash}
            if run.get("run_id") != "t07-" + canonical_sha256(identity)[:16]:
                raise ScenarioContractError("{} run_id does not match deterministic identity".format(key))


def load_scenario_manifest(path: Any, workspace_root: Optional[Any] = None) -> Dict[str, Any]:
    source = Path(path)
    try:
        value = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioContractError("cannot load scenario manifest {}: {}".format(source, exc))
    root = Path(workspace_root) if workspace_root is not None else source.resolve().parents[3]
    return validate_scenario_manifest(value, root)
