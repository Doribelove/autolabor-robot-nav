import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment.scenario import (
    THETA_ORDER, ScenarioContractError, build_perturbation_plan,
    canonical_sha256, load_scenario_manifest, validate_perturbation_plan,
    validate_scenario_manifest,
)


ROOT = Path(__file__).resolve().parents[4]
MANIFEST = ROOT / "experiments/manifests/t07/calibration_pilot.yaml"


def data():
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def test_checked_in_pilot_is_simulation_only_non_formal_and_paths_exist():
    manifest = load_scenario_manifest(MANIFEST, ROOT)
    assert manifest["simulation_only"] is True
    assert manifest["formal_experiment"] is False
    assert manifest["real_vehicle_use_forbidden"] is True
    assert all((ROOT / scene["world"]).is_file() for scene in manifest["scenes"])
    assert tuple(manifest["theta"]["order"]) == THETA_ORDER


def test_plan_has_baseline_and_paired_plus_minus_with_common_random_numbers():
    manifest = load_scenario_manifest(MANIFEST, ROOT)
    first = build_perturbation_plan(manifest, ROOT)
    second = build_perturbation_plan(manifest, ROOT)
    assert first == second
    assert len(first) == sum(len(scene["seeds"]) for scene in manifest["scenes"]) * (1 + 2 * len(THETA_ORDER))
    for scene in manifest["scenes"]:
        for seed in scene["seeds"]:
            paired = [run for run in first if run["scene_id"] == scene["scene_id"] and run["seed"] == seed]
            assert sum(run["condition"] == "baseline" for run in paired) == 1
            assert len({run["randomization_hash"] for run in paired}) == 1
            for name in THETA_ORDER:
                minus = next(run for run in paired if run["condition"] == name + "_minus")
                plus = next(run for run in paired if run["condition"] == name + "_plus")
                assert minus["theta"][name] < manifest["theta"]["baseline"][name] < plus["theta"][name]


@pytest.mark.parametrize("world", ["/tmp/fake.world", "../obstacle_test.world", "src/simulation/m2_gazebo/worlds/not_t02.world"])
def test_world_path_must_be_existing_t02_world(world):
    manifest = data()
    manifest["scenes"][0]["world"] = world
    with pytest.raises(ScenarioContractError, match="world"):
        validate_scenario_manifest(manifest, ROOT)


def test_hash_tampering_and_duplicate_ids_are_rejected():
    manifest = data()
    manifest["scenes"][0]["randomization"]["obstacle_jitter_m"] = 0.1
    with pytest.raises(ScenarioContractError, match="hash mismatch"):
        validate_scenario_manifest(manifest, ROOT)


def test_perturbations_must_remain_inside_simulation_candidate_bounds():
    manifest = data()
    manifest["theta"]["baseline"]["max_vel_x"] = 1.2
    with pytest.raises(ScenarioContractError, match="exceeds simulation candidates"):
        validate_scenario_manifest(manifest, ROOT)


def test_manifest_bounds_cannot_drift_from_current_simulation_candidates():
    manifest = data()
    manifest["theta"]["simulation_candidate_bounds"]["max_vel_x"] = [0.5, 1.2]
    with pytest.raises(ScenarioContractError, match="drift"):
        validate_scenario_manifest(manifest, ROOT)
    manifest = data()
    manifest["scenes"].append(copy.deepcopy(manifest["scenes"][0]))
    with pytest.raises(ScenarioContractError, match="duplicate scene_id"):
        validate_scenario_manifest(manifest, ROOT)


def test_same_scene_definition_cannot_leak_across_splits():
    manifest = data()
    leaked = copy.deepcopy(manifest["scenes"][0])
    leaked["scene_id"] = "leaked-copy"
    leaked["split"] = "test_id"
    manifest["scenes"].append(leaked)
    with pytest.raises(ScenarioContractError, match="leakage"):
        validate_scenario_manifest(manifest, ROOT)


def test_validator_rejects_missing_baseline_duplicate_run_and_broken_pairing():
    manifest = load_scenario_manifest(MANIFEST, ROOT)
    plan = build_perturbation_plan(manifest, ROOT)
    with pytest.raises(ScenarioContractError, match="baseline"):
        validate_perturbation_plan(plan[1:], manifest)
    duplicate = copy.deepcopy(plan)
    duplicate[1]["run_id"] = duplicate[0]["run_id"]
    with pytest.raises(ScenarioContractError, match="duplicate run_id"):
        validate_perturbation_plan(duplicate, manifest)
    broken = copy.deepcopy(plan)
    broken[1]["randomization_hash"] = "0" * 64
    with pytest.raises(ScenarioContractError, match="common random numbers"):
        validate_perturbation_plan(broken, manifest)
    broken_theta = copy.deepcopy(plan)
    broken_theta[0]["theta"]["max_vel_x"] += 0.01
    with pytest.raises(ScenarioContractError, match="theta does not match"):
        validate_perturbation_plan(broken_theta, manifest)


def test_canonical_hash_is_mapping_order_independent_and_rejects_nan():
    assert canonical_sha256({"a": 1, "b": 2}) == canonical_sha256({"b": 2, "a": 1})
    with pytest.raises(ValueError):
        canonical_sha256({"bad": float("nan")})
