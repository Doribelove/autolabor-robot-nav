import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_04c import (
    V204CError, build_v2_04c_plans, build_v2_04c_r2_plan, build_v2_04c_r3_plan,
    validate_v2_04c_contract, validate_v2_04c_r2_amendment,
    validate_v2_04c_r3_amendment,
)
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04c_refinement_contract.yaml"
R2_AMENDMENT = (
    WORKSPACE / "config/thesis_experiments/v2/v2_04c_ttc_qualification_r2_amendment.yaml"
)
R3_AMENDMENT = (
    WORKSPACE / "config/thesis_experiments/v2/v2_04c_ttc_qualification_r3_amendment.yaml"
)


def _contract():
    return yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))


def test_v2_04c_contract_is_simulation_only_and_resources_are_frozen():
    contract = _contract()
    assert validate_v2_04c_contract(contract, WORKSPACE, True) is contract
    assert contract["runtime_ready"] is False
    assert contract["training_allowed"] is False
    assert contract["real_vehicle_use_forbidden"] is True
    assert contract["split_boundary"]["permitted_splits"] == ["calibration"]


def test_v2_04c_contract_fails_closed_on_boundary_budget_or_aggregation_drift():
    mutations = (
        lambda data: data.update(runtime_ready=True),
        lambda data: data.update(training_allowed=True),
        lambda data: data["split_boundary"].update(permitted_splits=["calibration", "validation"]),
        lambda data: data["refinement_design"].update(planned_navigation_episode_count=179),
        lambda data: data["refinement_design"].update(early_stopping_allowed=True),
        lambda data: data["aggregation"].update(replicate_aggregator="mean"),
        lambda data: data["hard_gates"].update(collision_count_max_per_candidate=1),
    )
    for mutate in mutations:
        changed = copy.deepcopy(_contract())
        mutate(changed)
        with pytest.raises(V204CError):
            validate_v2_04c_contract(changed, WORKSPACE, False)


def test_v2_04c_plans_are_exact_joint_design_and_calibration_only():
    plans = build_v2_04c_plans(CONTRACT, WORKSPACE)
    qualification = plans["qualification"]
    refinement = plans["refinement"]
    assert qualification["candidate_count"] == 1
    assert qualification["planned_episode_count"] == 5
    assert refinement["candidate_count"] == 54
    assert refinement["planned_episode_count"] == 180
    assert all(
        item["split"] == "calibration"
        for plan in plans.values()
        for candidate in plan["candidates"]
        for item in candidate["evaluations"]
    )
    assert sum(row["design_role"] == "incumbent" for row in refinement["candidates"]) == 6
    assert sum(row["design_role"] == "joint_fractional_factor"
               for row in refinement["candidates"]) == 48
    by_anchor = {}
    for row in refinement["candidates"]:
        by_anchor.setdefault(row["anchor_id"], []).append(row)
    for rows in by_anchor.values():
        incumbent = next(row for row in rows if row["design_role"] == "incumbent")
        for row in rows:
            if row["design_role"] == "incumbent":
                continue
            coordinates = _contract()["refinement_design"]["factor_coordinates"][
                row["anchor_id"]
            ]
            assert all(row["values"][name] != incumbent["values"][name]
                       for name in coordinates)
            assert set(row["derived_parameter_changes"]).issubset(
                _contract()["refinement_design"][
                    "deterministic_derived_feasibility_parameters"
                ]
            )


def test_v2_04c_new_seeds_are_disjoint_from_v2_04b_and_non_calibration_splits():
    q = load_v2_scene_manifest(
        WORKSPACE / _contract()["frozen_inputs"]["qualification_scenes"]["path"], WORKSPACE
    )
    r = load_v2_scene_manifest(
        WORKSPACE / _contract()["frozen_inputs"]["refinement_scenes"]["path"], WORKSPACE
    )
    old = load_v2_scene_manifest(
        WORKSPACE / "experiments/manifests/v2/calibration/v2_04b_anchor_calibration_scenes.yaml",
        WORKSPACE,
    )
    foundation = load_v2_scene_manifest(
        WORKSPACE / "experiments/manifests/v2/scenes/v2_02_foundation_scenes.yaml", WORKSPACE
    )
    new_seeds = {row["seed"] for manifest in (q, r) for row in manifest["scenes"]}
    assert not new_seeds & {row["seed"] for row in old["scenes"]}
    assert not new_seeds & {row["seed"] for row in foundation["scenes"]}
    assert {row["split"] for manifest in (q, r) for row in manifest["scenes"]} == {"calibration"}


def test_episode_runner_records_three_state_ttc_contract():
    source = (WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04b_calibration_episode.py"
    ).read_text(encoding="utf-8")
    for status in ("OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID"):
        assert status in source
    assert "tracker_health_valid_fraction" in source
    assert "finite_ttc_sample_count" in source


def test_v2_04c_r2_is_a_fail_closed_timeout_only_retry():
    amendment = yaml.safe_load(R2_AMENDMENT.read_text(encoding="utf-8"))
    assert validate_v2_04c_r2_amendment(amendment, WORKSPACE, True) is amendment
    retry = build_v2_04c_r2_plan(R2_AMENDMENT, WORKSPACE)
    r1 = yaml.safe_load((
        WORKSPACE / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification_plan.yaml"
    ).read_text(encoding="utf-8"))
    assert retry["planned_episode_count"] == 5
    assert retry["candidates"][0]["values"] == r1["candidates"][0]["values"]
    assert retry["candidates"][0]["evaluations"] == r1["candidates"][0]["evaluations"]
    assert retry["candidates"][0]["candidate_id"] != r1["candidates"][0]["candidate_id"]
    changed = copy.deepcopy(amendment)
    changed["single_changed_factor"]["r2_value_s"] = 79.0
    with pytest.raises(V204CError):
        validate_v2_04c_r2_amendment(changed, WORKSPACE, False)


def test_batch_timeout_override_is_dynamic_only_and_explicit():
    source = (WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04b_calibration_batch.py"
    ).read_text(encoding="utf-8")
    assert 'scene["family"] == "DYNAMIC"' in source
    assert 'runner_command.append("--allow-timeout-override")' in source
    assert "--dynamic-timeout-override-s" in source


def test_v2_04c_r3_changes_only_the_tracker_to_teb_architecture_factor():
    amendment = yaml.safe_load(R3_AMENDMENT.read_text(encoding="utf-8"))
    assert validate_v2_04c_r3_amendment(amendment, WORKSPACE, True) is amendment
    retry = build_v2_04c_r3_plan(R3_AMENDMENT, WORKSPACE)
    r2 = yaml.safe_load((
        WORKSPACE / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification_r2_plan.yaml"
    ).read_text(encoding="utf-8"))
    assert retry["candidates"][0]["values"] == r2["candidates"][0]["values"]
    assert retry["candidates"][0]["evaluations"] == r2["candidates"][0]["evaluations"]
    assert retry["candidates"][0]["candidate_id"] != r2["candidates"][0]["candidate_id"]
    changed = copy.deepcopy(amendment)
    changed["single_changed_factor"]["minimum_track_confidence"] = 0.54
    with pytest.raises(V204CError):
        validate_v2_04c_r3_amendment(changed, WORKSPACE, False)
