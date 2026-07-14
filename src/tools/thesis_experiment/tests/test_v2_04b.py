import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_04b_calibration import build_anchor_calibration_plan
from thesis_experiment.v2_04b_freeze import assess_anchor_freeze
from thesis_experiment.v2_contract import (
    V2ContractError,
    validate_typed_calibration_contract,
)
from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/typed_transaction_calibration_contract.yaml"
SCENES = WORKSPACE / "experiments/manifests/v2/calibration/v2_04b_anchor_calibration_scenes.yaml"


def load_contract():
    return yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))


def test_v2_04b_contract_resources_validate_with_runtime_and_training_disabled():
    contract = load_contract()
    assert validate_typed_calibration_contract(
        contract, workspace=WORKSPACE, verify_resources=True
    ) is contract
    assert contract["runtime_ready"] is False
    assert contract["training_allowed"] is False
    assert contract["write_gate"]["real_vehicle_write_enabled"] is False
    assert contract["claims"]["anchor_calibration_complete"] is False


def test_v2_04b_contract_fails_closed_on_write_or_split_scope_drift():
    mutations = (
        lambda data: data.update(runtime_ready=True),
        lambda data: data.update(training_allowed=True),
        lambda data: data["write_gate"].update(active_gazebo_clock_required=False),
        lambda data: data["write_gate"].update(real_vehicle_write_enabled=True),
        lambda data: data["typed_transaction"].update(failure_restore_target="startup_snapshot"),
        lambda data: data["calibration"].update(permitted_manifest_splits=["calibration", "validation"]),
        lambda data: data["claims"].update(anchor_calibration_complete=True),
    )
    for mutation in mutations:
        changed = copy.deepcopy(load_contract())
        mutation(changed)
        with pytest.raises(V2ContractError):
            validate_typed_calibration_contract(changed)


def test_calibration_manifest_is_disjoint_and_plan_contains_only_calibration_episodes():
    manifest = load_v2_scene_manifest(SCENES, WORKSPACE)
    foundation = load_v2_scene_manifest(
        WORKSPACE / "experiments/manifests/v2/scenes/v2_02_foundation_scenes.yaml",
        WORKSPACE,
    )
    assert len(manifest["scenes"]) == 5
    assert {row["split"] for row in manifest["scenes"]} == {"calibration"}
    assert not (
        {row["scene_id"] for row in manifest["scenes"]}
        & {row["scene_id"] for row in foundation["scenes"]}
    )
    assert not (
        {row["seed"] for row in manifest["scenes"]}
        & {row["seed"] for row in foundation["scenes"]}
    )
    plan = build_anchor_calibration_plan(CONTRACT, WORKSPACE)
    assert plan["candidate_count"] == 54
    assert plan["planned_episode_count"] == 90
    assert plan["completed_navigation_episode_count"] == 0
    assert plan["test_or_validation_selection_used"] is False
    assert all(
        evaluation["split"] == "calibration"
        for candidate in plan["candidates"]
        for evaluation in candidate["evaluations"]
    )
    assert all(
        len(evaluation["effective_profile_sha256"]) == 64
        for candidate in plan["candidates"]
        for evaluation in candidate["evaluations"]
    )
    assert len({row["candidate_id"] for row in plan["candidates"]}) == 54
    assert all(len(row["values"]) == 20 for row in plan["candidates"])
    probes = {}
    for row in plan["candidates"]:
        if row["screen_coordinate"] is not None:
            probes[(row["anchor_id"], row["screen_coordinate"], row["screen_level"])] = row
    for anchor_id, coordinate, level in list(probes):
        if level == -1:
            low = probes[(anchor_id, coordinate, -1)]["values"][coordinate]
            high = probes[(anchor_id, coordinate, 1)]["values"][coordinate]
            assert low < high


def test_simulation_typed_node_has_independent_guards_and_no_learning_or_motion_output():
    source = (
        WORKSPACE
        / "src/application/teb_mode_manager/scripts/simulation_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    for guard in (
        "allow_simulation_teb_parameter_write", "/use_sim_time",
        "/m2_gazebo/simulation_only", "/clock", "EXPECTED_TEB_NAMESPACE",
    ):
        assert guard in source
    assert "RosTypedDynamicReconfigureAdapter" in source
    assert "backend.close()" in source
    assert "checkpoint" not in source.lower()
    assert "/cmd_vel" not in source
    assert "/gazebo/model_states" not in source


def test_completed_screen_freeze_assessment_fails_closed_without_posthoc_rules():
    progress_path = WORKSPACE / "artifacts/v2/calibration/v2_04b_batch_progress.yaml"
    if not progress_path.is_file():
        pytest.skip("completed Gazebo calibration evidence is not present")
    plan_path = WORKSPACE / "artifacts/v2/calibration/v2_04b_anchor_screen_plan.yaml"
    progress = yaml.safe_load(progress_path.read_text(encoding="utf-8"))
    plan = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    report = assess_anchor_freeze(load_contract(), plan, progress, progress_path)
    assert report["evidence"]["valid_evidence_episode_count"] == 90
    assert report["decision"]["enter_anchor_freeze"] is False
    assert report["claims"]["anchor_values_frozen"] is False
    assert set(report["freeze_blockers"]) == {
        "balanced_cross_family_aggregation_not_preregistered",
        "dynamic_primary_ttc_objective_unobserved",
        "bounded_refinement_not_preregistered",
    }
    assert report["provisional_single_family_screen_winners"]["anchor_cruise"]["rankable"]
