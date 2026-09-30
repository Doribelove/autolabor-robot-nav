import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

WORKSPACE = Path("/home/robot/robot_ws_base_rl")
FOUNDATION_SOURCE = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_foundation.py"
)
REVIEWER_SOURCE = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/review_v3_00_foundation.py"
)


def _foundation():
    spec = importlib.util.spec_from_file_location(
        "v3_00_foundation_under_test", FOUNDATION_SOURCE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


foundation = _foundation()


def _reviewer():
    spec = importlib.util.spec_from_file_location(
        "v3_00_reviewer_under_test", REVIEWER_SOURCE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_v3_00_bundle_is_byte_deterministic_and_persisted():
    first = foundation.build_bundle(WORKSPACE)
    second = foundation.build_bundle(WORKSPACE)
    assert first == second
    assert len(first) == 9
    foundation.validate_persisted_bundle(WORKSPACE, first)


def test_v3_00_contract_has_exact_zero_authority():
    contract = foundation.load_yaml(WORKSPACE, foundation.CONTRACT_RELATIVE)
    audit = foundation.validate_contract(WORKSPACE, contract)
    assert audit["contract_valid"] is True
    assert audit["source_freeze_count"] == 7
    state = contract["stage_state"]
    assert state["execution_authorized"] is False
    assert state["training_authorized"] is False
    assert state["evidence_budget_authorized"] == 0
    assert state["seed_allocation_count"] == 0
    assert state["schedule_row_count"] == 0


def test_candidate_fixture_rejects_duplicate_id_and_steering_limit_violation():
    contract = foundation.load_yaml(WORKSPACE, foundation.CONTRACT_RELATIVE)
    fixture = foundation.load_yaml(WORKSPACE, foundation.FIXTURE_RELATIVE)
    duplicate = copy.deepcopy(fixture)
    duplicate["candidates"][1]["candidate_id"] = duplicate["candidates"][0][
        "candidate_id"
    ]
    with pytest.raises(foundation.V3FoundationError):
        foundation.compile_candidate_fixture(contract, duplicate)

    steering = copy.deepcopy(fixture)
    steering["candidates"][1]["steering_angle_rad"] = 1.0
    with pytest.raises(foundation.V3FoundationError):
        foundation.compile_candidate_fixture(contract, steering)


def test_required_packages_and_message_registry_are_unique():
    packages = foundation.discover_required_packages(WORKSPACE)
    assert [row["package_name"] for row in packages] == list(
        foundation.REQUIRED_PACKAGES
    )
    assert len({row["source_root"] for row in packages}) == len(packages)
    messages = foundation.parse_message_registry(WORKSPACE)
    assert [row["message"] for row in messages] == [
        name[:-4] for name in foundation.MESSAGE_NAMES
    ]


def test_static_actor_fixture_is_not_simulation_evidence():
    actor = yaml.safe_load(
        (WORKSPACE / foundation.ACTOR_CONTRACT_RELATIVE).read_text(
            encoding="utf-8"
        )
    )
    assert actor["simulation_evidence"] is False
    assert actor["counterfactual_identifiability_demonstrated"] is False
    assert actor["clone_count"] == 24
    assert actor["all_propensities_strictly_positive"] is True
    assert actor["same_initial_state_digest_within_block"] is True
    assert all(
        row["actor_policy_exposed_to_runtime"] is False
        for row in actor["static_rows"]
    )


def test_persisted_independent_review_is_exactly_reproducible():
    reviewer = _reviewer()
    rebuilt = reviewer.build_review()
    persisted = yaml.safe_load(
        (WORKSPACE / foundation.REVIEW_RELATIVE).read_text(encoding="utf-8")
    )
    assert rebuilt == persisted
    assert rebuilt["status"] == "v3_00_pure_offline_foundation_closure_pass"
    assert rebuilt["offline_review_pass"] is True
    assert rebuilt["execution_authorized"] is False
    assert rebuilt["training_authorized"] is False
    assert rebuilt["evidence_budget_authorized"] == 0
    assert rebuilt["next_stage_automatically_authorized"] is False


def test_v3_00_forbidden_execution_state_is_absent():
    for name in (
        "execution_release.yaml",
        "simulation_authorization.yaml",
        "seed_allocation.yaml",
        "exact_schedule.yaml",
        "attempts",
        "journals",
        "models",
        "checkpoints",
    ):
        assert not (WORKSPACE / foundation.OUTPUT_ROOT / name).exists()
