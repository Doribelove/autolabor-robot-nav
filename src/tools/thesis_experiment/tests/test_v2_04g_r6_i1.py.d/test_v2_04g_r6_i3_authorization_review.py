import copy
import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[5]
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml"
)
AUTHORIZATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml"
)
REPORT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i3_authorization_review/"
    "v2_04g_r6_i3_authorization_review.yaml"
)


def _reviewer():
    path = WORKSPACE / (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py"
    )
    specification = importlib.util.spec_from_file_location(
        "v2_04g_r6_i3_authorization_reviewer_test", path
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _preregistration():
    return yaml.safe_load(PREREGISTRATION.read_text(encoding="utf-8"))


def test_r6_i3_machine_review_is_deterministic_and_non_executing():
    reviewer = _reviewer()
    first = reviewer.build_review(
        WORKSPACE, reviewer.EXPECTED_AUTHORIZATION_SHA256
    )
    second = reviewer.build_review(
        WORKSPACE, reviewer.EXPECTED_AUTHORIZATION_SHA256
    )
    persisted = yaml.safe_load(REPORT.read_text(encoding="utf-8"))
    assert first == second == persisted
    assert first["review_result"] == "pass"
    assert first["authorization_envelope_valid"] is True
    assert first["authorization_manifest_execution_authorized"] is True
    assert first["execution_release_received"] is False
    assert first["execution_ready"] is False
    assert (
        first["execution_absence_review"][
            "execution_release_manifest_present"
        ]
        is False
    )
    assert (
        first["execution_absence_review"]["closed_world_stage_prefix_scan"]
        is True
    )
    assert first["next_gate"]["execution_may_start_now"] is False
    assert (
        first["next_gate"]["dedicated_release_schema_and_validator_required"]
        is True
    )
    assert (
        first["next_gate"][
            "release_validation_before_any_journal_directory_or_subprocess"
        ]
        is True
    )
    assert first["side_effects"]["seed_or_evidence_units_consumed"] == 0
    assert first["side_effects"]["ros_started"] is False
    assert first["side_effects"]["gazebo_started"] is False


def test_r6_i3_schedule_and_budget_are_exact_and_paired():
    reviewer = _reviewer()
    preregistration = _preregistration()
    result = reviewer._verify_preregistration_document(
        preregistration, reviewer._authorization_module(WORKSPACE)
    )
    assert result["execution_seeds"] == [5151, 5152, 5153]
    assert result["compile_support_only_seeds"] == [5154, 5155, 5156, 5157]
    assert result["identity_count"] == 6
    assert result["evidence_budget_units"] == 6
    schedule = preregistration["schedule"]
    assert [row["sequence"] for row in schedule] == list(range(1, 7))
    pairs = {}
    for row in schedule:
        pairs.setdefault(row["scene_id"], set()).add(row["profile_id"])
    assert set(map(frozenset, pairs.values())) == {
        frozenset(reviewer.EXPECTED_PROFILES)
    }
    assert len(pairs) == 3


def test_r6_i3_authorization_exactly_copies_preregistered_schedule():
    preregistration = _preregistration()
    authorization = yaml.safe_load(AUTHORIZATION.read_text(encoding="utf-8"))
    assert authorization["exact_schedule"] == preregistration["schedule"]
    assert authorization["evidence_budget_authorized"] == 6
    assert authorization["fresh_execution_seeds"] == [5151, 5152, 5153]
    assert authorization["retry_or_resume_allowed"] is False
    assert authorization["seed_replacement_allowed"] is False
    assert authorization["budget_expansion_allowed"] is False
    assert "inherited_r6_i2_dependency_closure" in authorization[
        "bound_resources"
    ]
    assert "source_r6_i1_compiled_scene_index" in authorization[
        "bound_resources"
    ]
    assert "dependency_closure" not in authorization["bound_resources"]
    assert "compiled_scene_index" not in authorization["bound_resources"]


def test_r6_i3_release_is_a_separate_absent_fail_closed_artifact():
    preregistration = _preregistration()
    plan = preregistration["execution_release_plan"]
    assert plan["status"] == "required_not_created"
    assert plan["manifest_present"] is False
    assert plan["authorization_envelope_alone_sufficient_for_execution"] is False
    assert plan["future_entrypoint_must_fail_closed_without_valid_release"] is True
    assert plan["caller_supplied_exact_release_sha256_required"] is True
    assert plan["dedicated_release_schema_and_validator_required"] is True
    assert plan["release_schema_closed_and_type_sensitive"] is True
    assert plan["release_hash_and_parse_single_open_no_follow"] is True
    assert plan["release_validation_before_any_journal_directory_or_subprocess"] is True
    assert not (WORKSPACE / plan["canonical_manifest_path"]).exists()


def test_r6_i3_fresh_seed_audit_excludes_all_prior_allocations():
    reviewer = _reviewer()
    result = reviewer._verify_historical_seed_firewall(
        WORKSPACE, reviewer._authorization_module(WORKSPACE)
    )
    assert result["historical_high_watermark"] == 5147
    assert result["fresh_seed_block"] == list(range(5151, 5158))
    assert result["fresh_seed_prior_reference_count"] == 0


@pytest.mark.parametrize(
    "mutate,match",
    [
        (
            lambda document: document.update({"unexpected": False}),
            "preregistration keys drifted",
        ),
        (
            lambda document: document["schedule"].reverse(),
            "schedule drifted",
        ),
        (
            lambda document: document.update(
                {"execution_release_received": True}
            ),
            "execution_release_received drifted",
        ),
        (
            lambda document: document["fresh_seed_firewall"].update(
                {"execution_seeds": [5141, 5152, 5153]}
            ),
            "seed firewall drifted",
        ),
        (
            lambda document: document["frozen_common_values"].update(
                {"predicted_ttc_max_s": 1.5}
            ),
            "frozen common values drifted",
        ),
        (
            lambda document: document["fresh_scene_identity_plan"].update(
                {"fresh_compiled_scene_index_present": True}
            ),
            "scene plan drifted",
        ),
        (
            lambda document: document["execution_release_plan"].update(
                {"manifest_present": True}
            ),
            "execution release plan drifted",
        ),
    ],
)
def test_r6_i3_preregistration_semantic_mutations_fail_closed(mutate, match):
    reviewer = _reviewer()
    document = copy.deepcopy(_preregistration())
    mutate(document)
    with pytest.raises(reviewer.R6I3AuthorizationReviewError, match=match):
        reviewer._verify_preregistration_document(
            document, reviewer._authorization_module(WORKSPACE)
        )


def test_r6_i3_reviewer_rejects_non_caller_trust_anchor_hash():
    reviewer = _reviewer()
    with pytest.raises(
        reviewer.R6I3AuthorizationReviewError,
        match="caller authorization SHA256",
    ):
        reviewer.build_review(WORKSPACE, "0" * 64)


def test_r6_i3_execution_material_is_absent_by_contract():
    reviewer = _reviewer()
    result = reviewer._verify_execution_absence(WORKSPACE)
    assert result["fresh_scene_index_present"] is False
    assert result["actual_execution_entrypoint_present"] is False
    assert result["execution_journal_present"] is False
    assert result["evidence_units_consumed"] == 0
    assert result["execution_ready"] is False


@pytest.mark.parametrize(
    "relative",
    [
        (
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i3_execution_release.yaml"
        ),
        "artifacts/v2/integration/v2_04g_r6_i3/execution/journal.yaml",
        (
            "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/"
            "compiled_scenes/unbound.instance.yaml"
        ),
        (
            "src/tools/thesis_experiment/scripts/"
            "v2_04g_r6_i3_alternate_executor.py"
        ),
    ],
)
def test_r6_i3_closed_world_absence_rejects_any_execution_material(
    tmp_path, relative
):
    reviewer = _reviewer()
    target = tmp_path / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("not authorized\n", encoding="utf-8")
    with pytest.raises(
        reviewer.R6I3AuthorizationReviewError,
        match="execution material unexpectedly exists",
    ):
        reviewer._verify_execution_absence(tmp_path)


def test_r6_i3_review_binds_reviewer_and_directed_test_bytes():
    reviewer = _reviewer()
    report = yaml.safe_load(REPORT.read_text(encoding="utf-8"))
    integrity = report["review_source_integrity"]
    for label, relative in (
        ("reviewer", reviewer.REVIEWER_RELATIVE),
        ("directed_test", reviewer.DIRECTED_TEST_RELATIVE),
    ):
        payload = (WORKSPACE / relative).read_bytes()
        assert integrity[label]["path"] == relative.as_posix()
        assert integrity[label]["sha256"] == hashlib.sha256(payload).hexdigest()
    assert integrity["single_open_no_follow"] is True
