import copy
import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml

from thesis_experiment import (
    v2_04g_r6_i1_r6_i2_authorization as authorization,
)


WORKSPACE = Path(__file__).resolve().parents[5]
STAGE = "V2-04G-R6-I2"
SYNTHETIC_SEED = (1 << 30) + 1
REPORT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_authorization_assessment_review.yaml"
)


def _assessor():
    path = (
        WORKSPACE
        / "src/tools/thesis_experiment/scripts/"
          "v2_04g_r6_i1_r6_i2_assessor.py"
    )
    specification = importlib.util.spec_from_file_location(
        "v2_04g_r6_i1_r6_i2_assessor_test", path
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(value, sort_keys=False),
        encoding="utf-8",
    )


def _schedule():
    return [
        {
            "sequence": 1,
            "profile_id": "fixture_profile_a",
            "scene_id": "fixture_scene",
            "seed": SYNTHETIC_SEED,
            "attempt": 1,
            "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
            "expected_overlay_semantics": "none_iff_no_finite_ttc",
        },
        {
            "sequence": 2,
            "profile_id": "fixture_profile_b",
            "scene_id": "fixture_scene",
            "seed": SYNTHETIC_SEED,
            "attempt": 1,
            "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
            "expected_overlay_semantics": "none_iff_no_finite_ttc",
        },
    ]


def _preregistration():
    schedule = _schedule()
    return {
        "schema_version": "2.0",
        "stage": STAGE,
        "execution_authorized": False,
        "fresh_seed_firewall": {
            "execution_seeds": [SYNTHETIC_SEED],
        },
        "budget": {
            "evidence_units_authorizable": len(schedule),
            "attempt_limit_per_identity": 1,
            "retry_allowed": False,
            "resume_allowed": False,
            "replacement_seed_allowed": False,
            "budget_expansion_allowed": False,
        },
        "schedule": schedule,
    }


def _fixture_workspace(tmp_path):
    resource_paths = {
        "contract": "config/fixture_contract.yaml",
        "preregistration": "manifests/fixture_preregistration.yaml",
        "dependency_closure": "artifacts/fixture_closure.yaml",
        "integration_review": "artifacts/fixture_review.yaml",
        "compiled_scene_index": "artifacts/fixture_index.yaml",
        "r6_design_report": "artifacts/fixture_design.yaml",
    }
    documents = {
        "contract": {"stage": STAGE, "execution_authorized": False},
        "preregistration": _preregistration(),
        "dependency_closure": {
            "stage": STAGE,
            "closure_sha256": "d" * 64,
        },
        "integration_review": {
            "stage": STAGE,
            "review_result": "pass",
            "execution_authorized": False,
        },
        "compiled_scene_index": {"stage": STAGE, "scenes": []},
        "r6_design_report": {
            "stage": "V2-04G-R6",
            "review_result": "pass",
        },
    }
    for label, relative in resource_paths.items():
        _dump(tmp_path / relative, documents[label])
    bindings = {
        label: {"path": relative, "sha256": _digest(tmp_path / relative)}
        for label, relative in resource_paths.items()
    }
    schedule = documents["preregistration"]["schedule"]
    auth = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "authorization_id": "fixture_authorization",
        "status": "bounded_fresh_seed_simulation_authorized",
        "authorization_date": "fixture-date",
        "authorization_source":
            "explicit_user_instruction_after_independent_integration_review",
        "execution_authorized": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "scope": {
            "purpose":
                "runtime_evaluator_semantic_and_execution_integrity_validation",
            "stage_only": STAGE,
            "profiles": ["fixture_profile_a", "fixture_profile_b"],
            "fresh_execution_seeds": [SYNTHETIC_SEED],
            "exact_identity_count": len(schedule),
            "component_stage_authorized": False,
            "general_navigation_calibration_authorized": False,
            "winner_selection_authorized": False,
        },
        "evidence_budget_authorized": len(schedule),
        "fresh_execution_seeds": [SYNTHETIC_SEED],
        "attempt_limit_per_identity": 1,
        "retry_or_resume_allowed": False,
        "seed_replacement_allowed": False,
        "budget_expansion_allowed": False,
        "stop_on_first_terminal_failure": True,
        "forfeit_unattempted_units_after_terminal_failure": True,
        "i1_retry_or_resume_authorized": False,
        "i1_forfeited_units_reused": False,
        "prior_identity_reuse_allowed": False,
        "r5_retry_or_resume_authorized": False,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "rank_or_freeze_winner_authorized": False,
        "v2_05_authorized": False,
        "sac_or_training_authorized": False,
        "real_vehicle_authorized": False,
        "real_vehicle_teb_write_authorized": False,
        "exact_schedule": copy.deepcopy(schedule),
        "preregistration_schedule_sha256":
            authorization.canonical_document_sha256(schedule),
        "bound_resources": bindings,
        "dependency_closure_digest": "d" * 64,
        "authorization_trust_anchor": {
            "mechanism":
                "caller_supplied_exact_authorization_file_sha256",
            "self_hash_embedded": False,
            "guard_rejects_missing_or_mismatched_cli_hash": True,
        },
        "completion_boundary": {
            "maximum_claim":
                "fresh_simulation_runtime_evaluator_semantic_integration",
            "safety_performance_generalization_claim_allowed": False,
            "formal_result_must_remain_false": True,
            "runtime_ready_must_remain_false": True,
            "downstream_authorization_after_completion": False,
        },
    }
    auth_path = "manifests/fixture_authorization.yaml"
    _dump(tmp_path / auth_path, auth)
    return auth, auth_path, resource_paths


def _write_and_validate(tmp_path, auth, auth_path, resource_paths):
    _dump(tmp_path / auth_path, auth)
    return authorization.load_and_validate_authorization(
        tmp_path,
        auth_path,
        _digest(tmp_path / auth_path),
        STAGE,
        "preregistration",
        resource_paths,
        "d" * 64,
    )


def test_single_open_hash_and_parse_reuses_each_resource_snapshot(
    tmp_path, monkeypatch
):
    auth, auth_path, resource_paths = _fixture_workspace(tmp_path)
    counts = {}
    original = authorization.read_workspace_file_once

    def counted(workspace, declared_path, parse_yaml=False):
        counts[declared_path] = counts.get(declared_path, 0) + 1
        return original(workspace, declared_path, parse_yaml=parse_yaml)

    monkeypatch.setattr(
        authorization, "read_workspace_file_once", counted
    )
    result = _write_and_validate(
        tmp_path, auth, auth_path, resource_paths
    )
    assert result.identity_count == 2
    assert result.execution_seeds == (SYNTHETIC_SEED,)
    assert counts == {
        auth_path: 1,
        **{relative: 1 for relative in resource_paths.values()},
    }
    assert (
        result.preregistration
        is result.bound_resources["preregistration"]
    )


def test_single_open_reader_rejects_symlink_and_duplicate_key(tmp_path):
    _dump(tmp_path / "real.yaml", {"value": 1})
    (tmp_path / "link.yaml").symlink_to(tmp_path / "real.yaml")
    with pytest.raises(
        authorization.R6I2AuthorizationError, match="safely open"
    ):
        authorization.read_workspace_yaml_once(tmp_path, "link.yaml")
    (tmp_path / "duplicate.yaml").write_text(
        "value: 1\nvalue: 2\n", encoding="utf-8"
    )
    with pytest.raises(
        authorization.R6I2AuthorizationError, match="duplicate YAML key"
    ):
        authorization.read_workspace_yaml_once(
            tmp_path, "duplicate.yaml"
        )


def test_authorization_rejects_caller_hash_mismatch(tmp_path):
    _, auth_path, resource_paths = _fixture_workspace(tmp_path)
    with pytest.raises(
        authorization.R6I2AuthorizationError, match="trust-anchor"
    ):
        authorization.load_and_validate_authorization(
            tmp_path,
            auth_path,
            "0" * 64,
            STAGE,
            "preregistration",
            resource_paths,
            "d" * 64,
        )


@pytest.mark.parametrize(
    "mutate,match",
    [
        (
            lambda document: document.update({"unexpected": False}),
            "authorization keys drifted",
        ),
        (
            lambda document: document["scope"].update(
                {"unexpected": False}
            ),
            "authorization.scope keys drifted",
        ),
        (
            lambda document: document["exact_schedule"].reverse(),
            "exact schedule differs",
        ),
        (
            lambda document: document["exact_schedule"][0].update(
                {"seed": True}
            ),
            "exact schedule differs",
        ),
        (
            lambda document: document.update(
                {"i1_forfeited_units_reused": True}
            ),
            "i1_forfeited_units_reused drifted",
        ),
    ],
)
def test_closed_authorization_rejects_schema_schedule_and_firewall_drift(
    tmp_path, mutate, match
):
    auth, auth_path, resource_paths = _fixture_workspace(tmp_path)
    mutate(auth)
    with pytest.raises(authorization.R6I2AuthorizationError, match=match):
        _write_and_validate(tmp_path, auth, auth_path, resource_paths)


def test_authorization_rejects_bound_resource_hash_drift(tmp_path):
    auth, auth_path, resource_paths = _fixture_workspace(tmp_path)
    auth["bound_resources"]["contract"]["sha256"] = "0" * 64
    with pytest.raises(
        authorization.R6I2AuthorizationError,
        match="contract bound hash drifted",
    ):
        _write_and_validate(tmp_path, auth, auth_path, resource_paths)


def test_authorization_enforces_independent_logical_closure_digest(tmp_path):
    auth, auth_path, resource_paths = _fixture_workspace(tmp_path)
    closure_path = tmp_path / resource_paths["dependency_closure"]
    _dump(
        closure_path,
        {"stage": STAGE, "closure_sha256": "c" * 64},
    )
    auth["bound_resources"]["dependency_closure"]["sha256"] = _digest(
        closure_path
    )
    with pytest.raises(
        authorization.R6I2AuthorizationError,
        match="logical digest drifted",
    ):
        _write_and_validate(tmp_path, auth, auth_path, resource_paths)


def _empty_assessment_inputs():
    preregistration = {
        "stage": STAGE,
        "execution_authorized": False,
        "schedule": [],
    }
    stage_report = {
        "stage": STAGE,
        "status": "integration_review_only",
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "retry_count": 0,
        "resume_used": False,
        "attempt_ledger": [],
        "evidence_units_consumed": 0,
    }
    return preregistration, stage_report


def test_assessor_is_deterministic_and_has_explicit_stage_digest_scope():
    assessor = _assessor()
    preregistration, stage_report = _empty_assessment_inputs()
    arguments = {
        "stage": STAGE,
        "preregistration": preregistration,
        "preregistration_sha256": "a" * 64,
        "stage_report": stage_report,
        "stage_report_sha256": "b" * 64,
    }
    first = assessor.build_offline_assessment(**arguments)
    second = assessor.build_offline_assessment(**arguments)
    assert first == second
    assert first["status"] == "no_execution_state_reviewed"
    assert first["attempted_identity_count"] == 0
    assert first["source_bindings"]["stage_report_input_sha256"] == "b" * 64
    assert first["integration_validation_pass"] is False


def test_assessor_accepts_i2_repair_review_empty_execution_schedule():
    assessor = _assessor()
    preregistration = yaml.safe_load(
        (
            WORKSPACE
            / "experiments/manifests/v2/integration/"
              "v2_04g_r6_i2_repair_preregistration.yaml"
        ).read_text(encoding="utf-8")
    )
    _, stage_report = _empty_assessment_inputs()
    result = assessor.build_offline_assessment(
        stage=STAGE,
        preregistration=preregistration,
        preregistration_sha256="a" * 64,
        stage_report=stage_report,
        stage_report_sha256="b" * 64,
    )
    assert preregistration["seed_values"] == []
    assert result["planned_identity_count"] == 0
    assert result["attempted_identity_count"] == 0


def test_assessor_rejects_nonempty_ledger_without_direct_replay():
    assessor = _assessor()
    preregistration, stage_report = _empty_assessment_inputs()
    preregistration["schedule"] = _schedule()[:1]
    stage_report["attempt_ledger"] = [{
        "sequence": 1,
        "identity": {
            "stage": STAGE,
            "profile_id": "fixture_profile_a",
            "scene_id": "fixture_scene",
            "seed": SYNTHETIC_SEED,
            "attempt": 1,
        },
    }]
    stage_report["evidence_units_consumed"] = 1
    with pytest.raises(
        assessor.R6I2AssessmentError, match="requires direct persisted replay"
    ):
        assessor.build_offline_assessment(
            stage=STAGE,
            preregistration=preregistration,
            preregistration_sha256="a" * 64,
            stage_report=stage_report,
            stage_report_sha256="b" * 64,
        )


def test_assessor_rejects_non_exact_schedule_prefix():
    assessor = _assessor()
    preregistration, stage_report = _empty_assessment_inputs()
    preregistration["schedule"] = _schedule()[:1]
    stage_report["attempt_ledger"] = [{
        "sequence": 1,
        "identity": {
            "stage": STAGE,
            "profile_id": "fixture_profile_a",
            "scene_id": "fixture_scene",
            "seed": True,
            "attempt": 1,
        },
    }]
    with pytest.raises(
        assessor.R6I2AssessmentError, match="exact schedule prefix"
    ):
        assessor.build_offline_assessment(
            stage=STAGE,
            preregistration=preregistration,
            preregistration_sha256="a" * 64,
            stage_report=stage_report,
            stage_report_sha256="b" * 64,
            replay_attempt=lambda row: row,
        )


def test_machine_review_is_zero_authority_and_deterministic():
    assessor = _assessor()
    first = assessor.build_repair_review()
    second = assessor.build_repair_review()
    persisted = yaml.safe_load(REPORT.read_text(encoding="utf-8"))
    assert first == second == persisted
    assert first["execution_authorized"] is False
    assert first["real_authorization_created"] is False
    assert first["seed_values"] == []
    assert first["evidence_budget_units"] == 0
