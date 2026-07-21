import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_04g_r6_i1_dependency import (
    build_dependency_closure,
)
from thesis_experiment.v2_04g_r6_integrity import (
    R6IntegrityError,
    verify_dependency_closure,
)


WORKSPACE = Path(__file__).resolve().parents[4]
ROOT = WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1"
CONTRACT = WORKSPACE / (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_execution_integration_contract.yaml"
)
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_execution_preregistration.yaml"
)
CLOSURE = ROOT / "execution_dependency_closure.yaml"
REPORT = ROOT / "v2_04g_r6_i1_integration_review.yaml"
AUTHORIZATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_bounded_simulation_authorization.yaml"
)


def _module(name, path):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _reviewer():
    return _module(
        "review_v2_04g_r6_i1_test",
        WORKSPACE
        / "src/tools/thesis_experiment/scripts/review_v2_04g_r6_i1.py",
    )


def _runner():
    return _module(
        "v2_04g_r6_i1_runner_test",
        WORKSPACE
        / "src/tools/thesis_experiment/scripts/"
          "v2_04g_r6_i1_bounded_validation.py",
    )


def test_integration_contract_and_preregistration_remain_non_authorizing():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    prereg = yaml.safe_load(PREREGISTRATION.read_text(encoding="utf-8"))
    assert contract["stage"] == "V2-04G-R6-I1"
    assert contract["execution_authorized"] is False
    assert prereg["execution_authorized"] is False
    assert prereg["budget"]["evidence_units_authorizable"] == 6
    assert prereg["budget"]["evidence_units_consumed_before_authorization"] == 0
    assert prereg["fresh_seed_firewall"]["execution_seeds"] == [5141, 5142, 5143]
    assert prereg["fresh_seed_firewall"]["held_out_5001_5010_forbidden"] is True


def test_runtime_profiles_have_one_exact_leaf_difference():
    report = _reviewer().build_report()
    factor = report["single_factor_runtime_review"]
    assert factor["normalized_leaf_difference_count"] == 1
    assert factor["only_difference"] == "dynamic.conflict_estimator_id"


def test_fresh_scene_schedule_is_paired_and_compile_support_is_zero_evidence():
    prereg = yaml.safe_load(PREREGISTRATION.read_text(encoding="utf-8"))
    schedule = prereg["schedule"]
    pairs = {}
    for row in schedule:
        pairs.setdefault(row["seed"], set()).add(row["profile_id"])
    assert pairs == {
        5141: {"r6_semantics_legacy_control", "r6_semantics_circle_contact"},
        5142: {"r6_semantics_legacy_control", "r6_semantics_circle_contact"},
        5143: {"r6_semantics_legacy_control", "r6_semantics_circle_contact"},
    }
    assert prereg["budget"]["compile_support_evidence_units"] == 0


def test_dependency_closure_is_mechanically_reproducible():
    persisted = yaml.safe_load(CLOSURE.read_text(encoding="utf-8"))
    generated = build_dependency_closure(WORKSPACE)
    assert persisted == generated
    verified = verify_dependency_closure(
        WORKSPACE, persisted, generated["required_paths"]
    )
    assert verified["closure_sha256"] == persisted["closure_sha256"]
    assert verified["file_count"] >= 90


def test_dependency_closure_rejects_an_omitted_file():
    persisted = yaml.safe_load(CLOSURE.read_text(encoding="utf-8"))
    malformed = copy.deepcopy(persisted)
    malformed["files"].pop()
    with pytest.raises(R6IntegrityError, match="path set"):
        verify_dependency_closure(
            WORKSPACE, malformed, persisted["required_paths"]
        )


def test_runner_binds_all_six_integrity_protocol_call_sites():
    source = (
        WORKSPACE
        / "src/tools/thesis_experiment/scripts/"
          "v2_04g_r6_i1_bounded_validation.py"
    ).read_text(encoding="utf-8")
    for token in (
        "validate_readiness_raw_evidence",
        "acquire_compiled_scene_lease",
        "AtomicAttemptJournal",
        "bind_attempt_raw_evidence",
        "build_dependency_closure",
        "verify_teardown_restore",
        "authorize_launch_stop",
    ):
        assert token in source


def test_supervisor_adapter_keeps_xy_footprint_and_transaction_has_arm_gate():
    supervisor = (
        WORKSPACE
        / "src/application/teb_mode_manager/scripts/"
          "r6_rule_context_supervisor_node.py"
    ).read_text(encoding="utf-8")
    transaction = (
        WORKSPACE
        / "src/application/teb_mode_manager/scripts/"
          "v2_04g_r6_typed_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    assert "FootprintRuntimeTrack" in supervisor
    assert "(point.x, point.y)" in supervisor
    assert "config_sha256" in supervisor
    assert "execution_armed" in transaction
    assert "arm_execution" in transaction
    assert "restore_startup_two_phase" in transaction


def test_machine_review_is_deterministic_and_non_authorizing():
    reviewer = _reviewer()
    first = reviewer.build_report()
    second = reviewer.build_report()
    persisted = yaml.safe_load(REPORT.read_text(encoding="utf-8"))
    assert first == second == persisted
    assert first["review_result"] == "pass"
    assert first["execution_authorized"] is False
    assert first["seed_or_evidence_units_consumed"] == 0


def test_separate_authorization_binds_review_without_expanding_scope():
    authorization = yaml.safe_load(AUTHORIZATION.read_text(encoding="utf-8"))
    assert authorization["execution_authorized"] is True
    assert authorization["evidence_budget_authorized"] == 6
    assert authorization["fresh_execution_seeds"] == [5141, 5142, 5143]
    assert authorization["retry_or_resume_allowed"] is False
    assert authorization["held_out_5001_5010_accessed"] is False
    assert authorization["r5_remaining_units_consumed"] == 0
    review = authorization["bound_resources"]["integration_review"]
    assert review["sha256"] == _reviewer().sha256(REPORT)


def test_no_forbidden_runtime_process_is_live_after_validation():
    assert _runner()._process_matches() == []
