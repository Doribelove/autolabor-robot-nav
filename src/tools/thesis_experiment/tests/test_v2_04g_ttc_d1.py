import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
SCRIPT = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/diagnose_v2_04g_ttc_d1.py"
)
CONTRACT = (
    WORKSPACE
    / "config/thesis_experiments/v2/"
    "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
)
REPORT = (
    WORKSPACE
    / "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
    "v2_04g_ttc_d1_report.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location(
        "diagnose_v2_04g_ttc_d1_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_d1_reproduces_seed5111_semantic_timing_and_clearance_diagnosis():
    module = _module()
    report = module.build_report(WORKSPACE, CONTRACT)

    assert report["status"] == "complete_offline_diagnosis"
    assert report["source"]["seed"] == 5111
    assert report["source"]["new_evidence_units_consumed"] == 0
    assert report["frozen_r5_outcome"]["observed_ttc_status"] == (
        "NO_CONFLICT_IN_HORIZON"
    )
    assert report["semantic_comparison"][
        "runtime_crossing_overlay_with_zero_evaluator_finite_ttc"
    ] is True
    assert report["arrival_timing"]["actor_centerline_crossing_time_s"] == (
        pytest.approx(10.917137089133282)
    )
    assert report["arrival_timing"]["robot_crossing_point_arrival_time_s"] == (
        pytest.approx(14.922396673811173)
    )
    assert report["arrival_timing"]["robot_minus_actor_arrival_time_s"] == (
        pytest.approx(4.005259584677891)
    )
    envelope = report["ttc_and_circle_envelope"]
    assert envelope["evaluation_finite_ttc_sample_count"] == 0
    assert envelope["trace_finite_ttc_sample_count"] == 0
    assert envelope["proxy_finite_ttc_sample_count"] == 0
    assert envelope["minimum_predicted_circle_envelope"][
        "predicted_center_separation_m"
    ] == pytest.approx(1.693733092818502)
    assert envelope["minimum_predicted_circle_envelope"][
        "circle_envelope_margin_m"
    ] == pytest.approx(0.6848243631659008)
    truth = report["truth_clearance"]
    assert truth["frozen_async_gazebo_truth_minimum_m"] == pytest.approx(
        1.6251341291519297
    )
    assert truth["trace_synchronous_proxy_minimum"]["clearance_m"] == (
        pytest.approx(1.625170961143916)
    )


def test_d1_quantifies_frozen_candidate_non_identifiability():
    report = _module().build_report(WORKSPACE, CONTRACT)
    result = report["integrated_candidate_distinguishability"]

    assert result["reachable_crossing_sample_count"] == 21
    assert result[
        "all_frozen_candidates_equal_on_reachable_crossing_samples"
    ] is True
    assert result["frozen_candidate_crossing_difference_count"] == 0
    assert result["only_unknown_class_samples_distinguish_frozen_candidates"] is True
    pairwise = {
        (row["first_horizon_s"], row["second_horizon_s"]): row
        for row in result["pairwise"]
    }
    assert pairwise[(5.0, 4.5)]["difference_count"] == 1
    assert pairwise[(4.5, 4.0)]["difference_count"] == 3
    assert pairwise[(5.0, 4.0)]["difference_count"] == 4
    assert all(
        row["crossing_motion_class_difference_count"] == 0
        for row in pairwise.values()
    )


def test_d1_exploratory_15_and_10_are_distinguishable_but_not_authorized():
    report = _module().build_report(WORKSPACE, CONTRACT)
    exploratory = report["exploratory_future_horizons"]
    counts = exploratory["overlay_counts_by_horizon_s"]
    pairwise = exploratory["pairwise"]

    assert counts["1.5"]["CROSSING"] == 16
    assert counts["1.0"]["CROSSING"] == 11
    assert pairwise[0]["difference_count"] == 9
    assert pairwise[0]["crossing_motion_class_difference_count"] == 5
    assert pairwise[1]["difference_count"] == 5
    assert pairwise[1]["crossing_motion_class_difference_count"] == 5
    assert exploratory["suitable_for_future_r6_execution"] is None
    assert exploratory["r6_execution_authorization_created"] is False
    assert exploratory[
        "requires_fresh_preregistration_review_and_authorization"
    ] is True
    assert all(
        value is False
        for value in report["authorizations_after_diagnosis"].values()
    )


def test_d1_machine_audit_contains_all_six_confirmed_risks():
    module = _module()
    report = module.build_report(WORKSPACE, CONTRACT)
    audit = report["risk_audit"]
    findings = audit["findings"]

    assert audit["required_count"] == 6
    assert audit["confirmed_count"] == 6
    assert tuple(row["risk_id"] for row in findings) == module.EXPECTED_RISK_IDS
    assert all(row["status"] == "CONFIRMED" for row in findings)
    by_id = {row["risk_id"]: row for row in findings}
    direct = by_id["D1-RISK-READINESS-DIRECT-COUNTS"]["evidence"]
    assert direct["direct_tracker_message_count_check_present"] is False
    assert direct["direct_context_message_count_check_present"] is False
    toctou = by_id["D1-RISK-COMPILED-SCENE-TOCTOU"]["evidence"]
    assert toctou["child_sha256_revalidation_present_in_design_block"] is False
    sigint = by_id["D1-RISK-SIGINT-IN-PROGRESS"]["evidence"]
    assert sigint["keyboard_interrupt_handler_present"] is False
    binding = by_id["D1-RISK-ASSESSMENT-RAW-BINDING"]["evidence"]
    assert binding["activation_directly_bound"] is False
    assert binding["evaluation_directly_bound"] is False
    assert binding["trace_directly_bound"] is False
    closure = by_id["D1-RISK-EXECUTION-HASH-CLOSURE"]["evidence"]
    assert closure["missing_from_preregistered_closure_count"] > 0
    teardown = by_id["D1-RISK-TEARDOWN-RESTORE"]["evidence"]
    assert teardown["restore_failure_after_goal"] is True
    assert teardown["startup_profile_restored"] is False


def test_d1_report_is_deterministic_and_matches_checked_in_machine_report():
    module = _module()
    first = module.build_report(WORKSPACE, CONTRACT)
    second = module.build_report(WORKSPACE, CONTRACT)
    persisted = yaml.safe_load(REPORT.read_text(encoding="utf-8"))

    assert first == second
    assert persisted == first


def test_d1_rejects_duplicate_yaml_keys(tmp_path):
    module = _module()
    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text("stage: D1\nstage: drift\n", encoding="utf-8")

    with pytest.raises(module.D1DiagnosisError, match="duplicate YAML key"):
        module._load_yaml(duplicate)


def test_d1_rejects_any_declared_frozen_input_hash_drift():
    module = _module()
    contract = module._load_yaml(CONTRACT)
    contract["frozen_inputs"]["trace"]["sha256"] = "0" * 64

    with pytest.raises(module.D1DiagnosisError, match="trace"):
        module._verify_declared_inputs(WORKSPACE, contract)


def test_d1_rejects_noncanonical_output_without_writing():
    module = _module()
    forbidden = (
        WORKSPACE
        / "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
        "noncanonical-output.yaml"
    )
    assert not forbidden.exists()

    with pytest.raises(module.D1DiagnosisError, match="output path drifted"):
        module.diagnose(WORKSPACE, CONTRACT, forbidden)
    assert not forbidden.exists()


def test_d1_build_does_not_modify_any_r5_artifact():
    module = _module()
    before = module._snapshot_tree(WORKSPACE, module.R5_ARTIFACT_RELATIVE)
    module.build_report(WORKSPACE, CONTRACT)
    after = module._snapshot_tree(WORKSPACE, module.R5_ARTIFACT_RELATIVE)

    assert before == after
    assert before["file_count"] == 68
