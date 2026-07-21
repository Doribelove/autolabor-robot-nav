import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / (
    "experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml")
CONTRACT = WORKSPACE / (
    "config/thesis_experiments/v2/v2_04g_r5_ttc_robustness_contract.yaml")
BANK = WORKSPACE / (
    "experiments/manifests/v2/calibration/v2_04g_r5_ttc_timing_candidates.yaml")
AUDIT = WORKSPACE / (
    "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_dry_run_audit.yaml")
ASSESSOR_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/assess_v2_04g_r5.py")


def _module():
    spec = importlib.util.spec_from_file_location(
        "assess_v2_04g_r5_test", ASSESSOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ASSESSOR = _module()


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _base_readiness(prereg):
    reports = []
    for row in prereg["ttc_activation_coverage_readiness"]["schedule"]:
        reports.append({
            "sequence": row["sequence"],
            "identity": row["identity"],
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "attempt": 1,
            "expected_ttc_status": row["expected_status"],
            "observed_ttc_status": row["expected_status"],
            "status": "pass",
            "all_hard_gates_pass": True,
            "maximum_consecutive_stable_count": 10,
            "tracker_message_count": 30,
            "context_message_count": 30,
            "transaction_message_count": 30,
            "mechanism_message_count": 30,
            "transaction_activated_fraction": 1.0,
            "transaction_valid_fraction": 1.0,
            "transaction_join_valid_fraction": 1.0,
            "expected_context_hold_count": 0,
            "world_model_sequence_mismatch_count": 0,
            "world_model_input_join_fault_count": 0,
            "backend_transaction_fault_count": 0,
            "unknown_transaction_fault_count": 0,
            "fault_taxonomy_counts": {"CLEAN": 30},
            "training_used": False,
            "real_vehicle_used": False,
            "runtime_scene_labels_available": False,
            "runtime_policy_manifest_access": False,
        })
    return {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "status": "complete",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "attempts_per_identity_max": 1,
        "attempted_identity_count": 6,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_failure": None,
        "planned_probe_count": 6,
        "executed_probe_count": 6,
        "valid_probe_count": 6,
        "all_probe_hard_gates_pass": True,
        "ttc_coverage_pass": True,
        "ttc_component_authorized": True,
        "navigation_authorized": False,
        "reports": reports,
    }


def _base_component(prereg):
    probes = []
    for row in prereg["ttc_component_probe"]["schedule"]:
        probes.append({
            "sequence": row["sequence"],
            "identity": row["identity"],
            "attempt": 1,
            "expected_status": row["expected_status"],
            "observed_status": row["expected_status"],
            "pass": True,
        })
    return {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "status": "complete",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_used": False,
        "real_vehicle_used": False,
        "attempts_per_identity_max": 1,
        "attempted_identity_count": 3,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_failure": None,
        "probe_count": 3,
        "all_three_states_pass": True,
        "navigation_authorized": True,
        "probes": probes,
    }


def _family(scene_id):
    if "-cruise-" in scene_id:
        return "CRUISE"
    if "-dynamic-" in scene_id:
        return "DYNAMIC"
    if "-static-" in scene_id:
        return "STATIC_DENSE"
    if "-corridor-" in scene_id:
        return "CORRIDOR"
    if "-maneuver-" in scene_id:
        return "MANEUVER"
    raise AssertionError(scene_id)


def _evaluation(schedule_row, family, time_s):
    dynamic_scenes = (
        "dynamic-conflict-s5124", "dynamic-conflict-s5125")
    observed = (
        "OBSERVED_CONFLICT"
        if any(value in schedule_row["scene_id"] for value in dynamic_scenes)
        else "NO_CONFLICT_IN_HORIZON"
    )
    candidate = schedule_row["profile_id"] != "fixed_reference"
    overlay = (
        {"NONE": 10, "CROSSING": 4, "HEAD_ON": 0, "FOLLOW": 0,
         "OVERTAKE_OR_YIELD": 0}
        if candidate and observed == "OBSERVED_CONFLICT"
        else {"NONE": 10, "CROSSING": 0, "HEAD_ON": 0, "FOLLOW": 0,
              "OVERTAKE_OR_YIELD": 0}
    )
    reverse = 4 if candidate and family == "MANEUVER" else 0
    return {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "split": "calibration",
        "formal_result": False,
        "runtime_ready": False,
        "scene_id": schedule_row["scene_id"],
        "family": family,
        "seed": schedule_row["seed"],
        "method": schedule_row["method"],
        "supervisor_profile_id": schedule_row["profile_id"],
        "metrics": {"common": {
            "success": True,
            "collision": False,
            "navigation_time_s": time_s,
            "minimum_clearance_m": 0.31,
        }},
        "ttc_status": observed,
        "typed_transaction_valid": True,
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "runtime_scene_labels_available": False,
        "experiment_manager_validation_manifest_access": False,
        "active_anchor_switch_count": 1 if candidate else 0,
        "context_overlay_sample_counts": overlay,
        "mechanism_join_valid_fraction": 1.0 if candidate else None,
        "mechanism_join_reason_counts": (
            {"EXACT_SEQUENCE_JOIN": 30} if candidate else {}),
        "mechanism_topology_locked_sample_count": (
            2 if candidate and family == "STATIC_DENSE" else 0),
        "mechanism_corridor_centerline_sample_count": (
            2 if candidate and family == "CORRIDOR" else 0),
        "mechanism_maneuver_reverse_sample_count": reverse,
        "mechanism_topology_switch_count": 0,
        "clearance_audit": {
            "evaluator_only_gazebo_truth_used": True,
            "runtime_policy_received_truth": False,
            "minimum_signed_scan_clearance_m": 0.31,
            "minimum_truth_box_clearance_m": 0.32,
            "contact_count": 0,
        },
    }


def _base_navigation(prereg):
    schedule = prereg["navigation_schedule"]["schedule"]
    speed = {
        "fixed_reference": 10.0,
        "r5_ttc_control_h500": 9.9,
        "r5_ttc_h450": 9.5,
        "r5_ttc_h400": 9.7,
    }
    evaluations = [
        _evaluation(row, _family(row["scene_id"]), speed[row["profile_id"]])
        for row in schedule
    ]
    episodes = [
        {
            "sequence": row["sequence"],
            "method": row["method"],
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "attempt": 1,
        }
        for row in schedule
    ]
    progress = {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "status": "complete",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "attempts_per_identity_max": 1,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_failure": None,
        "planned_navigation_episode_count": 60,
        "valid_evidence_episode_count": 60,
        "attempted_identity_count": 60,
        "interface_failure_count": 0,
        "schedule_sha256": ASSESSOR.NAVIGATION_SCHEDULE_SHA256,
        "episodes": episodes,
        "attempt_ledger": [
            {
                "sequence": row["sequence"],
                "identity": {
                    "profile_id": row["profile_id"],
                    "method": row["method"],
                    "scene_id": row["scene_id"],
                },
                "attempt": 1,
                "status": "evidence_complete",
            }
            for row in schedule
        ],
    }
    scene_families = {
        row["scene_id"]: _family(row["scene_id"]) for row in schedule
    }
    return progress, evaluations, scene_families


def _documents():
    prereg = _load(PREREG)
    progress, evaluations, families = _base_navigation(prereg)
    return {
        "prereg": prereg,
        "contract": _load(CONTRACT),
        "bank": _load(BANK),
        "readiness": _base_readiness(prereg),
        "component": _base_component(prereg),
        "progress": progress,
        "evaluations": evaluations,
        "families": families,
    }


def _assess(documents):
    return ASSESSOR.assess_documents(
        documents["prereg"], documents["contract"], documents["bank"],
        documents["readiness"], documents.get("component"),
        documents.get("progress"), documents.get("evaluations"),
        documents.get("families"))


def test_current_frozen_design_hashes_and_resource_closure_are_accepted():
    frozen = ASSESSOR.verify_frozen_inputs(WORKSPACE, PREREG, AUDIT)
    assert frozen["preregistration"]["stage"] == "V2-04G-R5"
    assert frozen["candidate_bank"]["candidates"][0]["winner_eligible"] is False


def test_complete_69_ranks_only_eligible_candidates_and_never_authorizes_freeze():
    report = _assess(_documents())
    assert report["evidence_complete"] is True
    assert report["evidence_unit_counts"]["total"] == 69
    assert report["ranking_performed"] is True
    assert report["qualified_candidate_ids"] == [
        "r5_ttc_h450", "r5_ttc_h400"]
    assert report["reported_candidate_id"] == "r5_ttc_h450"
    assert report["winner_candidate_id"] is None
    assert report["candidate_summaries"][
        "r5_ttc_control_h500"]["all_hard_gates_pass"] is True
    assert report["candidate_summaries"][
        "r5_ttc_control_h500"]["selection_eligible"] is False
    assert all(value is False for value in report["decision"].values())


def test_legal_readiness_terminal_stop_reports_no_ranking_or_authorization():
    documents = _documents()
    readiness = documents["readiness"]
    readiness["reports"] = readiness["reports"][:1]
    readiness["reports"][0]["status"] = "terminal_failure"
    readiness["reports"][0]["all_hard_gates_pass"] = False
    readiness["reports"][0]["observed_ttc_status"] = "TRACKER_INVALID"
    readiness.update({
        "status": "terminal_failed",
        "executed_probe_count": 1,
        "attempted_identity_count": 1,
        "valid_probe_count": 0,
        "all_probe_hard_gates_pass": False,
        "ttc_coverage_pass": False,
        "ttc_component_authorized": False,
        "terminal_failure": {
            "identity": readiness["reports"][0]["identity"],
            "sequence": 1,
            "reason": "readiness TTC status mismatch",
        },
    })
    documents["component"] = None
    documents["progress"] = None
    documents["evaluations"] = None
    report = _assess(documents)
    assert report["status"] == "calibration_terminally_stopped"
    assert report["terminal_stop"]["phase"] == "readiness"
    assert report["ranking_performed"] is False
    assert report["winner_candidate_id"] is None
    assert all(value is False for value in report["decision"].values())


def test_missing_navigation_identity_without_terminal_marker_fails_closed():
    documents = _documents()
    documents["progress"]["episodes"].pop()
    documents["evaluations"].pop()
    with pytest.raises(ASSESSOR.R5AssessmentError):
        _assess(documents)


def test_legal_navigation_terminal_prefix_is_preserved_without_ranking():
    documents = _documents()
    schedule = documents["prereg"]["navigation_schedule"]["schedule"]
    completed = 15
    failed = schedule[completed]
    documents["progress"]["episodes"] = documents["progress"]["episodes"][:completed]
    documents["evaluations"] = documents["evaluations"][:completed]
    documents["progress"]["attempt_ledger"] = (
        documents["progress"]["attempt_ledger"][:completed] + [{
            "sequence": failed["sequence"],
            "identity": {
                "profile_id": failed["profile_id"],
                "method": failed["method"],
                "scene_id": failed["scene_id"],
            },
            "attempt": 1,
            "status": "terminal_failure",
        }])
    documents["progress"].update({
        "status": "terminal_failed",
        "valid_evidence_episode_count": completed,
        "attempted_identity_count": completed + 1,
        "interface_failure_count": 1,
        "terminal_failure": {
            "sequence": failed["sequence"],
            "identity": {
                "profile_id": failed["profile_id"],
                "method": failed["method"],
                "scene_id": failed["scene_id"],
            },
            "reason": "episode runner terminal interface failure",
        },
    })
    report = _assess(documents)
    assert report["status"] == "calibration_terminally_stopped"
    assert report["terminal_stop"]["phase"] == "navigation"
    assert report["evidence_unit_counts"]["total"] == 25
    assert report["valid_evidence_counts"]["total"] == 24
    assert report["ranking_performed"] is False
    assert report["qualified_candidate_ids"] == []
    assert all(value is False for value in report["decision"].values())


def test_control_can_never_enter_ranked_candidate_list():
    documents = _documents()
    for row in documents["evaluations"]:
        if row["supervisor_profile_id"] == "r5_ttc_control_h500":
            row["metrics"]["common"]["navigation_time_s"] = 1.0
            row["clearance_audit"]["minimum_truth_box_clearance_m"] = 1.0
    report = _assess(documents)
    assert "r5_ttc_control_h500" not in report["qualified_candidate_ids"]
    assert "r5_ttc_control_h500" not in report["ranked_qualified_candidate_ids"]


def test_missing_dynamic_overlay_disqualifies_only_affected_candidate():
    documents = _documents()
    for row in documents["evaluations"]:
        if (
            row["supervisor_profile_id"] == "r5_ttc_h450"
            and row["family"] == "DYNAMIC"
        ):
            row["context_overlay_sample_counts"] = {
                "NONE": 10, "CROSSING": 0, "HEAD_ON": 0, "FOLLOW": 0,
                "OVERTAKE_OR_YIELD": 0}
    report = _assess(documents)
    assert report["candidate_summaries"]["r5_ttc_h450"][
        "hard_gates"]["dynamic_overlay_activation"] is False
    assert report["qualified_candidate_ids"] == ["r5_ttc_h400"]


def test_fixed_ttc_failure_invalidates_all_candidate_selection():
    documents = _documents()
    fixed_dynamic = [
        row for row in documents["evaluations"]
        if row["supervisor_profile_id"] == "fixed_reference"
        and row["family"] == "DYNAMIC"
    ]
    fixed_dynamic[0]["ttc_status"] = "NO_CONFLICT_IN_HORIZON"
    report = _assess(documents)
    assert report["fixed_reference"]["validity_gate_pass"] is False
    assert report["qualified_candidate_ids"] == []
    assert report["status"] == "calibration_complete_no_candidate"
    assert report["winner_candidate_id"] is None
    assert all(value is False for value in report["decision"].values())


def test_retry_or_resume_metadata_is_rejected():
    for field, value in (("retry_count", 1), ("resume_used", True)):
        documents = _documents()
        documents["component"][field] = value
        with pytest.raises(ASSESSOR.R5AssessmentError):
            _assess(documents)
