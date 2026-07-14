import hashlib
from pathlib import Path

import yaml

from thesis_experiment.v2_scene import load_v2_scene_manifest


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04e_r1_preregistration.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_v2_04e_is_calibration_only_and_does_not_reuse_v2_04d_seeds():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04E"
    assert prereg["split"] == "calibration"
    assert prereg["runtime_ready"] is False
    assert prereg["training_started"] is False
    assert prereg["supersedes"]["accepted_evidence_episode_count"] == 0
    assert prereg["budget"]["planned_navigation_episode_count"] == 20
    assert set(prereg["data_firewall"]["allowed_seed_set"]).isdisjoint(
        prereg["data_firewall"]["forbidden_v2_04d_validation_seed_set"]
    )
    assert set(prereg["data_firewall"]["allowed_seed_set"]).isdisjoint(
        prereg["data_firewall"]["held_out_v2_04f_seed_set_reserved_and_unopened"]
    )
    historical_sources = {
        "repaired_supervisor_source", "episode_runner", "batch_runner"
    }
    for name, resource in prereg["resources"].items():
        if name in historical_sources:
            assert len(resource["sha256"]) == 64
            continue
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_v2_04e_candidates_enable_all_three_structural_repairs():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    candidate_path = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    bank = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
    assert len(bank["candidates"]) == 4
    for candidate in bank["candidates"]:
        geometry = candidate["geometry"]
        assert geometry["static_dense"]["persistence_density_full"] > 0.0
        assert geometry["maneuver"]["reverse_heading_error_min_rad"] > 0.0
        assert candidate["transition"]["switch_score_margin"] > 0.0
        assert all("exit_confidence" in values for values in geometry.values())


def test_v2_04e_scene_manifest_is_five_family_calibration_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    manifest = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["calibration_scenes"]["path"], WORKSPACE
    )
    assert len(manifest["scenes"]) == 5
    assert {scene["split"] for scene in manifest["scenes"]} == {"calibration"}
    assert {scene["seed"] for scene in manifest["scenes"]} == {4701, 4702, 4703, 4704, 4705}
    assert {scene["family"] for scene in manifest["scenes"]} == {
        "CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER"
    }


def test_v2_04e_runtime_launch_has_no_scene_or_manifest_policy_input():
    launch = (WORKSPACE /
        "src/simulation/m2_gazebo/launch/m2_v2_04e_04f_supervisor_experiment.launch"
    ).read_text(encoding="utf-8")
    supervisor = (WORKSPACE /
        "src/application/teb_mode_manager/src/teb_mode_manager/rule_supervisor.py"
    ).read_text(encoding="utf-8")
    assert "rule_supervisor_config" in launch
    assert "scene_family" not in launch
    assert "manifest" not in supervisor.lower()


def test_v2_04e2_uses_only_new_calibration_seeds_and_fixed_candidate_bank():
    path = WORKSPACE / "experiments/manifests/v2/calibration/v2_04e2_preregistration.yaml"
    prereg = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04E2"
    assert prereg["budget"]["planned_navigation_episode_count"] == 15
    allowed = set(prereg["data_firewall"]["allowed_seed_set"])
    assert allowed == {4711, 4712, 4713, 4714, 4715}
    assert allowed.isdisjoint(prereg["data_firewall"]["forbidden_v2_04d_validation_seed_set"])
    assert allowed.isdisjoint(prereg["data_firewall"]["forbidden_v2_04e_calibration_seed_set"])
    assert allowed.isdisjoint(prereg["data_firewall"]["held_out_v2_04f_seed_set_reserved_and_unopened"])
    for name, resource in prereg["resources"].items():
        if name in {"episode_runner", "batch_runner"}:
            assert len(resource["sha256"]) == 64
            continue
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_v2_04e2_candidates_add_pocket_evidence_and_exit_confirmation():
    path = WORKSPACE / "experiments/manifests/v2/calibration/v2_04e2_supervisor_candidates.yaml"
    bank = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert len(bank["candidates"]) == 3
    for candidate in bank["candidates"]:
        maneuver = candidate["geometry"]["maneuver"]
        assert maneuver["pocket_front_clearance_max_m"] > maneuver[
            "pocket_front_clearance_full_m"
        ]
        assert maneuver["pocket_side_clearance_max_m"] > maneuver[
            "pocket_side_clearance_full_m"
        ]
        assert candidate["transition"]["exit_confirmation_s"] > candidate[
            "transition"
        ]["enter_confirmation_s"]


def test_v2_04e3_changes_measurement_window_but_not_candidate_parameters():
    prereg_path = (
        WORKSPACE / "experiments/manifests/v2/calibration/v2_04e3_preregistration.yaml"
    )
    prereg = yaml.safe_load(prereg_path.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04E3"
    assert prereg["budget"]["planned_navigation_episode_count"] == 5
    for name, resource in prereg["resources"].items():
        if name in {"episode_runner", "batch_runner"}:
            assert len(resource["sha256"]) == 64
            continue
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    e2 = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e2_supervisor_candidates.yaml"
    ).read_text(encoding="utf-8"))
    e3 = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e3_supervisor_candidate.yaml"
    ).read_text(encoding="utf-8"))
    source = next(row for row in e2["candidates"]
                  if row["candidate_id"] == "pocket_g_wide_hold")
    confirmation = e3["candidates"][0]
    assert confirmation["geometry"] == source["geometry"]
    assert confirmation["transition"] == source["transition"]


def test_v2_04e4_is_exit_confirmation_single_factor_only():
    prereg = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e4_preregistration.yaml"
    ).read_text(encoding="utf-8"))
    assert prereg["budget"]["planned_navigation_episode_count"] == 5
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    e3 = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e3_supervisor_candidate.yaml"
    ).read_text(encoding="utf-8"))["candidates"][0]
    e4 = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e4_supervisor_candidate.yaml"
    ).read_text(encoding="utf-8"))["candidates"][0]
    assert e4["geometry"] == e3["geometry"]
    before = dict(e3["transition"])
    after = dict(e4["transition"])
    assert before.pop("exit_confirmation_s") == 4.5
    assert after.pop("exit_confirmation_s") == 10.0
    assert before == after


def test_v2_04f_is_fresh_held_out_three_method_pairing_after_freeze():
    prereg = yaml.safe_load((WORKSPACE /
        "experiments/manifests/v2/validation/v2_04f_preregistration.yaml"
    ).read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04F"
    assert prereg["method_ids"] == [
        "fixed_teb", "balanced_anchor", "rule_multi_anchor"
    ]
    assert prereg["budget"]["planned_navigation_episode_count"] == 30
    allowed = set(prereg["data_firewall"]["allowed_seed_set"])
    assert allowed == set(range(4801, 4811))
    assert allowed.isdisjoint(prereg["data_firewall"]["forbidden_v2_04d_validation_seed_set"])
    assert allowed.isdisjoint(prereg["data_firewall"]["forbidden_all_calibration_seed_set"])
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    scenes = load_v2_scene_manifest(
        WORKSPACE / prereg["resources"]["validation_scenes"]["path"], WORKSPACE
    )
    assert len(scenes["scenes"]) == 10
    assert {scene["split"] for scene in scenes["scenes"]} == {"validation"}


def test_v2_04f_completed_result_proves_success_but_blocks_performance_claim():
    path = WORKSPACE / "artifacts/v2/validation/v2_04f/v2_04f_paired_assessment.yaml"
    report = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert report["evidence"]["valid_paired_episode_count"] == 30
    assert report["decision"]["success_non_degradation_proven"] is True
    assert report["stage_1_hard_gates"]["all_stage_1_hard_gates_pass"] is False
    assert report["mechanism_checks"]["cruise_not_absorbed_by_static_pass"] is True
    assert report["mechanism_checks"]["maneuver_activation_pass"] is True
    assert report["mechanism_checks"]["chatter_pass"] is False
    assert report["decision"]["performance_effectiveness_proven"] is False
    assert report["decision"]["enter_v2_05_authorized"] is False
    assert report["decision"]["validation_may_modify_frozen_supervisor"] is False
    assert report["decision"]["sac_training_authorized"] is False
    assert report["decision"]["real_vehicle_authorized"] is False
