import copy
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r4_r1_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r4_r1_clearance_repair_contract.yaml"
BANK = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r4_r1_clearance_candidates.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r4_r1_resources_and_r4_boundary_are_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R4-R1"
    assert prereg["split"] == "calibration"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    for group in ("resources", "frozen_r4_boundary"):
        for resource in prereg[group].values():
            assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_r4_r1_seed_firewall_and_budget_are_fresh_and_complete():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    readiness = set(firewall["readiness_probe_only_seeds"])
    navigation = set(firewall["navigation_calibration_seeds"])
    forbidden = set(firewall["all_prior_v2_04g_calibration_and_probe_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert readiness == set(range(5081, 5087))
    assert navigation == set(range(5091, 5106))
    assert readiness.isdisjoint(navigation | forbidden | validation | held_out)
    assert navigation.isdisjoint(forbidden | validation | held_out)
    assert prereg["budget"]["planned_navigation_episode_count"] == 60
    assert prereg["budget"]["total_evidence_unit_budget"] == 69
    assert prereg["budget"]["budget_expansion_forbidden"] is True


def test_r4_r1_materializer_changes_only_tied_maneuver_clearance_fields():
    materializer = _module(
        "v2_04g_r4_r1_materializer_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_candidate_materializer.py")
    bank = yaml.safe_load(BANK.read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory() as directory:
        runtime = materializer.materialize_candidates(BANK, directory)
        assert set(runtime) == {row["candidate_id"] for row in bank["candidates"]}
        loaded = {candidate: {
            kind: yaml.safe_load(Path(paths[kind]).read_text(encoding="utf-8"))
            for kind in ("supervisor", "anchor_bank", "mechanism")
        } for candidate, paths in runtime.items()}
    control = loaded["r4r1_aggressive_control_m028"]
    for row in bank["candidates"]:
        candidate = loaded[row["candidate_id"]]
        value = row["maneuver_min_obstacle_dist_m"]
        for anchor_id in ("anchor_maneuver_forward", "anchor_maneuver_reverse"):
            assert candidate["anchor_bank"]["anchors"][anchor_id]["values"][
                "min_obstacle_dist"] == value
            assert candidate["anchor_bank"]["anchors"][anchor_id]["values"][
                "inflation_dist"] == 0.52
        supervisor = copy.deepcopy(candidate["supervisor"])
        supervisor["profile_id"] = control["supervisor"]["profile_id"]
        assert supervisor == control["supervisor"]
        mechanism = copy.deepcopy(candidate["mechanism"])
        mechanism["profile_id"] = control["mechanism"]["profile_id"]
        assert mechanism == control["mechanism"]


def test_r4_r1_contract_freezes_passed_mechanisms_and_strengthens_clearance_evidence():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    factor = contract["single_changed_factor"]
    assert factor["preregistered_values_m"] == [0.28, 0.30, 0.32]
    assert factor["maneuver_inflation_dist_m"] == 0.52
    frozen = contract["frozen_passed_mechanisms"]
    assert frozen["maneuver_speed_acceleration_and_time_values_changed"] is False
    assert frozen["maneuver_reverse_selection_and_residuals_changed"] is False
    assert frozen["typed_transaction_runtime_changed"] is False
    gate = contract["hard_gates"]["maneuver_clearance_repair"]
    assert gate["minimum_signed_scan_clearance_m_per_successful_maneuver"] == 0.25
    assert gate["minimum_truth_box_clearance_m_per_successful_maneuver"] == 0.25
    assert gate["runtime_truth_access_forbidden"] is True


def test_r4_r1_schedule_is_fixed_then_three_fifteen_episode_blocks():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    batch = _module(
        "v2_04g_r4_r1_batch_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_calibration_batch.py")
    instances = batch._BASE._load_instances(
        WORKSPACE / "artifacts/v2/calibration/v2_04g_r4_r1/compiled_scenes")
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._MAT.materialize_candidates(BANK, directory)
        rows = batch._R3.build_schedule(prereg, instances, runtime)
    assert len(rows) == 60
    assert {row["profile_id"] for row in rows[:15]} == {"fixed_reference"}
    for index, candidate in enumerate(prereg["candidate_ids"], start=1):
        block = rows[index * 15:(index + 1) * 15]
        assert len(block) == 15
        assert {row["profile_id"] for row in block} == {candidate}


def test_r4_r1_control_is_not_winner_eligible_and_freeze_is_fail_closed():
    bank = yaml.safe_load(BANK.read_text(encoding="utf-8"))
    eligibility = {row["candidate_id"]: row["winner_eligible"]
                   for row in bank["candidates"]}
    assert eligibility == {
        "r4r1_aggressive_control_m028": False,
        "r4r1_clearance_m030": True,
        "r4r1_clearance_m032": True,
    }
    script = WORKSPACE / "src/tools/thesis_experiment/scripts/freeze_v2_04g_r4_r1_winner.py"
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        assessment = root / "assessment.yaml"
        assessment.write_text(yaml.safe_dump({
            "stage": "V2-04G-R4-R1",
            "winner_candidate_id": "r4r1_aggressive_control_m028",
            "candidate_summaries": {"r4r1_aggressive_control_m028": {
                "all_hard_gates_pass": True}},
            "decision": {"freeze_authorized": True},
        }), encoding="utf-8")
        report = root / "report.yaml"
        result = subprocess.run([
            sys.executable, str(script), "--preregistration", str(PREREG),
            "--assessment", str(assessment), "--candidate-bank", str(BANK),
            "--output-prefix", str(root / "winner"), "--report", str(report),
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        assert result.returncode != 0
        assert not report.exists()


def test_r4_r1_runtime_sources_do_not_read_labels_or_load_learning():
    for relative in (
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_candidate_materializer.py",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_calibration_batch.py",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_mechanism_episode.py",
    ):
        source = (WORKSPACE / relative).read_text(encoding="utf-8")
        assert "/scene_label" not in source
        assert "learned_policy_loaded: true" not in source.lower()
