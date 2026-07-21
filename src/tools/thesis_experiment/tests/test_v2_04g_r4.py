import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r4_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r4_full_calibration_contract.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r4_resources_and_frozen_repair_boundary_are_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R4"
    assert prereg["split"] == "calibration"
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["real_vehicle_use_forbidden"] is True
    for group in ("resources", "frozen_repair_boundary"):
        for resource in prereg[group].values():
            assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_r4_seed_firewall_budget_and_execution_order_are_fixed():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    readiness = set(firewall["readiness_probe_only_seeds"])
    navigation = set(firewall["navigation_calibration_seeds"])
    forbidden = set(firewall["all_prior_v2_04g_calibration_and_probe_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert readiness == set(range(5051, 5057))
    assert navigation == set(range(5061, 5076))
    assert readiness.isdisjoint(navigation | forbidden | validation | held_out)
    assert navigation.isdisjoint(forbidden | validation | held_out)
    assert prereg["budget"] == {
        "activation_readiness_probe_count": 6,
        "attempts_per_readiness_probe_max": 1,
        "ttc_component_probe_count": 3,
        "fixed_reference_episode_count": 15,
        "candidate_count": 3,
        "episode_count_per_candidate": 15,
        "planned_navigation_episode_count": 60,
        "attempts_per_navigation_episode_max": 2,
        "total_evidence_unit_budget": 69,
        "budget_expansion_forbidden": True,
    }


def test_r4_schedule_is_fixed_then_three_candidates_with_fifteen_each():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    batch = _module(
        "v2_04g_r4_batch_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_calibration_batch.py",
    )
    instances = batch._BASE._load_instances(
        WORKSPACE / "artifacts/v2/calibration/v2_04g_r4/compiled_scenes"
    )
    candidate_path = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._R2.materialize_candidates(candidate_path, directory)
        rows = batch._R3.build_schedule(prereg, instances, runtime)
    assert len(rows) == 60
    assert [row["sequence"] for row in rows] == list(range(1, 61))
    assert {row["profile_id"] for row in rows[:15]} == {"fixed_reference"}
    for offset, candidate in enumerate(prereg["candidate_ids"], start=1):
        block = rows[offset * 15:(offset + 1) * 15]
        assert len(block) == 15
        assert {row["profile_id"] for row in block} == {candidate}


def test_r4_readiness_summary_fails_closed_on_atomic_input_faults():
    batch = _module(
        "v2_04g_r4_readiness_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r4_activation_probe_batch.py",
    )
    base = {
        "profile_id": "r2_target_balanced", "repeat": 1, "seed": 5051,
        "status": "pass", "all_hard_gates_pass": True,
        "fault_taxonomy_counts": {}, "fault_reason_counts": {},
        "world_model_sequence_mismatch_count": 0,
        "world_model_input_join_fault_count": 0,
        "_report_path": "report.yaml", "_report_sha256": "0" * 64,
    }
    clean = batch._summary(PREREG, [base], [base], "complete", None)
    assert clean["all_probe_hard_gates_pass"] is True
    assert clean["navigation_authorized"] is True
    failed = dict(base, world_model_input_join_fault_count=1)
    result = batch._summary(PREREG, [failed], [failed], "complete", None)
    assert result["atomic_world_model_input_alignment_pass"] is False
    assert result["all_probe_hard_gates_pass"] is False
    assert result["ttc_probe_authorized"] is False
    assert result["navigation_authorized"] is False


def test_r4_contract_requires_success_safety_efficiency_and_maneuver_gates():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    gates = contract["hard_gates"]
    assert gates["fixed_reference_validity"] == {
        "success_count_min": 14, "collision_count_max": 0,
    }
    assert gates["candidate_safety"]["success_count_not_below_fixed"] is True
    assert gates["candidate_safety"]["minimum_clearance_m_min_per_successful_episode"] == 0.25
    assert gates["efficiency_vs_fixed"]["total_navigation_time_ratio_max"] == 1.05
    assert gates["priority_family_efficiency"]["families"] == [
        "STATIC_DENSE", "CORRIDOR", "MANEUVER"
    ]
    assert gates["mechanism_activation"]["maneuver_reverse_episode_count_min"] == 2
    assert contract["winner_freeze"]["runtime_ready_after_freeze"] is False


def test_r4_freezer_refuses_to_write_without_an_all_gate_winner():
    script = WORKSPACE / "src/tools/thesis_experiment/scripts/freeze_v2_04g_r4_winner.py"
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        assessment = root / "assessment.yaml"
        assessment.write_text(yaml.safe_dump({
            "stage": "V2-04G-R4", "winner_candidate_id": None,
            "candidate_summaries": {},
        }), encoding="utf-8")
        output_prefix = root / "winner"
        report = root / "freeze_report.yaml"
        result = subprocess.run([
            sys.executable, str(script), "--preregistration", str(PREREG),
            "--assessment", str(assessment), "--candidate-bank",
            str(WORKSPACE / prereg["resources"]["candidate_bank"]["path"]),
            "--output-prefix", str(output_prefix), "--report", str(report),
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        assert result.returncode != 0
        assert not report.exists()
        assert not list(root.glob("winner_*.yaml"))


def test_r4_runtime_sources_do_not_read_scene_labels_or_allow_training():
    sources = [
        WORKSPACE / "src/application/teb_mode_manager/scripts/rule_context_supervisor_node.py",
        WORKSPACE / "src/tools/thesis_experiment/scripts/v2_04g_r4_mechanism_episode.py",
        WORKSPACE / "src/tools/thesis_experiment/scripts/v2_04g_r4_calibration_batch.py",
    ]
    for path in sources:
        source = path.read_text(encoding="utf-8")
        assert "/scene_label" not in source
        assert "SAC" not in source
