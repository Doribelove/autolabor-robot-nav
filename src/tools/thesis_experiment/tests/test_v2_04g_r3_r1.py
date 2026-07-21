import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r3_r1_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r3_r1_world_model_input_join_contract.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r3_r1_resources_and_r3_failure_boundary_are_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R3-R1"
    assert prereg["readiness_only"] is True
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["navigation_allowed_in_this_stage"] is False
    assert prereg["ttc_allowed_in_this_stage"] is False
    for group in ("resources", "frozen_readiness_boundary"):
        for resource in prereg[group].values():
            assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_r3_r1_is_one_factor_and_keeps_all_required_dependencies_frozen():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["single_changed_factor"] == {
        "name": "atomic_bounded_geometry_tracks_health_join",
        "r1_transaction_join_changed": False,
        "r2_candidate_values_changed": False,
        "typed_transaction_runtime_changed": False,
        "readiness_taxonomy_changed": False,
        "supervisor_thresholds_changed": False,
        "evaluator_changed": False,
    }
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    join = contract["atomic_world_model_input_join"]
    assert join["streams"] == ["geometry", "tracks", "health"]
    assert join["maximum_entries_per_stream"] == 32
    assert join["maximum_arrival_age_s"] == 1.0
    assert join["maximum_sequence_lag"] == 2
    assert join["maximum_timestamp_spread_s"] == 0.05
    assert join["cross_sequence_synthesis_allowed"] is False


def test_r3_r1_seed_firewall_and_budget_are_fresh_and_readiness_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    seeds = set(firewall["readiness_probe_seeds"])
    prior = set(firewall["all_prior_v2_04g_calibration_and_probe_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert seeds == set(range(5041, 5047))
    assert seeds.isdisjoint(prior | validation | held_out)
    assert [row["seed"] for row in prereg[
        "activation_readiness_probe"]["schedule"]] == list(range(5041, 5047))
    assert prereg["budget"] == {
        "planned_probe_count": 6,
        "attempts_per_probe_max": 1,
        "planned_navigation_episode_count": 0,
        "ttc_component_probe_count": 0,
        "budget_expansion_forbidden": True,
    }


def test_r3_r1_listener_retains_taxonomy_and_adds_zero_atomic_fault_gate():
    listener = _module(
        "v2_04g_r3_r1_listener_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r3_r1_activation_probe_listener.py",
    )
    clean = {"fault_samples": [], "hard_gates": {"frozen_taxonomy": True}}
    assert listener.add_atomic_input_gate(clean)["all_hard_gates_pass"] is True
    failed = {
        "fault_samples": [{"latest_context": {
            "reason": "world_model_sequence_mismatch"}}],
        "hard_gates": {"frozen_taxonomy": True},
    }
    result = listener.add_atomic_input_gate(failed)
    assert result["world_model_sequence_mismatch_count"] == 1
    assert result["all_hard_gates_pass"] is False
    assert listener._FROZEN.EXPECTED_CONTEXT_HOLD_REASON == (
        "invalid_or_faulted_context_hold_previous_executed")


def test_r3_r1_supervisor_no_longer_combines_independent_latest_messages():
    source = (WORKSPACE /
        "src/application/teb_mode_manager/scripts/rule_context_supervisor_node.py"
    ).read_text(encoding="utf-8")
    assert "BoundedWorldModelInputJoin" in source
    assert 'self._add_input("geometry", message)' in source
    assert 'self._add_input("tracks", message)' in source
    assert 'self._add_input("health", message)' in source
    assert "self.geometry = message" not in source
    assert "self.tracks = message" not in source
    assert "self.health = message" not in source
    assert 'world_model_sequence_mismatch' not in source


def test_r3_r1_batch_is_fail_closed_and_has_no_navigation_or_ttc_path():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    batch = _module(
        "v2_04g_r3_r1_batch_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r3_r1_activation_probe_batch.py",
    )
    batch._R3.verify_resources(prereg)
    candidate_path = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._R2.materialize_candidates(candidate_path, directory)
        assert set(runtime) == {
            "r2_control_g2", "r2_target_balanced", "r2_target_aggressive"
        }
    source = (WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r3_r1_activation_probe_batch.py"
    ).read_text(encoding="utf-8")
    assert "attempts_per_probe" not in source
    assert "move_base_simple/goal" not in source
    assert "probe_v2_04g" not in source
    assert prereg["pass_boundary"]["current_stage_navigation_authorized"] is False
    assert prereg["pass_boundary"]["current_stage_ttc_authorized"] is False
