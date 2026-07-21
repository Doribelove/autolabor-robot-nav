import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PREREG = WORKSPACE / "experiments/manifests/v2/calibration/v2_04g_r2_r1_preregistration.yaml"
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/v2_04g_r2_r1_readiness_taxonomy_contract.yaml"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _module(name, relative_path):
    path = WORKSPACE / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_r2_r1_resources_and_failed_r2_boundary_are_exact():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    assert prereg["stage"] == "V2-04G-R2-R1"
    assert prereg["readiness_only"] is True
    assert prereg["runtime_ready"] is False
    assert prereg["training_allowed"] is False
    assert prereg["navigation_allowed_in_this_stage"] is False
    for resource in prereg["resources"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]
    for resource in prereg["frozen_r2_failure_boundary"].values():
        assert _sha256(WORKSPACE / resource["path"]) == resource["sha256"]


def test_r2_r1_fault_taxonomy_implementation_matches_contract():
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    listener = _module(
        "r2_r1_listener_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r2_r1_activation_probe_listener.py",
    )
    taxonomy = contract["fault_taxonomy"]["classes"]
    assert taxonomy["EXPECTED_FAIL_CLOSED_CONTEXT_HOLD"]["exact_reason"] == (
        listener.EXPECTED_CONTEXT_HOLD_REASON
    )
    assert tuple(taxonomy["BACKEND_TRANSACTION_FAULT"][
        "case_insensitive_substrings"
    ]) == listener.BACKEND_FAULT_PATTERNS
    examples = {
        "": "CLEAN",
        listener.EXPECTED_CONTEXT_HOLD_REASON: "EXPECTED_FAIL_CLOSED_CONTEXT_HOLD",
        "configuration acknowledgement timed out": "BACKEND_TRANSACTION_FAULT",
        "configuration readback differs": "BACKEND_TRANSACTION_FAULT",
        "novel nonempty failure": "UNKNOWN_TRANSACTION_FAULT",
    }
    for reason, expected in examples.items():
        assert listener.classify_fault_reason(reason) == expected


def test_r2_r1_stable_window_is_consecutive_and_resets_on_instability():
    listener = _module(
        "r2_r1_window_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r2_r1_activation_probe_listener.py",
    )
    window = listener.ConsecutiveStableWindow(3)
    observed = [
        window.update(value)
        for value in (True, True, False, True, True, True)
    ]
    assert observed == [False, False, False, False, False, True]
    assert window.maximum_count == 3
    assert window.current_count == 3


def test_r2_r1_seed_firewall_and_budget_are_fresh_and_readiness_only():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    firewall = prereg["seed_firewall"]
    seeds = set(firewall["readiness_probe_seeds"])
    prior = set(firewall["all_prior_v2_04g_and_probe_seeds_forbidden"])
    validation = set(firewall["previous_validation_seeds_forbidden"])
    held_out = set(firewall["reserved_future_held_out_seeds"])
    assert seeds == set(range(4981, 4987))
    assert seeds.isdisjoint(prior | validation | held_out)
    schedule = prereg["activation_readiness_probe"]["schedule"]
    assert [row["seed"] for row in schedule] == list(range(4981, 4987))
    assert prereg["budget"] == {
        "planned_probe_count": 6,
        "attempts_per_probe_max": 1,
        "planned_navigation_episode_count": 0,
        "ttc_component_probe_count": 0,
        "budget_expansion_forbidden": True,
    }


def test_r2_r1_batch_is_fail_closed_and_materializes_frozen_candidates():
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    batch = _module(
        "r2_r1_batch_test",
        "src/tools/thesis_experiment/scripts/v2_04g_r2_r1_activation_probe_batch.py",
    )
    batch.verify_resources(prereg)
    candidate_path = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    with tempfile.TemporaryDirectory() as directory:
        runtime = batch._R2.materialize_candidates(candidate_path, directory)
        assert set(runtime) == {
            "r2_control_g2", "r2_target_balanced", "r2_target_aggressive"
        }
    source = (WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r2_r1_activation_probe_batch.py"
    ).read_text(encoding="utf-8")
    assert "attempts_per_probe" not in source
    assert "start_typed_transaction:=true" in source
    assert "move_base_simple/goal" not in source
    assert prereg["pass_boundary"]["current_stage_navigation_authorized"] is False
