import importlib.util
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[4]
SCRIPTS = WORKSPACE / "src/tools/thesis_experiment/scripts"


def _module(filename, name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_readiness_schedule_is_exact_once_only_and_held_out_free():
    module = _module(
        "v2_04g_r5_readiness_batch.py", "r5_readiness_schedule_test"
    )
    prereg, _, _ = module._GUARD.verify_frozen_start()
    gate, schedule, instances = module._validate_design(prereg)
    assert gate["planned_probe_count"] == 6
    assert len(schedule) == 6
    assert [row["sequence"] for row in schedule] == list(range(1, 7))
    assert {row["attempt_limit"] for row in schedule} == {1}
    assert [row["seed"] for row in schedule] == [5111, 5112, 5113] * 2
    assert set(row["scene_id"] for row in schedule).issubset(instances)
    assert set(row["seed"] for row in schedule).isdisjoint(
        prereg["seed_firewall"]["reserved_future_held_out_seeds"]
    )
    assert set(row["seed"] for row in schedule).isdisjoint(
        prereg["seed_firewall"]["readiness_compile_support_only_seeds"]
    )


def test_readiness_complete_schema_matches_navigation_and_assessment(monkeypatch):
    module = _module(
        "v2_04g_r5_readiness_batch.py", "r5_readiness_schema_test"
    )
    prereg, _, _ = module._GUARD.verify_frozen_start()
    gate, schedule, _ = module._validate_design(prereg)
    summary = module._base_summary(prereg, schedule)
    summary["attempt_ledger"] = [
        {"identity": row["identity"], "status": "evidence_complete"}
        for row in schedule
    ]
    summary["reports"] = [{} for _ in schedule]

    def aggregate(value):
        value["atomic_world_model_input_alignment_pass"] = True
        value["aggregate_fault_taxonomy_counts"] = {}
        value["observed_status_counts_by_profile"] = {
            "r5_ttc_h450": {
                "OBSERVED_CONFLICT": 2,
                "NO_CONFLICT_IN_HORIZON": 1,
                "TRACKER_INVALID": 0,
            },
            "r5_ttc_h400": {
                "OBSERVED_CONFLICT": 2,
                "NO_CONFLICT_IN_HORIZON": 1,
                "TRACKER_INVALID": 0,
            },
        }

    monkeypatch.setattr(module, "_refresh_aggregates", aggregate)
    monkeypatch.setattr(module, "_write_summary", lambda value: None)
    assert module._complete(summary, gate) is True
    assert summary["executed_probe_count"] == 6
    assert summary["attempts_per_identity_max"] == 1
    assert summary["retry_count"] == 0
    assert summary["resume_used"] is False
    assert summary["resume_forbidden"] is True
    assert summary["ttc_component_authorized"] is True
    assert summary["navigation_authorized"] is False


def test_component_fixture_covers_exact_three_states_without_seed():
    module = _module(
        "probe_v2_04g_r5_ttc_states.py", "r5_component_fixture_test"
    )
    prereg, _, _ = module._GUARD.verify_frozen_start()
    schedule = module._validate_schedule(prereg)
    observed = [
        module._fixture(row["expected_status"])["status"] for row in schedule
    ]
    assert observed == [
        "OBSERVED_CONFLICT",
        "NO_CONFLICT_IN_HORIZON",
        "TRACKER_INVALID",
    ]
    assert all("seed" not in row for row in schedule)
    assert {row["attempt_limit"] for row in schedule} == {1}
