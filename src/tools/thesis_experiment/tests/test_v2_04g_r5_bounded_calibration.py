import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
SCRIPT = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r5_bounded_calibration.py"
)
PREREGISTRATION = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/"
    "v2_04g_r5_preregistration.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location(
        "v2_04g_r5_bounded_calibration_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _runtime_paths(root):
    return {
        candidate: {
            "supervisor": Path(root) / candidate / "supervisor.yaml",
            "anchor_bank": Path(root) / candidate / "anchor_bank.yaml",
            "mechanism": Path(root) / candidate / "mechanism.yaml",
        }
        for candidate in (
            "r5_ttc_control_h500",
            "r5_ttc_h450",
            "r5_ttc_h400",
        )
    }


def _exact_schedule(module, runtime_root):
    prereg = yaml.safe_load(PREREGISTRATION.read_text(encoding="utf-8"))
    instances = module._load_instances(WORKSPACE, prereg)
    rows, digest = module.build_schedule(
        prereg, instances, _runtime_paths(runtime_root)
    )
    return prereg, instances, rows, digest


def _prerequisites(module, root):
    readiness = {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "status": "complete",
        "simulation_only": True,
        "runtime_ready": False,
        "planned_probe_count": 6,
        "executed_probe_count": 6,
        "valid_probe_count": 6,
        "attempts_per_identity_max": 1,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_failure": None,
        "all_probe_hard_gates_pass": True,
        "ttc_coverage_pass": True,
        "ttc_component_authorized": True,
        "navigation_authorized": False,
        "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {
            "sha256": module.EXPECTED_PREREGISTRATION_SHA256
        },
    }
    readiness_path = Path(root) / "readiness/v2_04g_r5_readiness_summary.yaml"
    _write_yaml(readiness_path, readiness)
    component = {
        "schema_version": "2.0",
        "stage": "V2-04G-R5",
        "status": "complete",
        "simulation_only": True,
        "runtime_ready": False,
        "probe_count": 3,
        "attempts_per_identity_max": 1,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_failure": None,
        "expected_status_order": [
            "OBSERVED_CONFLICT",
            "NO_CONFLICT_IN_HORIZON",
            "TRACKER_INVALID",
        ],
        "observed_status_order": [
            "OBSERVED_CONFLICT",
            "NO_CONFLICT_IN_HORIZON",
            "TRACKER_INVALID",
        ],
        "all_three_states_pass": True,
        "navigation_authorized": True,
        "training_used": False,
        "real_vehicle_used": False,
        "preregistration": {
            "sha256": module.EXPECTED_PREREGISTRATION_SHA256
        },
        "readiness_summary": {
            "sha256": module._sha256(readiness_path)
        },
    }
    component_path = Path(root) / "v2_04g_r5_ttc_three_state_probe.yaml"
    _write_yaml(component_path, component)
    return readiness_path, component_path


def test_exact_navigation_schedule_is_frozen_60_identity_product(tmp_path):
    module = _module()
    prereg, _, rows, digest = _exact_schedule(module, tmp_path / "runtime")
    assert digest == module.EXPECTED_SCHEDULE_SHA256
    assert len(rows) == 60
    assert len({
        (row["profile_id"], row["method"], row["scene_id"])
        for row in rows
    }) == 60
    assert [row["profile_id"] for row in rows] == (
        ["fixed_reference"] * 15
        + ["r5_ttc_control_h500"] * 15
        + ["r5_ttc_h450"] * 15
        + ["r5_ttc_h400"] * 15
    )
    assert [row["seed"] for row in rows[:15]] == list(range(5121, 5136))
    assert {row["seed"] for row in rows}.isdisjoint(
        prereg["seed_firewall"]["reserved_future_held_out_seeds"]
    )
    assert {row["seed"] for row in rows}.isdisjoint(
        prereg["seed_firewall"]["readiness_compile_support_only_seeds"]
    )


def test_prerequisites_require_exact_readiness_and_component_chain(tmp_path):
    module = _module()
    readiness, component = _prerequisites(module, tmp_path)
    refs = module.verify_prerequisites(readiness, component)
    assert refs["readiness_summary"]["sha256"] == module._sha256(readiness)
    assert refs["ttc_component_probe"]["sha256"] == module._sha256(component)

    value = yaml.safe_load(readiness.read_text(encoding="utf-8"))
    value["resume_forbidden"] = False
    _write_yaml(readiness, value)
    with pytest.raises(RuntimeError, match="readiness prerequisite failed"):
        module.verify_prerequisites(readiness, component)


def test_dry_run_prints_exact_schedule_without_writing_artifacts(
        tmp_path, capsys):
    module = _module()
    _, _, rows, _ = _exact_schedule(module, tmp_path / "runtime")
    forbidden_artifact_root = tmp_path / "artifacts"
    module.print_dry_run(rows)
    output = capsys.readouterr().out
    assert "60 navigation identities" in output
    assert output.count("attempt=1") == 60
    assert "001 fixed_reference v2-04g-r5-cruise-s5121" in output
    assert "060 r5_ttc_h400 v2-04g-r5-maneuver-s5135" in output
    assert not forbidden_artifact_root.exists()


def test_successful_executor_records_each_identity_once_and_forbids_resume(
        tmp_path):
    module = _module()
    _, _, rows, digest = _exact_schedule(module, tmp_path / "runtime")
    output_root = tmp_path / "artifacts/v2/calibration/v2_04g_r5"
    progress_path = output_root / "v2_04g_r5_progress.yaml"
    calls = []

    def fake_episode(row, output_dir):
        calls.append(row["sequence"])
        output_dir.mkdir(parents=True)
        trace = output_dir / "trace.csv"
        trace.write_text("frozen-test-trace\n", encoding="utf-8")
        evaluation = {
            "metrics": {
                "common": {"success": True, "collision": False}
            },
            "raw_trace_sha256": module._sha256(trace),
        }
        evaluation_path = output_dir / "evaluation.yaml"
        _write_yaml(evaluation_path, evaluation)
        return evaluation_path, evaluation

    progress = module.execute_navigation_schedule(
        schedule=rows,
        output_root=output_root,
        progress_path=progress_path,
        preregistration_path=PREREGISTRATION,
        schedule_hash=digest,
        prerequisites={
            "readiness_summary": {"path": "test", "sha256": "a" * 64},
            "ttc_component_probe": {"path": "test", "sha256": "b" * 64},
        },
        episode_callable=fake_episode,
        evidence_validator=lambda row, path, value, output: value,
    )
    assert calls == list(range(1, 61))
    assert progress["status"] == "complete"
    assert progress["resume_forbidden"] is True
    assert progress["retry_count"] == 0
    assert progress["resume_used"] is False
    assert progress["attempted_identity_count"] == 60
    assert progress["valid_evidence_episode_count"] == 60
    assert progress["terminal_failure"] is None
    assert len(progress["attempt_ledger"]) == 60
    assert {row["attempt"] for row in progress["attempt_ledger"]} == {1}
    assert {row["status"] for row in progress["attempt_ledger"]} == {
        "evidence_complete"
    }
    assert {row["attempt"] for row in progress["episodes"]} == {1}
    with pytest.raises(module.StageAlreadyStarted, match="resume is forbidden"):
        module.execute_navigation_schedule(
            rows, output_root, progress_path, PREREGISTRATION, digest, {},
            fake_episode, lambda row, path, value, output: value,
        )
    assert calls == list(range(1, 61))


def test_terminal_failure_is_preserved_and_never_retried_or_resumed(tmp_path):
    module = _module()
    _, _, rows, digest = _exact_schedule(module, tmp_path / "runtime")
    output_root = tmp_path / "artifacts/v2/calibration/v2_04g_r5"
    progress_path = output_root / "v2_04g_r5_progress.yaml"
    calls = []

    def failed_episode(row, output_dir):
        calls.append(row["sequence"])
        output_dir.mkdir(parents=True)
        (output_dir / "launch.log").write_text(
            "preserved failure\n", encoding="utf-8"
        )
        raise RuntimeError("synthetic terminal runner failure")

    with pytest.raises(module.TerminalEvidenceFailure):
        module.execute_navigation_schedule(
            schedule=rows,
            output_root=output_root,
            progress_path=progress_path,
            preregistration_path=PREREGISTRATION,
            schedule_hash=digest,
            prerequisites={
                "readiness_summary": {"path": "test", "sha256": "a" * 64},
                "ttc_component_probe": {"path": "test", "sha256": "b" * 64},
            },
            episode_callable=failed_episode,
            evidence_validator=lambda row, path, value, output: value,
        )
    progress = yaml.safe_load(progress_path.read_text(encoding="utf-8"))
    assert calls == [1]
    assert progress["status"] == "terminal_failure"
    assert progress["resume_forbidden"] is True
    assert progress["retry_count"] == 0
    assert progress["attempted_identity_count"] == 1
    assert progress["valid_evidence_episode_count"] == 0
    assert progress["interface_failure_count"] == 1
    assert len(progress["attempt_ledger"]) == 1
    assert progress["attempt_ledger"][0]["attempt"] == 1
    assert progress["attempt_ledger"][0]["status"] == "terminal_failure"
    assert progress["terminal_failure"]["sequence"] == 1
    assert (
        output_root
        / "episodes/ep_001__fixed_reference__v2-04g-r5-cruise-s5121/"
        "launch.log"
    ).read_text(encoding="utf-8") == "preserved failure\n"

    with pytest.raises(module.StageAlreadyStarted, match="resume is forbidden"):
        module.execute_navigation_schedule(
            rows, output_root, progress_path, PREREGISTRATION, digest, {},
            failed_episode, lambda row, path, value, output: value,
        )
    assert calls == [1]
