import ast
import copy
import hashlib
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


MODULE_PATH = Path(__file__).resolve().parents[2] / (
    "src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_assessment.py"
)
SPECIFICATION = importlib.util.spec_from_file_location(
    "v2_04g_r6_i5_assessment_under_test", MODULE_PATH
)
assessment = importlib.util.module_from_spec(SPECIFICATION)
sys.modules[SPECIFICATION.name] = assessment
SPECIFICATION.loader.exec_module(assessment)

REAL_WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _dump(path, document):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _write(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(payload)


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _identity(row):
    return {
        "stage": assessment.STAGE,
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }


def _semantic(row):
    role = row["expected_overlay_semantics"]
    if role in {"non_none", "non_none_iff_finite_ttc"}:
        return 5, {"NONE": 0, "PASSING": 5}
    if role == "legacy_non_none_identifiability":
        return 0, {"NONE": 0, "PASSING": 5}
    return 0, {"NONE": 5}


def _attempt(root, row):
    identity = _identity(row)
    attempt_name = "{:02d}__{}__{}".format(
        row["sequence"], row["profile_id"], row["scene_id"]
    )
    attempt_root = Path(assessment.ATTEMPTS_ROOT) / attempt_name
    raw_root = attempt_root / "raw"
    scene_root = attempt_root / "work/scene_snapshot"
    journal_path = Path(assessment.JOURNALS_ROOT) / (
        "attempt_{:02d}.yaml".format(row["sequence"])
    )

    trace = ("sequence,x,y\n{},0,0\n".format(row["sequence"])).encode()
    process_log = ("sanitized process log {}\n".format(row["sequence"])).encode()
    _write(root / raw_root / "trace.csv", trace)
    _write(root / raw_root / "process.log", process_log)
    finite, overlay = _semantic(row)
    activation = {
        **identity,
        "all_hard_gates_pass": True,
        "tracker_message_count": 24,
        "context_message_count": 25,
    }
    evaluation = {
        **identity,
        "ttc_status": row["expected_ttc_status"],
        "formal_result": False,
        "runtime_ready": False,
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "runtime_scene_labels_available": False,
        "tracker_message_count": 26,
        "context_message_count": 27,
        "finite_ttc_sample_count": finite,
        "context_overlay_sample_counts": overlay,
        "raw_trace_sha256": hashlib.sha256(trace).hexdigest(),
    }
    clearance = {**identity, "minimum_clearance_m": 0.8}
    startup_sha = hashlib.sha256(
        ("startup {}".format(row["sequence"])).encode()
    ).hexdigest()
    teardown = {
        **identity,
        "restore_requested_while_backend_alive": True,
        "transaction_acknowledged": True,
        "transaction_readback_match": True,
        "independent_readback_match": True,
        "service_response_success": True,
        "startup_profile_sha256": startup_sha,
        "transaction_readback_sha256": startup_sha,
        "independent_readback_sha256": startup_sha,
    }
    _dump(root / raw_root / "activation.yaml", activation)
    _dump(root / raw_root / "evaluation.yaml", evaluation)
    _dump(root / raw_root / "clearance.yaml", clearance)
    _dump(root / raw_root / "teardown_receipt.yaml", teardown)

    resources = {
        label: {
            "path": (raw_root / filename).as_posix(),
            "sha256": _digest(root / raw_root / filename),
        }
        for label, filename in assessment.RAW_FILENAMES.items()
    }
    instance = scene_root / (
        row["scene_id"] + "__" + ("a" * 64) + ".instance.yaml"
    )
    world = scene_root / (row["scene_id"] + "__" + ("b" * 64) + ".world")
    _dump(root / instance, {"scene": {"scene_id": row["scene_id"], "seed": row["seed"]}})
    _write(root / world, b"synthetic world\n")
    instance_row = {
        "path": str(root / instance),
        "sha256": _digest(root / instance),
    }
    world_row = {"path": str(root / world), "sha256": _digest(root / world)}
    scene_snapshot = {
        "scene_id": row["scene_id"],
        "index": {"path": "synthetic/index.yaml", "sha256": "1" * 64},
        "source_instance": {"path": "synthetic/source.instance.yaml", "sha256": "2" * 64},
        "source_world": {"path": "synthetic/source.world", "sha256": "3" * 64},
        "snapshot_instance": instance_row,
        "snapshot_world": world_row,
    }
    pre_scene = {
        "scene_id": row["scene_id"],
        "verification_phase": "pre_spawn",
        "resources": {
            "snapshot_instance": instance_row,
            "snapshot_world": world_row,
        },
    }
    post_scene = copy.deepcopy(pre_scene)
    post_scene["verification_phase"] = "post_episode"
    readiness = {
        "identity": identity,
        "minimum_message_count": 20,
        "direct_counts": {
            "activation_tracker_message_count": 24,
            "activation_context_message_count": 25,
            "evaluation_tracker_message_count": 26,
            "evaluation_context_message_count": 27,
        },
        "pass": True,
    }
    verified_teardown = {
        "identity": identity,
        "status": "pass",
        "journal_state_path": str(root / journal_path),
        "startup_profile_sha256": startup_sha,
        "two_phase_restore_verified": True,
        "launch_stop_allowed": True,
        "post_episode_scene_verification": post_scene,
    }
    evidence_binding = {
        "identity": identity,
        "raw_evidence_bound": True,
        "readiness_direct_counts": readiness,
        "teardown_restore": verified_teardown,
        "resources": resources,
    }
    journal = {
        "schema_version": "2.0",
        "stage": assessment.STAGE,
        "identity": identity,
        "status": "evidence_complete",
        "lifecycle_phase": "evidence_complete",
        "resume_forbidden": True,
        "active_identity": None,
        "downstream_authorized": False,
        "evidence_binding": evidence_binding,
        "startup_profile_sha256": startup_sha,
        "scene_snapshot": scene_snapshot,
        "pre_spawn_scene_verification": pre_scene,
        "post_episode_scene_verification": post_scene,
        "launch_stop_authorization": {
            "launch_stop_allowed": True,
            "identity": identity,
            "teardown_restore": verified_teardown,
        },
    }
    _dump(root / journal_path, journal)
    semantic = {
        "finite_ttc_sample_count": finite,
        "non_none_overlay_count": sum(
            value for key, value in overlay.items() if key != "NONE"
        ),
    }
    ledger = {
        "sequence": row["sequence"],
        "identity": identity,
        "status": "evidence_complete",
        "seed_consumed": True,
        "evidence_units_consumed": 1,
        "attempt_limit": 1,
        "resume_forbidden": True,
        "journal_root": assessment.JOURNALS_ROOT,
        "journal": journal_path.as_posix(),
        "raw_evidence_root": raw_root.as_posix(),
        "credential_safe_environment_audit": {"pass": True},
        "bootstrap_receipt": {"service_wait_allowed": True},
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "raw_resources": resources,
        "expected_ttc_status": row["expected_ttc_status"],
        "observed_ttc_status": row["expected_ttc_status"],
        "semantic_observation": semantic,
        "supervisor_config_sha256": "4" * 64,
    }
    return {
        "ledger": ledger,
        "journal": journal,
        "journal_path": journal_path.as_posix(),
        "raw_root": raw_root.as_posix(),
    }


def _fixture(tmp_path):
    root = tmp_path.resolve()
    prereg_source = REAL_WORKSPACE / assessment.PREREGISTRATION_PATH
    _write(root / assessment.PREREGISTRATION_PATH, prereg_source.read_bytes())
    attempts = [_attempt(root, row) for row in assessment.EXPECTED_SCHEDULE]
    stage = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": assessment.STAGE,
        "status": "execution_complete_pending_assessment",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "execution_release": {
            "path": assessment.CANONICAL_RELEASE_PATH,
            "sha256": "5" * 64,
        },
        "authorization_envelope": {
            "path": assessment.CANONICAL_AUTHORIZATION_PATH,
            "sha256": assessment.EXPECTED_AUTHORIZATION_SHA256,
        },
        "evidence_budget_authorized": 6,
        "evidence_units_consumed": 6,
        "r5_remaining_units_consumed": 0,
        "r6_i1_forfeited_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "retry_count": 0,
        "resume_used": False,
        "attempt_limit_per_identity": 1,
        "planned_identity_count": 6,
        "attempt_ledger": [item["ledger"] for item in attempts],
        "terminal_failure": None,
        "assessment_complete": False,
        "winner_ranked_or_frozen": False,
        "unattempted_budget_forfeited": 0,
        "resume_forbidden": True,
        "conservative_pre_popen_consumption_commit": True,
    }
    _dump(root / assessment.STAGE_REPORT_PATH, stage)
    return {"root": root, "stage": stage, "attempts": attempts}


def _build(fixture, **overrides):
    root = fixture["root"]
    arguments = {
        "workspace": root,
        "preregistration_path": assessment.PREREGISTRATION_PATH,
        "caller_preregistration_sha256": _digest(
            root / assessment.PREREGISTRATION_PATH
        ),
        "stage_report_path": assessment.STAGE_REPORT_PATH,
        "caller_stage_report_sha256": _digest(root / assessment.STAGE_REPORT_PATH),
    }
    arguments.update(overrides)
    return assessment.build_assessment(**arguments)


def _rewrite_stage(fixture):
    _dump(fixture["root"] / assessment.STAGE_REPORT_PATH, fixture["stage"])


def _rewrite_journal(fixture, index):
    item = fixture["attempts"][index]
    _dump(fixture["root"] / item["journal_path"], item["journal"])


def test_full_six_identity_assessment_passes_deterministically(tmp_path):
    fixture = _fixture(tmp_path)
    first = _build(fixture)
    second = _build(fixture)
    assert first == second
    assert first["assessment_result"] == "pass"
    assert first["status"] == "simulation_integration_validation_pass"
    assert first["completed_identity_count"] == 6
    assert len(first["attempt_replays"]) == 6
    assert first["integrity_failures"] == []
    assert first["integration_validation_pass"] is True
    assert first["formal_result"] is False
    assert first["runtime_ready"] is False
    assert first["winner_ranked_or_frozen"] is False
    assert first["downstream_authorized"] is False


def test_every_input_is_opened_once_and_parse_reuses_bytes(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path)
    counts = {}
    original = assessment._read_workspace_bytes_once

    def counted(workspace, declared_path):
        counts[declared_path] = counts.get(declared_path, 0) + 1
        return original(workspace, declared_path)

    monkeypatch.setattr(assessment, "_read_workspace_bytes_once", counted)
    result = _build(fixture)
    assert result["assessment_result"] == "pass"
    assert counts
    assert set(counts.values()) == {1}
    assert assessment.PREREGISTRATION_PATH in counts
    assert assessment.STAGE_REPORT_PATH in counts


def test_caller_hashes_are_independent_and_exact(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(
        assessment.R6I5AssessmentError, match="frozen I5 authority"
    ):
        _build(fixture, caller_preregistration_sha256="0" * 64)
    with pytest.raises(
        assessment.R6I5AssessmentError, match="stage report trust-anchor"
    ):
        _build(fixture, caller_stage_report_sha256="0" * 64)


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("formal_result", True, "formal_result drifted"),
        ("evidence_units_consumed", True, "consumption drifted"),
        ("winner_ranked_or_frozen", True, "winner_ranked_or_frozen drifted"),
        ("retry_count", 1, "retry_count drifted"),
    ],
)
def test_stage_safety_and_types_fail_closed(tmp_path, field, value, match):
    fixture = _fixture(tmp_path)
    fixture["stage"][field] = value
    _rewrite_stage(fixture)
    with pytest.raises(assessment.R6I5AssessmentError, match=match):
        _build(fixture)


def test_raw_byte_drift_produces_deterministic_fail_report(tmp_path):
    fixture = _fixture(tmp_path)
    trace = fixture["root"] / fixture["attempts"][0]["raw_root"] / "trace.csv"
    with trace.open("ab") as stream:
        stream.write(b"drift\n")
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert result["completed_identity_count"] == 5
    assert result["integrity_failures"][0]["sequence"] == 1
    assert "raw resource hash drifted: trace" in result["integrity_failures"][0]["error"]


def test_journal_binding_drift_produces_fail_report(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["attempts"][1]["journal"]["evidence_binding"][
        "raw_evidence_bound"
    ] = False
    _rewrite_journal(fixture, 1)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert result["integrity_failures"][0]["sequence"] == 2
    assert "raw evidence is not bound" in result["integrity_failures"][0]["error"]


def test_readiness_binding_is_recomputed_from_raw_counts(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["attempts"][2]["journal"]["evidence_binding"][
        "readiness_direct_counts"
    ]["direct_counts"]["activation_tracker_message_count"] = 999
    _rewrite_journal(fixture, 2)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert "readiness binding drifted" in result["integrity_failures"][0]["error"]


def test_semantic_schedule_is_recomputed_from_raw_evaluation(tmp_path):
    fixture = _fixture(tmp_path)
    path = fixture["root"] / fixture["attempts"][4]["raw_root"] / "evaluation.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["finite_ttc_sample_count"] = 1
    _dump(path, document)
    resource = fixture["attempts"][4]["journal"]["evidence_binding"]["resources"][
        "evaluation"
    ]
    resource["sha256"] = _digest(path)
    fixture["attempts"][4]["ledger"]["raw_resources"]["evaluation"][
        "sha256"
    ] = resource["sha256"]
    _rewrite_journal(fixture, 4)
    _rewrite_stage(fixture)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert "legacy clear semantics failed" in result["integrity_failures"][0]["error"]


def test_teardown_hashes_and_launch_stop_binding_are_replayed(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["attempts"][3]["journal"]["evidence_binding"]["teardown_restore"][
        "two_phase_restore_verified"
    ] = False
    _rewrite_journal(fixture, 3)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert "teardown binding drifted" in result["integrity_failures"][0]["error"]


def test_scene_snapshot_is_rehashed_after_episode(tmp_path):
    fixture = _fixture(tmp_path)
    row = fixture["attempts"][5]["journal"]["scene_snapshot"]["snapshot_world"]
    world = Path(row["path"])
    with world.open("ab") as stream:
        stream.write(b"drift\n")
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert "scene snapshot hash drifted" in result["integrity_failures"][0]["error"]


def test_hash_only_trace_and_process_log_are_not_yaml_parsed(tmp_path):
    fixture = _fixture(tmp_path)
    for label in ("trace", "process_log"):
        row = fixture["attempts"][0]["journal"]["evidence_binding"]["resources"][label]
        path = fixture["root"] / row["path"]
        _write(path, b"5161: integer-key-like-but-hash-only\n")
        row["sha256"] = _digest(path)
        fixture["attempts"][0]["ledger"]["raw_resources"][label]["sha256"] = row[
            "sha256"
        ]
    evaluation_path = (
        fixture["root"] / fixture["attempts"][0]["raw_root"] / "evaluation.yaml"
    )
    evaluation = yaml.safe_load(evaluation_path.read_text(encoding="utf-8"))
    evaluation["raw_trace_sha256"] = fixture["attempts"][0]["journal"][
        "evidence_binding"
    ]["resources"]["trace"]["sha256"]
    _dump(evaluation_path, evaluation)
    evaluation_sha = _digest(evaluation_path)
    fixture["attempts"][0]["journal"]["evidence_binding"]["resources"][
        "evaluation"
    ]["sha256"] = evaluation_sha
    fixture["attempts"][0]["ledger"]["raw_resources"]["evaluation"][
        "sha256"
    ] = evaluation_sha
    _rewrite_journal(fixture, 0)
    _rewrite_stage(fixture)
    assert _build(fixture)["assessment_result"] == "pass"


def test_parsed_raw_non_string_key_fails_without_crashing_assessor(tmp_path):
    fixture = _fixture(tmp_path)
    row = fixture["attempts"][0]["journal"]["evidence_binding"]["resources"][
        "activation"
    ]
    path = fixture["root"] / row["path"]
    with path.open("ab") as stream:
        stream.write(b"5161: forbidden-key\n")
    row["sha256"] = _digest(path)
    fixture["attempts"][0]["ledger"]["raw_resources"]["activation"][
        "sha256"
    ] = row["sha256"]
    _rewrite_journal(fixture, 0)
    _rewrite_stage(fixture)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert "non-string key" in result["integrity_failures"][0]["error"]


def test_terminal_failure_and_explicit_forfeitures_are_preserved(tmp_path):
    fixture = _fixture(tmp_path)
    ledger = fixture["stage"]["attempt_ledger"]
    ledger[2].update(
        {
            "status": "terminal_failure",
            "seed_consumed": False,
            "evidence_units_consumed": 0,
            "failure_type": "SyntheticFailure",
            "failure_reason": "bounded",
        }
    )
    for row in ledger[3:]:
        row.update(
            {
                "status": "forfeited_unattempted_after_terminal_failure",
                "seed_consumed": False,
                "evidence_units_consumed": 0,
                "retry_forbidden": True,
                "resume_forbidden": True,
            }
        )
    fixture["stage"].update(
        {
            "status": "terminal_failure",
            "evidence_units_consumed": 2,
            "terminal_failure": {
                "identity": ledger[2]["identity"],
                "failure_type": "SyntheticFailure",
                "reason": "bounded",
            },
            "unattempted_budget_forfeited": 4,
        }
    )
    _rewrite_stage(fixture)
    result = _build(fixture)
    assert result["assessment_result"] == "fail"
    assert result["status"] == "terminal_execution_failure_preserved"
    assert result["completed_identity_count"] == 2
    assert result["unattempted_budget_forfeited"] == 4
    assert result["all_completed_journals_directly_replayed"] is True


def test_ledger_identity_bool_int_confusion_is_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    fixture["stage"]["attempt_ledger"][0]["identity"]["seed"] = True
    _rewrite_stage(fixture)
    with pytest.raises(assessment.R6I5AssessmentError, match="ledger identity drifted"):
        _build(fixture)


def test_duplicate_stage_yaml_key_is_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    path = fixture["root"] / assessment.STAGE_REPORT_PATH
    with path.open("ab") as stream:
        stream.write(b"stage: V2-04G-R6-I5\n")
    with pytest.raises(assessment.R6I5AssessmentError, match="duplicate YAML key"):
        _build(fixture)


@pytest.mark.parametrize("target", ["stage", "raw"])
def test_symlinked_inputs_fail_no_follow(tmp_path, target):
    fixture = _fixture(tmp_path)
    if target == "stage":
        path = fixture["root"] / assessment.STAGE_REPORT_PATH
    else:
        path = fixture["root"] / fixture["attempts"][0]["raw_root"] / "trace.csv"
    real = path.with_name(path.name + ".real")
    path.rename(real)
    path.symlink_to(real.name)
    if target == "stage":
        with pytest.raises(assessment.R6I5AssessmentError, match="cannot safely open"):
            _build(fixture)
    else:
        result = _build(fixture)
        assert result["assessment_result"] == "fail"
        assert "cannot safely open" in result["integrity_failures"][0]["error"]


def test_write_once_persists_exact_report_and_refuses_overwrite(tmp_path):
    fixture = _fixture(tmp_path)
    report = _build(fixture)
    before_stage = _digest(fixture["root"] / assessment.STAGE_REPORT_PATH)
    receipt = assessment.write_assessment_once(fixture["root"], report)
    target = fixture["root"] / assessment.EXECUTION_REPORT_PATH
    assert target.is_file()
    assert receipt == {
        "path": assessment.EXECUTION_REPORT_PATH,
        "sha256": _digest(target),
    }
    assert _digest(fixture["root"] / assessment.STAGE_REPORT_PATH) == before_stage
    with pytest.raises(FileExistsError):
        assessment.write_assessment_once(fixture["root"], report)


def test_assessor_source_has_no_ros_or_process_launch_imports():
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    imports = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add((node.module or "").split(".")[0])
    assert not imports & {
        "actionlib",
        "dynamic_reconfigure",
        "gazebo_msgs",
        "roslaunch",
        "rospy",
        "subprocess",
    }

