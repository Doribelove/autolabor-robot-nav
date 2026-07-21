import copy
import hashlib
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


MODULE_PATH = Path(__file__).resolve().parents[2] / (
    "src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release.py"
)
SPECIFICATION = importlib.util.spec_from_file_location(
    "v2_04g_r6_i5_release_validator_under_test", MODULE_PATH
)
release = importlib.util.module_from_spec(SPECIFICATION)
sys.modules[SPECIFICATION.name] = release
SPECIFICATION.loader.exec_module(release)


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


def _seal_logical(document):
    value = copy.deepcopy(document)
    value.pop("closure_sha256", None)
    document["closure_sha256"] = release.canonical_document_sha256(value)
    return document


def _scope():
    return {
        "purpose": "runtime_evaluator_semantic_and_execution_integrity_validation",
        "stage_only": release.STAGE,
        "profiles": list(release.EXPECTED_PROFILES),
        "fresh_execution_seeds": list(release.EXPECTED_EXECUTION_SEEDS),
        "exact_identity_count": len(release.EXPECTED_SCHEDULE),
        "component_stage_authorized": False,
        "general_navigation_calibration_authorized": False,
        "winner_selection_authorized": False,
    }


def _safety():
    return {
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "evidence_budget_authorized": 6,
        "fresh_execution_seeds": [5161, 5162, 5163],
        "attempt_limit_per_identity": 1,
        "retry_or_resume_allowed": False,
        "seed_replacement_allowed": False,
        "budget_expansion_allowed": False,
        "stop_on_first_terminal_failure": True,
        "forfeit_unattempted_units_after_terminal_failure": True,
        "i1_retry_or_resume_authorized": False,
        "i1_forfeited_units_reused": False,
        "prior_identity_reuse_allowed": False,
        "r5_retry_or_resume_authorized": False,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "rank_or_freeze_winner_authorized": False,
        "v2_05_authorized": False,
        "sac_or_training_authorized": False,
        "real_vehicle_authorized": False,
        "real_vehicle_teb_write_authorized": False,
    }


def _completion():
    return {
        "maximum_claim": "fresh_simulation_runtime_evaluator_semantic_integration",
        "safety_performance_generalization_claim_allowed": False,
        "formal_result_must_remain_false": True,
        "runtime_ready_must_remain_false": True,
        "downstream_authorization_after_completion": False,
    }


def _preregistration():
    return {
        "schema_version": "2.1",
        "stage": release.STAGE,
        "execution_authorized": False,
        "execution_release_required": True,
        "budget": {
            "evidence_units_authorizable": 6,
            "attempt_limit_per_identity": 1,
            "retry_allowed": False,
            "resume_allowed": False,
            "replacement_seed_allowed": False,
            "budget_expansion_allowed": False,
        },
        "schedule": copy.deepcopy(release.EXPECTED_SCHEDULE),
    }


def _authorization(root, i4_closure):
    schedule = copy.deepcopy(release.EXPECTED_SCHEDULE)
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": release.STAGE,
        "authorization_id": "synthetic_r6_i5_authorization",
        "status": "bounded_fresh_seed_simulation_authorized",
        "authorization_date": "2099-01-01",
        "authorization_source": (
            "explicit_user_instruction_after_independent_integration_review"
        ),
        "execution_authorized": True,
        **_safety(),
        "scope": _scope(),
        "exact_schedule": schedule,
        "preregistration_schedule_sha256": release.EXPECTED_SCHEDULE_SHA256,
        "bound_resources": {
            label: {"path": path, "sha256": _digest(root / path)}
            for label, path in release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS.items()
        },
        "dependency_closure_digest": i4_closure["closure_sha256"],
        "authorization_trust_anchor": {
            "mechanism": "caller_supplied_exact_authorization_file_sha256",
            "self_hash_embedded": False,
            "guard_rejects_missing_or_mismatched_cli_hash": True,
        },
        "completion_boundary": _completion(),
    }


def _release(root, closure_digest):
    schedule = copy.deepcopy(release.EXPECTED_SCHEDULE)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": release.STAGE,
        "release_id": "synthetic_r6_i5_execution_release",
        "status": "bounded_simulation_execution_released",
        "release_date": "2099-01-02",
        "release_source": (
            "same_turn_explicit_user_instruction_after_i5_execution_readiness_closure"
        ),
        "explicit_user_execution_instruction_received": True,
        "execution_release_authorized": True,
        **_safety(),
        "scope": _scope(),
        "authorization_envelope_alone_sufficient_for_execution": False,
        "exact_schedule": schedule,
        "exact_schedule_sha256": release.EXPECTED_SCHEDULE_SHA256,
        "bound_resources": {
            label: {"path": path, "sha256": _digest(root / path)}
            for label, path in release.EXPECTED_RELEASE_RESOURCE_PATHS.items()
        },
        "dependency_closure_digest": closure_digest,
        "release_trust_anchor": {
            "mechanism": "caller_supplied_exact_execution_release_file_sha256",
            "self_hash_embedded": False,
            "guard_rejects_missing_or_mismatched_cli_hash": True,
            "authorization_hash_independently_supplied": True,
        },
        "prejournal_gate": {
            "release_validation_before_execution_state_creation_required": True,
            "authorization_revalidation_required": True,
            "all_bound_resources_rehashed_required": True,
            "closure_logical_digest_recomputed_required": True,
            "scene_children_rehashed_required": True,
            "machine_review_pass_required": True,
            "existing_execution_state_absent_required": True,
            "forbidden_processes_absent_required": True,
            "execution_state_creation_before_validation_allowed": False,
        },
        "completion_boundary": _completion(),
    }


def _machine_review():
    return {
        "schema_version": "1.0",
        "stage": release.STAGE,
        "status": release.EXPECTED_MACHINE_REVIEW_STATUS,
        "review_result": "pass",
        "execution_ready": False,
        "separate_execution_release_required": True,
        "separate_execution_release_present": False,
        "formal_result": False,
        "runtime_ready": False,
        "execution_absence_review": {
            "release_manifest_present": False,
            "attempt_root_present": False,
            "journal_root_present": False,
            "receipt_present": False,
            "raw_or_semantic_evidence_present": False,
            "stage_execution_report_present": False,
            "evidence_units_consumed": 0,
            "process_start_performed_by_review": False,
            "execution_ready": False,
            "pass": True,
        },
        "side_effects": {
            "execution_release_created": False,
            "attempt_root_created": False,
            "journal_created": False,
            "subprocess_started_by_review": False,
            "ros_started_by_review": False,
            "gazebo_started_by_review": False,
            "move_base_or_teb_started_by_review": False,
            "evidence_units_consumed": 0,
            "training_started": False,
        },
    }


def _fixture(tmp_path):
    root = tmp_path.resolve()
    preregistration_path = release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "preregistration"
    ]
    i4_closure_path = release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "inherited_i4_dependency_closure"
    ]
    _dump(root / preregistration_path, _preregistration())
    i4_closure = _seal_logical(
        {
            "schema_version": "synthetic",
            "stage": release.BASIS_STAGE,
            "execution_authorized": False,
            "execution_ready": False,
            "unresolved": [],
        }
    )
    _dump(root / i4_closure_path, i4_closure)

    for label, path in release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS.items():
        if path in {preregistration_path, i4_closure_path}:
            continue
        if path.endswith((".yaml", ".yml")):
            _dump(
                root / path,
                {
                    "fixture_label": label,
                    "legacy_integer_key_roster": {
                        5141: "dynamic_conflict_single",
                        5142: "dynamic_conflict_multi",
                    },
                },
            )
        else:
            _write(root / path, ("synthetic {}\n".format(label)).encode())

    authorization = _authorization(root, i4_closure)
    _dump(root / release.CANONICAL_AUTHORIZATION_PATH, authorization)

    release_paths = release.EXPECTED_RELEASE_RESOURCE_PATHS
    for label in ("stage_transition", "scene_derivation"):
        _dump(
            root / release_paths[label],
            {
                "stage": release.STAGE,
                "fixture_label": label,
                "seed_roles": {5161: "single", 5162: "multi", 5163: "clear"},
            },
        )
    for label in ("execution_entrypoint", "release_validator", "release_validator_tests"):
        _write(root / release_paths[label], ("synthetic {}\n".format(label)).encode())

    child_labels = sorted(
        label
        for label in release_paths
        if label.startswith(release.SCENE_CHILD_LABEL_PREFIX)
    )
    for index, label in enumerate(child_labels, start=1):
        path = release_paths[label]
        if path.endswith(".instance.yaml"):
            _dump(
                root / path,
                {
                    "stage": release.STAGE,
                    "fixture_index": index,
                    "legacy_integer_seed": {5161 + index: "hash_only"},
                },
            )
        else:
            _write(root / path, ("synthetic world {}\n".format(index)).encode())
    scene_index = {
        "schema_version": "2.0",
        "formal_result": False,
        "runtime_ready": False,
        "scene_count": 7,
        "files": [
            {"path": release_paths[label], "sha256": _digest(root / release_paths[label])}
            for label in child_labels
        ],
    }
    _dump(root / release_paths["fresh_scene_index"], scene_index)

    machine_review = _machine_review()
    _dump(root / release_paths["execution_machine_review"], machine_review)

    closure_local_paths = sorted(
        (
            set(release_paths.values())
            - {
                release_paths["execution_dependency_closure"],
                release_paths["execution_machine_review"],
            }
        )
        | set(release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS.values())
    )
    external_python = root / "synthetic_external/python3"
    _write(external_python, b"synthetic python interpreter\n")
    external = _seal_logical(
        {
            "python_interpreter": {
                "canonical_path": external_python.as_posix(),
                "sha256": _digest(external_python),
                "size_bytes": external_python.stat().st_size,
            },
            "runtime_bindings": [],
            "files": [
                {
                    "canonical_path": external_python.as_posix(),
                    "sha256": _digest(external_python),
                    "size_bytes": external_python.stat().st_size,
                }
            ],
            "unresolved": [],
        }
    )
    closure = _seal_logical(
        {
            "schema_version": "synthetic",
            "stage": release.STAGE,
            "execution_authorized": False,
            "local": {
                "files": [
                    {
                        "path": path,
                        "sha256": _digest(root / path),
                        "size_bytes": (root / path).stat().st_size,
                    }
                    for path in closure_local_paths
                ],
                "required_paths": closure_local_paths,
            },
            "external": external,
            "unresolved": [],
        }
    )
    _dump(root / release_paths["execution_dependency_closure"], closure)

    release_document = _release(root, closure["closure_sha256"])
    _dump(root / release.CANONICAL_RELEASE_PATH, release_document)
    return {
        "root": root,
        "authorization": authorization,
        "i4_closure": i4_closure,
        "closure": closure,
        "machine_review": machine_review,
        "release": release_document,
    }


def _validate(fixture, **overrides):
    root = fixture["root"]
    arguments = {
        "workspace": root,
        "release_path": release.CANONICAL_RELEASE_PATH,
        "caller_release_sha256": _digest(root / release.CANONICAL_RELEASE_PATH),
        "authorization_path": release.CANONICAL_AUTHORIZATION_PATH,
        "caller_authorization_sha256": _digest(
            root / release.CANONICAL_AUTHORIZATION_PATH
        ),
        "expected_resource_paths": release.EXPECTED_RELEASE_RESOURCE_PATHS,
        "expected_machine_review_status": release.EXPECTED_MACHINE_REVIEW_STATUS,
    }
    arguments.update(overrides)
    return release.load_and_validate_execution_release(**arguments)


def _refresh_closure_local_rows(fixture):
    root = fixture["root"]
    for row in fixture["closure"]["local"]["files"]:
        row["sha256"] = _digest(root / row["path"])
        row["size_bytes"] = (root / row["path"]).stat().st_size
    _seal_logical(fixture["closure"])
    _dump(
        root
        / release.EXPECTED_RELEASE_RESOURCE_PATHS["execution_dependency_closure"],
        fixture["closure"],
    )


def _refresh_release(fixture):
    root = fixture["root"]
    fixture["release"]["dependency_closure_digest"] = fixture["closure"][
        "closure_sha256"
    ]
    for row in fixture["release"]["bound_resources"].values():
        row["sha256"] = _digest(root / row["path"])
    _dump(root / release.CANONICAL_RELEASE_PATH, fixture["release"])


def _commit_authorization_and_outer(fixture):
    root = fixture["root"]
    for row in fixture["authorization"]["bound_resources"].values():
        row["sha256"] = _digest(root / row["path"])
    fixture["authorization"]["dependency_closure_digest"] = fixture[
        "i4_closure"
    ]["closure_sha256"]
    _dump(root / release.CANONICAL_AUTHORIZATION_PATH, fixture["authorization"])
    _refresh_closure_local_rows(fixture)
    _refresh_release(fixture)


def _commit_i5_closure(fixture):
    root = fixture["root"]
    _seal_logical(fixture["closure"])
    _dump(
        root
        / release.EXPECTED_RELEASE_RESOURCE_PATHS["execution_dependency_closure"],
        fixture["closure"],
    )
    _refresh_release(fixture)


def _commit_machine_review(fixture):
    _dump(
        fixture["root"]
        / release.EXPECTED_RELEASE_RESOURCE_PATHS["execution_machine_review"],
        fixture["machine_review"],
    )
    _refresh_release(fixture)


def _omit_entrypoint_from_closure(document):
    path = release.EXPECTED_RELEASE_RESOURCE_PATHS["execution_entrypoint"]
    document["local"]["files"] = [
        row for row in document["local"]["files"] if row["path"] != path
    ]
    document["local"]["required_paths"].remove(path)


def test_valid_synthetic_i5_release_returns_exact_closed_receipt(tmp_path):
    fixture = _fixture(tmp_path)
    result = _validate(fixture)
    assert result.identity_count == 6
    assert result.execution_seeds == (5161, 5162, 5163)
    assert result.schedule_sha256 == release.EXPECTED_SCHEDULE_SHA256
    assert len(result.bound_resources) == 29
    assert len(result.authorization_bound_resources) == 12
    assert result.release_parsed_labels == tuple(
        sorted(release.RELEASE_PARSED_RESOURCE_LABELS)
    )
    assert len(result.release_parsed_labels) == 5
    assert len(result.release_hash_only_labels) == 24
    assert result.authorization_parsed_labels == tuple(
        sorted(release.AUTHORIZATION_PARSED_RESOURCE_LABELS)
    )
    assert len(result.authorization_parsed_labels) == 2
    assert len(result.authorization_hash_only_labels) == 10
    assert result.runtime_executables["python_interpreter"].endswith(
        "/synthetic_external/python3"
    )


def test_every_resource_is_single_open_and_snapshot_reused(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path)
    workspace_counts = {}
    external_counts = {}
    original_workspace = release._read_workspace_relative_bytes_once
    original_external = release._read_external_absolute_bytes_once

    def counted_workspace(workspace, declared_path):
        workspace_counts[declared_path] = workspace_counts.get(declared_path, 0) + 1
        return original_workspace(workspace, declared_path)

    def counted_external(declared_path):
        external_counts[declared_path] = external_counts.get(declared_path, 0) + 1
        return original_external(declared_path)

    monkeypatch.setattr(
        release, "_read_workspace_relative_bytes_once", counted_workspace
    )
    monkeypatch.setattr(
        release, "_read_external_absolute_bytes_once", counted_external
    )
    result = _validate(fixture)
    assert set(workspace_counts.values()) == {1}
    assert set(external_counts.values()) == {1}
    assert result.preregistration is result.bound_resources["preregistration"]


def test_hash_only_legacy_integer_key_yaml_is_never_parsed(tmp_path):
    fixture = _fixture(tmp_path)
    result = _validate(fixture)
    assert result.bound_resources["execution_contract"].document is None
    assert result.bound_resources["scene_derivation"].document is None
    assert result.bound_resources["fresh_scene_child_01"].document is None
    assert (
        result.authorization_bound_resources["source_i1_scene_manifest"].document
        is None
    )
    for label in release.RELEASE_PARSED_RESOURCE_LABELS:
        assert result.bound_resources[label].document is not None
    for label in release.AUTHORIZATION_PARSED_RESOURCE_LABELS:
        assert result.authorization_bound_resources[label].document is not None


@pytest.mark.parametrize(
    "mutation,match",
    [
        (lambda document: document.update({"unexpected": False}), "keys drifted"),
        (
            lambda document: document.update({"evidence_budget_authorized": True}),
            "evidence_budget_authorized drifted",
        ),
        (lambda document: document["exact_schedule"].reverse(), "schedule drifted"),
        (
            lambda document: document["exact_schedule"][0].update({"seed": True}),
            "schedule drifted",
        ),
        (
            lambda document: document["prejournal_gate"].update(
                {"execution_state_creation_before_validation_allowed": True}
            ),
            "prejournal policy drifted",
        ),
    ],
)
def test_release_schema_schedule_and_types_fail_closed(tmp_path, mutation, match):
    fixture = _fixture(tmp_path)
    mutation(fixture["release"])
    _dump(
        fixture["root"] / release.CANONICAL_RELEASE_PATH,
        fixture["release"],
    )
    with pytest.raises(release.R6I5ExecutionReleaseError, match=match):
        _validate(fixture)


@pytest.mark.parametrize("kind", ["missing", "extra", "path_swap"])
def test_release_bound_resource_roster_is_exact(tmp_path, kind):
    fixture = _fixture(tmp_path)
    bindings = fixture["release"]["bound_resources"]
    if kind == "missing":
        bindings.pop("failed_i3_release")
    elif kind == "extra":
        bindings["unexpected"] = copy.deepcopy(bindings["failed_i3_release"])
    else:
        bindings["failed_i3_release"]["path"] = bindings["stage_transition"]["path"]
    _dump(
        fixture["root"] / release.CANONICAL_RELEASE_PATH,
        fixture["release"],
    )
    with pytest.raises(release.R6I5ExecutionReleaseError):
        _validate(fixture)


@pytest.mark.parametrize("kind", ["missing", "extra", "path_swap"])
def test_authorization_bound_resource_roster_is_exact(tmp_path, kind):
    fixture = _fixture(tmp_path)
    bindings = fixture["authorization"]["bound_resources"]
    if kind == "missing":
        bindings.pop("r6_design_report")
    elif kind == "extra":
        bindings["unexpected"] = copy.deepcopy(bindings["r6_design_report"])
    else:
        bindings["r6_design_report"]["path"] = bindings["frozen_evaluator"]["path"]
    _commit_authorization_and_outer(fixture)
    with pytest.raises(release.R6I5ExecutionReleaseError):
        _validate(fixture)


@pytest.mark.parametrize("kind", ["missing", "extra", "path_swap"])
def test_caller_trusted_release_roster_is_version_locked(tmp_path, kind):
    fixture = _fixture(tmp_path)
    paths = dict(release.EXPECTED_RELEASE_RESOURCE_PATHS)
    if kind == "missing":
        paths.pop("failed_i3_release")
    elif kind == "extra":
        paths["unexpected"] = paths["failed_i3_release"]
    else:
        paths["failed_i3_release"] = paths["stage_transition"]
    with pytest.raises(
        release.R6I5ExecutionReleaseError,
        match="trusted release resource roster drifted",
    ):
        _validate(fixture, expected_resource_paths=paths)


def test_independent_release_and_authorization_caller_hashes_are_required(tmp_path):
    fixture = _fixture(tmp_path)
    zero = "0" * 64
    with pytest.raises(release.R6I5ExecutionReleaseError, match="release trust-anchor"):
        _validate(fixture, caller_release_sha256=zero)
    with pytest.raises(
        release.R6I5ExecutionReleaseError, match="authorization trust-anchor"
    ):
        _validate(fixture, caller_authorization_sha256=zero)


def test_hash_only_resource_byte_drift_is_rejected_without_parsing(tmp_path):
    fixture = _fixture(tmp_path)
    path = release.EXPECTED_RELEASE_RESOURCE_PATHS["scene_derivation"]
    with (fixture["root"] / path).open("ab") as stream:
        stream.write(b"# drift\n")
    with pytest.raises(
        release.R6I5ExecutionReleaseError,
        match="release-bound resource drifted: scene_derivation",
    ):
        _validate(fixture)


@pytest.mark.parametrize(
    "payload,match",
    [
        (b"stage: V2-04G-R6-I5\nstage: V2-04G-R6-I5\n", "duplicate YAML key"),
        (b"stage: V2-04G-R6-I5\n5161: legacy\n", "non-string key"),
        (
            b"base: &base\n  stage: V2-04G-R6-I5\n<<: *base\n",
            "merge keys are forbidden",
        ),
    ],
)
def test_parsed_preregistration_rejects_ambiguous_or_non_string_yaml(
    tmp_path, payload, match
):
    fixture = _fixture(tmp_path)
    preregistration_path = release.EXPECTED_RELEASE_RESOURCE_PATHS[
        "preregistration"
    ]
    _write(fixture["root"] / preregistration_path, payload)
    _commit_authorization_and_outer(fixture)
    with pytest.raises(release.R6I5ExecutionReleaseError, match=match):
        _validate(fixture)


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("stage", "V2-04G-R6-I3", "closure stage drifted"),
        ("execution_authorized", True, "must remain non-authorizing"),
        ("execution_ready", True, "readiness drifted"),
        ("unresolved", ["missing"], "closure is unresolved"),
    ],
)
def test_inherited_i4_authorization_closure_semantics_are_enforced(
    tmp_path, field, value, match
):
    fixture = _fixture(tmp_path)
    fixture["i4_closure"][field] = value
    _seal_logical(fixture["i4_closure"])
    _dump(
        fixture["root"]
        / release.EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
            "inherited_i4_dependency_closure"
        ],
        fixture["i4_closure"],
    )
    _commit_authorization_and_outer(fixture)
    with pytest.raises(release.R6I5ExecutionReleaseError, match=match):
        _validate(fixture)


@pytest.mark.parametrize(
    "mutate,match",
    [
        (lambda document: document.update({"stage": "V2-04G-R6-I4"}), "stage drifted"),
        (
            lambda document: document.update({"execution_authorized": True}),
            "must remain non-authorizing",
        ),
        (lambda document: document.update({"unresolved": ["missing"]}), "is unresolved"),
        (
            _omit_entrypoint_from_closure,
            "omits a release-bound runtime resource",
        ),
    ],
)
def test_i5_execution_dependency_closure_fails_closed(tmp_path, mutate, match):
    fixture = _fixture(tmp_path)
    mutate(fixture["closure"])
    _commit_i5_closure(fixture)
    with pytest.raises(release.R6I5ExecutionReleaseError, match=match):
        _validate(fixture)


@pytest.mark.parametrize(
    "mutate,match",
    [
        (lambda document: document.update({"stage": "V2-04G-R6-I4"}), "stage drifted"),
        (lambda document: document.update({"status": "pass"}), "status drifted"),
        (
            lambda document: document["execution_absence_review"].update(
                {"evidence_units_consumed": 1}
            ),
            "evidence_units_consumed drifted",
        ),
        (
            lambda document: document["side_effects"].update(
                {"ros_started_by_review": True}
            ),
            "ros_started_by_review drifted",
        ),
    ],
)
def test_i5_machine_review_status_absence_and_side_effects_are_enforced(
    tmp_path, mutate, match
):
    fixture = _fixture(tmp_path)
    mutate(fixture["machine_review"])
    _commit_machine_review(fixture)
    with pytest.raises(release.R6I5ExecutionReleaseError, match=match):
        _validate(fixture)


def test_machine_review_expected_status_argument_is_version_locked(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(
        release.R6I5ExecutionReleaseError,
        match="trusted machine review status drifted",
    ):
        _validate(fixture, expected_machine_review_status="caller_selected_status")


@pytest.mark.parametrize("target", ["release", "hash_only_resource"])
def test_release_and_resource_symlinks_are_rejected(tmp_path, target):
    fixture = _fixture(tmp_path)
    if target == "release":
        path = fixture["root"] / release.CANONICAL_RELEASE_PATH
    else:
        path = fixture["root"] / release.EXPECTED_RELEASE_RESOURCE_PATHS[
            "scene_derivation"
        ]
    real_path = path.with_name(path.name + ".real")
    path.rename(real_path)
    path.symlink_to(real_path.name)
    with pytest.raises(release.R6I5ExecutionReleaseError, match="cannot safely open"):
        _validate(fixture)


def test_validation_is_read_only_and_deterministic(tmp_path):
    fixture = _fixture(tmp_path)
    root = fixture["root"]
    before = {
        path.relative_to(root).as_posix(): _digest(path)
        for path in root.rglob("*")
        if path.is_file()
    }
    first = _validate(fixture)
    second = _validate(fixture)
    after = {
        path.relative_to(root).as_posix(): _digest(path)
        for path in root.rglob("*")
        if path.is_file()
    }
    assert after == before
    assert first.release.sha256 == second.release.sha256
    assert first.authorization.sha256 == second.authorization.sha256
    assert first.schedule_sha256 == second.schedule_sha256
