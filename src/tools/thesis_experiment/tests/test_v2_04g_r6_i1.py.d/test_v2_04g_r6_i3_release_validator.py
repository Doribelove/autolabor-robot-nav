import copy
import hashlib
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


MODULE_PATH = Path(__file__).resolve().parents[2] / (
    "src/thesis_experiment/v2_04g_r6_i1_r6_i2_r6_i3_release.py"
)
SPECIFICATION = importlib.util.spec_from_file_location(
    "v2_04g_r6_i3_release_validator_under_test", MODULE_PATH
)
release = importlib.util.module_from_spec(SPECIFICATION)
sys.modules[SPECIFICATION.name] = release
SPECIFICATION.loader.exec_module(release)


MACHINE_REVIEW_STATUS = (
    "execution_readiness_closure_pass_release_absent"
)
PREREGISTRATION_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml"
)
SCENE_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i3_execution/compiled_scenes"
)
SCENE_INDEX_PATH = SCENE_ROOT + "/compiled_scene_index.yaml"
RUNNER_PATH = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py"
)
VALIDATOR_PATH = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_release.py"
)
VALIDATOR_TEST_PATH = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_release_validator.py"
)
CLOSURE_PATH = (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i3_execution/execution_dependency_closure.yaml"
)
MACHINE_REVIEW_PATH = (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i3_execution/v2_04g_r6_i3_execution_readiness_review.yaml"
)
I2_CLOSURE_PATH = (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i2_repair_review/execution_dependency_closure.yaml"
)


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
        "fresh_execution_seeds": [5151, 5152, 5153],
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


def _scene_paths():
    stems = [
        "v2-04g-r6-i3-dynamic-conflict-single-s5151",
        "v2-04g-r6-i3-dynamic-conflict-multi-s5152",
        "v2-04g-r6-i3-dynamic-semantic-clear-s5153",
        "v2-04g-r6-i3-compile-support-cruise-s5154",
        "v2-04g-r6-i3-compile-support-static-s5155",
        "v2-04g-r6-i3-compile-support-corridor-s5156",
        "v2-04g-r6-i3-compile-support-maneuver-s5157",
    ]
    result = []
    for stem in stems:
        result.extend(
            [
                SCENE_ROOT + "/" + stem + ".instance.yaml",
                SCENE_ROOT + "/" + stem + ".world",
            ]
        )
    return result


def _authorization(preregistration_sha, i2_closure_sha, i2_logical):
    schedule = copy.deepcopy(release.EXPECTED_SCHEDULE)
    document = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": release.STAGE,
        "authorization_id": "synthetic_r6_i3_authorization",
        "status": "bounded_fresh_seed_simulation_authorized",
        "authorization_date": "2099-01-01",
        "authorization_source": (
            "explicit_user_instruction_after_independent_integration_review"
        ),
        "execution_authorized": True,
        **_safety(),
        "scope": _scope(),
        "exact_schedule": schedule,
        "preregistration_schedule_sha256": (
            release.canonical_document_sha256(schedule)
        ),
        "bound_resources": {
            "preregistration": {
                "path": PREREGISTRATION_PATH,
                "sha256": preregistration_sha,
            },
            "inherited_r6_i2_dependency_closure": {
                "path": I2_CLOSURE_PATH,
                "sha256": i2_closure_sha,
            },
        },
        "dependency_closure_digest": i2_logical,
        "authorization_trust_anchor": {
            "mechanism": "caller_supplied_exact_authorization_file_sha256",
            "self_hash_embedded": False,
            "guard_rejects_missing_or_mismatched_cli_hash": True,
        },
        "completion_boundary": _completion(),
    }
    return document


def _release(resource_paths, resource_hashes, closure_digest):
    schedule = copy.deepcopy(release.EXPECTED_SCHEDULE)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": release.STAGE,
        "release_id": "synthetic_r6_i3_execution_release",
        "status": "bounded_simulation_execution_released",
        "release_date": "2099-01-02",
        "release_source": (
            "explicit_user_instruction_after_execution_readiness_closure"
        ),
        "explicit_user_execution_instruction_received": True,
        "execution_release_authorized": True,
        **_safety(),
        "scope": _scope(),
        "authorization_envelope_alone_sufficient_for_execution": False,
        "exact_schedule": schedule,
        "exact_schedule_sha256": release.canonical_document_sha256(schedule),
        "bound_resources": {
            label: {"path": path, "sha256": resource_hashes[label]}
            for label, path in resource_paths.items()
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


def _fixture(tmp_path):
    root = tmp_path.resolve()
    schedule = copy.deepcopy(release.EXPECTED_SCHEDULE)
    preregistration = {
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
        "schedule": schedule,
    }
    _dump(root / PREREGISTRATION_PATH, preregistration)

    i2_closure = _seal_logical(
        {
            "schema_version": "synthetic",
            "stage": "V2-04G-R6-I2",
            "unresolved": [],
        }
    )
    _dump(root / I2_CLOSURE_PATH, i2_closure)
    authorization = _authorization(
        _digest(root / PREREGISTRATION_PATH),
        _digest(root / I2_CLOSURE_PATH),
        i2_closure["closure_sha256"],
    )
    _dump(root / release.CANONICAL_AUTHORIZATION_PATH, authorization)

    scene_paths = _scene_paths()
    for index, path in enumerate(scene_paths, start=1):
        if path.endswith(".instance.yaml"):
            _dump(
                root / path,
                {
                    "stage": release.STAGE,
                    "fixture_index": index,
                    "formal_result": False,
                    "runtime_ready": False,
                },
            )
        else:
            _write(root / path, ("synthetic world {}\n".format(index)).encode())
    scene_index = {
        "schema_version": "2.0",
        "manifest_id": "synthetic_r6_i3_compiled_scenes",
        "formal_result": False,
        "runtime_ready": False,
        "scene_count": 7,
        "families": [
            "DYNAMIC",
            "DYNAMIC",
            "DYNAMIC",
            "CRUISE",
            "STATIC_DENSE",
            "CORRIDOR",
            "MANEUVER",
        ],
        "files": [
            {"path": path, "sha256": _digest(root / path)}
            for path in scene_paths
        ],
    }
    _dump(root / SCENE_INDEX_PATH, scene_index)
    _write(root / RUNNER_PATH, b"# synthetic reviewed runner\n")
    _write(root / VALIDATOR_PATH, b"# synthetic reviewed release validator\n")
    _write(root / VALIDATOR_TEST_PATH, b"# synthetic validator tests\n")

    resource_paths = {
        "preregistration": PREREGISTRATION_PATH,
        "authorization_envelope": release.CANONICAL_AUTHORIZATION_PATH,
        "fresh_scene_index": SCENE_INDEX_PATH,
        "execution_entrypoint": RUNNER_PATH,
        "release_validator": VALIDATOR_PATH,
        "release_validator_tests": VALIDATOR_TEST_PATH,
        "execution_dependency_closure": CLOSURE_PATH,
        "execution_machine_review": MACHINE_REVIEW_PATH,
    }
    for index, path in enumerate(scene_paths, start=1):
        resource_paths["fresh_scene_child_{:02d}".format(index)] = path

    closure_local_paths = sorted(
        set(resource_paths.values())
        - {CLOSURE_PATH, MACHINE_REVIEW_PATH}
        | {I2_CLOSURE_PATH}
    )
    external_python = root / "synthetic_external/python3"
    external_roslaunch = root / "synthetic_external/roslaunch"
    _write(external_python, b"synthetic python interpreter\n")
    _write(external_roslaunch, b"synthetic roslaunch executable\n")
    external_rows = [
        {
            "canonical_path": path.as_posix(),
            "sha256": _digest(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted([external_python, external_roslaunch])
    ]
    external = _seal_logical(
        {
            "python_interpreter": {
                "canonical_path": external_python.as_posix(),
                "sha256": _digest(external_python),
                "size_bytes": external_python.stat().st_size,
            },
            "python_bindings": [],
            "runtime_bindings": [
                {
                    "binding": "package-executable:roslaunch:roslaunch",
                    "resolution_kind": "synthetic_fixture",
                    "package": "roslaunch",
                    "package_root": root.as_posix(),
                    "target_canonical_path": external_roslaunch.as_posix(),
                    "canonical_paths": [external_roslaunch.as_posix()],
                }
            ],
            "files": external_rows,
            "unresolved": [],
        }
    )
    local_rows = [
        {
            "path": path,
            "sha256": _digest(root / path),
            "size_bytes": (root / path).stat().st_size,
        }
        for path in closure_local_paths
    ]
    closure = _seal_logical(
        {
            "schema_version": "3.0",
            "stage": release.STAGE,
            "review_scope": "synthetic_execution_readiness_closure_only",
            "execution_authorized": False,
            "seed_or_evidence_units_allocated": 0,
            "seed_or_evidence_units_consumed": 0,
            "authorization_resources": [],
            "generator": "synthetic_fixture",
            "local": {
                "entrypoints": [RUNNER_PATH],
                "files": local_rows,
                "edges": [],
                "external_python_names": [],
                "external_runtime_names": [
                    "package-executable:roslaunch:roslaunch"
                ],
                "required_paths": closure_local_paths,
            },
            "external": external,
            "unresolved": [],
        }
    )
    _dump(root / CLOSURE_PATH, closure)

    machine_review = {
        "schema_version": "1.0",
        "stage": release.STAGE,
        "status": MACHINE_REVIEW_STATUS,
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
            "host_process_exclusivity_check_deferred_to_future_release_gate": True,
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
    _dump(root / MACHINE_REVIEW_PATH, machine_review)

    resource_hashes = {
        label: _digest(root / path) for label, path in resource_paths.items()
    }
    release_document = _release(
        resource_paths, resource_hashes, closure["closure_sha256"]
    )
    _dump(root / release.CANONICAL_RELEASE_PATH, release_document)
    return {
        "root": root,
        "resource_paths": resource_paths,
        "release": release_document,
        "authorization": authorization,
    }


def _validate(fixture, release_sha=None, authorization_sha=None):
    root = fixture["root"]
    return release.load_and_validate_execution_release(
        root,
        release.CANONICAL_RELEASE_PATH,
        release_sha or _digest(root / release.CANONICAL_RELEASE_PATH),
        release.CANONICAL_AUTHORIZATION_PATH,
        authorization_sha
        or _digest(root / release.CANONICAL_AUTHORIZATION_PATH),
        expected_resource_paths=fixture["resource_paths"],
        expected_machine_review_status=MACHINE_REVIEW_STATUS,
    )


def _rewrite_release(fixture):
    _dump(
        fixture["root"] / release.CANONICAL_RELEASE_PATH,
        fixture["release"],
    )


def _bind_current_file(fixture, label):
    path = fixture["resource_paths"][label]
    fixture["release"]["bound_resources"][label]["sha256"] = _digest(
        fixture["root"] / path
    )


def test_valid_synthetic_release_returns_closed_runtime_receipt(tmp_path):
    fixture = _fixture(tmp_path)
    result = _validate(fixture)
    assert result.identity_count == 6
    assert result.execution_seeds == (5151, 5152, 5153)
    assert result.schedule_sha256 == release.canonical_document_sha256(
        release.EXPECTED_SCHEDULE
    )
    assert result.release.document["execution_release_authorized"] is True
    assert result.machine_review.document["execution_ready"] is False
    assert (
        result.runtime_executables["package-executable:roslaunch:roslaunch"]
        .endswith("/synthetic_external/roslaunch")
    )
    assert result.runtime_executables["python_interpreter"].endswith(
        "/synthetic_external/python3"
    )


def test_each_workspace_resource_is_opened_once_and_snapshots_are_reused(
    tmp_path, monkeypatch
):
    fixture = _fixture(tmp_path)
    counts = {}
    original = release._read_workspace_relative_bytes_once

    def counted(workspace, declared_path):
        counts[declared_path] = counts.get(declared_path, 0) + 1
        return original(workspace, declared_path)

    monkeypatch.setattr(release, "_read_workspace_relative_bytes_once", counted)
    result = _validate(fixture)
    assert counts
    assert set(counts.values()) == {1}
    assert (
        result.preregistration
        is result.bound_resources["preregistration"]
    )


@pytest.mark.parametrize(
    "mutate,match",
    [
        (
            lambda document: document.update({"unexpected": False}),
            "keys drifted",
        ),
        (
            lambda document: document.update(
                {"evidence_budget_authorized": True}
            ),
            "evidence_budget_authorized drifted",
        ),
        (
            lambda document: document["exact_schedule"].reverse(),
            "exact_schedule drifted",
        ),
        (
            lambda document: document["exact_schedule"][0].update(
                {"seed": True}
            ),
            "exact_schedule drifted",
        ),
        (
            lambda document: document["prejournal_gate"].update(
                {"execution_state_creation_before_validation_allowed": True}
            ),
            "prejournal policy drifted",
        ),
        (
            lambda document: document["bound_resources"].pop(
                "fresh_scene_child_14"
            ),
            "bound_resources keys drifted",
        ),
    ],
)
def test_closed_type_sensitive_release_rejects_mutations(
    tmp_path, mutate, match
):
    fixture = _fixture(tmp_path)
    mutate(fixture["release"])
    _rewrite_release(fixture)
    with pytest.raises(release.R6I3ExecutionReleaseError, match=match):
        _validate(fixture)


def test_release_and_authorization_require_independent_caller_hashes(tmp_path):
    fixture = _fixture(tmp_path)
    with pytest.raises(release.R6I3ExecutionReleaseError, match="release trust-anchor"):
        _validate(fixture, release_sha="0" * 64)
    with pytest.raises(
        release.R6I3ExecutionReleaseError, match="authorization trust-anchor"
    ):
        _validate(fixture, authorization_sha="0" * 64)


def test_duplicate_key_and_merge_are_rejected_from_single_snapshot(tmp_path):
    fixture = _fixture(tmp_path)
    release_path = fixture["root"] / release.CANONICAL_RELEASE_PATH
    release_path.write_text(
        release_path.read_text(encoding="utf-8") + "stage: V2-04G-R6-I3\n",
        encoding="utf-8",
    )
    with pytest.raises(release.R6I3ExecutionReleaseError, match="duplicate YAML key"):
        _validate(fixture)

    merged = fixture["root"] / "synthetic/merge.yaml"
    _write(merged, b"first: 1\nsecond:\n  <<: {hidden: true}\n")
    with pytest.raises(release.R6I3ExecutionReleaseError, match="merge keys are forbidden"):
        release.read_workspace_file_once(
            fixture["root"], "synthetic/merge.yaml", parse_yaml=True
        )


def test_no_follow_reader_rejects_release_and_resource_symlinks(tmp_path):
    fixture = _fixture(tmp_path)
    target = fixture["root"] / release.CANONICAL_RELEASE_PATH
    real = target.with_name("synthetic_release.real")
    target.rename(real)
    target.symlink_to(real)
    with pytest.raises(release.R6I3ExecutionReleaseError, match="cannot safely open"):
        _validate(fixture, release_sha=_digest(real))

    fixture = _fixture(tmp_path / "second")
    child = fixture["root"] / fixture["resource_paths"]["fresh_scene_child_01"]
    real_child = child.with_name(child.name + ".real")
    child.rename(real_child)
    child.symlink_to(real_child)
    with pytest.raises(release.R6I3ExecutionReleaseError, match="cannot safely open"):
        _validate(fixture)


def test_authorization_semantic_drift_fails_even_with_new_caller_hash(tmp_path):
    fixture = _fixture(tmp_path)
    auth_path = fixture["root"] / release.CANONICAL_AUTHORIZATION_PATH
    fixture["authorization"]["retry_or_resume_allowed"] = True
    _dump(auth_path, fixture["authorization"])
    _bind_current_file(fixture, "authorization_envelope")
    _rewrite_release(fixture)
    with pytest.raises(
        release.R6I3ExecutionReleaseError,
        match="authorization retry_or_resume_allowed drifted",
    ):
        _validate(fixture, authorization_sha=_digest(auth_path))


def test_scene_index_drift_fails_after_physical_hash_is_rebound(tmp_path):
    fixture = _fixture(tmp_path)
    index_path = fixture["root"] / SCENE_INDEX_PATH
    index = yaml.safe_load(index_path.read_text(encoding="utf-8"))
    index["files"].pop()
    _dump(index_path, index)
    _bind_current_file(fixture, "fresh_scene_index")
    _rewrite_release(fixture)
    with pytest.raises(release.R6I3ExecutionReleaseError, match="scene index child closure"):
        _validate(fixture)


def test_closure_logical_digest_drift_fails_after_file_hash_is_rebound(tmp_path):
    fixture = _fixture(tmp_path)
    closure_path = fixture["root"] / CLOSURE_PATH
    closure = yaml.safe_load(closure_path.read_text(encoding="utf-8"))
    closure["generator"] = "mutated_without_resealing_logical_digest"
    _dump(closure_path, closure)
    _bind_current_file(fixture, "execution_dependency_closure")
    _rewrite_release(fixture)
    with pytest.raises(
        release.R6I3ExecutionReleaseError,
        match="execution dependency closure logical digest drifted",
    ):
        _validate(fixture)


def test_machine_review_must_remain_nonexecuting_pass(tmp_path):
    fixture = _fixture(tmp_path)
    review_path = fixture["root"] / MACHINE_REVIEW_PATH
    review = yaml.safe_load(review_path.read_text(encoding="utf-8"))
    review["execution_absence_review"]["attempt_root_present"] = True
    _dump(review_path, review)
    _bind_current_file(fixture, "execution_machine_review")
    _rewrite_release(fixture)
    with pytest.raises(
        release.R6I3ExecutionReleaseError,
        match="execution absence review attempt_root_present drifted",
    ):
        _validate(fixture)


def test_validation_is_read_only_and_deterministic(tmp_path):
    fixture = _fixture(tmp_path)

    def tree_digest():
        rows = []
        for path in sorted(item for item in fixture["root"].rglob("*") if item.is_file()):
            rows.append((path.relative_to(fixture["root"]).as_posix(), _digest(path)))
        return rows

    before = tree_digest()
    first = _validate(fixture)
    second = _validate(fixture)
    assert first.schedule_sha256 == second.schedule_sha256
    assert first.execution_seeds == second.execution_seeds
    assert tree_digest() == before
