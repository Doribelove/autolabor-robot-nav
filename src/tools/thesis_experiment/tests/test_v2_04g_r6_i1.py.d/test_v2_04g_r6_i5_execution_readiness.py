"""Directed offline tests for R6-I5 execution readiness and prejournal order."""

import ast
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import types

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
RUNNER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_bounded_validation.py"
)
VALIDATOR_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release.py"
)
VALIDATOR_TEST_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_release_validator.py"
)
DEPENDENCY_CLOSURE_PATH = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/"
    "execution_dependency_closure.yaml"
)
MACHINE_REVIEW_PATH = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/"
    "v2_04g_r6_i5_execution_readiness_review.yaml"
)
REVIEWER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_"
    "execution_readiness_reviewer.py"
)
AUTHORIZATION_PATH = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
RELEASE_PATH = WORKSPACE / (
    "experiments/manifests/v2/integration/v2_04g_r6_i5_execution_release.yaml"
)
FAILED_I3_RELEASE_PATH = WORKSPACE / (
    "experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml"
)
EXPECTED_AUTHORIZATION_SHA256 = (
    "bc59820b0140b50503657966d735511a8007d9ec8e14f3f2cf237791ff170592"
)
EXPECTED_SCHEDULE_SHA256 = (
    "b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402"
)


def _load(path, name):
    specification = importlib.util.spec_from_file_location(name, path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def _load_runner(name):
    return _load(RUNNER_PATH, name)


def _load_thesis_module(module_name):
    package = sys.modules.get("thesis_experiment")
    if package is None:
        package = types.ModuleType("thesis_experiment")
        package.__package__ = "thesis_experiment"
        package.__path__ = [
            str(
                WORKSPACE
                / "src/tools/thesis_experiment/src/thesis_experiment"
            )
        ]
        sys.modules["thesis_experiment"] = package
    return importlib.import_module("thesis_experiment." + module_name)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _function(tree, name):
    return next(
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    )


def _call_name(node):
    function = node.func
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ""


def test_runner_ast_places_all_mutation_after_complete_prejournal():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(RUNNER_PATH))
    imports = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "").split(".")[0])
    assert not (
        set(imports)
        & {
            "actionlib",
            "dynamic_reconfigure",
            "gazebo_msgs",
            "rosgraph",
            "roslaunch",
            "rospy",
            "subprocess",
            "thesis_experiment",
            "yaml",
        }
    )

    preflight = _function(tree, "_execution_preflight")
    unit_gate = _function(tree, "_prejournal_unit_gate")
    preflight_calls = {
        _call_name(node)
        for function in (preflight, unit_gate)
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
    }
    assert not (
        preflight_calls
        & {
            "mkdir",
            "Popen",
            "_subprocess_module",
            "_spawn_sanitized",
            "_capture_command",
            "_run_command",
            "_atomic_yaml",
            "_exclusive_bytes",
        }
    )
    assert not any(
        isinstance(node, (ast.Import, ast.ImportFrom))
        for function in (preflight, unit_gate)
        for node in ast.walk(function)
    )

    execute = _function(tree, "_execute_validated")
    statements = [
        node
        for node in execute.body
        if not (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        )
    ]
    assert isinstance(statements[0], ast.Assign)
    assert isinstance(statements[0].value, ast.Call)
    assert _call_name(statements[0].value) == "_execution_preflight"
    mutation_calls = [
        node
        for node in ast.walk(execute)
        if isinstance(node, ast.Call) and _call_name(node) == "mkdir"
    ]
    assert mutation_calls
    assert statements[0].value.lineno < min(node.lineno for node in mutation_calls)


def test_complete_simulated_prejournal_has_no_mkdir_popen_or_ros_import(
    monkeypatch,
):
    runner = _load_runner("r6_i5_complete_prejournal_barrier_test")
    mutation = {"mkdir": 0, "popen": 0}
    loaded_names = []

    class FakeSnapshot:
        def __init__(self, **values):
            self.__dict__.update(values)

    validation = types.SimpleNamespace(
        preregistration=types.SimpleNamespace(
            document={"schedule": runner.EXPECTED_SCHEDULE}
        ),
        schedule_sha256=runner.EXPECTED_SCHEDULE_SHA256,
        identity_count=6,
        execution_seeds=(5161, 5162, 5163),
        runtime_executables={},
        bound_resources={
            "fresh_scene_index": types.SimpleNamespace(sha256="2" * 64),
        },
    )
    seed_by_scene = {
        row["scene_id"]: row["seed"] for row in runner.EXPECTED_SCHEDULE
    }

    class FakeIntegrity:
        @staticmethod
        def acquire_compiled_scene_lease(workspace, index, digest, scene_id):
            assert workspace == runner.WORKSPACE
            assert index == runner.COMPILED_INDEX_RELATIVE
            assert digest == "2" * 64
            return types.SimpleNamespace(
                instance_bytes=json.dumps(
                    {
                        "scene": {
                            "scene_id": scene_id,
                            "seed": seed_by_scene[scene_id],
                            "family": "DYNAMIC",
                        }
                    }
                ).encode("utf-8")
            )

    release_module = types.SimpleNamespace(
        FileSnapshot=FakeSnapshot,
        load_and_validate_execution_release=lambda *args, **kwargs: validation,
        _parse_yaml_mapping=lambda payload, label: json.loads(payload.decode("utf-8")),
    )
    assessment_module = types.SimpleNamespace(stage="V2-04G-R6-I5")
    snapshots = {
        relative: {
            "relative": relative.as_posix(),
            "path": str(WORKSPACE / relative),
            "sha256": "1" * 64,
            "payload": b"pass\n",
        }
        for relative in runner.TRUSTED_MODULE_SHA256
    }

    def fake_load(snapshot, name):
        del snapshot
        loaded_names.append(name)
        if name.endswith("r6_i5_release"):
            return release_module
        if name.endswith("r6_integrity"):
            return FakeIntegrity
        if name.endswith("r6_i5_assessment"):
            return assessment_module
        return types.SimpleNamespace()

    def forbidden_mkdir(*args, **kwargs):
        del args, kwargs
        mutation["mkdir"] += 1
        raise AssertionError("prejournal attempted mkdir")

    def forbidden_subprocess():
        mutation["popen"] += 1
        raise AssertionError("prejournal attempted subprocess import")

    expected_runtime_state = runner._expected_runtime_state(True)
    monkeypatch.setattr(runner, "_trusted_module_snapshots", lambda: snapshots)
    monkeypatch.setattr(runner, "_load_verified_module", fake_load)
    monkeypatch.setattr(
        runner, "_runtime_executable", lambda validation, binding: "/bin/true"
    )
    checked_profiles = []

    def fake_runtime_profile(row, context):
        del context
        checked_profiles.append(row["profile_id"])
        return {"profile_id": row["profile_id"]}

    monkeypatch.setattr(runner, "_runtime_profile", fake_runtime_profile)
    monkeypatch.setattr(runner, "_process_matches", lambda: [])
    monkeypatch.setattr(
        runner,
        "_runtime_state_present",
        lambda: expected_runtime_state,
    )
    monkeypatch.setattr(runner.Path, "mkdir", forbidden_mkdir)
    monkeypatch.setattr(runner, "_subprocess_module", forbidden_subprocess)

    result = runner._execution_preflight(
        runner.RELEASE_RELATIVE.as_posix(),
        "0" * 64,
        runner.AUTHORIZATION_RELATIVE.as_posix(),
        runner.EXPECTED_AUTHORIZATION_SHA256,
    )
    assert result["validation"] is validation
    assert result["assessment"] is assessment_module
    assert mutation == {"mkdir": 0, "popen": 0}
    assert checked_profiles == [
        "r6_semantics_legacy_control",
        "r6_semantics_circle_contact",
    ]
    assert loaded_names == [
        "thesis_experiment."
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release",
        "thesis_experiment.v2_04g_r6_i1_r6_i2_bootstrap",
        "thesis_experiment.v2_04g_r6_integrity",
        "thesis_experiment."
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_assessment",
    ]
    assert not any(
        name.split(".")[-1]
        in {
            "actionlib",
            "dynamic_reconfigure",
            "gazebo_msgs",
            "roslaunch",
            "rospy",
        }
        for name in loaded_names
    )


def test_failed_prejournal_cannot_reach_execution_mutation(monkeypatch):
    runner = _load_runner("r6_i5_failed_prejournal_barrier_test")
    mutation = {"mkdir": 0, "popen": 0}

    class SentinelError(RuntimeError):
        pass

    def fail_preflight(*args, **kwargs):
        del args, kwargs
        raise SentinelError("synthetic complete prejournal failure")

    def forbidden_mkdir(*args, **kwargs):
        del args, kwargs
        mutation["mkdir"] += 1
        raise AssertionError("failed prejournal reached mkdir")

    def forbidden_subprocess():
        mutation["popen"] += 1
        raise AssertionError("failed prejournal reached subprocess")

    monkeypatch.setattr(runner, "_execution_preflight", fail_preflight)
    monkeypatch.setattr(runner.Path, "mkdir", forbidden_mkdir)
    monkeypatch.setattr(runner, "_subprocess_module", forbidden_subprocess)
    with pytest.raises(SentinelError, match="complete prejournal failure"):
        runner._execute_validated("release", "0" * 64, "authorization", "1" * 64)
    assert mutation == {"mkdir": 0, "popen": 0}


class _FakeCmdline:
    def __init__(self, payload=None, error=None):
        self.payload = payload
        self.error = error

    def read_bytes(self):
        if self.error is not None:
            raise self.error
        return self.payload


class _FakeProcEntry:
    def __init__(self, pid, payload=None, error=None):
        self.name = str(pid)
        self.payload = payload
        self.error = error

    def __truediv__(self, name):
        assert name == "cmdline"
        return _FakeCmdline(self.payload, self.error)


def test_process_scan_excludes_only_self(monkeypatch):
    runner = _load_runner("r6_i5_process_self_only_test")
    current = os.getpid()
    other = current + 100000
    entries = [
        _FakeProcEntry(current, b"roslaunch\0self.launch"),
        _FakeProcEntry(other, b"roslaunch\0other.launch"),
        _FakeProcEntry(other + 1, b"python3\0ordinary.py"),
    ]

    def fake_iterdir(path):
        assert path.as_posix() == "/proc"
        return iter(entries)

    monkeypatch.setattr(runner.Path, "iterdir", fake_iterdir)
    matches = runner._process_matches()
    assert [row["pid"] for row in matches] == [other]
    source = RUNNER_PATH.read_text(encoding="utf-8")
    process_source = ast.get_source_segment(
        source, _function(ast.parse(source), "_process_matches")
    )
    assert "os.getpid()" in process_source
    assert "getppid" not in process_source
    assert "ancestor" not in process_source.lower()


def test_proc_enumeration_failure_is_fail_closed(monkeypatch):
    runner = _load_runner("r6_i5_proc_enumeration_failure_test")

    def denied(path):
        if path.as_posix() == "/proc":
            raise PermissionError("synthetic /proc denial")
        return iter(())

    monkeypatch.setattr(runner.Path, "iterdir", denied)
    with pytest.raises(
        runner.R6I5ExecutionError,
        match="cannot enumerate /proc for host isolation",
    ):
        runner._process_matches()


def test_proc_entry_nontransient_failure_is_fail_closed(monkeypatch):
    runner = _load_runner("r6_i5_proc_entry_failure_test")
    other = os.getpid() + 100000
    monkeypatch.setattr(
        runner.Path,
        "iterdir",
        lambda path: iter(
            [_FakeProcEntry(other, error=PermissionError("synthetic denial"))]
        ),
    )
    with pytest.raises(
        runner.R6I5ExecutionError,
        match="cannot inspect process",
    ):
        runner._process_matches()


def test_runner_trusted_hashes_match_exact_sources_and_authority():
    runner = _load_runner("r6_i5_runner_trusted_hash_test")
    validator = _load(VALIDATOR_PATH, "r6_i5_validator_path_parity_test")
    assert _sha256(AUTHORIZATION_PATH) == EXPECTED_AUTHORIZATION_SHA256
    assert runner.EXPECTED_AUTHORIZATION_SHA256 == EXPECTED_AUTHORIZATION_SHA256
    assert runner.EXPECTED_SCHEDULE_SHA256 == EXPECTED_SCHEDULE_SHA256
    assert _sha256(VALIDATOR_PATH) == runner.EXPECTED_RELEASE_VALIDATOR_SHA256
    assert (
        _sha256(VALIDATOR_TEST_PATH)
        == runner.EXPECTED_RELEASE_VALIDATOR_TEST_SHA256
    )
    assert (
        runner.TRUSTED_MODULE_SHA256[runner.RELEASE_VALIDATOR_RELATIVE]
        == runner.EXPECTED_RELEASE_VALIDATOR_SHA256
    )
    assert (
        runner.TRUSTED_MODULE_SHA256[runner.RELEASE_VALIDATOR_TEST_RELATIVE]
        == runner.EXPECTED_RELEASE_VALIDATOR_TEST_SHA256
    )
    for relative, expected in runner.TRUSTED_MODULE_SHA256.items():
        assert len(expected) == 64
        assert set(expected) <= set("0123456789abcdef")
        assert _sha256(WORKSPACE / relative) == expected
    assert runner.EXPECTED_RELEASE_RESOURCE_PATHS == validator.EXPECTED_RELEASE_RESOURCE_PATHS
    assert runner.EXPECTED_MACHINE_REVIEW_STATUS == (
        "r6_i5_execution_readiness_closure_pass_release_absent"
    )


def test_authority_schedule_roster_and_historical_nonreuse_are_exact():
    authorization = yaml.safe_load(AUTHORIZATION_PATH.read_text(encoding="utf-8"))
    preregistration = yaml.safe_load(
        (WORKSPACE / authorization["bound_resources"]["preregistration"]["path"])
        .read_text(encoding="utf-8")
    )
    schedule = preregistration["schedule"]
    digest = hashlib.sha256(
        json.dumps(schedule, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    assert digest == EXPECTED_SCHEDULE_SHA256
    assert authorization["exact_schedule"] == schedule
    assert authorization["execution_authorized"] is True
    assert authorization["attempt_limit_per_identity"] == 1
    assert authorization["retry_or_resume_allowed"] is False
    assert authorization["seed_replacement_allowed"] is False
    assert authorization["budget_expansion_allowed"] is False
    assert list(authorization["bound_resources"]) == [
        "contract",
        "preregistration",
        "i4_validator",
        "inherited_i4_dependency_closure",
        "i4_machine_review",
        "failed_i3_release",
        "frozen_evaluator",
        "legacy_supervisor",
        "aligned_supervisor",
        "source_i1_scene_manifest",
        "source_i1_compiled_scene_index",
        "r6_design_report",
    ]
    assert _sha256(FAILED_I3_RELEASE_PATH) == (
        "5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6"
    )
    assert not RELEASE_PATH.exists()


@pytest.mark.parametrize(
    "relative",
    [
        "attempts",
        "journals",
        "receipts",
        "raw_evidence",
        "semantic_evidence",
        "ros_home",
        "ros_logs",
        "v2_04g_r6_i5_stage_report.yaml",
        "v2_04g_r6_i5_execution_report.yaml",
    ],
)
def test_each_i5_execution_state_path_fails_dependency_gate(monkeypatch, relative):
    dependency = _load_thesis_module(
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency"
    )

    target = (dependency.ARTIFACT_ROOT / relative).as_posix()
    original = dependency.os.path.lexists

    def fake_lexists(path):
        try:
            workspace_relative = Path(path).relative_to(WORKSPACE).as_posix()
        except ValueError:
            return original(path)
        if workspace_relative == target:
            return True
        return original(path)

    monkeypatch.setattr(dependency.os.path, "lexists", fake_lexists)
    with pytest.raises(
        dependency.R6I5DependencyError,
        match="forbidden execution state exists",
    ):
        dependency._verify_state_boundary(WORKSPACE)


def test_dependency_closure_rebuild_is_deterministic_and_process_free(monkeypatch):
    dependency = _load_thesis_module(
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency"
    )
    i2_dependency = _load_thesis_module(
        "v2_04g_r6_i1_r6_i2_dependency"
    )

    def forbidden_subprocess(*args, **kwargs):
        del args, kwargs
        raise AssertionError("I5 dependency closure attempted a subprocess")

    monkeypatch.setattr(i2_dependency.subprocess, "run", forbidden_subprocess)
    first = dependency.build_dependency_closure(WORKSPACE)
    second = dependency.build_dependency_closure(WORKSPACE)
    assert first == second
    assert first["execution_authorized"] is False
    assert first["execution_ready"] is False
    assert first["execution_release_present"] is False
    assert first["execution_seeds"] == [5161, 5162, 5163]
    assert first["compile_support_only_seeds"] == [5164, 5165, 5166, 5167]
    assert first["exact_schedule_sha256"] == EXPECTED_SCHEDULE_SHA256
    assert first["authorization_resource_audit"]["resource_count"] == 12
    assert first["authorization_resource_audit"]["parsed_count"] == 2
    assert first["authorization_resource_audit"]["hash_only_count"] == 10
    assert first["fresh_scene_revalidation"]["compiled_child_count"] == 14
    assert first["inherited_i4_revalidation"]["local_file_count"] == 54
    assert first["inherited_i4_revalidation"]["external_file_count"] == 307
    assert first["inherited_i4_revalidation"]["external_runtime_binding_count"] == 9
    assert len(first["external"]["files"]) == 313
    assert len(first["external"]["python_bindings"]) == 49
    assert len(first["external"]["runtime_bindings"]) == 9
    assert first["i5_external_extension_audit"] == {
        "inherited_external_file_count": 307,
        "inherited_python_binding_count": 47,
        "inherited_runtime_binding_count": 9,
        "additional_python_bindings": ["ctypes", "secrets"],
        "workspace_local_script_bindings": [
            {
                "binding": (
                    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_"
                    "scene_materializer"
                ),
                "from": dependency.I5_SCENE_REVIEWER,
                "to": dependency.I5_SCENE_MATERIALIZER,
            }
        ],
        "final_external_file_count": len(first["external"]["files"]),
        "final_python_binding_count": 49,
        "final_runtime_binding_count": 9,
        "final_external_closure_sha256": first["external"][
            "closure_sha256"
        ],
        "additional_bindings_resolved_without_import": True,
        "all_external_files_mechanically_rehashed": True,
        "pass": True,
    }
    assert first["unresolved"] == []
    local_paths = {row["path"] for row in first["local"]["files"]}
    assert dependency.I5_DEPENDENCY in local_paths
    assert dependency.I5_REVIEWER in local_paths
    assert dependency.I5_READINESS_TEST in local_paths
    assert dependency.I5_SCENE_MATERIALIZER in local_paths
    assert dependency.I5_RELEASE_VALIDATOR in local_paths
    assert dependency.I5_ASSESSOR in local_paths
    assert dependency.I5_ASSESSOR_TEST in local_paths
    assert dependency.EXECUTION_CLOSURE.as_posix() not in local_paths
    assert dependency.MACHINE_REVIEW.as_posix() not in local_paths
    assert dependency.FUTURE_I5_RELEASE.as_posix() not in local_paths
    assert first["hash_graph_boundary"] == {
        "closure_self_included": False,
        "final_machine_review_artifact_included": False,
        "future_i5_release_included": False,
        "failed_i3_release_included": True,
        "i4_validator_closure_and_review_included": True,
        "future_release_must_bind_closure_and_review": True,
    }


def test_reviewer_ast_has_no_ros_or_subprocess_and_build_is_read_only():
    source = REVIEWER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(REVIEWER_PATH))
    imports = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "").split(".")[0])
    assert not (
        set(imports)
        & {
            "actionlib",
            "dynamic_reconfigure",
            "gazebo_msgs",
            "roslaunch",
            "rospy",
            "subprocess",
        }
    )
    build = _function(tree, "build_review")
    calls = {_call_name(node) for node in ast.walk(build) if isinstance(node, ast.Call)}
    assert not calls & {"mkdir", "Popen", "_atomic_yaml"}


def test_persisted_closure_and_machine_review_rebuild_exactly():
    dependency = _load_thesis_module(
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency"
    )

    assert DEPENDENCY_CLOSURE_PATH.is_file()
    assert MACHINE_REVIEW_PATH.is_file()
    assert not RELEASE_PATH.exists()
    closure = yaml.safe_load(DEPENDENCY_CLOSURE_PATH.read_text(encoding="utf-8"))
    receipt = dependency.verify_dependency_closure(WORKSPACE, closure)
    assert receipt["pass"] is True
    assert receipt["compiled_scene_child_count"] == 14
    assert receipt["external_file_count"] == 313
    assert receipt["external_python_binding_count"] == 49
    assert receipt["external_runtime_binding_count"] == 9
    reviewer = _load(REVIEWER_PATH, "r6_i5_persisted_machine_review_test")
    first = reviewer.build_review(WORKSPACE)
    second = reviewer.build_review(WORKSPACE)
    assert first == second
    persisted = yaml.safe_load(MACHINE_REVIEW_PATH.read_text(encoding="utf-8"))
    assert persisted == first
    assert first["status"] == (
        "r6_i5_execution_readiness_closure_pass_release_absent"
    )
    assert first["execution_authorized"] is False
    assert first["execution_ready"] is False
    assert first["future_i5_release"]["present"] is False


def test_actual_machine_review_builder_passes_release_validator_schema():
    assert DEPENDENCY_CLOSURE_PATH.is_file()
    assert not RELEASE_PATH.exists()
    reviewer = _load(REVIEWER_PATH, "r6_i5_machine_review_cross_feed_test")
    validator = _load(VALIDATOR_PATH, "r6_i5_machine_review_validator_test")
    review = reviewer.build_review(WORKSPACE)
    validator._validate_machine_review(
        review,
        "r6_i5_execution_readiness_closure_pass_release_absent",
    )
    assert review["release_validator_cross_feed"] == {
        "validator_path": reviewer.I5_RELEASE_VALIDATOR,
        "expected_machine_review_status": (
            "r6_i5_execution_readiness_closure_pass_release_absent"
        ),
        "actual_build_review_schema_validated": True,
    }
