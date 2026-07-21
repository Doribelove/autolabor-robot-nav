import copy
import hashlib
from pathlib import Path

import pytest

from thesis_experiment.v2_04g_r6_i1_r6_i2_dependency import (
    ENTRYPOINTS,
    R6I2DependencyError,
    build_dependency_closure,
    build_external_dependency_closure,
    canonical_file_record,
    resolve_runtime_binding,
    verify_external_files,
)


WORKSPACE = Path(__file__).resolve().parents[5]
I1_CLOSURE = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml"
)


def _i1_external_names():
    import yaml

    document = yaml.safe_load(I1_CLOSURE.read_text(encoding="utf-8"))
    return (
        document["external_python_modules"],
        document["external_runtime_bindings"],
    )


def test_all_i1_external_python_names_close_to_canonical_files():
    python_names, _ = _i1_external_names()
    external = build_external_dependency_closure(
        WORKSPACE, python_names, []
    )
    result = verify_external_files(external)
    assert result["python_binding_count"] == 39
    assert result["runtime_binding_count"] == 0
    assert result["external_file_count"] > 39
    assert external["unresolved"] == []
    for row in external["python_bindings"]:
        assert row["canonical_paths"]
        assert all(Path(path).is_absolute() for path in row["canonical_paths"])


def test_runtime_binary_and_launch_bindings_include_exact_target_sha():
    bindings = (
        "$(find gazebo_ros)/launch/empty_world.launch",
        "node:gazebo_ros:spawn_model",
        "node:move_base:move_base",
        "node:robot_state_publisher:robot_state_publisher",
        "package-executable:xacro:xacro",
    )
    external = build_external_dependency_closure(WORKSPACE, [], bindings)
    result = verify_external_files(external)
    assert result["runtime_binding_count"] == 5
    records = {
        row["canonical_path"]: row
        for row in external["files"]
    }
    for binding in external["runtime_bindings"]:
        target = binding["target_canonical_path"]
        assert target in records
        assert records[target]["sha256"] == hashlib.sha256(
            Path(target).read_bytes()
        ).hexdigest()


def test_frozen_i1_xacro_substitution_is_rejected_as_unresolved():
    with pytest.raises(
        R6I2DependencyError,
        match="launch substitution is unresolved",
    ):
        resolve_runtime_binding(
            WORKSPACE, "$(find xacro)/xacro"
        )


def test_external_verifier_rejects_hash_drift(tmp_path):
    dependency = tmp_path / "external.bin"
    dependency.write_bytes(b"before")
    record = canonical_file_record(dependency)
    interpreter = canonical_file_record("/usr/bin/python3")
    document = {
        "python_interpreter": interpreter,
        "python_bindings": [{
            "binding": "synthetic",
            "resolution_kind": "python_module_file",
            "module_origin": record["canonical_path"],
            "canonical_paths": [record["canonical_path"]],
        }],
        "runtime_bindings": [],
        "files": sorted(
            [interpreter, record],
            key=lambda row: row["canonical_path"],
        ),
        "unresolved": [],
    }
    import json

    document["closure_sha256"] = hashlib.sha256(
        json.dumps(
            document, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    dependency.write_bytes(b"after")
    with pytest.raises(R6I2DependencyError, match="drifted"):
        verify_external_files(document)


def test_external_verifier_rejects_unbound_binding_path():
    python_names, _ = _i1_external_names()
    external = build_external_dependency_closure(
        WORKSPACE, python_names[:1], []
    )
    malformed = copy.deepcopy(external)
    malformed["python_bindings"][0]["canonical_paths"].append(
        "/not/a/recorded/dependency"
    )
    malformed["python_bindings"][0]["canonical_paths"].sort()
    with pytest.raises(
        R6I2DependencyError, match="unrecorded file"
    ):
        verify_external_files(malformed)


def test_review_closure_source_declares_no_authorization():
    source = (
        WORKSPACE
        / "src/tools/thesis_experiment/src/thesis_experiment/"
          "v2_04g_r6_i1_r6_i2_dependency.py"
    ).read_text(encoding="utf-8")
    assert "authorization_resources" in source
    assert '"authorization_resources": []' in source
    assert "seed_or_evidence_units_allocated" in source


def test_complete_default_closure_covers_contract_and_i1_inheritance():
    import yaml

    closure = build_dependency_closure(WORKSPACE)
    local = closure["local"]
    external = closure["external"]
    contract_path = WORKSPACE / (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml"
    )
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    contract_resources = {
        row["path"] for row in contract["resources"].values()
    }
    local_paths = set(local["required_paths"])
    inherited_python, _ = _i1_external_names()

    assert local["entrypoints"] == list(ENTRYPOINTS)
    assert contract_resources.issubset(local_paths)
    assert set(inherited_python).issubset(
        set(local["external_python_names"])
    )
    assert len(inherited_python) == 39
    assert {
        "__future__",
        "hmac",
        "pytest",
        "shutil",
        "urllib",
    }.issubset(set(local["external_python_names"]))
    assert len(local["external_python_names"]) > 39
    assert local["external_runtime_names"] == [
        "$(find gazebo_ros)/launch/empty_world.launch",
        "node:gazebo_ros:spawn_model",
        "node:move_base:move_base",
        "node:robot_state_publisher:robot_state_publisher",
    ]
    assert [row["binding"] for row in external["runtime_bindings"]] == [
        "$(find gazebo_ros)/launch/empty_world.launch",
        "node:gazebo_ros:spawn_model",
        "node:move_base:move_base",
        "node:robot_state_publisher:robot_state_publisher",
        "package-executable:xacro:xacro",
    ]
    compiled_children = {
        path
        for path in local_paths
        if "/compiled_scenes/" in path
        and not path.endswith("compiled_scene_index.yaml")
    }
    assert len(compiled_children) == 14
    assert (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i2_repair_review/"
        "v2_04g_r6_i2_authorization_assessment_review.yaml"
    ) in local_paths
    assert not any("HANDOFF" in path for path in local_paths)
    assert not any(
        path.endswith("v2_04g_r6_i2_integration_review.yaml")
        for path in local_paths
    )
    assert not any(
        "v2_04g_r6_i2" in Path(path).name
        and "authorization" in Path(path).name
        and path.startswith("experiments/manifests/")
        for path in local_paths
    )
    assert closure["execution_authorized"] is False
    assert closure["authorization_resources"] == []
    assert closure["seed_or_evidence_units_allocated"] == 0
    assert closure["seed_or_evidence_units_consumed"] == 0
