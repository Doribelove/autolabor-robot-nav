#!/usr/bin/env python3
"""Generate/check the offline P1-R1 path+SHA dependency closure.

No execution, ROS, network, authorization, attempt, or journal capability is
present.  The closure is a pre-release offline snapshot and therefore excludes
itself and the final independent review from its own hash DAG.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, Mapping, Set

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness"
)
CLOSURE_RELATIVE = OUTPUT_ROOT / "p1_r1_dependency_closure.yaml"
FINAL_REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_offline_review.yaml"

DIRECT_ROOTS = (
    "config/thesis_experiments/v2/v2_04g_r6_i1_p1_r1_offline_readiness_contract.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_r1_scene_derivation.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_r1_method_bindings.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i6_future_multiseed_performance_design.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/v2_04g_r6_i6_result_interpretation_review.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/i5_evidence_freeze.sha256",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/execution_dependency_closure.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/v2_04g_r6_i5_execution_report.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/v2_04g_r6_i5_stage_report.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/v2_04g_r6_i5_scenes.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/compiled_scenes/compiled_scene_index.yaml",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_statistics.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_builder.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_dependency_generator.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_reviewer.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i1_p1_statistics.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i1_p1_offline_builder.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i1_p1_offline_reviewer.py",
    "src/simulation/m2_gazebo/config/fixed_teb.yaml",
    "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch",
    "src/simulation/m2_gazebo/config/simulation_candidates.yaml",
    "config/thesis_experiments/v2/simulation_contract.yaml",
    "src/tools/thesis_experiment/scripts/v2_supervisor_repair_episode.py",
)


class P1DependencyError(ValueError):
    pass


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1DependencyError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _safe_relative(value: Any) -> Path:
    _require(type(value) is str and value, "dependency path must be a string")
    path = Path(value)
    _require(
        not path.is_absolute()
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe dependency path: {}".format(value),
    )
    return path


def _snapshot(workspace: Path, relative: Path) -> Dict[str, Any]:
    path = workspace / relative
    _require(path.is_file() and not path.is_symlink(), "regular non-symlink required: {}".format(relative))
    payload = path.read_bytes()
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _walk_bindings(value: Any) -> Iterable[Mapping[str, Any]]:
    if isinstance(value, dict):
        if set(value) >= {"path", "sha256"} and isinstance(value["path"], str):
            yield value
        for child in value.values():
            yield from _walk_bindings(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_bindings(child)


def _add_historical_graph(
    workspace: Path,
    relative: Path,
    resources: Dict[str, Dict[str, Any]],
    visited_documents: Set[str],
) -> None:
    key = relative.as_posix()
    if key in visited_documents:
        return
    visited_documents.add(key)
    snapshot = _snapshot(workspace, relative)
    resources[key] = snapshot
    if relative.suffix not in {".yaml", ".yml"}:
        return
    try:
        document = yaml.safe_load((workspace / relative).read_text(encoding="utf-8"))
    except (UnicodeError, yaml.YAMLError):
        return
    for binding in _walk_bindings(document):
        try:
            child = _safe_relative(binding["path"])
        except P1DependencyError:
            continue
        child_path = workspace / child
        if not child_path.is_file() or child_path.is_symlink():
            continue
        actual = _snapshot(workspace, child)
        _require(actual["sha256"] == binding["sha256"], "historical binding drift: {}".format(child))
        resources[child.as_posix()] = actual
        if "dependency_closure" in child.name or child.name == "execution_dependency_closure.yaml":
            _add_historical_graph(workspace, child, resources, visited_documents)


def _logical_sha(document: Mapping[str, Any]) -> str:
    payload = json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def build_closure(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    resources: Dict[str, Dict[str, Any]] = {}
    for item in DIRECT_ROOTS:
        relative = _safe_relative(item)
        resources[item] = _snapshot(workspace, relative)
    for path in sorted((workspace / OUTPUT_ROOT).rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(workspace)
        if relative in {CLOSURE_RELATIVE, FINAL_REVIEW_RELATIVE}:
            continue
        resources[relative.as_posix()] = _snapshot(workspace, relative)

    historical_resources: Dict[str, Dict[str, Any]] = {}
    _add_historical_graph(
        workspace,
        Path("artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/execution_dependency_closure.yaml"),
        historical_resources,
        set(),
    )
    resources.update(historical_resources)
    ordered = [resources[key] for key in sorted(resources)]

    python_real = Path(sys.executable).resolve(strict=True)
    pyyaml_real = Path(yaml.__file__).resolve(strict=True)
    external = [
        {
            "provider": "python_interpreter",
            "path": str(python_real),
            "sha256": _sha(python_real.read_bytes()),
            "version": sys.version.split()[0],
        },
        {
            "provider": "pyyaml",
            "path": str(pyyaml_real),
            "sha256": _sha(pyyaml_real.read_bytes()),
            "version": yaml.__version__,
        },
        {
            "provider": "python_standard_library_statistics_stack",
            "modules": ["hashlib", "json", "math"],
            "third_party_statistics_dependency": False,
        },
    ]
    document = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P1-R1",
        "status": "offline_dependency_closure_release_absent",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "evidence_units_consumed": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "roots": list(DIRECT_ROOTS),
        "local_resources": ordered,
        "local_resource_count": len(ordered),
        "historical_transitive_resource_count": len(historical_resources),
        "p1_compiled_child_count": 188,
        "p1_schedule_episode_count": 270,
        "external_providers": external,
        "unresolved": [],
        "hash_graph_boundary": {
            "closure_excludes_itself": True,
            "final_review_excluded": True,
            "future_release_excluded_and_absent": True,
            "future_attempts_and_journals_excluded_and_absent": True,
        },
        "fresh_seed_lifecycle": {
            "reserved": list(range(5201, 5295)),
            "materialized_evidence": list(range(5201, 5291)),
            "materialized_compile_support": list(range(5291, 5295)),
            "authorized": [],
            "consumed": [],
        },
        "logical_sha256": "",
    }
    logical_input = dict(document)
    logical_input.pop("logical_sha256")
    document["logical_sha256"] = _logical_sha(logical_input)
    return document


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    document = build_closure(workspace)
    payload = _yaml_bytes(document)
    target = workspace / CLOSURE_RELATIVE
    if args.materialize:
        _require(not target.exists(), "closure already exists")
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file(), "closure is absent")
        _require(target.read_bytes() == payload, "closure bytes differ")
    print("P1-R1 dependency closure {}: {} local resources".format(
        "materialized" if args.materialize else "verified",
        document["local_resource_count"],
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

