#!/usr/bin/env python3
"""Build the complete path+SHA dependency closure for P1-E1 offline review."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Set

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P1-E1"
P1_CLOSURE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "p1_r1_dependency_closure.yaml"
)
I5_CLOSURE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/"
    "execution_dependency_closure.yaml"
)
OUTPUT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/"
    "offline_readiness_dependency_closure.yaml"
)
REVIEW = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/"
    "p1_e1_offline_readiness_review.yaml"
)
RELEASE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p1_e1_execution_release.yaml"
)

ROOTS = (
    "config/thesis_experiments/v2/v2_04g_r6_i1_p1_e1_execution_contract.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_e1_simulation_authorization.yaml",
    "docs/thesis_experiment/CURRENT_V2_04G_P1_R1_OFFLINE_READINESS_HANDOFF.md",
    P1_CLOSURE.as_posix(),
    I5_CLOSURE.as_posix(),
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/v2_04g_p1_r1_exact_schedule.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_r1_method_bindings.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_r1_scene_derivation.yaml",
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p1_e1_execution.launch",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_episode.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_runtime_control.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_activation_probe_listener.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_fixed_snapshot.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_assess.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_exact_runner.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_dependency_generator.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_offline_reviewer.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_e1_release.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_e1_replay.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_statistics.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i1_p1_e1_offline.py",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/i5_evidence_freeze.sha256",
)


class P1ClosureError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise P1ClosureError(message)


def _sha(payload):
    return hashlib.sha256(payload).hexdigest()


def _canonical_sha(value):
    return _sha(json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8"))


def _load(path):
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root must be mapping: {}".format(path))
    return value


def _safe_relative(value):
    path = Path(value)
    _require(
        not path.is_absolute() and bool(path.parts)
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe relative dependency path: {}".format(value),
    )
    return path


def _local_receipt(workspace, relative):
    path = workspace / _safe_relative(relative)
    _require(path.is_file() and not path.is_symlink(), "local dependency absent: {}".format(relative))
    payload = path.read_bytes()
    return {"path": Path(relative).as_posix(), "sha256": _sha(payload), "size_bytes": len(payload)}


def _external_receipt(path_text):
    path = Path(path_text)
    _require(path.is_absolute(), "external dependency is not absolute")
    canonical = path.resolve(strict=True)
    _require(canonical.is_file(), "external dependency absent: {}".format(path))
    payload = canonical.read_bytes()
    return {"path": str(canonical), "sha256": _sha(payload), "size_bytes": len(payload)}


def _collect_i5_external_paths(document):
    paths = set()
    external = document["external"]
    interpreter = external.get("python_interpreter", {})
    if interpreter.get("canonical_path"):
        paths.add(interpreter["canonical_path"])
    for row in external.get("files", []):
        if row.get("canonical_path"):
            paths.add(row["canonical_path"])
    return paths


def build_closure(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(not (workspace / RELEASE).exists(), "release must be absent during closure build")
    p1 = _load(workspace / P1_CLOSURE)
    i5 = _load(workspace / I5_CLOSURE)
    local_paths: Set[str] = set(ROOTS)
    local_paths.update(row["path"] for row in p1["local_resources"])
    local_paths.update(row["path"] for row in i5["local"]["files"])
    external_paths = set()
    for provider in p1.get("external_providers", []):
        if provider.get("path"):
            external_paths.add(provider["path"])
    external_paths.update(_collect_i5_external_paths(i5))
    for absolute in list(external_paths):
        path = Path(absolute).resolve(strict=True)
        try:
            relative = path.relative_to(workspace)
        except ValueError:
            continue
        local_paths.add(relative.as_posix())
        external_paths.remove(absolute)
    local_paths.discard(OUTPUT.as_posix())
    local_paths.discard(REVIEW.as_posix())
    local_paths.discard(RELEASE.as_posix())
    local = [_local_receipt(workspace, value) for value in sorted(local_paths)]
    external = [_external_receipt(value) for value in sorted(external_paths)]
    schedule = _load(workspace / Path(ROOTS[5]))
    document = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_execution_dependency_closure_release_absent",
        "simulation_only": True,
        "execution_authorized": True,
        "authorized_evidence_units": 270,
        "evidence_units_consumed": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "i5_rerun_or_resume_forbidden": True,
        "roots": list(ROOTS),
        "local_resources": local,
        "local_resource_count": len(local),
        "external_resources": external,
        "external_resource_count": len(external),
        "inherited_p1_r1_local_resource_count": len(p1["local_resources"]),
        "inherited_i5_local_resource_count": len(i5["local"]["files"]),
        "p1_compiled_child_count": 188,
        "schedule_episode_count": len(schedule["episodes"]),
        "schedule_episode_digest": schedule["canonical_episode_digest"],
        "unresolved": [],
        "hash_graph_boundary": [
            "closure_excludes_itself",
            "offline_review_artifact_excluded_and_release_binds_it",
            "unique_release_excluded_and_absent_during_review",
            "attempts_journals_consumption_and_terminal_results_excluded_and_absent",
        ],
    }
    document["logical_sha256"] = _canonical_sha(document)
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    document = build_closure(WORKSPACE)
    payload = yaml.safe_dump(document, sort_keys=False, allow_unicode=True).encode("utf-8")
    if args.write:
        target = WORKSPACE / OUTPUT
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            raise P1ClosureError("closure already exists; remove only through a new reviewed stage")
        target.write_bytes(payload)
    else:
        print(payload.decode("utf-8"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
