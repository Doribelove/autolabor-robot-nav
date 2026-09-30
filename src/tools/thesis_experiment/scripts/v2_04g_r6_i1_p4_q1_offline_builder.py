#!/usr/bin/env python3
"""Build strict runtime-schema P4 Anchor Bank candidates offline."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P4-Q1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p4_q1_offline_runtime_bank_readiness_contract.yaml"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p4_q1_offline_builder.py"
)
SOURCE_METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_q1_runtime_bank_readiness"
)
CANDIDATE_ROOT = OUTPUT_ROOT / "candidate_configs"
METHODS_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_method_bindings.yaml"
AUDIT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_construction_audit.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p4_q1_source_freeze.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_readiness_report.yaml"


class P4Q1BuildError(ValueError):
    """Raised when the single schema-promotion factor differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4Q1BuildError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_sha(value: Any) -> str:
    return _sha(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    )


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _receipt(relative: Path, payload: bytes | None = None) -> Dict[str, Any]:
    if payload is None:
        payload = (WORKSPACE / relative).read_bytes()
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _leaf_differences(left: Any, right: Any, prefix: str = "") -> list[str]:
    if type(left) is not type(right):
        return [prefix]
    if isinstance(left, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in left or key not in right:
                result.append(child)
            else:
                result.extend(_leaf_differences(left[key], right[key], child))
        return result
    if isinstance(left, list):
        if len(left) != len(right):
            return [prefix]
        result = []
        for index, pair in enumerate(zip(left, right)):
            result.extend(
                _leaf_differences(pair[0], pair[1], "{}[{}]".format(prefix, index))
            )
        return result
    return [] if left == right else [prefix]


def _load_anchor_bank_class():
    root = WORKSPACE / "src/application/teb_mode_manager/src"
    sys.path.insert(0, str(root))
    try:
        from teb_mode_manager.action_pipeline import AnchorBank
    finally:
        sys.path.pop(0)
    return AnchorBank


def _verify_sources(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    for label, binding in contract["source_bindings"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "source SHA differs")


def build_outputs(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _verify_sources(contract)
    AnchorBank = _load_anchor_bank_class()
    source_methods = _load_yaml(workspace / SOURCE_METHODS_RELATIVE)
    _require(len(source_methods["methods"]) == 11, "source method count differs")
    factor = contract["single_factor"]
    circle_ids = factor["methods"]
    allowed = factor["allowed_leaf_differences"]
    files: Dict[Path, bytes] = {}
    audits = []
    promoted_receipts = {}
    for method_id in circle_ids:
        method = next(
            row for row in source_methods["methods"] if row["method_id"] == method_id
        )
        source_path = Path(method["materialized_anchor_bank"]["path"])
        source_payload = (workspace / source_path).read_bytes()
        _require(
            _sha(source_payload) == method["materialized_anchor_bank"]["sha256"],
            "source bank receipt differs",
        )
        source = _load_yaml(workspace / source_path)
        original_error = None
        try:
            AnchorBank(source)
        except Exception as exc:
            original_error = "{}: {}".format(type(exc).__name__, exc)
        _require(
            original_error == "ActionPipelineError: anchor bank safety boundary drifted",
            "source rejection differs",
        )
        promoted = deepcopy(source)
        promoted["status"] = factor["status_transition"]["to"]
        removed = promoted["source_provenance"].pop("p2_q1")
        differences = _leaf_differences(source, promoted)
        _require(differences == allowed, "candidate diff escaped exact allowlist")
        parsed = AnchorBank(promoted)
        _require(
            len(parsed.parameter_names) == 20
            and promoted["runtime_ready"] is False
            and promoted["parameter_write_enabled"] is False
            and promoted["training_allowed"] is False
            and promoted["real_vehicle_use_forbidden"] is True,
            "promoted safety boundary differs",
        )
        relative = CANDIDATE_ROOT / "{}_anchor_bank.yaml".format(method_id)
        payload = _yaml_bytes(promoted)
        files[relative] = payload
        promoted_receipts[method_id] = _receipt(relative, payload)
        audits.append(
            {
                "method_id": method_id,
                "source": _receipt(source_path, source_payload),
                "promoted": promoted_receipts[method_id],
                "source_rejection": original_error,
                "promoted_strict_parse": "pass",
                "leaf_differences": differences,
                "removed_provenance_stage": removed["stage"],
                "behavioral_TEB_field_count_changed": 0,
                "parameter_registry_and_values_equal": (
                    source["parameters"] == promoted["parameters"]
                    and source["anchors"] == promoted["anchors"]
                    and source["overlays"] == promoted["overlays"]
                    and source["transaction"] == promoted["transaction"]
                    and source["policy_boundary"] == promoted["policy_boundary"]
                ),
            }
        )
    _require(all(row["parameter_registry_and_values_equal"] for row in audits), "behavior drift")

    methods = deepcopy(source_methods)
    methods.update(
        {
            "stage": STAGE,
            "status": "offline_runtime_schema_promoted_bindings_not_authorized",
            "execution_authorized": False,
            "evidence_budget_authorized": 0,
        }
    )
    for method in methods["methods"]:
        if method["method_id"] in promoted_receipts:
            method["materialized_anchor_bank"] = promoted_receipts[method["method_id"]]
    logical_methods = deepcopy(methods)
    logical_methods.pop("method_binding_digest", None)
    methods["method_binding_digest"] = _canonical_sha(logical_methods)
    methods_payload = _yaml_bytes(methods)
    files[METHODS_RELATIVE] = methods_payload

    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_runtime_bank_schema_promotion_audit_pass",
        "factor_count": 1,
        "factor_id": factor["factor_id"],
        "candidate_count": 5,
        "allowed_leaf_differences": allowed,
        "behavioral_TEB_field_count_changed": 0,
        "candidate_audits": audits,
        "method_count": len(methods["methods"]),
        "fixed_and_TEB_Tuned_bindings_unchanged": all(
            left == right
            for left, right in zip(source_methods["methods"], methods["methods"])
            if left["method_id"] not in circle_ids
        ),
        "held_out_test_accessed": False,
    }
    _require(audit["fixed_and_TEB_Tuned_bindings_unchanged"], "baseline binding drifted")
    files[AUDIT_RELATIVE] = _yaml_bytes(audit)

    source_paths = [CONTRACT_RELATIVE, BUILDER_RELATIVE, SOURCE_METHODS_RELATIVE]
    source_paths.extend(Path(binding["path"]) for binding in contract["source_bindings"].values())
    unique = {path.as_posix(): _receipt(path) for path in source_paths}
    source_freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_runtime_bank_readiness_source_freeze",
        "resources": [unique[key] for key in sorted(unique)],
        "resource_count": len(unique),
        "unresolved": [],
    }
    source_freeze["logical_sha256"] = _canonical_sha(source_freeze)
    files[SOURCE_FREEZE_RELATIVE] = _yaml_bytes(source_freeze)
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_runtime_bank_readiness_pass_release_absent",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "factor_count": 1,
        "factor_id": factor["factor_id"],
        "candidate_count": 5,
        "strict_parser_pass_count": 5,
        "behavioral_TEB_field_count_changed": 0,
        "method_count": 11,
        "fair_baseline_rule_unchanged": True,
        "training_steps": 0,
        "held_out_test_accessed": False,
        "ROS_or_Gazebo_started": False,
        "next_safe_entry": "independent_offline_review_then_exact_I1_release_construction",
    }
    files[REPORT_RELATIVE] = _yaml_bytes(report)
    return files


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P4-Q1 output root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=target.name + ".tmp.", dir=target.parent))
    try:
        for relative, payload in sorted(files.items(), key=lambda item: item[0].as_posix()):
            local = temporary / relative.relative_to(OUTPUT_ROOT)
            local.parent.mkdir(parents=True, exist_ok=True)
            with local.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        os.rename(str(temporary), str(target))
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def _check_only(files: Mapping[Path, bytes], workspace: Path) -> None:
    for relative, expected in files.items():
        path = workspace / relative
        _require(path.is_file() and path.read_bytes() == expected, "output drifted")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    files = build_outputs(WORKSPACE)
    if args.check:
        _check_only(files, WORKSPACE)
    else:
        _materialize(files, WORKSPACE)
    print(
        yaml.safe_dump(
            {
                "stage": STAGE,
                "status": "check_pass" if args.check else "materialized",
                "output_count": len(files),
            },
            sort_keys=False,
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
