#!/usr/bin/env python3
"""Independent fail-closed review and terminal seal for V3-00."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _load_foundation():
    source = (
        WORKSPACE
        / "src/tools/thesis_experiment/src/thesis_experiment/v3_foundation.py"
    )
    spec = importlib.util.spec_from_file_location("_v3_foundation_reviewer", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load {}".format(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


foundation = _load_foundation()


HANDOFF_RELATIVE = Path(
    "docs/thesis_experiment/CURRENT_V3_00_FOUNDATION_HANDOFF.md"
)


def _load_persisted(relative: Path) -> Mapping[str, Any]:
    return foundation.load_yaml_bytes(
        foundation.read_workspace_file_once(WORKSPACE, relative),
        relative.as_posix(),
    )


def _verify_evidence_freeze() -> int:
    payload = foundation.read_workspace_file_once(
        WORKSPACE, foundation.EVIDENCE_FREEZE_RELATIVE
    )
    count = 0
    previous = ""
    for line in payload.decode("utf-8").splitlines():
        digest, path_text = line.split(None, 1)
        relative = Path(path_text.strip())
        foundation.require(
            relative.as_posix() > previous,
            "evidence freeze is not strictly sorted",
        )
        actual = foundation.read_workspace_file_once(WORKSPACE, relative)
        foundation.require(
            foundation.sha256_bytes(actual) == digest,
            "evidence freeze drift: {}".format(relative),
        )
        previous = relative.as_posix()
        count += 1
    foundation.require(count >= 20, "evidence freeze roster unexpectedly small")
    return count


def _independent_candidate_audit(candidate: Mapping[str, Any]) -> Dict[str, Any]:
    candidates = candidate["candidates"]
    foundation.require(candidate["simulation_evidence"] is False, "fixture is evidence")
    foundation.require(candidate["candidate_count"] == 8, "candidate count differs")
    foundation.require(
        len({row["candidate_id"] for row in candidates}) == 8,
        "candidate IDs are not unique",
    )
    foundation.require(
        candidate["certified_stop_count"] >= 1,
        "certified stop absent",
    )
    foundation.require(candidate["teb_fallback_count"] >= 1, "TEB fallback absent")
    maximum_residual = 0.0
    for row in candidates:
        points = row["points"]
        times = [point["time_from_start_s"] for point in points]
        foundation.require(
            all(right > left for left, right in zip(times[:-1], times[1:])),
            "candidate time is not strictly increasing",
        )
        foundation.require(
            row["candidate_digest"] == foundation.canonical_sha(
                {
                    key: value
                    for key, value in row.items()
                    if key != "candidate_digest"
                }
            ),
            "candidate digest differs",
        )
        maximum_residual = max(
            maximum_residual, float(row["maximum_discrete_residual"])
        )
    foundation.require(
        maximum_residual <= 1.0e-9,
        "candidate kinematic residual exceeds reviewer bound",
    )
    return {
        "candidate_count": len(candidates),
        "unique_candidate_ids": True,
        "certified_stop_present": True,
        "teb_fallback_present": True,
        "maximum_discrete_residual": maximum_residual,
    }


def _independent_actor_audit(actor: Mapping[str, Any]) -> Dict[str, Any]:
    rows = actor["static_rows"]
    policy_ids = actor["actor_policy_ids"]
    candidate_ids = {
        row["candidate_id"]
        for row in rows
        if row["actor_policy_id_evaluator_only"] == policy_ids[0]
    }
    foundation.require(actor["simulation_evidence"] is False, "actor fixture is evidence")
    foundation.require(
        actor["counterfactual_identifiability_demonstrated"] is False,
        "fixture claims identifiability",
    )
    foundation.require(len(policy_ids) == 3, "actor policy count differs")
    foundation.require(len(rows) == 24, "actor clone count differs")
    foundation.require(
        len({row["scene_snapshot_id"] for row in rows}) == 1,
        "clone initial-state digest differs",
    )
    for policy_id in policy_ids:
        policy_rows = [
            row
            for row in rows
            if row["actor_policy_id_evaluator_only"] == policy_id
        ]
        foundation.require(
            {row["candidate_id"] for row in policy_rows} == candidate_ids,
            "candidate coverage differs: {}".format(policy_id),
        )
        foundation.require(
            all(row["behavior_propensity"] > 0.0 for row in policy_rows),
            "non-positive propensity",
        )
        foundation.require(
            abs(sum(row["behavior_propensity"] for row in policy_rows) - 1.0)
            <= 1.0e-12,
            "policy propensity sum differs",
        )
        foundation.require(
            all(row["actor_policy_exposed_to_runtime"] is False for row in policy_rows),
            "actor hidden label leaks to runtime",
        )
    foundation.require(
        all(row["response_effect_fixture_pass"] for row in actor["policy_audits"]),
        "response-effect schema fixture failed",
    )
    return {
        "policy_count": len(policy_ids),
        "clone_count": len(rows),
        "same_initial_state_digest": True,
        "candidate_coverage_each_policy": True,
        "positive_propensity_each_candidate": True,
        "hidden_label_firewall": True,
        "simulation_identifiability_claim": False,
    }


def _independent_package_audit(package_closure: Mapping[str, Any]) -> Dict[str, Any]:
    rows = package_closure["required_packages"]
    foundation.require(len(rows) == 6, "required package count differs")
    names = [row["package_name"] for row in rows]
    foundation.require(
        names == list(foundation.REQUIRED_PACKAGES),
        "required package order differs",
    )
    foundation.require(
        len({row["source_root"] for row in rows}) == len(rows),
        "required package source roots are not unique",
    )
    for row in rows:
        relative = Path(row["package_xml_path"])
        payload = foundation.read_workspace_file_once(WORKSPACE, relative)
        foundation.require(
            foundation.sha256_bytes(payload) == row["package_xml_sha256"],
            "package XML receipt differs: {}".format(row["package_name"]),
        )
    policy = package_closure["future_environment_policy"]
    foundation.require(
        policy["process_start"] == "env_i_non_login_shell"
        and policy["package_resolution_probe_required_before_consumption"] is True
        and policy[
            "package_resolution_failure_may_cross_consumption_boundary"
        ]
        is False
        and policy["credential_keys_in_child_environment_allowed"] is False,
        "future environment policy differs",
    )
    return {
        "required_package_count": len(rows),
        "unique_source_root_each_package": True,
        "package_xml_receipts_valid": True,
        "preconsumption_resolution_probe_required": True,
    }


def build_review() -> Dict[str, Any]:
    contract = foundation.load_yaml(WORKSPACE, foundation.CONTRACT_RELATIVE)
    contract_audit = foundation.validate_contract(WORKSPACE, contract)
    preregistration = foundation.load_yaml(
        WORKSPACE, foundation.PREREGISTRATION_RELATIVE
    )
    foundation.validate_preregistration(preregistration)

    bundle = foundation.build_bundle(WORKSPACE)
    foundation.validate_persisted_bundle(WORKSPACE, bundle)
    evidence_count = _verify_evidence_freeze()

    closure = _load_persisted(foundation.DEPENDENCY_CLOSURE_RELATIVE)
    logical = dict(closure)
    digest = logical.pop("logical_sha256")
    foundation.require(
        foundation.canonical_sha(logical) == digest,
        "dependency logical digest differs",
    )
    foundation.require(closure["unresolved"] == [], "dependency unresolved")
    for row in closure["local_resources"]:
        payload = foundation.read_workspace_file_once(
            WORKSPACE, Path(row["path"])
        )
        foundation.require(
            foundation.sha256_bytes(payload) == row["sha256"]
            and len(payload) == row["size_bytes"],
            "dependency receipt drift: {}".format(row["path"]),
        )

    candidate = _load_persisted(foundation.CANDIDATE_FIXTURE_RELATIVE)
    actor = _load_persisted(foundation.ACTOR_CONTRACT_RELATIVE)
    packages = _load_persisted(foundation.PACKAGE_CLOSURE_RELATIVE)
    interface = _load_persisted(foundation.INTERFACE_RELATIVE)
    zero = _load_persisted(foundation.ZERO_AUTHORITY_RELATIVE)

    foundation.require(interface["message_count"] == 5, "message count differs")
    foundation.require(
        interface["publish_boundary"]["only_allowed_command_topic"]
        == "/cmd_vel_navigation"
        and interface["publish_boundary"]["direct_cmd_vel_publish_forbidden"]
        is True,
        "command publish boundary differs",
    )
    for field in (
        "runtime_ready",
        "execution_ready",
        "execution_authorized",
        "training_authorized",
        "real_vehicle_authorized",
    ):
        foundation.require(zero[field] is False, "zero authority {} differs".format(field))
    for field in (
        "evidence_budget_authorized",
        "training_budget_steps",
        "seed_allocation_count",
        "schedule_row_count",
        "active_forbidden_process_count",
    ):
        foundation.require(
            type(zero[field]) is int and zero[field] == 0,
            "zero authority {} is not exact zero".format(field),
        )
    foundation.require(
        not foundation.active_forbidden_processes(),
        "forbidden runtime process active during review",
    )

    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "v3_00_pure_offline_foundation_closure_pass",
        "review_result": "pass",
        "offline_review_pass": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "evidence_budget_authorized": 0,
        "seed_allocation_count": 0,
        "schedule_row_count": 0,
        "real_vehicle_authorized": False,
        "source_freeze_count": contract_audit["source_freeze_count"],
        "dependency_resource_count": closure["resource_count"],
        "evidence_freeze_resource_count": evidence_count,
        "interface_audit": {
            "message_count": interface["message_count"],
            "only_allowed_future_command_topic": "/cmd_vel_navigation",
            "direct_cmd_vel_publish_forbidden": True,
        },
        "package_audit": _independent_package_audit(packages),
        "candidate_audit": _independent_candidate_audit(candidate),
        "actor_clone_audit": _independent_actor_audit(actor),
        "claim_state": {
            "foundation_tooling_qualified": True,
            "actor_response_validated_in_simulation": False,
            "counterfactual_identifiability_demonstrated": False,
            "predictor_trained_or_calibrated": False,
            "planner_runtime_validated": False,
            "performance_improvement_demonstrated": False,
            "safety_or_deployment_ready": False,
        },
        "active_forbidden_process_count": 0,
        "next_stage_automatically_authorized": False,
        "next_safe_entry": "V3-01 independent offline design and release review",
    }
    review["review_digest"] = foundation.canonical_sha(review)
    return review


def _write_review(review: Mapping[str, Any]) -> None:
    path = WORKSPACE / foundation.REVIEW_RELATIVE
    foundation.require(not path.exists(), "V3-00 review already exists")
    foundation.require(
        not (WORKSPACE / foundation.TERMINAL_FREEZE_RELATIVE).exists(),
        "V3-00 terminal seal already exists",
    )
    path.write_bytes(foundation.yaml_bytes(review))


def _terminal_freeze_payload(handoff_relative: Path) -> bytes:
    root = WORKSPACE / foundation.OUTPUT_ROOT
    foundation.require(root.is_dir(), "V3-00 artifact root absent")
    paths = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            relative = path.relative_to(WORKSPACE)
            if relative != foundation.TERMINAL_FREEZE_RELATIVE:
                paths.append(relative)
    paths.append(handoff_relative)
    unique = sorted(set(paths), key=lambda path: path.as_posix())
    rows = []
    for relative in unique:
        payload = foundation.read_workspace_file_once(WORKSPACE, relative)
        rows.append(
            "{}  {}\n".format(foundation.sha256_bytes(payload), relative.as_posix())
        )
    return "".join(rows).encode("utf-8")


def _seal_handoff(relative: Path) -> None:
    foundation.require(
        relative == HANDOFF_RELATIVE,
        "unexpected V3-00 handoff path",
    )
    handoff = foundation.read_workspace_file_once(WORKSPACE, relative)
    foundation.require(
        b"v3_00_pure_offline_foundation_closure_pass" in handoff,
        "handoff does not contain passing review status",
    )
    foundation.require(
        (WORKSPACE / foundation.REVIEW_RELATIVE).is_file(),
        "persisted review absent",
    )
    target = WORKSPACE / foundation.TERMINAL_FREEZE_RELATIVE
    foundation.require(not target.exists(), "terminal freeze already exists")
    target.write_bytes(_terminal_freeze_payload(relative))


def _check_terminal_freeze() -> int:
    path = WORKSPACE / foundation.TERMINAL_FREEZE_RELATIVE
    if not path.exists():
        return 0
    payload = foundation.read_workspace_file_once(
        WORKSPACE, foundation.TERMINAL_FREEZE_RELATIVE
    )
    count = 0
    for line in payload.decode("utf-8").splitlines():
        digest, path_text = line.split(None, 1)
        relative = Path(path_text.strip())
        actual = foundation.read_workspace_file_once(WORKSPACE, relative)
        foundation.require(
            foundation.sha256_bytes(actual) == digest,
            "terminal freeze drift: {}".format(relative),
        )
        count += 1
    foundation.require(
        payload == _terminal_freeze_payload(HANDOFF_RELATIVE),
        "terminal freeze roster is not exact",
    )
    return count


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--seal-handoff", metavar="WORKSPACE_RELATIVE_PATH")
    args = parser.parse_args()

    review = build_review()
    if args.write:
        _write_review(review)
    elif args.seal_handoff:
        _seal_handoff(Path(args.seal_handoff))

    persisted = WORKSPACE / foundation.REVIEW_RELATIVE
    if persisted.exists():
        foundation.require(
            foundation.read_workspace_file_once(
                WORKSPACE, foundation.REVIEW_RELATIVE
            )
            == foundation.yaml_bytes(review),
            "persisted independent review is not exactly reproducible",
        )
    terminal_count = _check_terminal_freeze()
    print(
        json.dumps(
            {
                "status": review["status"],
                "review_written": bool(args.write),
                "handoff_sealed": bool(args.seal_handoff),
                "terminal_freeze_resource_count": terminal_count,
                "execution_authorized": False,
                "training_authorized": False,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
