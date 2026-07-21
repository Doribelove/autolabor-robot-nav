#!/usr/bin/env python3
"""Shared fail-closed guards for the separately authorized R5 calibration."""

import hashlib
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
PREREGISTRATION = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml"
)
DRY_RUN_AUDIT = (
    WORKSPACE
    / "artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_dry_run_audit.yaml"
)
EXECUTION_AUTHORIZATION = (
    WORKSPACE
    / "experiments/manifests/v2/calibration/"
    "v2_04g_r5_bounded_execution_authorization.yaml"
)
PREREGISTRATION_SHA256 = (
    "0adcfd6a7a686b799b6dc55394cdf1e90fa140cee636d4283e0fb807f14134c6"
)
DRY_RUN_AUDIT_SHA256 = (
    "d7a3113c89b08889dc754a72f4e792c422225f19504ab3218d9712cf46dee8e1"
)
RUNTIME_PROCESS_MARKERS = (
    "roscore",
    "rosmaster",
    "roslaunch",
    "gzserver",
    "gzclient",
    "move_base",
    "rule_context_supervisor_node.py",
    "typed_teb_transaction_node.py",
    "nav_world_model_node.py",
    "v2_04g_r5_mechanism_episode.py",
    "v2_04g_r5_readiness_batch.py",
    "v2_04g_r5_bounded_calibration.py",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def atomic_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def _resource_entries(value, prefix=""):
    if isinstance(value, dict):
        if set(value) == {"path", "sha256"}:
            yield prefix, value
            return
        for key, child in value.items():
            child_prefix = "{}.{}".format(prefix, key) if prefix else str(key)
            yield from _resource_entries(child, child_prefix)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _resource_entries(
                child, "{}[{}]".format(prefix, index)
            )


def verify_frozen_start():
    """Verify the exact user-frozen start plus every declared resource."""
    if sha256(PREREGISTRATION) != PREREGISTRATION_SHA256:
        raise RuntimeError("R5 preregistration hash drifted")
    if sha256(DRY_RUN_AUDIT) != DRY_RUN_AUDIT_SHA256:
        raise RuntimeError("R5 dry-run audit hash drifted")
    prereg = load_yaml(PREREGISTRATION)
    audit = load_yaml(DRY_RUN_AUDIT)
    authorization = load_yaml(EXECUTION_AUTHORIZATION)
    if not (
        prereg.get("stage") == STAGE
        and prereg.get("split") == "calibration"
        and prereg.get("simulation_only") is True
        and prereg.get("calibration_only") is True
        and prereg.get("training_allowed") is False
        and prereg.get("runtime_ready") is False
        and prereg.get("formal_result") is False
        and prereg.get("real_vehicle_use_forbidden") is True
    ):
        raise RuntimeError("R5 preregistration safety boundary drifted")
    if not (
        audit.get("stage") == STAGE
        and audit.get("status") == "dry_run_audit_pass"
        and audit.get("resource_audit", {}).get(
            "all_declared_hashes_match"
        ) is True
        and audit.get("preregistration", {}).get("sha256")
        == PREREGISTRATION_SHA256
    ):
        raise RuntimeError("R5 dry-run audit boundary drifted")
    expected_actions = {
        "revalidate_frozen_start": True,
        "implement_minimal_execution_and_assessment_wrappers": True,
        "readiness_episode_count": 6,
        "deterministic_ttc_component_identity_count": 3,
        "navigation_episode_count": 60,
        "total_evidence_unit_budget": 69,
        "fail_closed_assessment": True,
    }
    if not (
        authorization.get("stage") == STAGE
        and authorization.get("split") == "calibration"
        and authorization.get("simulation_only") is True
        and authorization.get("calibration_only") is True
        and authorization.get("training_allowed") is False
        and authorization.get("authorized_actions") == expected_actions
        and set(authorization.get("unauthorized_actions", {}).values())
        == {False}
        and authorization.get("frozen_start", {})
        .get("preregistration", {})
        .get("sha256")
        == PREREGISTRATION_SHA256
        and authorization.get("frozen_start", {})
        .get("dry_run_audit", {})
        .get("sha256")
        == DRY_RUN_AUDIT_SHA256
    ):
        raise RuntimeError("R5 bounded execution authorization drifted")
    declared = list(_resource_entries({
        "resources": prereg.get("resources", {}),
        "frozen_r4_r1_boundary": prereg.get("frozen_r4_r1_boundary", {}),
    }))
    seen = set()
    for label, resource in declared:
        relative = Path(resource["path"])
        if relative.is_absolute():
            raise RuntimeError("absolute frozen resource path: {}".format(label))
        path = (WORKSPACE / relative).resolve()
        try:
            path.relative_to(WORKSPACE)
        except ValueError as exc:
            raise RuntimeError(
                "frozen resource escapes workspace: {}".format(label)
            ) from exc
        if path in seen:
            continue
        seen.add(path)
        if not path.is_file() or sha256(path) != resource["sha256"]:
            raise RuntimeError("frozen resource drifted: {}".format(label))
    if len(seen) != 39:
        raise RuntimeError("R5 frozen resource closure count drifted")
    return prereg, audit, authorization


def assert_thesis_workspace_environment():
    """Refuse execution from the stable robot workspace or an unsourced shell."""
    prefix = os.environ.get("CMAKE_PREFIX_PATH", "").split(":")
    expected = str(WORKSPACE / "devel")
    if not prefix or Path(prefix[0]).resolve() != Path(expected).resolve():
        raise RuntimeError(
            "source only /home/robot/robot_ws_base_rl/devel/setup.bash first"
        )
    for name in ("CMAKE_PREFIX_PATH", "ROS_PACKAGE_PATH"):
        entries = [entry for entry in os.environ.get(name, "").split(":") if entry]
        if any(
            Path(entry).resolve() == Path("/home/robot/robot_ws")
            or str(Path(entry).resolve()).startswith("/home/robot/robot_ws/")
            for entry in entries
        ):
            raise RuntimeError("stable real-robot workspace leaked into {}".format(name))


def _ancestor_pids():
    ancestors = {os.getpid()}
    current = os.getpid()
    while current > 1:
        try:
            fields = Path("/proc/{}/stat".format(current)).read_text(
                encoding="utf-8"
            ).split()
            current = int(fields[3])
        except (OSError, ValueError, IndexError):
            break
        ancestors.add(current)
    return ancestors


def live_runtime_processes():
    """Return relevant live processes, excluding this command's ancestor chain."""
    excluded = _ancestor_pids()
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except OSError:
            continue
        if command and any(marker in command for marker in RUNTIME_PROCESS_MARKERS):
            matches.append({"pid": int(entry.name), "command": command.strip()})
    return sorted(matches, key=lambda row: row["pid"])


def assert_no_live_runtime_processes():
    matches = live_runtime_processes()
    if matches:
        raise RuntimeError("pre-existing ROS/Gazebo/R5 runtime process detected")
    return matches
