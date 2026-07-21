#!/usr/bin/env python3
"""Run the once-only, fail-closed V2-04G-R5 navigation calibration."""

import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
SCRIPT_DIR = Path(__file__).resolve().parent
GUARD_PATH = SCRIPT_DIR / "v2_04g_r5_execution_guard.py"
FROZEN_BATCH_PATH = SCRIPT_DIR / "v2_04g_r3_calibration_batch.py"
EPISODE_RUNNER = SCRIPT_DIR / "v2_04g_r5_mechanism_episode.py"
OUTPUT_ROOT = WORKSPACE / "artifacts/v2/calibration/v2_04g_r5"
PROGRESS_PATH = OUTPUT_ROOT / "v2_04g_r5_progress.yaml"
READINESS_SUMMARY = (
    OUTPUT_ROOT / "readiness/v2_04g_r5_readiness_summary.yaml"
)
TTC_COMPONENT_REPORT = OUTPUT_ROOT / "v2_04g_r5_ttc_three_state_probe.yaml"
RUNTIME_CONFIG_ROOT = OUTPUT_ROOT / "runtime_candidate_configs"
EXPECTED_PREREGISTRATION_SHA256 = (
    "0adcfd6a7a686b799b6dc55394cdf1e90fa140cee636d4283e0fb807f14134c6"
)
EXPECTED_DRY_RUN_AUDIT_SHA256 = (
    "d7a3113c89b08889dc754a72f4e792c422225f19504ab3218d9712cf46dee8e1"
)
EXPECTED_SCHEDULE_SHA256 = (
    "5daf5a4dcdf0e68c4b034f343e0ae5f85504c25e6158e94699c6f1a8cc80513a"
)
PROFILE_ORDER = (
    "fixed_reference",
    "r5_ttc_control_h500",
    "r5_ttc_h450",
    "r5_ttc_h400",
)
METHOD_BY_PROFILE = {
    "fixed_reference": "fixed_teb",
    "r5_ttc_control_h500": "rule_multi_anchor",
    "r5_ttc_h450": "rule_multi_anchor",
    "r5_ttc_h400": "rule_multi_anchor",
}
TTC_STATUSES = {
    "OBSERVED_CONFLICT",
    "NO_CONFLICT_IN_HORIZON",
    "TRACKER_INVALID",
}


class TerminalEvidenceFailure(RuntimeError):
    """Raised after an attempted identity permanently terminates the stage."""


class StageAlreadyStarted(RuntimeError):
    """Raised when a second invocation would constitute a resume."""


def _load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_yaml(path):
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError("YAML document must be a mapping: {}".format(path))
    return value


def _atomic_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(value, sort_keys=False), encoding="utf-8"
    )
    temporary.replace(path)


def _inside(path, root, label):
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(Path(root).resolve())
    except ValueError as exc:
        raise RuntimeError("{} escapes {}".format(label, root)) from exc
    return resolved


def _display_path(path, workspace=WORKSPACE):
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(Path(workspace).resolve()).as_posix()
    except ValueError:
        return str(resolved)


def _identity(row):
    return {
        "profile_id": row["profile_id"],
        "method": row["method"],
        "scene_id": row["scene_id"],
    }


def _identity_key(row):
    return (row["profile_id"], row["method"], row["scene_id"])


def _load_instances(workspace, preregistration):
    workspace = Path(workspace).resolve()
    resource = preregistration["resources"]["navigation_compiled_scene_index"]
    index_path = _inside(workspace / resource["path"], workspace, "scene index")
    if not index_path.is_file() or _sha256(index_path) != resource["sha256"]:
        raise RuntimeError("R5 compiled scene index drifted")
    index = _load_yaml(index_path)
    if index.get("scene_count") != 15 or len(index.get("files", [])) != 30:
        raise RuntimeError("R5 compiled scene index is incomplete")
    for entry in index["files"]:
        path = _inside(workspace / entry["path"], workspace, "compiled scene")
        if not path.is_file() or _sha256(path) != entry["sha256"]:
            raise RuntimeError("compiled scene drifted: {}".format(entry["path"]))
    directory = index_path.parent
    instances = {}
    for instance_path in sorted(directory.glob("*.instance.yaml")):
        instance = _load_yaml(instance_path)
        scene = instance.get("scene", {})
        scene_id = scene.get("scene_id")
        if not isinstance(scene_id, str) or scene_id in instances:
            raise RuntimeError("duplicate or invalid compiled scene identity")
        world_path = directory / (scene_id + ".world")
        if not world_path.is_file():
            raise RuntimeError("compiled world is missing for {}".format(scene_id))
        instances[scene_id] = (
            instance,
            instance_path.resolve(),
            world_path.resolve(),
        )
    if set(instances) != set(preregistration["scene_ids"]):
        raise RuntimeError("R5 compiled navigation scene set drifted")
    return instances


def _canonical_schedule_rows(preregistration, instances):
    declared = preregistration.get("navigation_schedule", {})
    if not (
        declared.get("method_order") == list(PROFILE_ORDER)
        and declared.get("scene_order") == preregistration.get("scene_ids")
        and declared.get("exact_cartesian_product_required") is True
        and declared.get("planned_episode_count") == 60
        and declared.get("attempts_per_identity_max") == 1
        and declared.get("duplicate_identity_count_max") == 0
    ):
        raise RuntimeError("R5 navigation schedule boundary drifted")
    expected = []
    for profile_id in PROFILE_ORDER:
        method = METHOD_BY_PROFILE[profile_id]
        for scene_id in preregistration["scene_ids"]:
            scene = instances[scene_id][0]["scene"]
            expected.append({
                "sequence": len(expected) + 1,
                "stage": STAGE,
                "split": "calibration",
                "method": method,
                "profile_id": profile_id,
                "scene_id": scene_id,
                "family": scene["family"],
                "seed": scene["seed"],
            })
    expected_declaration = [{
        "sequence": row["sequence"],
        "method": row["method"],
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt_limit": 1,
    } for row in expected]
    if declared.get("schedule") != expected_declaration:
        raise RuntimeError("R5 exact 60-identity schedule drifted")
    identities = {_identity_key(row) for row in expected}
    if len(expected) != 60 or len(identities) != 60:
        raise RuntimeError("R5 navigation schedule is not an exact 60 product")
    payload = json.dumps(
        expected, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    digest = hashlib.sha256(payload).hexdigest()
    if digest != EXPECTED_SCHEDULE_SHA256:
        raise RuntimeError("R5 navigation schedule hash drifted")
    return expected, digest


def build_schedule(preregistration, instances, runtime_configs):
    """Build and verify the exact 60 rows, then attach frozen runtime paths."""
    canonical, digest = _canonical_schedule_rows(preregistration, instances)
    if set(runtime_configs) != set(PROFILE_ORDER[1:]):
        raise RuntimeError("R5 runtime candidate set drifted")
    fixed_supervisor = WORKSPACE / preregistration["resources"][
        "frozen_m030_supervisor"
    ]["path"]
    fixed_anchor = WORKSPACE / preregistration["resources"][
        "frozen_m030_anchor_bank"
    ]["path"]
    rows = []
    for canonical_row in canonical:
        row = dict(canonical_row)
        profile_id = row["profile_id"]
        if profile_id == "fixed_reference":
            row.update({
                "runtime_config": str(fixed_supervisor.resolve()),
                "anchor_bank": str(fixed_anchor.resolve()),
                "mechanism_config": "",
            })
        else:
            runtime = runtime_configs[profile_id]
            row.update({
                "runtime_config": str(Path(runtime["supervisor"]).resolve()),
                "anchor_bank": str(Path(runtime["anchor_bank"]).resolve()),
                "mechanism_config": str(Path(runtime["mechanism"]).resolve()),
            })
        rows.append(row)
    return rows, digest


def _verify_runtime_configs(preregistration):
    """Verify readiness-created configs in memory without rewriting any file."""
    if not RUNTIME_CONFIG_ROOT.is_dir():
        raise RuntimeError("readiness runtime candidate configs are missing")
    bank_path = WORKSPACE / preregistration["resources"]["candidate_bank"]["path"]
    bank = _load_yaml(bank_path)
    frozen = bank["frozen_m030_input"]
    base_supervisor = _load_yaml(WORKSPACE / frozen["supervisor"]["path"])
    base_anchor = _load_yaml(WORKSPACE / frozen["anchor_bank"]["path"])
    base_mechanism = _load_yaml(WORKSPACE / frozen["mechanism"]["path"])
    candidates = {row["candidate_id"]: row for row in bank["candidates"]}
    if set(candidates) != set(PROFILE_ORDER[1:]):
        raise RuntimeError("R5 candidate bank set drifted")
    if {path.name for path in RUNTIME_CONFIG_ROOT.iterdir()} != set(candidates):
        raise RuntimeError("R5 runtime candidate directory set drifted")
    actual = {}
    for candidate_id in PROFILE_ORDER[1:]:
        candidate_root = RUNTIME_CONFIG_ROOT / candidate_id
        paths = {
            "supervisor": candidate_root / "supervisor.yaml",
            "anchor_bank": candidate_root / "anchor_bank.yaml",
            "mechanism": candidate_root / "mechanism.yaml",
        }
        if {path.name for path in candidate_root.iterdir()} != {
            "supervisor.yaml", "anchor_bank.yaml", "mechanism.yaml"
        }:
            raise RuntimeError(
                "readiness runtime config file set drifted: {}".format(
                    candidate_id
                )
            )
        expected_supervisor = copy.deepcopy(base_supervisor)
        expected_supervisor["profile_id"] = (
            "fam_teb_v2_04g_r5_{}_supervisor".format(candidate_id)
        )
        expected_supervisor["dynamic"]["predicted_ttc_max_s"] = float(
            candidates[candidate_id]["predicted_ttc_max_s"]
        )
        expected_anchor = copy.deepcopy(base_anchor)
        expected_anchor["bank_id"] = (
            "fam_teb_v2_04g_r5_{}_anchor_input".format(candidate_id)
        )
        expected_mechanism = copy.deepcopy(base_mechanism)
        expected_mechanism["profile_id"] = (
            "fam_teb_v2_04g_r5_{}_mechanism".format(candidate_id)
        )
        expected = {
            "supervisor": expected_supervisor,
            "anchor_bank": expected_anchor,
            "mechanism": expected_mechanism,
        }
        for kind, path in paths.items():
            if not path.is_file() or _load_yaml(path) != expected[kind]:
                raise RuntimeError(
                    "readiness runtime config drifted: {}.{}".format(
                        candidate_id, kind
                    )
                )
        actual[candidate_id] = paths
    return actual


def verify_prerequisites(
        readiness_path,
        component_path,
        preregistration_sha256=EXPECTED_PREREGISTRATION_SHA256,
):
    """Require the completed 6-evidence readiness and 3-state component gate."""
    readiness_path = Path(readiness_path).resolve()
    component_path = Path(component_path).resolve()
    if not readiness_path.is_file() or not component_path.is_file():
        raise RuntimeError("R5 readiness or TTC component prerequisite is missing")
    readiness = _load_yaml(readiness_path)
    component = _load_yaml(component_path)
    common_readiness = (
        readiness.get("stage") == STAGE
        and readiness.get("status") == "complete"
        and readiness.get("simulation_only") is True
        and readiness.get("runtime_ready") is False
        and readiness.get("planned_probe_count") == 6
        and readiness.get("executed_probe_count") == 6
        and readiness.get("valid_probe_count") == 6
        and readiness.get("attempts_per_identity_max") == 1
        and readiness.get("retry_count") == 0
        and readiness.get("resume_used") is False
        and readiness.get("resume_forbidden") is True
        and readiness.get("terminal_failure") is None
        and readiness.get("all_probe_hard_gates_pass") is True
        and readiness.get("ttc_coverage_pass") is True
        and readiness.get("ttc_component_authorized") is True
        and readiness.get("navigation_authorized") is False
        and readiness.get("training_started") is False
        and readiness.get("real_vehicle_used") is False
        and readiness.get("preregistration", {}).get("sha256")
        == preregistration_sha256
    )
    if not common_readiness:
        raise RuntimeError("R5 TTC readiness prerequisite failed")
    expected_status_order = [
        "OBSERVED_CONFLICT",
        "NO_CONFLICT_IN_HORIZON",
        "TRACKER_INVALID",
    ]
    common_component = (
        component.get("stage") == STAGE
        and component.get("status") == "complete"
        and component.get("simulation_only") is True
        and component.get("runtime_ready") is False
        and component.get("probe_count") == 3
        and component.get("attempts_per_identity_max") == 1
        and component.get("retry_count") == 0
        and component.get("resume_used") is False
        and component.get("resume_forbidden") is True
        and component.get("terminal_failure") is None
        and component.get("expected_status_order") == expected_status_order
        and component.get("observed_status_order") == expected_status_order
        and component.get("all_three_states_pass") is True
        and component.get("navigation_authorized") is True
        and component.get("training_used") is False
        and component.get("real_vehicle_used") is False
        and component.get("preregistration", {}).get("sha256")
        == preregistration_sha256
        and component.get("readiness_summary", {}).get("sha256")
        == _sha256(readiness_path)
    )
    if not common_component:
        raise RuntimeError("R5 TTC component prerequisite failed")
    return {
        "readiness_summary": {
            "path": _display_path(readiness_path),
            "sha256": _sha256(readiness_path),
        },
        "ttc_component_probe": {
            "path": _display_path(component_path),
            "sha256": _sha256(component_path),
        },
    }


def _new_progress(preregistration_path, schedule_hash, prerequisites):
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "in_progress",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {
            "path": _display_path(preregistration_path),
            "sha256": _sha256(preregistration_path),
        },
        "dry_run_audit": {
            "path": _display_path(
                OUTPUT_ROOT / "v2_04g_r5_dry_run_audit.yaml"
            ),
            "sha256": EXPECTED_DRY_RUN_AUDIT_SHA256,
        },
        **prerequisites,
        "schedule_sha256": schedule_hash,
        "planned_navigation_episode_count": 60,
        "attempts_per_identity_max": 1,
        "attempted_identity_count": 0,
        "valid_evidence_episode_count": 0,
        "interface_failure_count": 0,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": False,
        "active_identity": None,
        "attempt_ledger": [],
        "episodes": [],
        "terminal_failure": None,
        "interface_failures": [],
    }


def _expected_episode_dir(output_root, row):
    return Path(output_root) / "episodes" / (
        "ep_{:03d}__{}__{}".format(
            row["sequence"], row["profile_id"], row["scene_id"]
        )
    )


def _progress_episode(row, evaluation_path, evaluation):
    trace_path = Path(evaluation_path).parent / "trace.csv"
    return {
        "sequence": row["sequence"],
        "profile_id": row["profile_id"],
        "method": row["method"],
        "scene_id": row["scene_id"],
        "family": row["family"],
        "seed": row["seed"],
        "attempt": 1,
        "evaluation": str(Path(evaluation_path).resolve()),
        "evaluation_sha256": _sha256(evaluation_path),
        "trace_sha256": _sha256(trace_path),
        "success": evaluation["metrics"]["common"]["success"],
        "collision": evaluation["metrics"]["common"]["collision"],
    }


def execute_navigation_schedule(
        schedule,
        output_root,
        progress_path,
        preregistration_path,
        schedule_hash,
        prerequisites,
        episode_callable,
        evidence_validator,
):
    """Execute each identity once, atomically recording any terminal failure."""
    if len(schedule) != 60 or [row["sequence"] for row in schedule] != list(
        range(1, 61)
    ):
        raise RuntimeError("R5 executor refuses a non-exact navigation schedule")
    if any(row.get("attempt_limit", 1) != 1 for row in schedule):
        raise RuntimeError("R5 executor refuses an attempt budget above one")
    output_root = Path(output_root).resolve()
    progress_path = Path(progress_path).resolve()
    if progress_path.exists():
        existing = _load_yaml(progress_path)
        raise StageAlreadyStarted(
            "R5 navigation already {}: resume is forbidden".format(
                existing.get("status", "started")
            )
        )
    episodes_root = output_root / "episodes"
    if episodes_root.exists() and any(episodes_root.iterdir()):
        raise StageAlreadyStarted(
            "untracked R5 navigation output exists; resume is forbidden"
        )
    for row in schedule:
        if _expected_episode_dir(output_root, row).exists():
            raise StageAlreadyStarted(
                "untracked R5 identity output exists; resume is forbidden"
            )
    progress = _new_progress(
        preregistration_path, schedule_hash, prerequisites
    )
    _atomic_yaml(progress_path, progress)
    for row in schedule:
        output_dir = _expected_episode_dir(output_root, row)
        ledger = {
            "sequence": row["sequence"],
            "identity": _identity(row),
            "attempt": 1,
            "status": "attempt_started",
            "output_dir": str(output_dir.resolve()),
        }
        progress["attempt_ledger"].append(ledger)
        progress["attempted_identity_count"] = len(progress["attempt_ledger"])
        progress["active_identity"] = {
            "sequence": row["sequence"],
            **_identity(row),
        }
        _atomic_yaml(progress_path, progress)
        try:
            if output_dir.exists():
                raise RuntimeError(
                    "R5 identity output appeared after attempt registration"
                )
            evaluation_path, evaluation = episode_callable(row, output_dir)
            evaluation = evidence_validator(
                row, evaluation_path, evaluation, output_dir
            )
        except BaseException as exc:
            failure = {
                "sequence": row["sequence"],
                "identity": _identity(row),
                "attempt": 1,
                "phase": "navigation_episode",
                "reason": "{}: {}".format(type(exc).__name__, exc),
                "output_dir": str(output_dir.resolve()),
            }
            ledger.update({
                "status": "terminal_failure",
                "phase": failure["phase"],
                "reason": failure["reason"],
            })
            progress.update({
                "status": "terminal_failure",
                "interface_failure_count": 1,
                "resume_forbidden": True,
                "active_identity": None,
                "terminal_failure": failure,
                "interface_failures": [failure],
            })
            _atomic_yaml(progress_path, progress)
            raise TerminalEvidenceFailure(
                "R5 terminal evidence failure at sequence {}".format(
                    row["sequence"]
                )
            ) from exc
        episode = _progress_episode(row, evaluation_path, evaluation)
        progress["episodes"].append(episode)
        progress["valid_evidence_episode_count"] = len(progress["episodes"])
        progress["active_identity"] = None
        ledger.update({
            "status": "evidence_complete",
            "evaluation": episode["evaluation"],
            "evaluation_sha256": episode["evaluation_sha256"],
            "trace_sha256": episode["trace_sha256"],
        })
        _atomic_yaml(progress_path, progress)
    progress.update({
        "status": "complete",
        "resume_forbidden": True,
        "active_identity": None,
    })
    _atomic_yaml(progress_path, progress)
    return progress


def _validate_actual_evidence(instances, row, path, evaluation, output_dir):
    path = Path(path).resolve()
    output_dir = Path(output_dir).resolve()
    if path != output_dir / "evaluation.yaml" or not path.is_file():
        raise RuntimeError("R5 evaluation path drifted")
    trace_path = output_dir / "trace.csv"
    if not trace_path.is_file():
        raise RuntimeError("R5 trace is missing")
    reloaded = _load_yaml(path)
    if reloaded != evaluation:
        raise RuntimeError("R5 returned evaluation differs from persisted evidence")
    scene_instance = instances[row["scene_id"]][0]
    expected = {
        "stage": STAGE,
        "split": "calibration",
        "method": row["method"],
        "scene_id": row["scene_id"],
        "family": row["family"],
        "seed": row["seed"],
        "instance_sha256": scene_instance["instance_sha256"],
        "supervisor_profile_id": row["profile_id"],
        "formal_result": False,
        "runtime_ready": False,
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "runtime_scene_labels_available": False,
        "experiment_manager_validation_manifest_access": False,
        "experiment_manager_calibration_manifest_access": True,
        "typed_transaction_valid": True,
    }
    for key, value in expected.items():
        if evaluation.get(key) != value:
            raise RuntimeError("R5 evaluation {} drifted".format(key))
    if evaluation.get("raw_trace_sha256") != _sha256(trace_path):
        raise RuntimeError("R5 raw trace hash drifted")
    if evaluation.get("ttc_status") not in TTC_STATUSES:
        raise RuntimeError("R5 TTC evidence status drifted")
    audit = evaluation.get("clearance_audit")
    if not (
        isinstance(audit, dict)
        and audit.get("stage") == STAGE
        and audit.get("evaluator_only_gazebo_truth_used") is True
        and audit.get("runtime_policy_received_truth") is False
    ):
        raise RuntimeError("R5 clearance audit boundary drifted")
    if row["method"] == "fixed_teb":
        if evaluation.get("transaction_message_count") != 0:
            raise RuntimeError("R5 Fixed unexpectedly received transactions")
    elif evaluation.get("transaction_activated_count", 0) <= 0:
        raise RuntimeError("R5 candidate has no activated transaction")
    return evaluation


def print_dry_run(schedule):
    """Print the exact schedule without creating or changing artifact files."""
    print(
        "{} dry-run: {} navigation identities; no ROS/Gazebo or artifact writes".format(
            STAGE, len(schedule)
        )
    )
    for row in schedule:
        print(
            "{:03d} {} {} seed={} attempt=1".format(
                row["sequence"],
                row["profile_id"],
                row["scene_id"],
                row["seed"],
            )
        )


def _actual_episode_callable(guard, instances):
    frozen = _load_module(
        FROZEN_BATCH_PATH, "v2_04g_r3_frozen_batch_for_r5_navigation"
    )
    frozen.STAGE = STAGE
    args = SimpleNamespace(
        output_root=OUTPUT_ROOT,
        episode_runner=EPISODE_RUNNER,
    )
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"

    def run(row, expected_output):
        if _expected_episode_dir(OUTPUT_ROOT, row).resolve() != Path(
            expected_output
        ).resolve():
            raise RuntimeError("R5 episode output identity drifted")
        guard.assert_no_live_runtime_processes()
        result = frozen.run_episode(row, args, instances, environment)
        guard.assert_no_live_runtime_processes()
        return result

    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    guard = _load_module(GUARD_PATH, "v2_04g_r5_execution_guard_for_navigation")
    preregistration, audit, _ = guard.verify_frozen_start()
    instances = _load_instances(WORKSPACE, preregistration)
    canonical_schedule, schedule_hash = _canonical_schedule_rows(
        preregistration, instances
    )
    if (
        audit.get("navigation_plan", {}).get("schedule_sha256")
        != schedule_hash
        or guard.sha256(guard.PREREGISTRATION)
        != EXPECTED_PREREGISTRATION_SHA256
        or guard.sha256(guard.DRY_RUN_AUDIT)
        != EXPECTED_DRY_RUN_AUDIT_SHA256
    ):
        raise RuntimeError("R5 frozen schedule or start hash drifted")
    if PROGRESS_PATH.exists():
        existing = _load_yaml(PROGRESS_PATH)
        raise StageAlreadyStarted(
            "R5 navigation already {}: no resume".format(
                existing.get("status", "started")
            )
        )
    if args.dry_run:
        print_dry_run(canonical_schedule)
        return 0

    guard.assert_thesis_workspace_environment()
    guard.assert_no_live_runtime_processes()
    prerequisites = verify_prerequisites(
        READINESS_SUMMARY,
        TTC_COMPONENT_REPORT,
        guard.PREREGISTRATION_SHA256,
    )
    runtime_configs = _verify_runtime_configs(preregistration)
    schedule, attached_schedule_hash = build_schedule(
        preregistration, instances, runtime_configs
    )
    if attached_schedule_hash != schedule_hash:
        raise RuntimeError("R5 attached navigation schedule hash drifted")
    progress = execute_navigation_schedule(
        schedule=schedule,
        output_root=OUTPUT_ROOT,
        progress_path=PROGRESS_PATH,
        preregistration_path=guard.PREREGISTRATION,
        schedule_hash=schedule_hash,
        prerequisites=prerequisites,
        episode_callable=_actual_episode_callable(guard, instances),
        evidence_validator=lambda row, path, evidence, output: (
            _validate_actual_evidence(
                instances, row, path, evidence, output
            )
        ),
    )
    print(
        "{} navigation complete: {}/60 valid evidence; assessment required".format(
            STAGE, progress["valid_evidence_episode_count"]
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
