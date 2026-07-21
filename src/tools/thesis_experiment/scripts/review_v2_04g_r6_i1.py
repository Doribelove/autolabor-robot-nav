#!/usr/bin/env python3
"""Offline review of the independent R6-I1 execution integration."""

import argparse
import hashlib
import os
from pathlib import Path
import tempfile

import yaml

from thesis_experiment.v2_04g_r6_i1_dependency import (
    build_dependency_closure,
)
from thesis_experiment.v2_04g_r6_integrity import (
    strict_yaml,
    verify_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I1"
CONTRACT = WORKSPACE / (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_execution_integration_contract.yaml"
)
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_execution_preregistration.yaml"
)
CLOSURE = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml"
)
OUTPUT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "v2_04g_r6_i1_integration_review.yaml"
)
AUTHORIZATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_bounded_simulation_authorization.yaml"
)
EXECUTION_ROOT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/execution"
)
STAGE_REPORT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "v2_04g_r6_i1_stage_report.yaml"
)
EXPECTED_DESIGN_RESOURCES = {
    "r6_design_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_semantic_alignment_design_contract.yaml",
        "16dfcbd9d4d34758653b61b5b73018415d956d5e337597cd1163021c2e228be1",
    ),
    "r6_design_preregistration": (
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_alignment_preregistration.yaml",
        "1e923a316d8dd8675628da4cfd7feab51653fd543df76e226b919a198019d912",
    ),
    "r6_design_candidate_bank": (
        "experiments/manifests/v2/preregistrations/"
        "v2_04g_r6_semantic_candidates.yaml",
        "6732b132067591f71c365463b5677bbe5dc161b9fb5b891ebac8a74b70c63c5d",
    ),
    "r6_semantic_reference": (
        "src/application/teb_mode_manager/src/teb_mode_manager/"
        "r6_relative_ttc_supervisor.py",
        "da7ebfbe361137da603c2b7767ef3040ae91751603097dd14b70225ac44a1b83",
    ),
    "r6_design_integrity": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_integrity.py",
        "65887068fcc1d98296a04eb1b8d87f2d6e29139365555bdfa27323efee9b89f8",
    ),
    "r6_design_report": (
        "artifacts/v2/design_review/v2_04g_r6/"
        "v2_04g_r6_design_review.yaml",
        "7cd08db3ead76c31f20fc76c23c3ec7ad86ddbe39d095b599ee75b6fea76cb1e",
    ),
    "r6_design_handoff": (
        "docs/thesis_experiment/CURRENT_V2_04G_R6_DESIGN_HANDOFF.md",
        "134a7a838571392353d84f64d2f1045d78f2922816e718443d7b70db7380f1c6",
    ),
}
EXPECTED_R5_TREE_SHA256 = (
    "ecb1f33093dee469008c2ad2d783b3e8ffd1c0739db7903b5df273717e270984"
)
EXPECTED_RISK_IDS = (
    "D1-RISK-READINESS-DIRECT-COUNTS",
    "D1-RISK-COMPILED-SCENE-TOCTOU",
    "D1-RISK-SIGINT-IN-PROGRESS",
    "D1-RISK-ASSESSMENT-RAW-BINDING",
    "D1-RISK-EXECUTION-HASH-CLOSURE",
    "D1-RISK-TEARDOWN-RESTORE",
)


class R6I1ReviewError(ValueError):
    """Raised when the execution integration fails closed."""


def _require(condition, message):
    if not condition:
        raise R6I1ReviewError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, value):
    target = Path(path)
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
        directory = os.open(str(target.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _tree_snapshot(path):
    root = Path(path)
    rows = []
    for item in sorted(root.rglob("*")):
        if item.is_file() and not item.is_symlink():
            rows.append(
                "{} {}\n".format(
                    item.relative_to(WORKSPACE).as_posix(), sha256(item)
                )
            )
    return {
        "file_count": len(rows),
        "tree_sha256": hashlib.sha256(
            "".join(rows).encode("utf-8")
        ).hexdigest(),
    }


def _verify_resource_map(contract):
    resources = contract.get("resources")
    _require(isinstance(resources, dict) and resources, "resource map is empty")
    verified = {}
    for label, row in resources.items():
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "{} resource schema drifted".format(label),
        )
        relative = Path(row["path"])
        _require(
            not relative.is_absolute() and ".." not in relative.parts,
            "{} resource path is unsafe".format(label),
        )
        path = (WORKSPACE / relative).resolve()
        _require(
            path.is_file()
            and not path.is_symlink()
            and sha256(path) == row["sha256"],
            "{} resource hash drifted".format(label),
        )
        verified[label] = dict(row)
    for label, (path, digest) in EXPECTED_DESIGN_RESOURCES.items():
        _require(
            verified.get(label) == {"path": path, "sha256": digest},
            "{} frozen design binding drifted".format(label),
        )
    return verified


def _flatten(value, prefix=()):
    if isinstance(value, dict):
        result = {}
        for key in sorted(value):
            result.update(_flatten(value[key], prefix + (str(key),)))
        return result
    if isinstance(value, list):
        result = {}
        for index, item in enumerate(value):
            result.update(_flatten(item, prefix + (str(index),)))
        return result
    return {".".join(prefix): value}


def _verify_runtime_profiles(resources):
    supervisors = {}
    for profile in (
        "r6_semantics_legacy_control",
        "r6_semantics_circle_contact",
    ):
        label = profile + "_supervisor"
        path = WORKSPACE / resources[label]["path"]
        supervisors[profile] = strict_yaml(path)
        directory = path.parent
        for name in ("anchor_bank.yaml", "mechanism.yaml"):
            other = (
                WORKSPACE
                / resources[
                    profile + "_" + name[:-5]
                ]["path"]
            )
            _require(
                other.parent == directory,
                "runtime profile directory binding drifted",
            )
    left = _flatten(supervisors["r6_semantics_legacy_control"])
    right = _flatten(supervisors["r6_semantics_circle_contact"])
    differences = [
        key for key in sorted(set(left) | set(right))
        if type(left.get(key)) is not type(right.get(key))
        or left.get(key) != right.get(key)
    ]
    _require(
        differences == ["dynamic.conflict_estimator_id"],
        "runtime profiles differ outside the single factor: {}".format(
            differences
        ),
    )
    expected = {
        "r6_semantics_legacy_control":
            "legacy_class_conditioned_geometry_v1",
        "r6_semantics_circle_contact":
            "shared_circle_envelope_first_contact_v1",
    }
    for profile, document in supervisors.items():
        dynamic = document["dynamic"]
        _require(
            dynamic == {
                "minimum_track_confidence": 0.45,
                "predicted_ttc_max_s": 5.0,
                "closest_approach_max_m": 1.35,
                "robot_radius_m": 0.62,
                "minimum_relative_speed_mps": 0.05,
                "conflict_estimator_id": expected[profile],
            }
            and document["transition"][
                "overlay_release_confirmation_s"
            ] == 0.20,
            "runtime frozen values drifted",
        )
    _require(
        resources["r6_semantics_legacy_control_anchor_bank"]["sha256"]
        == resources["r6_semantics_circle_contact_anchor_bank"]["sha256"]
        and resources["r6_semantics_legacy_control_mechanism"]["sha256"]
        == resources["r6_semantics_circle_contact_mechanism"]["sha256"],
        "anchor/mechanism bytes differ between factor levels",
    )
    return {
        "profile_count": 2,
        "normalized_leaf_difference_count": 1,
        "only_difference": differences[0],
        "pass": True,
    }


def _verify_scenes(resources, preregistration):
    manifest = strict_yaml(
        WORKSPACE / resources["scene_manifest"]["path"]
    )
    index_path = WORKSPACE / resources["compiled_scene_index"]["path"]
    index = strict_yaml(index_path)
    scenes = manifest["scenes"]
    _require(
        len(scenes) == 7
        and [scene["seed"] for scene in scenes]
        == [5141, 5142, 5143, 5144, 5145, 5146, 5147],
        "fresh scene seed schedule drifted",
    )
    dynamic = {scene["seed"]: scene for scene in scenes[:3]}
    crossing_times = {
        seed: [
            round(0.5 * (
                actor["trajectory"][0]["time_s"]
                + actor["trajectory"][-1]["time_s"]
            ), 3)
            for actor in scene["dynamic_agents"]
        ]
        for seed, scene in dynamic.items()
    }
    _require(
        crossing_times == {
            5141: [14.9],
            5142: [14.5, 16.3],
            5143: [11.0],
        }
        and all(
            scene["randomization"]["agent_time_jitter_s"] == 0.0
            for scene in dynamic.values()
        ),
        "preregistered dynamic timing screen drifted",
    )
    _require(
        index.get("scene_count") == 7
        and len(index.get("files", [])) == 14,
        "compiled scene index cardinality drifted",
    )
    for row in index["files"]:
        path = WORKSPACE / row["path"]
        _require(
            path.is_file() and sha256(path) == row["sha256"],
            "compiled scene child hash drifted",
        )
    schedule = preregistration["schedule"]
    _require(
        len(schedule) == 6
        and [row["sequence"] for row in schedule] == list(range(1, 7))
        and [row["attempt"] for row in schedule] == [1] * 6
        and {row["seed"] for row in schedule} == {5141, 5142, 5143}
        and all(row["seed"] not in range(5001, 5011) for row in schedule)
        and all(row["seed"] not in range(5111, 5136) for row in schedule),
        "paired fresh execution schedule drifted",
    )
    return {
        "manifest_scene_count": 7,
        "compiled_child_count": 14,
        "execution_scene_count": 3,
        "compile_support_only_count": 4,
        "fresh_execution_seeds": [5141, 5142, 5143],
        "timing_screen_pass": True,
        "pass": True,
    }


def _require_text(path, needles, label):
    text = Path(path).read_text(encoding="utf-8")
    missing = [needle for needle in needles if needle not in text]
    _require(not missing, "{} call sites missing {}".format(label, missing))
    return text


def _verify_call_sites(resources):
    runner = _require_text(
        WORKSPACE / resources["bounded_runner"]["path"],
        (
            "build_dependency_closure",
            "verify_dependency_closure",
            "AtomicAttemptJournal",
            "acquire_compiled_scene_lease",
            "materialize_scene_snapshot",
            "revalidate_scene_snapshot",
            "validate_readiness_raw_evidence",
            "bind_attempt_raw_evidence",
            "bind_terminal_attempt_evidence",
            "verify_teardown_restore",
            "authorize_launch_stop",
            "_process_matches",
        ),
        "bounded runner",
    )
    _require(
        runner.index("_verify_preflight(")
        < runner.index("EXECUTION_ROOT.mkdir"),
        "preflight is not before persistent execution state",
    )
    _require_text(
        WORKSPACE / resources["activation_listener"]["path"],
        (
            "TrackedObstacleArray",
            "tracker_message_count",
            "context_message_count",
            "scene_id",
            "attempt",
        ),
        "activation listener",
    )
    _require_text(
        WORKSPACE / resources["runtime_supervisor_node"]["path"],
        (
            "FootprintRuntimeTrack",
            "tuple(",
            "point.x, point.y",
            "config_sha256",
            "attempt_profile_id",
        ),
        "runtime supervisor node",
    )
    _require_text(
        WORKSPACE / resources["transaction_node"]["path"],
        (
            "execution_armed",
            "arm_execution",
            "restore_startup_two_phase",
            "independent_adapter",
            "supervisor_config_sha256",
        ),
        "transaction node",
    )
    _require_text(
        WORKSPACE / resources["assessor"]["path"],
        (
            "validate_persisted_attempt",
            "attempt_ledger",
            "integrity_failures",
        ),
        "persisted assessor",
    )
    return {
        "required_risk_ids": list(EXPECTED_RISK_IDS),
        "real_call_site_count": 6,
        "runner_preflight_before_ledger_or_subprocess": True,
        "startup_arm_write_barrier_present": True,
        "persisted_journal_assessor_present": True,
        "pass": True,
    }


def _verify_closure():
    frozen = strict_yaml(CLOSURE)
    generated = build_dependency_closure(WORKSPACE)
    _require(frozen == generated, "dependency closure is not reproducible")
    verification = verify_dependency_closure(
        WORKSPACE, frozen, generated["required_paths"]
    )
    _require(
        verification["closure_sha256"] == frozen["closure_sha256"],
        "dependency closure digest drifted",
    )
    return {
        **verification,
        "external_python_module_count": len(
            frozen["external_python_modules"]
        ),
        "external_runtime_binding_count": len(
            frozen["external_runtime_bindings"]
        ),
        "mechanically_reproduced": True,
    }


def build_report():
    contract = strict_yaml(CONTRACT)
    preregistration = strict_yaml(PREREGISTRATION)
    _require(
        contract.get("stage") == STAGE
        and contract.get("execution_authorized") is False
        and contract.get("runtime_ready") is False
        and preregistration.get("stage") == STAGE
        and preregistration.get("execution_authorized") is False
        and preregistration["budget"]["evidence_units_authorizable"] == 6
        and preregistration["budget"][
            "evidence_units_consumed_before_authorization"
        ] == 0,
        "integration review safety boundary drifted",
    )
    resources = _verify_resource_map(contract)
    r5_before = _tree_snapshot(
        WORKSPACE / "artifacts/v2/calibration/v2_04g_r5"
    )
    _require(
        r5_before == {
            "file_count": 68,
            "tree_sha256": EXPECTED_R5_TREE_SHA256,
        },
        "frozen R5 artifact tree drifted",
    )
    runtime = _verify_runtime_profiles(resources)
    scenes = _verify_scenes(resources, preregistration)
    call_sites = _verify_call_sites(resources)
    closure = _verify_closure()
    r5_after = _tree_snapshot(
        WORKSPACE / "artifacts/v2/calibration/v2_04g_r5"
    )
    _require(r5_before == r5_after, "review modified frozen R5 artifacts")
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": "fam_teb_v2_04g_r6_i1_execution_integration_review_1",
        "status": "execution_integration_review_pass_not_authorized",
        "review_result": "pass",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": True,
        "execution_authorized": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "single_factor_runtime_review": runtime,
        "fresh_scene_and_budget_review": scenes,
        "integrity_call_site_review": call_sites,
        "dependency_closure_review": closure,
        "resource_integrity": {
            "resource_count": len(resources),
            "all_hashes_match": True,
            "resources": resources,
            "r5_artifact_tree": r5_before,
            "r5_before_and_after_identical": True,
        },
        "budget": {
            "evidence_units_preregistered": 6,
            "evidence_units_authorized_by_review": 0,
            "evidence_units_consumed": 0,
            "attempt_limit_per_identity": 1,
            "retry_or_resume_allowed": False,
        },
        "authorization_boundary": {
            "authorization_created_by_reviewer": False,
            "separate_authorization_required": True,
            "cli_authorization_sha256_trust_anchor_required": True,
        },
        "seed_or_evidence_units_consumed": 0,
        "ros_started": False,
        "gazebo_started": False,
        "move_base_started": False,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "winner_ranked_or_frozen": False,
        "downstream_authorized": False,
        "claim_limit": "integration_review_only_no_execution_evidence",
    }


def review():
    if OUTPUT.exists():
        raise FileExistsError("integration review output already exists")
    if AUTHORIZATION.exists() or EXECUTION_ROOT.exists() or STAGE_REPORT.exists():
        raise R6I1ReviewError(
            "review must precede authorization and execution state"
        )
    report = build_report()
    _atomic_yaml(OUTPUT, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    report = review() if args.write else build_report()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
