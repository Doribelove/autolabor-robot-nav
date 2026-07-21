"""Deterministic calibration-only Anchor screening plan generation."""

import hashlib
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml

from teb_mode_manager.action_pipeline import AnchorBank, FeasibleActionDecoder

from .v2_contract import validate_typed_calibration_contract
from .v2_scene import canonical_sha256, load_v2_scene_manifest


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _screen_values(base: float, lower: float, upper: float, fraction: float):
    step = fraction * (upper - lower)
    low = max(lower, base - step)
    high = min(upper, base + step)
    # At a boundary retain two distinct, inward one-factor probes.
    if low == base:
        low = min(upper, base + 2.0 * step)
    if high == base:
        high = max(lower, base - 2.0 * step)
    if len({float(low), float(base), float(high)}) != 3:
        raise ValueError("cannot construct three distinct bounded screen values")
    lower_probe, upper_probe = sorted((low, high))
    return {-1: lower_probe, 1: upper_probe}


def apply_candidate_overlay(bank, values, overlay_id):
    """Apply the preregistered factorized overlay to an arbitrary candidate."""

    base = bank.validate_values(values, "calibration candidate base")
    if overlay_id not in bank.overlays:
        raise ValueError("unknown calibration overlay {}".format(overlay_id))
    overlay = bank.overlays[overlay_id]
    effective = dict(base)
    for name, factor in overlay.scale.items():
        effective[name] *= factor
    for name, offset in overlay.offset.items():
        effective[name] += offset
    feasible, reason_mask = FeasibleActionDecoder(bank)._intrinsic_feasible(effective, None)
    if reason_mask != 0:
        raise ValueError("candidate overlay required terminal projection")
    return feasible


def build_anchor_calibration_plan(
    contract_path: Any, workspace: Any,
) -> Dict[str, Any]:
    root = Path(workspace).resolve()
    contract_source = Path(contract_path).resolve()
    contract = yaml.safe_load(contract_source.read_text(encoding="utf-8"))
    validate_typed_calibration_contract(
        contract, workspace=root, verify_resources=True
    )
    resources = contract["resources"]
    bank_path = root / resources["anchor_bank"]["path"]
    scene_path = root / resources["calibration_scene_manifest"]["path"]
    bank = AnchorBank.from_file(bank_path)
    manifest = load_v2_scene_manifest(scene_path, root)
    calibration = contract["calibration"]
    step_fraction = float(calibration["coordinate_step_fraction_of_domain"])
    scenes_by_family = {}
    for scene in manifest["scenes"]:
        if scene["split"] != "calibration":
            raise ValueError("non-calibration scene reached candidate planner")
        scenes_by_family.setdefault(scene["family"], []).append(scene)
    decoder = FeasibleActionDecoder(bank)
    candidates = []
    planned_episodes = 0
    for anchor_id, coordinate_names in calibration["screening_coordinates"].items():
        anchor = bank.anchors[anchor_id]
        candidate_rows = [("center", None, 0, dict(anchor.values))]
        for name in coordinate_names:
            definition = bank.definitions[name]
            screen = _screen_values(
                float(anchor.values[name]), definition.lower, definition.upper,
                step_fraction,
            )
            for level in (-1, 1):
                values = dict(anchor.values)
                values[name] = float(screen[level])
                feasible, reason_mask = decoder._intrinsic_feasible(values, None)
                if reason_mask != 0:
                    raise ValueError("candidate required terminal projection")
                candidate_rows.append((name + ("_low" if level < 0 else "_high"), name, level, feasible))
        if len(candidate_rows) != calibration["candidate_budget_per_anchor"]:
            raise ValueError("candidate budget construction drifted")
        for local_index, (screen_id, coordinate, level, values) in enumerate(candidate_rows):
            validated = bank.validate_values(values, "calibration candidate")
            evaluations = []
            for family in calibration["anchor_scene_families"][anchor_id]:
                overlay = calibration["dynamic_overlay_by_family"][family]
                effective_values = apply_candidate_overlay(bank, validated, overlay)
                for scene in scenes_by_family[family]:
                    evaluations.append({
                        "scene_id": scene["scene_id"],
                        "split": scene["split"],
                        "seed": scene["seed"],
                        "family": family,
                        "dynamic_overlay": overlay,
                        "effective_profile_sha256": canonical_sha256(effective_values),
                    })
            candidate_id = "{}-c{:02d}-{}".format(anchor_id, local_index, screen_id)
            candidates.append({
                "candidate_id": candidate_id,
                "anchor_id": anchor_id,
                "base_profile_id": anchor.profile_id,
                "screen_coordinate": coordinate,
                "screen_level": level,
                "values": validated,
                "profile_sha256": canonical_sha256(validated),
                "evaluations": evaluations,
            })
            planned_episodes += len(evaluations)
    unique_ids = {row["candidate_id"] for row in candidates}
    if len(unique_ids) != len(candidates):
        raise ValueError("candidate IDs are not unique")
    expected = contract["acceptance_gates"]
    if len(candidates) != expected["generated_candidate_count"]:
        raise ValueError("generated candidate count drifted")
    if planned_episodes != expected["planned_calibration_episode_count"]:
        raise ValueError("planned calibration episode count drifted")
    return {
        "schema_version": "2.0",
        "stage": "V2-04B",
        "plan_id": "fam_teb_v2_04b_anchor_screen_plan_1",
        "status": "calibration_started",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "test_or_validation_selection_used": False,
        "contract": {
            "path": contract_source.relative_to(root).as_posix(),
            "sha256": _file_sha256(contract_source),
        },
        "anchor_bank": dict(resources["anchor_bank"]),
        "scene_manifest": dict(resources["calibration_scene_manifest"]),
        "strategy": calibration["strategy"],
        "selection_order": calibration["selection_order"],
        "candidate_count": len(candidates),
        "planned_episode_count": planned_episodes,
        "completed_navigation_episode_count": 0,
        "candidates": candidates,
        "claims": {
            "candidate_screen_preregistered": True,
            "anchor_calibration_complete": False,
            "anchor_values_frozen": False,
            "performance_improvement_observed": False,
        },
    }


def write_anchor_calibration_plan(plan: Mapping[str, Any], output_path: Any) -> None:
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(dict(plan), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
