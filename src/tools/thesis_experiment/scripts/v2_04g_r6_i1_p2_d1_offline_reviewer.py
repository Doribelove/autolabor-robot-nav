#!/usr/bin/env python3
"""Independent, pure-offline review of the V2-04G-P2-D1 attribution closure."""

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile

import yaml


WORKSPACE_DEFAULT = Path("/home/robot/robot_ws_base_rl")
MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d1.py"
)
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d1_offline_trace_attribution_contract.yaml"
)
BLOCKS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "v2_04g_p2_d1_trace_attribution_blocks.json"
)
REPORT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "v2_04g_p2_d1_trace_attribution_report.yaml"
)
SOURCE_FREEZE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "p2_d1_source_evidence_freeze.sha256"
)
REVIEW_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "v2_04g_p2_d1_offline_review.yaml"
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class ReviewError(ValueError):
    """Raised when the independent P2-D1 review fails closed."""


def require(condition, message):
    if not condition:
        raise ReviewError(message)


def safe_relative(value):
    relative = value if isinstance(value, Path) else Path(value)
    require(not relative.is_absolute() and relative.parts and ".." not in relative.parts, "unsafe relative path")
    return relative


def stable_read(workspace, relative):
    relative = safe_relative(relative)
    current = workspace
    for component in relative.parts:
        current = current / component
        require(not current.is_symlink(), "symbolic link forbidden: {}".format(relative))
    descriptor = os.open(
        str(workspace / relative),
        os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "not a regular file: {}".format(relative))
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "file changed during review: {}".format(relative),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(relative))
        return payload
    finally:
        os.close(descriptor)


def sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(payload, label):
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise ReviewError("{} parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} is not a mapping".format(label))
    return value


def load_json(payload, label):
    try:
        value = json.loads(payload.decode("utf-8"))
    except Exception as exc:
        raise ReviewError("{} parse failed: {}".format(label, exc))
    return value


def load_attribution_module(workspace):
    path = workspace / MODULE_RELATIVE
    spec = importlib.util.spec_from_file_location("p2_d1_review_rebuild", path)
    require(spec is not None and spec.loader is not None, "cannot load attribution module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def geometric_mean(values):
    require(values and all(type(value) in (int, float) and value > 0.0 for value in values), "geometric mean inputs differ")
    return math.exp(sum(math.log(float(value)) for value in values) / len(values))


def independent_minimum_time(distance, start_velocity, end_velocity, maximum_velocity, acceleration):
    accelerate_distance = (maximum_velocity ** 2 - start_velocity ** 2) / (2.0 * acceleration)
    decelerate_distance = (maximum_velocity ** 2 - end_velocity ** 2) / (2.0 * acceleration)
    if distance >= accelerate_distance + decelerate_distance:
        return (
            (maximum_velocity - start_velocity) / acceleration
            + (maximum_velocity - end_velocity) / acceleration
            + (distance - accelerate_distance - decelerate_distance) / maximum_velocity
        )
    peak = math.sqrt(acceleration * distance + (start_velocity ** 2 + end_velocity ** 2) / 2.0)
    return (peak - start_velocity) / acceleration + (peak - end_velocity) / acceleration


def verify_source_freeze(workspace, payload):
    entries = []
    for line in payload.decode("utf-8").splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, "source freeze line differs")
        expected, path_text = match.groups()
        relative = safe_relative(path_text)
        require(sha256(stable_read(workspace, relative)) == expected, "source freeze target differs: {}".format(relative))
        entries.append({"path": relative.as_posix(), "sha256": expected})
    require(len(entries) == 109, "source freeze count differs")
    require(len({entry["path"] for entry in entries}) == 109, "source freeze path duplicated")
    require(entries == sorted(entries, key=lambda entry: entry["path"]), "source freeze order differs")
    return entries


def independent_review(workspace):
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    blocks_payload = stable_read(workspace, BLOCKS_RELATIVE)
    report_payload = stable_read(workspace, REPORT_RELATIVE)
    freeze_payload = stable_read(workspace, SOURCE_FREEZE_RELATIVE)
    contract = load_yaml(contract_payload, "contract")
    blocks = load_json(blocks_payload, "block table")
    report = load_yaml(report_payload, "report")
    source_entries = verify_source_freeze(workspace, freeze_payload)

    require(contract.get("stage") == "V2-04G-P2-D1", "contract stage differs")
    require(contract.get("offline_only") is True, "contract is not offline-only")
    require(contract.get("execution_authorized") is False, "contract authorizes execution")
    require(contract.get("evidence_budget_authorized") == 0, "contract evidence budget differs")
    require(contract.get("training_budget_steps") == 0, "contract training budget differs")
    require(all(contract.get("forbidden_side_effects", {}).values()), "contract side-effect boundary differs")
    require(report.get("status") == "offline_trace_attribution_physical_room_pass", "report status differs")
    require(report.get("contract") == {
        "path": CONTRACT_RELATIVE.as_posix(),
        "sha256": sha256(contract_payload),
    }, "report contract binding differs")
    require(report.get("artifacts", {}).get("block_table") == {
        "path": BLOCKS_RELATIVE.as_posix(),
        "sha256": sha256(blocks_payload),
        "rows": 30,
    }, "report block binding differs")
    require(type(blocks) is list and len(blocks) == 30, "block table count differs")
    require([row.get("seed") for row in blocks] == list(range(5261, 5291)), "block seed roster differs")
    require(len({row.get("block_id") for row in blocks}) == 30, "block IDs differ")

    module = load_attribution_module(workspace)
    rebuilt_report, rebuilt_blocks, rebuilt_closure = module.build_outputs(workspace)
    require(module.canonical_blocks_bytes(rebuilt_blocks) == blocks_payload, "independent deterministic block rebuild differs")
    require(module.report_bytes(rebuilt_report) == report_payload, "independent deterministic report rebuild differs")
    require(len(rebuilt_closure) == 108, "rebuilt source closure count differs")

    for row in blocks:
        independently_retimed = (
            row["first_cruise_elapsed_s"]
            + independent_minimum_time(
                row["post_cruise_path_m"],
                row["first_cruise_velocity_mps"],
                0.0,
                1.2,
                0.8,
            )
            + row["post_terminal_accounting_s"]
        )
        require(abs(independently_retimed - row["post_cruise_envelope_navigation_time_s"]) <= 1e-9, "independent kinematic retiming differs")
        require(abs(
            row["treatment_navigation_time_s"]
            - row["post_cruise_envelope_navigation_time_s"]
            - row["post_cruise_recoverable_time_s"]
        ) <= 1e-9, "recoverable-time closure differs")

    legacy_geometric = geometric_mean([row["legacy_navigation_time_s"] for row in blocks])
    treatment_geometric = geometric_mean([row["treatment_navigation_time_s"] for row in blocks])
    envelope_geometric = geometric_mean([row["post_cruise_envelope_navigation_time_s"] for row in blocks])
    envelope_reduction = 1.0 - envelope_geometric / legacy_geometric
    primary = report["same_path_longitudinal_envelopes"]["frozen_prefix_post_cruise_primary"]
    require(abs(legacy_geometric - report["frozen_p1_result"]["legacy_geometric_restricted_time_s"]) <= 1e-12, "legacy geometric time differs")
    require(abs(treatment_geometric - report["frozen_p1_result"]["treatment_geometric_restricted_time_s"]) <= 1e-12, "treatment geometric time differs")
    require(abs(envelope_geometric - primary["geometric_time_s"]) <= 1e-12, "envelope geometric time differs")
    require(abs(envelope_reduction - primary["reduction_fraction_vs_paired_legacy"]) <= 1e-12, "envelope reduction differs")
    require(envelope_reduction >= 0.10, "independent envelope point does not reach ten percent")
    require(primary["paired_bootstrap_one_sided_95_percent_lower"] >= 0.10, "bootstrap lower does not reach ten percent")
    require(primary["blocks_at_or_above_8_percent"] == 29, "8-percent block count differs")
    require(primary["blocks_at_or_above_10_percent"] == 24, "10-percent block count differs")

    target_checks = {}
    for label, target in (("8percent", 0.08), ("10percent", 0.10)):
        target_row = report["target_closure"][label]
        recovery = target_row["required_fraction_of_post_cruise_recoverable_time"]
        recovered_times = [
            row["treatment_navigation_time_s"]
            - recovery * row["post_cruise_recoverable_time_s"]
            for row in blocks
        ]
        recovered_reduction = 1.0 - geometric_mean(recovered_times) / legacy_geometric
        require(abs(recovered_reduction - target) <= 1e-12, "target recovery closure differs")
        multiplier = target_row["first_order_actual_cap_dwell_multiplier"]
        cap_scaled_times = [
            row["treatment_navigation_time_s"]
            - row["actual_cap_bound_dwell_s"] * (1.0 - 1.0 / multiplier)
            for row in blocks
        ]
        cap_scaled_reduction = 1.0 - geometric_mean(cap_scaled_times) / legacy_geometric
        require(abs(cap_scaled_reduction - target) <= 1e-12, "cap exposure closure differs")
        require(target_row["inside_current_anchor_bank_speed_bound"] is False, "out-of-bank estimate was hidden")
        target_checks[label] = {
            "recovered_reduction_fraction": recovered_reduction,
            "cap_exposure_reduction_fraction": cap_scaled_reduction,
        }

    historical = report["historical_single_coordinate_check"]
    historical_rows = historical["rows"]
    independent_speed_effect = 1.0 - historical_rows["center"]["navigation_time_s"] / historical_rows["speed_high"]["navigation_time_s"]
    independent_weight_effect = 1.0 - historical_rows["weight_high"]["navigation_time_s"] / historical_rows["weight_low"]["navigation_time_s"]
    require(abs(independent_speed_effect - historical["speed_ceiling_high_1_09_to_center_1_20_time_reduction_fraction"]) <= 1e-12, "historical speed effect differs")
    require(abs(independent_weight_effect - historical["weight_optimaltime_1_03_to_1_37_time_reduction_fraction"]) <= 1e-12, "historical weight effect differs")
    require(independent_speed_effect >= 0.08, "historical speed direction check differs")
    require(independent_weight_effect < 0.08, "historical weight limitation differs")

    expected_decision = {
        "aggregate_physical_room_for_8_to_10_percent": True,
        "factor_hypothesis_retained_for_design": True,
        "specific_executable_runtime_factor_qualified": False,
        "counterfactual_gain_achieved": False,
        "causal_performance_improvement_demonstrated": False,
        "new_speed_bound_safety_qualified": False,
        "simulation_authorized": False,
        "decision_reason": "post_cruise_same_path_point_and_one_sided_lower_exceed_ten_percent_but_runtime_mapping_and_causal_gain_remain_unqualified",
    }
    require(report.get("decision") == expected_decision, "decision boundary differs")
    require(report.get("frozen_p1_result", {}).get("performance_improvement_demonstrated") is False, "P1 was positively reinterpreted")
    require(all(value is False for value in report.get("side_effect_audit", {}).values()), "offline side-effect audit differs")

    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-D1",
        "review_id": "fam_teb_v2_04g_p2_d1_independent_offline_review_1",
        "status": "offline_trace_attribution_independent_review_pass",
        "review_result": "pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "bindings": {
            "contract": {"path": CONTRACT_RELATIVE.as_posix(), "sha256": sha256(contract_payload)},
            "source_evidence_freeze": {"path": SOURCE_FREEZE_RELATIVE.as_posix(), "sha256": sha256(freeze_payload)},
            "block_table": {"path": BLOCKS_RELATIVE.as_posix(), "sha256": sha256(blocks_payload)},
            "attribution_report": {"path": REPORT_RELATIVE.as_posix(), "sha256": sha256(report_payload)},
        },
        "integrity_review": {
            "source_paths_rehashed": len(source_entries),
            "paired_blocks_recomputed": len(blocks),
            "deterministic_block_rebuild_byte_equal": True,
            "deterministic_report_rebuild_byte_equal": True,
            "independent_kinematic_rows_recomputed": len(blocks),
            "integrity_failures": [],
        },
        "independent_result": {
            "legacy_geometric_time_s": legacy_geometric,
            "treatment_geometric_time_s": treatment_geometric,
            "post_cruise_envelope_geometric_time_s": envelope_geometric,
            "post_cruise_envelope_reduction_fraction": envelope_reduction,
            "one_sided_95_percent_lower_fraction": primary["paired_bootstrap_one_sided_95_percent_lower"],
            "target_checks": target_checks,
            "historical_speed_direction_effect_fraction": independent_speed_effect,
            "historical_weight_direction_effect_fraction": independent_weight_effect,
            "physical_room_for_8_to_10_percent": True,
        },
        "claim_review": {
            "specific_runtime_factor_qualified": False,
            "causal_gain_demonstrated": False,
            "p1_negative_result_preserved": True,
            "new_speed_bound_safety_qualified": False,
            "simulation_authorized": False,
        },
        "side_effect_audit": report["side_effect_audit"],
    }


def review_bytes(review):
    return yaml.safe_dump(review, sort_keys=False, allow_unicode=True).encode("utf-8")


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".{}-".format(path.name), dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE_DEFAULT)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    arguments = parser.parse_args()
    workspace = arguments.workspace.resolve()
    review = independent_review(workspace)
    payload = review_bytes(review)
    if arguments.write:
        atomic_write(workspace / REVIEW_RELATIVE, payload)
    else:
        require(stable_read(workspace, REVIEW_RELATIVE) == payload, "review report is not byte-equal to rebuild")
    print(review["status"])
    print("source_paths={}".format(review["integrity_review"]["source_paths_rehashed"]))
    print("review_sha256={}".format(sha256(payload)))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ReviewError, OSError, ValueError) as exc:
        print("p2_d1_offline_review_failed: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)

