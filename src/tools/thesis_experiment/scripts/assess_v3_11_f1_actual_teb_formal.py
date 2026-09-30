#!/usr/bin/env python3
"""Assess V3-11-F1 from raw paired actual-TEB formal evidence."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_11_f1_actual_teb_formal_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_formal_release.yaml"
)
SCHEDULE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/exact_schedule.yaml"
)
DEPENDENCY_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/dependency_freeze.yaml"
)
ANALYSIS_RELEASE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/formal_analysis_release.yaml"
)
EXECUTION_RELATIVE = Path("artifacts/v3/actual_teb/v3_11_f1_execution")
ASSESSMENT_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "formal_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "formal_evidence_freeze.yaml"
)
EXPECTED_CONTRACT_SHA256 = (
    "2f142590e81fa32af4dfde37d4aa442f1abb17973959cbad8893dc080214e53d"
)
EXPECTED_AUTHORIZATION_SHA256 = (
    "26a2173f11b32185409721e4027c419f87dd02e6afe965c0dfe790350705f0c2"
)
EXPECTED_RELEASE_SHA256 = (
    "b21835d5d55a55fd130aae836c609fd117481b9d4cd54155ba8fc78b57dc9ea6"
)
EXPECTED_SCHEDULE_SHA256 = (
    "0b84b742469ebf693b0e6ec9837e505035b4ac42b0305bfc0c0762346214d57e"
)
EXPECTED_DEPENDENCY_FREEZE_SHA256 = (
    "79c8225588af2a381571457417a9450177152ffc2bec832c1eb559b45484b661"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_bytes(path):
    require(path.is_file(), "required file absent: {}".format(path))
    return path.read_bytes()


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    return sha256_bytes(read_bytes(path))


def load_yaml(path):
    return yaml.safe_load(read_bytes(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_once(path, payload):
    require(not path.exists(), "immutable output exists: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def close(a, b, tolerance=1e-12):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)


def expected_rows(contract):
    formal = contract["formal_preregistration"]
    methods = list(contract["methods"]["identifiers"])
    rows = []
    sequence = 0
    for zero_index, seed in enumerate(
        range(int(formal["seed_start"]), int(formal["seed_end"]) + 1)
    ):
        stratum = formal["strata"][zero_index % len(formal["strata"])]
        order = methods if zero_index % 2 == 0 else list(reversed(methods))
        for method in order:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "block_index": zero_index + 1,
                    "seed": seed,
                    "scene_id": stratum["scene_id"],
                    "actor_policy": stratum["actor_policy"],
                    "method": method,
                }
            )
    return rows


def expected_actor_parameters(seed):
    return {
        "start_coordinate_m": -(
            2.85 + ((int(seed) * 7) % 9) * 0.05
        ),
        "base_speed_mps": 0.90 + ((int(seed) * 3) % 5) * 0.05,
    }


def episode_directory(row):
    return WORKSPACE / EXECUTION_RELATIVE / "episodes" / (
        "{:03d}_{}_{}_{}_{}".format(
            row["sequence"],
            row["seed"],
            row["scene_id"],
            row["actor_policy"],
            row["method"],
        )
    )


def validate_preexecution_closure(analysis_release_sha256):
    expected = {
        WORKSPACE / CONTRACT_RELATIVE: EXPECTED_CONTRACT_SHA256,
        WORKSPACE / AUTHORIZATION_RELATIVE: EXPECTED_AUTHORIZATION_SHA256,
        WORKSPACE / RELEASE_RELATIVE: EXPECTED_RELEASE_SHA256,
        WORKSPACE / SCHEDULE_RELATIVE: EXPECTED_SCHEDULE_SHA256,
        WORKSPACE / DEPENDENCY_FREEZE_RELATIVE: (
            EXPECTED_DEPENDENCY_FREEZE_SHA256
        ),
    }
    for path, digest in expected.items():
        require(sha256_file(path) == digest, "released resource changed: {}".format(path))
    analysis_release_path = WORKSPACE / ANALYSIS_RELEASE_RELATIVE
    require(
        sha256_file(analysis_release_path) == analysis_release_sha256,
        "analysis release hash differs",
    )
    analysis_release = load_yaml(analysis_release_path)
    require(
        analysis_release["status"] == "immutable_preexecution_analysis_release",
        "analysis release status differs",
    )
    require(
        analysis_release["formal_execution_release_sha256"]
        == EXPECTED_RELEASE_SHA256,
        "analysis release binds a different execution release",
    )
    for resource in analysis_release["resources"]:
        path = WORKSPACE / resource["path"]
        require(
            sha256_file(path) == resource["sha256"],
            "analysis resource changed: {}".format(resource["path"]),
        )
        require(
            path.stat().st_size == resource["size_bytes"],
            "analysis resource size changed: {}".format(resource["path"]),
        )
    dependency = load_yaml(WORKSPACE / DEPENDENCY_FREEZE_RELATIVE)
    require(
        dependency["status"] == "formal_dependency_freeze_pass",
        "formal dependency freeze did not pass",
    )
    for resource in dependency["resources"]:
        path = WORKSPACE / resource["path"]
        require(
            sha256_file(path) == resource["sha256"],
            "formal dependency changed: {}".format(resource["path"]),
        )
        require(
            path.stat().st_size == resource["size_bytes"],
            "formal dependency size changed: {}".format(resource["path"]),
        )
    return analysis_release


def read_journal(path):
    records = []
    for line_number, raw in enumerate(read_bytes(path).splitlines(), start=1):
        require(raw.strip(), "blank journal line {}".format(line_number))
        value = json.loads(raw)
        require(isinstance(value, dict), "journal record is not an object")
        records.append(value)
    return records


def replay_execution(contract, rows):
    execution_root = WORKSPACE / EXECUTION_RELATIVE
    journal_path = execution_root / "attempt_journal.jsonl"
    summary_path = execution_root / "matrix_summary.yaml"
    journal = read_journal(journal_path)
    require(len(journal) == 2 * len(rows) + 2, "journal event count differs")
    opened = journal[0]
    require(opened["event"] == "execution_opened", "journal does not open")
    require(opened["stage"] == "V3-11-F1", "journal stage differs")
    require(opened["phase"] == "formal", "journal phase differs")
    require(
        opened["release_sha256"] == EXPECTED_RELEASE_SHA256,
        "journal release hash differs",
    )
    require(
        opened["contract_sha256"] == EXPECTED_CONTRACT_SHA256,
        "journal contract hash differs",
    )
    require(
        opened["authorization_sha256"] == EXPECTED_AUTHORIZATION_SHA256,
        "journal authorization hash differs",
    )
    require(
        opened["schedule_sha256"] == EXPECTED_SCHEDULE_SHA256,
        "journal schedule hash differs",
    )
    require(opened["authorized_units"] == len(rows), "journal budget differs")
    require(opened["training_steps"] == 0, "journal training boundary differs")
    reports = []
    evidence_paths = [
        CONTRACT_RELATIVE,
        AUTHORIZATION_RELATIVE,
        RELEASE_RELATIVE,
        SCHEDULE_RELATIVE,
        DEPENDENCY_FREEZE_RELATIVE,
        ANALYSIS_RELEASE_RELATIVE,
        EXECUTION_RELATIVE / "attempt_journal.jsonl",
        EXECUTION_RELATIVE / "matrix_summary.yaml",
    ]
    for index, row in enumerate(rows):
        consumed = journal[1 + 2 * index]
        complete = journal[2 + 2 * index]
        require(consumed["event"] == "unit_consumed", "unit was not consumed")
        require(consumed["row"] == row, "consumed row differs")
        require(consumed["sequence"] == row["sequence"], "sequence differs")
        require(consumed["attempt"] == 1, "attempt identity differs")
        require(complete["event"] == "unit_complete", "unit did not complete")
        require(complete["row"] == row, "complete row differs")
        require(complete["sequence"] == row["sequence"], "sequence differs")
        root = episode_directory(row)
        require(root.is_dir(), "episode directory absent: {}".format(root))
        report_path = root / "episode_report.yaml"
        receipt_path = root / "matrix_receipt.yaml"
        world_path = root / "world.sdf"
        log_path = root / "roslaunch.log"
        report = load_yaml(report_path)
        receipt = load_yaml(receipt_path)
        require(receipt["row"] == row, "receipt row differs")
        expected_actor = expected_actor_parameters(row["seed"])
        for key, value in expected_actor.items():
            require(
                close(receipt["actor_generated_parameters"][key], value),
                "actor derivation differs for {}".format(key),
            )
        require(receipt["episode_report"] == report, "embedded report differs")
        require(
            receipt["episode_report_sha256"] == sha256_file(report_path),
            "report hash differs",
        )
        require(
            receipt["world_sha256"] == sha256_file(world_path),
            "world hash differs",
        )
        require(
            receipt["roslaunch_log_sha256"] == sha256_file(log_path),
            "launch log hash differs",
        )
        require(receipt["roslaunch_return_code"] == 0, "roslaunch failed")
        require(receipt["timed_out"] is False, "episode timed out")
        require(
            complete["matrix_receipt_sha256"] == sha256_file(receipt_path),
            "receipt hash differs",
        )
        require(
            complete["episode_pass"] == report["episode_pass"],
            "journal pass flag differs",
        )
        require(
            complete["goal_reached"] == report["result"]["goal_reached"],
            "journal goal flag differs",
        )
        require(
            close(
                complete["restricted_elapsed_s"],
                report["result"]["restricted_elapsed_s"],
            ),
            "journal time differs",
        )
        require(
            close(
                complete["minimum_conservative_clearance_m"],
                report["result"]["minimum_conservative_clearance_m"],
            ),
            "journal clearance differs",
        )
        require(report["stage"] == "V3-11", "runtime report stage differs")
        require(report["method"] == row["method"], "method differs")
        require(report["scene_id"] == row["scene_id"], "scene differs")
        require(report["seed"] == row["seed"], "seed differs")
        require(
            report["actor_policy_evaluator_only"] == row["actor_policy"],
            "actor policy differs",
        )
        require(report["simulation_only"] is True, "simulation boundary differs")
        require(report["training_steps"] == 0, "training boundary differs")
        require(report["hardware_accessed"] is False, "hardware boundary differs")
        require(report["real_vehicle_used"] is False, "real vehicle boundary differs")
        require(
            report["planner_contract"]["sha256"]
            == contract["upstream"]["planner_contract_sha256"],
            "planner contract differs",
        )
        require(
            report["predictor_model"]["sha256"]
            == contract["upstream"]["predictor_model_sha256"],
            "predictor model differs",
        )
        require(
            0.0 <= float(report["result"]["restricted_elapsed_s"]) <= 22.0,
            "restricted time is outside frozen support",
        )
        reports.append(report)
        for path in sorted(item for item in root.rglob("*") if item.is_file()):
            evidence_paths.append(path.relative_to(WORKSPACE))
    terminal = journal[-1]
    require(terminal["event"] == "execution_complete", "journal did not close")
    for key in ("authorized_units", "consumed_units", "completed_units"):
        require(terminal[key] == len(rows), "{} count differs".format(key))
    require(terminal["forfeited_units"] == 0, "formal units were forfeited")
    require(
        terminal["summary_sha256"] == sha256_file(summary_path),
        "terminal summary hash differs",
    )
    require(
        terminal["formal_assessment_complete"] is False,
        "runner improperly made a formal claim",
    )
    return reports, sorted(set(evidence_paths), key=str)


def bootstrap_lower(v3_times, teb_times, seed, replicates, alpha):
    require(len(v3_times) == len(teb_times), "pair lengths differ")
    count = len(v3_times)
    generator = random.Random(int(seed))
    estimates = []
    for _ in range(int(replicates)):
        v3_sum = 0.0
        teb_sum = 0.0
        for _draw in range(count):
            index = generator.randrange(count)
            v3_sum += v3_times[index]
            teb_sum += teb_times[index]
        estimates.append(100.0 * (teb_sum - v3_sum) / teb_sum)
    estimates.sort()
    order_index = int(math.floor(float(alpha) * int(replicates)))
    require(0 <= order_index < len(estimates), "bootstrap order index invalid")
    return {
        "one_sided_lower_reduction_percent": estimates[order_index],
        "order_index_zero_based": order_index,
        "minimum_bootstrap_reduction_percent": estimates[0],
        "median_bootstrap_reduction_percent": estimates[len(estimates) // 2],
        "maximum_bootstrap_reduction_percent": estimates[-1],
        "estimates_digest": sha256_bytes(
            "\n".join("{:.17g}".format(value) for value in estimates).encode(
                "ascii"
            )
        ),
    }


def compute_metrics(reports, rows, statistics):
    by_method = {
        method: [report for report in reports if report["method"] == method]
        for method in ("v3_spatial_safe", "actual_teb")
    }
    by_block = {}
    for row, report in zip(rows, reports):
        by_block.setdefault(row["block_index"], {})[row["method"]] = report
    require(len(by_block) == 30, "formal paired block count differs")
    pairs = []
    pair_differences = []
    for block_index in sorted(by_block):
        block = by_block[block_index]
        require(
            set(block) == {"v3_spatial_safe", "actual_teb"},
            "paired method set differs",
        )
        v3 = block["v3_spatial_safe"]
        teb = block["actual_teb"]
        require(v3["scene_id"] == teb["scene_id"], "paired scene differs")
        require(
            v3["actor_policy_evaluator_only"]
            == teb["actor_policy_evaluator_only"],
            "paired policy differs",
        )
        v3_time = float(v3["result"]["restricted_elapsed_s"])
        teb_time = float(teb["result"]["restricted_elapsed_s"])
        pairs.append(
            {
                "block_index": block_index,
                "seed": rows[2 * (block_index - 1)]["seed"],
                "scene_id": v3["scene_id"],
                "actor_policy": v3["actor_policy_evaluator_only"],
                "v3_restricted_elapsed_s": v3_time,
                "actual_teb_restricted_elapsed_s": teb_time,
                "v3_minus_teb_s": v3_time - teb_time,
                "block_reduction_percent": (
                    100.0 * (teb_time - v3_time) / teb_time
                ),
            }
        )
        v3_actor = v3["warmup"]["gate_actor_track"]
        teb_actor = teb["warmup"]["gate_actor_track"]
        pair_differences.append(
            {
                "block_index": block_index,
                "actor_coordinate_absolute_difference_m": abs(
                    float(v3_actor["coordinate_m"])
                    - float(teb_actor["coordinate_m"])
                ),
                "actor_speed_absolute_difference_mps": abs(
                    float(v3_actor["coordinate_speed_mps"])
                    - float(teb_actor["coordinate_speed_mps"])
                ),
            }
        )
    v3_times = [item["v3_restricted_elapsed_s"] for item in pairs]
    teb_times = [item["actual_teb_restricted_elapsed_s"] for item in pairs]
    mean_v3 = sum(v3_times) / len(v3_times)
    mean_teb = sum(teb_times) / len(teb_times)
    point = 100.0 * (mean_teb - mean_v3) / mean_teb
    bootstrap = bootstrap_lower(
        v3_times,
        teb_times,
        statistics["bootstrap_seed"],
        statistics["bootstrap_replicates"],
        statistics["one_sided_alpha"],
    )
    strata = {}
    for item in pairs:
        key = "{}__{}".format(item["scene_id"], item["actor_policy"])
        strata.setdefault(key, []).append(item)
    require(
        set(strata)
        == {
            "open_crossing_90__cautious",
            "open_crossing_90__hesitant",
            "wide_corridor_crossing_70__cautious",
            "wide_corridor_crossing_70__hesitant",
            "bounded_corridor_crossing_110__cautious",
            "bounded_corridor_crossing_110__hesitant",
        },
        "formal stratum support differs",
    )
    stratum_metrics = {}
    for key, items in sorted(strata.items()):
        require(len(items) == 5, "stratum block count differs: {}".format(key))
        local_v3 = sum(x["v3_restricted_elapsed_s"] for x in items) / len(items)
        local_teb = (
            sum(x["actual_teb_restricted_elapsed_s"] for x in items)
            / len(items)
        )
        stratum_metrics[key] = {
            "paired_blocks": len(items),
            "mean_v3_restricted_elapsed_s": local_v3,
            "mean_actual_teb_restricted_elapsed_s": local_teb,
            "mean_reduction_percent": (
                100.0 * (local_teb - local_v3) / local_teb
            ),
        }
    return {
        "episode_count": len(reports),
        "paired_block_count": len(pairs),
        "episodes_by_method": {
            method: len(method_reports)
            for method, method_reports in by_method.items()
        },
        "episode_pass_count_by_method": {
            method: sum(report["episode_pass"] is True for report in method_reports)
            for method, method_reports in by_method.items()
        },
        "goal_success_rate_by_method": {
            method: (
                sum(
                    report["result"]["goal_reached"] is True
                    for report in method_reports
                )
                / len(method_reports)
            )
            for method, method_reports in by_method.items()
        },
        "goal_success_rate_difference_v3_minus_actual_teb": (
            sum(
                report["result"]["goal_reached"] is True
                for report in by_method["v3_spatial_safe"]
            )
            / len(by_method["v3_spatial_safe"])
            - sum(
                report["result"]["goal_reached"] is True
                for report in by_method["actual_teb"]
            )
            / len(by_method["actual_teb"])
        ),
        "mean_restricted_elapsed_s_by_method": {
            "v3_spatial_safe": mean_v3,
            "actual_teb": mean_teb,
        },
        "point_reduction_percent": point,
        "one_sided_alpha": statistics["one_sided_alpha"],
        "bootstrap_seed": statistics["bootstrap_seed"],
        "bootstrap_replicates": statistics["bootstrap_replicates"],
        "bootstrap": bootstrap,
        "stratum_metrics": stratum_metrics,
        "minimum_conservative_clearance_m_by_method": {
            method: min(
                float(report["result"]["minimum_conservative_clearance_m"])
                for report in method_reports
            )
            for method, method_reports in by_method.items()
        },
        "contact_state_count_total": sum(
            int(report["result"]["contact_state_count"]) for report in reports
        ),
        "common_gate_valid_rate": (
            sum(report["gate_physical_valid"] is True for report in reports)
            / len(reports)
        ),
        "actor_mode_match_rate": (
            sum(
                report["warmup"]["actor_mode_matches_evaluator_policy"] is True
                for report in reports
            )
            / len(reports)
        ),
        "v3_spatial_selected_rate": (
            sum(
                report["v3_plan"]["selected_candidate_id"] == "spatial_left_v120"
                for report in by_method["v3_spatial_safe"]
            )
            / len(by_method["v3_spatial_safe"])
        ),
        "maximum_paired_gate_actor_coordinate_difference_m": max(
            item["actor_coordinate_absolute_difference_m"]
            for item in pair_differences
        ),
        "maximum_paired_gate_actor_speed_difference_mps": max(
            item["actor_speed_absolute_difference_mps"]
            for item in pair_differences
        ),
        "maximum_v3_tracking_error_m": max(
            float(report["result"]["tracking_error_max_m"])
            for report in by_method["v3_spatial_safe"]
        ),
        "minimum_actual_teb_local_plan_count": min(
            int(report["result"]["local_plan_count"])
            for report in by_method["actual_teb"]
        ),
        "actual_teb_action_succeeded_rate": (
            sum(
                report["result"]["action_state_name"] == "SUCCEEDED"
                for report in by_method["actual_teb"]
            )
            / len(by_method["actual_teb"])
        ),
        "paired_blocks": pairs,
        "gate_pair_differences": pair_differences,
    }


def recompute_assessment(analysis_release_sha256):
    analysis_release = validate_preexecution_closure(analysis_release_sha256)
    contract = load_yaml(WORKSPACE / CONTRACT_RELATIVE)
    authorization = load_yaml(WORKSPACE / AUTHORIZATION_RELATIVE)
    release = load_yaml(WORKSPACE / RELEASE_RELATIVE)
    schedule = load_yaml(WORKSPACE / SCHEDULE_RELATIVE)
    require(contract["stage"] == "V3-11-F1", "contract stage differs")
    require(
        authorization["formal_execution_authorized"] is True,
        "formal execution was not authorized",
    )
    require(
        release["formal_assessment_required_before_claim"] is True,
        "release claim boundary differs",
    )
    rows = expected_rows(contract)
    require(schedule["rows"] == rows, "formal schedule differs")
    require(schedule["row_count"] == 60, "formal schedule count differs")
    reports, evidence_paths = replay_execution(contract, rows)
    statistics = contract["formal_preregistration"]["statistics"]
    metrics = compute_metrics(reports, rows, statistics)
    qualification_contract = load_yaml(
        WORKSPACE
        / "config/thesis_experiments/v3/"
        "v3_11_a0_actual_teb_fair_baseline_contract.yaml"
    )
    integrity = qualification_contract["qualification_gates"]
    safety = statistics["safety_task_gates"]
    effect = statistics["meaningful_effect_gate"]
    gates = {
        "complete_exact_fresh_schedule_pass": (
            metrics["episode_count"] == statistics["schedule_rows"]
            and metrics["paired_block_count"] == statistics["paired_blocks"]
        ),
        "attempt_identity_no_retry_or_forfeiture_pass": True,
        "simulation_zero_training_boundary_pass": all(
            report["simulation_only"] is True
            and report["training_steps"] == 0
            and report["hardware_accessed"] is False
            and report["real_vehicle_used"] is False
            for report in reports
        ),
        "common_gate_integrity_pass": (
            metrics["common_gate_valid_rate"]
            >= integrity["common_gate_valid_rate_min"]
        ),
        "actor_mode_integrity_pass": (
            metrics["actor_mode_match_rate"]
            >= integrity[
                "actor_mode_matches_evaluator_policy_rate_min"
            ]
        ),
        "paired_gate_actor_coordinate_integrity_pass": (
            metrics["maximum_paired_gate_actor_coordinate_difference_m"]
            <= integrity["paired_gate_actor_coordinate_difference_max_m"]
        ),
        "paired_gate_actor_speed_integrity_pass": (
            metrics["maximum_paired_gate_actor_speed_difference_mps"]
            <= integrity["paired_gate_actor_speed_difference_max_mps"]
        ),
        "v3_spatial_selection_integrity_pass": (
            metrics["v3_spatial_selected_rate"]
            >= integrity["v3_spatial_selected_rate_min"]
        ),
        "v3_tracking_integrity_pass": (
            metrics["maximum_v3_tracking_error_m"]
            <= integrity["v3_tracking_error_max_m"]
        ),
        "actual_teb_execution_integrity_pass": (
            metrics["minimum_actual_teb_local_plan_count"] > 0
            and close(metrics["actual_teb_action_succeeded_rate"], 1.0)
        ),
        "collision_gate_pass": (
            metrics["contact_state_count_total"]
            <= safety["collision_count_max"]
        ),
        "conservative_clearance_gate_pass": all(
            value >= safety["conservative_clearance_min_m"]
            for value in metrics[
                "minimum_conservative_clearance_m_by_method"
            ].values()
        ),
        "v3_goal_success_noninferiority_pass": (
            metrics["goal_success_rate_difference_v3_minus_actual_teb"]
            >= -float(safety["v3_goal_success_noninferiority_margin"])
        ),
        "point_meaningful_effect_pass": (
            metrics["point_reduction_percent"]
            >= effect["point_reduction_percent_min"]
        ),
        "one_sided_lower_meaningful_effect_pass": (
            metrics["bootstrap"]["one_sided_lower_reduction_percent"]
            >= effect["one_sided_lower_reduction_percent_min"]
        ),
        "each_scene_policy_stratum_nonnegative_pass": all(
            item["mean_reduction_percent"]
            >= effect["each_scene_policy_stratum_mean_reduction_percent_min"]
            for item in metrics["stratum_metrics"].values()
        ),
        "preexecution_analysis_release_pass": (
            analysis_release["formal_execution_release_sha256"]
            == EXPECTED_RELEASE_SHA256
        ),
    }
    integrity_gate_names = [
        "complete_exact_fresh_schedule_pass",
        "attempt_identity_no_retry_or_forfeiture_pass",
        "simulation_zero_training_boundary_pass",
        "common_gate_integrity_pass",
        "actor_mode_integrity_pass",
        "paired_gate_actor_coordinate_integrity_pass",
        "paired_gate_actor_speed_integrity_pass",
        "v3_spatial_selection_integrity_pass",
        "v3_tracking_integrity_pass",
        "actual_teb_execution_integrity_pass",
        "preexecution_analysis_release_pass",
    ]
    formal_result = all(gates[name] for name in integrity_gate_names)
    performance = formal_result and all(gates.values())
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1",
        "status": (
            "actual_teb_fair_formal_meaningful_improvement_demonstrated"
            if performance
            else (
                "actual_teb_fair_formal_valid_negative"
                if formal_result
                else "actual_teb_fair_formal_integrity_failure"
            )
        ),
        "execution_release_sha256": EXPECTED_RELEASE_SHA256,
        "analysis_release_sha256": analysis_release_sha256,
        "contract_sha256": EXPECTED_CONTRACT_SHA256,
        "authorization_sha256": EXPECTED_AUTHORIZATION_SHA256,
        "schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "execution_summary_sha256": sha256_file(
            WORKSPACE / EXECUTION_RELATIVE / "matrix_summary.yaml"
        ),
        "execution_journal_sha256": sha256_file(
            WORKSPACE / EXECUTION_RELATIVE / "attempt_journal.jsonl"
        ),
        "assessment_basis": (
            "raw 30-block paired replay with pre-released deterministic "
            "200000-replicate bootstrap"
        ),
        "metrics": metrics,
        "gates": gates,
        "formal_result": formal_result,
        "performance_improvement_demonstrated": performance,
        "training_steps": 0,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
        "claim_scope": {
            "actual_teb_baseline": True,
            "three_frozen_single_actor_simulation_templates": True,
            "supported_actor_policies": [cautious, hesitant],
            "oracle_actor_track_shared_by_both_methods": True,
            "perception_robustness_claim_authorized": False,
            "general_navigation_safety_claim_authorized": False,
            "deployment_claim_authorized": False,
            "real_vehicle_claim_authorized": False,
        },
    }
    return assessment, evidence_paths


def evidence_freeze(evidence_paths, assessment_payload):
    resources = [
        {
            "path": str(path),
            "sha256": sha256_file(WORKSPACE / path),
            "size_bytes": (WORKSPACE / path).stat().st_size,
        }
        for path in evidence_paths
    ]
    resources.append(
        {
            "path": str(ASSESSMENT_RELATIVE),
            "sha256": sha256_bytes(assessment_payload),
            "size_bytes": len(assessment_payload),
        }
    )
    return {
        "schema_version": "1.0",
        "stage": "V3-11-F1",
        "status": "formal_evidence_freeze",
        "resource_count": len(resources),
        "resources": resources,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-release-sha256", required=True)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    assessment, evidence_paths = recompute_assessment(
        arguments.analysis_release_sha256
    )
    assessment_payload = yaml_bytes(assessment)
    freeze = evidence_freeze(evidence_paths, assessment_payload)
    freeze_payload = yaml_bytes(freeze)
    assessment_path = WORKSPACE / ASSESSMENT_RELATIVE
    freeze_path = WORKSPACE / EVIDENCE_FREEZE_RELATIVE
    if arguments.check:
        require(
            read_bytes(assessment_path) == assessment_payload,
            "formal assessment replay differs",
        )
        require(
            read_bytes(freeze_path) == freeze_payload,
            "formal evidence freeze replay differs",
        )
    else:
        write_once(assessment_path, assessment_payload)
        write_once(freeze_path, freeze_payload)
    print(
        "status={} formal_result={} demonstrated={} point={:.6f}% "
        "lower={:.6f}% assessment_sha256={} evidence_freeze_sha256={}".format(
            assessment["status"],
            assessment["formal_result"],
            assessment["performance_improvement_demonstrated"],
            assessment["metrics"]["point_reduction_percent"],
            assessment["metrics"]["bootstrap"][
                "one_sided_lower_reduction_percent"
            ],
            sha256_bytes(assessment_payload),
            sha256_bytes(freeze_payload),
        )
    )


if __name__ == "__main__":
    main()
