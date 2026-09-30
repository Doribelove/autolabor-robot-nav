#!/usr/bin/env python3
"""Independently assess and freeze the V3-11-Q1-R1 qualification evidence."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_11_a0_actual_teb_fair_baseline_contract.yaml"
)
EXECUTION_RELATIVE = Path("artifacts/v3/actual_teb/v3_11_q1_r1")
ASSESSMENT_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "qualification_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "evidence_freeze.yaml"
)
EXPECTED_CONTRACT_SHA256 = (
    "387aff179de115f5a2e748ee119473add9c5b4e8bd488349f1a9a905ea705000"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_once(path):
    require(path.is_file(), "required file absent: {}".format(path))
    return path.read_bytes()


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    return sha256_bytes(read_once(path))


def load_yaml(path):
    return yaml.safe_load(read_once(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_once(path, payload):
    require(not path.exists(), "immutable output already exists: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def close(a, b, tolerance=1e-12):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)


def expected_schedule(contract):
    matrix = contract["evaluation"]["qualification"]
    methods = list(contract["methods"]["identifiers"])
    rows = []
    sequence = 0
    for zero_index, seed in enumerate(
        range(int(matrix["seed_start"]), int(matrix["seed_end"]) + 1)
    ):
        stratum = matrix["strata"][zero_index % len(matrix["strata"])]
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


def read_journal(path):
    records = []
    for line_number, raw in enumerate(read_once(path).splitlines(), start=1):
        require(raw.strip(), "blank journal line {}".format(line_number))
        value = json.loads(raw)
        require(
            isinstance(value, dict),
            "journal line {} is not an object".format(line_number),
        )
        records.append(value)
    return records


def replay_raw_evidence(contract, rows):
    execution_root = WORKSPACE / EXECUTION_RELATIVE
    schedule_path = execution_root / "exact_schedule.yaml"
    journal_path = execution_root / "attempt_journal.jsonl"
    summary_path = execution_root / "matrix_summary.yaml"
    schedule = load_yaml(schedule_path)
    require(schedule["phase"] == "qualification", "schedule phase differs")
    require(schedule["row_count"] == len(rows), "schedule row count differs")
    require(schedule["rows"] == rows, "schedule differs from contract derivation")

    identities = {
        (
            row["seed"],
            row["scene_id"],
            row["actor_policy"],
            row["method"],
        )
        for row in rows
    }
    require(len(identities) == len(rows), "schedule contains duplicate identity")

    journal = read_journal(journal_path)
    require(len(journal) == 2 * len(rows) + 2, "journal event count differs")
    opened = journal[0]
    require(opened["event"] == "execution_opened", "journal does not open")
    require(opened["phase"] == "qualification", "journal phase differs")
    require(
        opened["contract_sha256"] == EXPECTED_CONTRACT_SHA256,
        "journal contract hash differs",
    )
    require(opened["authorized_units"] == len(rows), "authorization count differs")
    require(close(opened["lookahead_m"], 0.70), "lookahead differs")

    reports = []
    receipts = []
    evidence_paths = [
        CONTRACT_RELATIVE,
        EXECUTION_RELATIVE / "exact_schedule.yaml",
        EXECUTION_RELATIVE / "attempt_journal.jsonl",
        EXECUTION_RELATIVE / "matrix_summary.yaml",
    ]
    for index, row in enumerate(rows):
        consumed = journal[1 + 2 * index]
        complete = journal[2 + 2 * index]
        require(consumed["event"] == "unit_consumed", "unit was not consumed")
        require(consumed["sequence"] == row["sequence"], "consumed sequence differs")
        require(consumed["row"] == row, "consumed row differs")
        require(consumed["attempt"] == 1, "attempt identity differs")
        require(complete["event"] == "unit_complete", "unit did not complete")
        require(complete["sequence"] == row["sequence"], "complete sequence differs")
        require(complete["row"] == row, "complete row differs")

        root = episode_directory(row)
        require(root.is_dir(), "episode directory absent: {}".format(root))
        report_path = root / "episode_report.yaml"
        receipt_path = root / "matrix_receipt.yaml"
        world_path = root / "world.sdf"
        log_path = root / "roslaunch.log"
        report = load_yaml(report_path)
        receipt = load_yaml(receipt_path)
        require(receipt["row"] == row, "receipt row differs at {}".format(index + 1))
        expected_actor = expected_actor_parameters(row["seed"])
        for key, value in expected_actor.items():
            require(
                close(receipt["actor_generated_parameters"][key], value),
                "actor derivation differs at {} {}".format(index + 1, key),
            )
        require(
            receipt["episode_report"] == report,
            "receipt embedded report differs at {}".format(index + 1),
        )
        require(
            receipt["episode_report_sha256"] == sha256_file(report_path),
            "report hash differs at {}".format(index + 1),
        )
        require(
            receipt["world_sha256"] == sha256_file(world_path),
            "world hash differs at {}".format(index + 1),
        )
        require(
            receipt["roslaunch_log_sha256"] == sha256_file(log_path),
            "launch log hash differs at {}".format(index + 1),
        )
        require(receipt["roslaunch_return_code"] == 0, "roslaunch failed")
        require(receipt["timed_out"] is False, "episode timed out")
        require(
            complete["matrix_receipt_sha256"] == sha256_file(receipt_path),
            "journal receipt hash differs at {}".format(index + 1),
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
            "journal elapsed time differs",
        )
        require(
            close(
                complete["minimum_conservative_clearance_m"],
                report["result"]["minimum_conservative_clearance_m"],
            ),
            "journal clearance differs",
        )
        require(report["stage"] == "V3-11", "report stage differs")
        require(report["method"] == row["method"], "report method differs")
        require(report["scene_id"] == row["scene_id"], "report scene differs")
        require(report["seed"] == row["seed"], "report seed differs")
        require(
            report["actor_policy_evaluator_only"] == row["actor_policy"],
            "report actor policy differs",
        )
        require(report["simulation_only"] is True, "simulation boundary differs")
        require(report["training_steps"] == 0, "training steps differ")
        require(report["hardware_accessed"] is False, "hardware boundary differs")
        require(report["real_vehicle_used"] is False, "real-vehicle boundary differs")
        reports.append(report)
        receipts.append(receipt)
        for path in sorted(
            item for item in root.rglob("*") if item.is_file()
        ):
            evidence_paths.append(path.relative_to(WORKSPACE))

    terminal = journal[-1]
    require(terminal["event"] == "execution_complete", "journal did not close")
    for key in ("authorized_units", "consumed_units", "completed_units"):
        require(terminal[key] == len(rows), "{} count differs".format(key))
    require(terminal["forfeited_units"] == 0, "units were forfeited")
    require(
        terminal["summary_sha256"] == sha256_file(summary_path),
        "journal summary hash differs",
    )
    return reports, receipts, sorted(set(evidence_paths), key=str)


def recompute_metrics(reports, rows):
    by_method = {
        method: [report for report in reports if report["method"] == method]
        for method in ("v3_spatial_safe", "actual_teb")
    }
    by_block = {}
    for row, report in zip(rows, reports):
        by_block.setdefault(row["block_index"], {})[row["method"]] = report
    require(len(by_block) == 12, "paired block count differs")

    paired = []
    pair_differences = []
    for block_index in sorted(by_block):
        block = by_block[block_index]
        require(
            set(block) == {"v3_spatial_safe", "actual_teb"},
            "paired methods incomplete",
        )
        v3 = block["v3_spatial_safe"]
        teb = block["actual_teb"]
        require(v3["scene_id"] == teb["scene_id"], "paired scene differs")
        require(
            v3["actor_policy_evaluator_only"]
            == teb["actor_policy_evaluator_only"],
            "paired actor policy differs",
        )
        v3_time = float(v3["result"]["restricted_elapsed_s"])
        teb_time = float(teb["result"]["restricted_elapsed_s"])
        paired.append(
            {
                "block_index": block_index,
                "scene_id": v3["scene_id"],
                "actor_policy": v3["actor_policy_evaluator_only"],
                "v3_restricted_elapsed_s": v3_time,
                "actual_teb_restricted_elapsed_s": teb_time,
                "reduction_percent": 100.0 * (teb_time - v3_time) / teb_time,
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

    mean_v3 = sum(
        item["v3_restricted_elapsed_s"] for item in paired
    ) / len(paired)
    mean_teb = sum(
        item["actual_teb_restricted_elapsed_s"] for item in paired
    ) / len(paired)
    return {
        "episode_count": len(reports),
        "paired_block_count": len(paired),
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
        "mean_restricted_elapsed_s_by_method": {
            "v3_spatial_safe": mean_v3,
            "actual_teb": mean_teb,
        },
        "exploratory_paired_mean_time_reduction_percent": (
            100.0 * (mean_teb - mean_v3) / mean_teb
        ),
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
        "paired_blocks": paired,
        "gate_pair_differences": pair_differences,
    }


def recompute_assessment():
    contract_path = WORKSPACE / CONTRACT_RELATIVE
    contract_payload = read_once(contract_path)
    require(
        sha256_bytes(contract_payload) == EXPECTED_CONTRACT_SHA256,
        "frozen qualification contract changed",
    )
    contract = yaml.safe_load(contract_payload)
    require(contract["stage"] == "V3-11-A0-R8", "contract stage differs")
    require(
        contract["authority"]["integration_and_qualification_authorized"] is True,
        "qualification was not authorized",
    )
    require(
        contract["authority"]["formal_execution_authorized"] is False,
        "qualification contract improperly authorizes formal execution",
    )
    require(
        contract["authority"]["training_authorized"] is False,
        "training boundary differs",
    )
    dependency_results = []
    for resource in contract["qualification_dependency_freeze"]:
        path = WORKSPACE / resource["path"]
        observed = sha256_file(path)
        dependency_results.append(
            {
                "path": resource["path"],
                "expected_sha256": resource["sha256"],
                "observed_sha256": observed,
                "pass": observed == resource["sha256"],
            }
        )
    require(
        all(item["pass"] for item in dependency_results),
        "qualification dependency freeze differs",
    )

    rows = expected_schedule(contract)
    reports, _receipts, evidence_paths = replay_raw_evidence(contract, rows)
    metrics = recompute_metrics(reports, rows)
    gates_contract = contract["qualification_gates"]
    gates = {
        "schedule_rows_exact_pass": (
            metrics["episode_count"] == gates_contract["schedule_rows_exact"]
        ),
        "attempt_identity_and_no_retry_pass": True,
        "common_gate_valid_rate_pass": (
            metrics["common_gate_valid_rate"]
            >= gates_contract["common_gate_valid_rate_min"]
        ),
        "actor_mode_match_rate_pass": (
            metrics["actor_mode_match_rate"]
            >= gates_contract[
                "actor_mode_matches_evaluator_policy_rate_min"
            ]
        ),
        "paired_gate_actor_coordinate_difference_pass": (
            metrics["maximum_paired_gate_actor_coordinate_difference_m"]
            <= gates_contract[
                "paired_gate_actor_coordinate_difference_max_m"
            ]
        ),
        "paired_gate_actor_speed_difference_pass": (
            metrics["maximum_paired_gate_actor_speed_difference_mps"]
            <= gates_contract["paired_gate_actor_speed_difference_max_mps"]
        ),
        "v3_spatial_selected_rate_pass": (
            metrics["v3_spatial_selected_rate"]
            >= gates_contract["v3_spatial_selected_rate_min"]
        ),
        "goal_success_rate_each_method_pass": all(
            value >= gates_contract["goal_success_rate_each_method_min"]
            for value in metrics["goal_success_rate_by_method"].values()
        ),
        "collision_count_pass": (
            metrics["contact_state_count_total"]
            <= gates_contract["collision_count_max"]
        ),
        "conservative_clearance_pass": all(
            value >= gates_contract["conservative_clearance_min_m"]
            for value in metrics[
                "minimum_conservative_clearance_m_by_method"
            ].values()
        ),
        "v3_tracking_error_pass": (
            metrics["maximum_v3_tracking_error_m"]
            <= gates_contract["v3_tracking_error_max_m"]
        ),
        "actual_teb_local_plan_pass": (
            metrics["minimum_actual_teb_local_plan_count"] > 0
            and close(metrics["actual_teb_action_succeeded_rate"], 1.0)
        ),
        "simulation_and_zero_training_boundary_pass": all(
            report["simulation_only"] is True
            and report["training_steps"] == 0
            and report["hardware_accessed"] is False
            and report["real_vehicle_used"] is False
            for report in reports
        ),
        "dependency_freeze_pass": all(
            item["pass"] for item in dependency_results
        ),
    }
    passed = all(gates.values())
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-Q1-R1",
        "status": (
            "actual_teb_fair_multiscene_qualification_pass"
            if passed
            else "actual_teb_fair_multiscene_qualification_valid_negative"
        ),
        "contract_path": str(CONTRACT_RELATIVE),
        "contract_sha256": EXPECTED_CONTRACT_SHA256,
        "execution_root": str(EXECUTION_RELATIVE),
        "raw_execution_summary_sha256": sha256_file(
            WORKSPACE / EXECUTION_RELATIVE / "matrix_summary.yaml"
        ),
        "raw_attempt_journal_sha256": sha256_file(
            WORKSPACE / EXECUTION_RELATIVE / "attempt_journal.jsonl"
        ),
        "raw_exact_schedule_sha256": sha256_file(
            WORKSPACE / EXECUTION_RELATIVE / "exact_schedule.yaml"
        ),
        "assessment_basis": (
            "independent_replay_of_contract_schedule_journal_receipts_"
            "raw_episode_reports_worlds_and_logs"
        ),
        "metrics": metrics,
        "gates": gates,
        "overall_pass": passed,
        "qualified_for_fresh_formal_actual_teb_evaluation": passed,
        "formal_performance_claim_authorized": False,
        "training_steps": 0,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "runtime_ready": False,
        "dependency_results": dependency_results,
        "claim_boundary": {
            "qualified_claim": (
                "fair execution integration and multi-scene qualification "
                "against actual TebLocalPlannerROS"
            ),
            "exploratory_effect_is_not_formal_result": True,
            "perception_robustness_claim_authorized": False,
            "general_navigation_safety_claim_authorized": False,
            "deployment_claim_authorized": False,
        },
    }
    return value, evidence_paths


def build_evidence_freeze(evidence_paths, assessment_payload):
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
        "stage": "V3-11-Q1-R1",
        "status": "qualification_evidence_freeze",
        "resource_count": len(resources),
        "resources": resources,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    assessment, evidence_paths = recompute_assessment()
    assessment_payload = yaml_bytes(assessment)
    freeze = build_evidence_freeze(evidence_paths, assessment_payload)
    freeze_payload = yaml_bytes(freeze)
    assessment_path = WORKSPACE / ASSESSMENT_RELATIVE
    freeze_path = WORKSPACE / EVIDENCE_FREEZE_RELATIVE
    if arguments.check:
        require(
            read_once(assessment_path) == assessment_payload,
            "qualification assessment replay differs",
        )
        require(
            read_once(freeze_path) == freeze_payload,
            "qualification evidence freeze replay differs",
        )
    else:
        write_once(assessment_path, assessment_payload)
        write_once(freeze_path, freeze_payload)
    print(
        "status={} qualified={} exploratory_reduction={:.6f}% "
        "assessment_sha256={} evidence_freeze_sha256={}".format(
            assessment["status"],
            assessment[
                "qualified_for_fresh_formal_actual_teb_evaluation"
            ],
            assessment["metrics"][
                "exploratory_paired_mean_time_reduction_percent"
            ],
            sha256_bytes(assessment_payload),
            sha256_bytes(freeze_payload),
        )
    )


if __name__ == "__main__":
    main()
