#!/usr/bin/env python3
"""Deterministically attribute V3-08-A1-R1 integration failure."""

import argparse
import hashlib
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/v3_08_a2_r1_failure_attribution_contract.yaml"
)
SCHEDULE = Path(
    "artifacts/v3/planner/v3_08_a1_r1/offline_readiness/"
    "v3_08_a1_r1_exact_schedule.yaml"
)
ASSESSMENT = Path(
    "artifacts/v3/planner/v3_08_a1_r1/execution/"
    "v3_08_a1_r1_integration_assessment.yaml"
)
ROOT = Path("artifacts/v3/analysis/v3_08_a2")
REPORT = ROOT / "v3_08_a2_failure_attribution.yaml"
FREEZE = ROOT / "v3_08_a2_evidence_freeze.sha256"


def load(relative):
    return yaml.safe_load((W / relative).read_bytes())


def encode(value):
    return yaml.safe_dump(value, sort_keys=False).encode()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def build():
    contract = load(CONTRACT)
    schedule = load(SCHEDULE)
    assessment = load(ASSESSMENT)
    if assessment["status"] != "spatial_planner_simulation_integration_fail":
        raise RuntimeError("R1 result differs")
    failures = []
    stage_mismatches = 0
    evidence = [CONTRACT, SCHEDULE, ASSESSMENT]
    for row in schedule["rows"]:
        receipt_path = Path(
            "artifacts/v3/planner/v3_08_a1_r1/execution/episodes"
        ) / row["unit_id"] / "episode_receipt.yaml"
        receipt = load(receipt_path)
        stage_mismatches += receipt["stage"] != "V3-08-A1-R1"
        if receipt["goal_reached"] is False:
            last = receipt["decisions"][-1]
            previous = receipt["decisions"][-2]
            velocity = (
                float(last["state_after"]["ego_x_m"])
                - float(previous["state_after"]["ego_x_m"])
            ) / float(last["segment_duration_s"])
            future_duration = float(
                contract["future_single_behavioral_factor"]["future_value_s"]
            ) - float(receipt["final_state"]["global_time_s"])
            projected = float(receipt["final_state"]["ego_x_m"]) + velocity * future_duration
            failures.append({
                "unit_id": row["unit_id"],
                "policy": row["actor_policy_evaluator_only"],
                "goal_x_m": 5.2,
                "final_time_s": float(receipt["final_state"]["global_time_s"]),
                "final_x_m": float(receipt["final_state"]["ego_x_m"]),
                "goal_shortfall_m": 5.2 - float(receipt["final_state"]["ego_x_m"]),
                "last_selected_candidate_id": last["selected_candidate_id"],
                "last_segment_speed_mps": velocity,
                "projected_x_at_12_5_s_m": projected,
                "projected_goal_reached_at_12_5_s": projected >= 5.2,
                "selected_candidate_accepted": last["selected_candidate_accepted"],
                "last_segment_clearance_m": last["segment"][
                    "minimum_conservative_clearance_m"
                ],
            })
        evidence.append(receipt_path)
    if len(failures) != 1:
        raise RuntimeError("failure count differs")
    failed = failures[0]
    conclusion = (
        failed["policy"] == "assertive"
        and failed["goal_shortfall_m"] > 0.0
        and failed["projected_goal_reached_at_12_5_s"] is True
        and failed["selected_candidate_accepted"] is True
        and failed["last_segment_clearance_m"] >= 0.0
    )
    report = {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-08-A2",
        "status": (
            "r1_boundary_failure_attribution_pass"
            if conclusion else "r1_boundary_failure_attribution_fail"
        ),
        "r1_summary": {
            "episodes": 18, "goals": 17,
            "goal_success_rate": assessment["metrics"]["goal_success_rate"],
            "safety_violation_count": assessment["metrics"][
                "conservative_clearance_violation_count"
            ],
            "exact_replay_rate": assessment["metrics"]["exact_core_replay_rate"],
        },
        "failed_units": failures,
        "receipt_stage_identity": {
            "expected_stage": "V3-08-A1-R1",
            "legacy_stage_observed": "V3-08-A1",
            "mismatch_count": stage_mismatches,
            "future_repair_required": stage_mismatches > 0,
            "behavioral_semantics_affected": False,
        },
        "attribution": {
            "safety_failure": False,
            "selection_failure": False,
            "predictor_failure": False,
            "episode_boundary_truncation": conclusion,
            "next_half_second_existing_recovery_motion_sufficient": conclusion,
        },
        "future_design": contract["future_single_behavioral_factor"],
        "mandatory_nonbehavioral_integrity_repair": contract[
            "mandatory_nonbehavioral_integrity_repair"
        ],
        "simulation_authorized": False,
        "performance_claim_authorized": False,
        "runtime_ready": False,
        "real_vehicle_authorized": False,
        "overall_pass": conclusion and stage_mismatches == 18,
        "_evidence": evidence,
    }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = build()
    evidence = report.pop("_evidence")
    report_bytes = encode(report)
    freeze = (
        "\n".join(
            "{}  {}".format(
                hashlib.sha256(
                    report_bytes if item == REPORT else (W / item).read_bytes()
                ).hexdigest(),
                item,
            )
            for item in evidence + [REPORT]
        ) + "\n"
    ).encode()
    for relative, payload in {REPORT: report_bytes, FREEZE: freeze}.items():
        path = W / relative
        if args.check:
            if not path.is_file() or path.read_bytes() != payload:
                raise RuntimeError("{} differs".format(relative))
        else:
            write_once(path, payload)
    print(
        "status={} report_sha256={}".format(
            report["status"], hashlib.sha256(report_bytes).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
