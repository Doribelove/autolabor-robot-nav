#!/usr/bin/env python3
"""Assess V3-03-A1 probe-select-execute integration from raw traces."""

import argparse
import importlib.util
import json
import math
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"
HORIZONS = (1.5, 2.5, 3.5, 4.5)


def module():
    spec = importlib.util.spec_from_file_location("v3_03_a1_assessor", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def candidate_position(candidate, relative_time):
    points = candidate["points"]
    if relative_time <= 0.0:
        return float(points[0]["x_m"]), float(points[0]["y_m"])
    for left, right in zip(points, points[1:]):
        if relative_time <= float(right["time_from_start_s"]) + 1e-12:
            span = float(right["time_from_start_s"]) - float(
                left["time_from_start_s"]
            )
            ratio = (
                relative_time - float(left["time_from_start_s"])
            ) / span
            return (
                float(left["x_m"])
                + ratio * (float(right["x_m"]) - float(left["x_m"])),
                float(left["y_m"])
                + ratio * (float(right["y_m"]) - float(left["y_m"])),
            )
    return float(points[-1]["x_m"]), float(points[-1]["y_m"])


def actor_coordinate(v, trace, angle_deg, stamp):
    angle = math.radians(float(angle_deg))
    return (
        v.interpolate(trace, "actor_x_m", stamp) * math.cos(angle)
        + v.interpolate(trace, "actor_y_m", stamp) * math.sin(angle)
    )


def assess(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    planner_contract = v.load_yaml_path(
        W / contract["upstream"]["planner_contract_path"]
    )
    model = v.load_yaml_path(W / contract["upstream"]["predictor_path"])
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        journal[-1]["event"] == "execution_complete"
        and journal[-1]["consumed_units"] == 18
        and journal[-1]["completed_units"] == 18
        and journal[-1]["completed_decisions"] == 9
        and journal[-1]["retry_units"] == 0,
        "journal differs",
    )
    outcomes = []
    selected_accepted = []
    stop_available = []
    collision_free = []
    covered = []
    per_horizon = {str(value): [] for value in HORIZONS}
    coordinate_spreads = []
    speed_spreads = []
    trace_errors = []
    progresses = []
    latencies = []
    nonfinite_count = 0
    radius = v.core._combined_radius(planner_contract)
    for probe_row, execution_row in zip(
        schedule["rows"][::2], schedule["rows"][1::2]
    ):
        decision_root = (
            W / v.EXECUTION_RELATIVE / "decisions" / probe_row["decision_id"]
        )
        receipt_path = decision_root / "decision_receipt.yaml"
        receipt = v.load_yaml_path(receipt_path)
        for key, phase_root in (
            ("probe_phase", decision_root / "probe"),
            ("execution_phase", decision_root / "execution"),
        ):
            raw_bytes = v.read_file_once(phase_root / "raw_pose_trace.csv")
            v.require(
                receipt[key]["raw_csv_sha256"] == v.sha256_bytes(raw_bytes),
                "raw hash differs",
            )
        probe_trace = v.load_trace(
            decision_root / "probe/raw_pose_trace.csv"
        )
        execution_trace = v.load_trace(
            decision_root / "execution/raw_pose_trace.csv"
        )
        snapshot = receipt["snapshot_runtime_input_without_policy"]
        replay = v.core.plan(snapshot, model, planner_contract)
        v.require(
            replay == receipt["planner_result"],
            "planner replay differs",
        )
        selected_id = replay["decision"]["selected_candidate_id"]
        selected = next(
            item
            for item in replay["candidate_set"]["candidates"]
            if item["candidate_id"] == selected_id
        )
        selected_eval = next(
            item
            for item in replay["evaluations"]
            if item["candidate_id"] == selected_id
        )
        selected_accepted.append(selected_eval["accepted"])
        stop_available.append(
            sum(
                item["certified_stop"]
                for item in replay["candidate_set"]["candidates"]
            )
            == 1
        )
        actual_clearances = [
            math.hypot(
                float(item["ego_x_m"]) - float(item["actor_x_m"]),
                float(item["ego_y_m"]) - float(item["actor_y_m"]),
            )
            - radius
            for item in execution_trace
            if float(item["sim_time_s"]) >= 0.5 - 1e-12
        ]
        actual_minimum_clearance = min(actual_clearances)
        collision_free.append(actual_minimum_clearance >= 0.0)
        tube = v.core.predict_occupancy_tube(
            snapshot, selected, model, planner_contract
        )
        horizon_rows = []
        angle = float(execution_row["angle_deg"])
        for index, absolute_time in enumerate(HORIZONS, start=1):
            truth = actor_coordinate(
                v, execution_trace, angle, absolute_time
            )
            occupancy = tube["samples"][index * 20]
            hit = (
                float(occupancy["lower_coordinate_m"]) - 1e-12
                <= truth
                <= float(occupancy["upper_coordinate_m"]) + 1e-12
            )
            covered.append(hit)
            per_horizon[str(absolute_time)].append(hit)
            horizon_rows.append(
                {
                    "absolute_time_s": absolute_time,
                    "truth_actor_coordinate_m": truth,
                    "lower_coordinate_m": occupancy[
                        "lower_coordinate_m"
                    ],
                    "upper_coordinate_m": occupancy[
                        "upper_coordinate_m"
                    ],
                    "covered": hit,
                }
            )
        coordinate_spread = abs(
            float(receipt["probe_observation"]["actor_coordinate_m"])
            - float(receipt["execution_observation"]["actor_coordinate_m"])
        )
        speed_spread = abs(
            float(receipt["probe_observation"]["actor_speed_mps"])
            - float(receipt["execution_observation"]["actor_speed_mps"])
        )
        coordinate_spreads.append(coordinate_spread)
        speed_spreads.append(speed_spread)
        candidate_errors = []
        for item in execution_trace:
            stamp = float(item["sim_time_s"])
            if stamp < 0.5 - 1e-12:
                continue
            expected_x, expected_y = candidate_position(
                selected, stamp - 0.5
            )
            candidate_errors.append(
                math.hypot(
                    float(item["ego_x_m"]) - expected_x,
                    float(item["ego_y_m"]) - expected_y,
                )
            )
        trace_error = max(candidate_errors)
        trace_errors.append(trace_error)
        progress = (
            v.interpolate(execution_trace, "ego_x_m", 4.5)
            - v.interpolate(execution_trace, "ego_x_m", 0.5)
        )
        progresses.append(progress)
        latency = float(receipt["planner_latency_ms"])
        latencies.append(latency)
        numeric = [
            actual_minimum_clearance,
            coordinate_spread,
            speed_spread,
            trace_error,
            progress,
            latency,
        ]
        nonfinite_count += sum(not math.isfinite(value) for value in numeric)
        outcomes.append(
            {
                "decision_id": probe_row["decision_id"],
                "block_id": probe_row["block_id"],
                "seed": probe_row["seed"],
                "actor_policy_evaluator_only": probe_row[
                    "actor_policy_evaluator_only"
                ],
                "selected_candidate_id": selected_id,
                "selected_candidate_accepted": selected_eval["accepted"],
                "predicted_minimum_clearance_m": selected_eval[
                    "minimum_clearance_m"
                ],
                "actual_minimum_clearance_with_margin_m": (
                    actual_minimum_clearance
                ),
                "progress_at_horizon_m": progress,
                "planner_latency_ms": latency,
                "probe_execution_actor_coordinate_spread_m": (
                    coordinate_spread
                ),
                "probe_execution_actor_speed_spread_mps": speed_spread,
                "selected_trajectory_trace_error_m": trace_error,
                "horizon_coverage": horizon_rows,
                "decision_receipt_sha256": v.sha256_bytes(
                    v.read_file_once(receipt_path)
                ),
            }
        )
    sorted_latency = sorted(latencies)
    p99_index = min(
        len(sorted_latency) - 1,
        max(0, int(math.ceil(0.99 * len(sorted_latency))) - 1),
    )
    metrics = {
        "decision_count": len(outcomes),
        "simulation_unit_count": 18,
        "selected_candidate_accepted_rate": sum(selected_accepted)
        / len(selected_accepted),
        "certified_stop_available_rate": sum(stop_available)
        / len(stop_available),
        "no_actual_footprint_intersection_rate": sum(collision_free)
        / len(collision_free),
        "minimum_actual_clearance_with_margin_m": min(
            item["actual_minimum_clearance_with_margin_m"]
            for item in outcomes
        ),
        "horizon_actor_envelope_coverage": sum(covered) / len(covered),
        "per_horizon_actor_envelope_coverage": {
            key: sum(values) / len(values)
            for key, values in per_horizon.items()
        },
        "maximum_common_prefix_probe_execution_coordinate_spread_m": max(
            coordinate_spreads
        ),
        "maximum_common_prefix_probe_execution_speed_spread_mps": max(
            speed_spreads
        ),
        "maximum_selected_trajectory_trace_error_m": max(trace_errors),
        "minimum_progress_at_horizon_m": min(progresses),
        "planner_latency_p99_ms": sorted_latency[p99_index],
        "planner_latency_max_ms": max(latencies),
        "nonfinite_metric_count": nonfinite_count,
        "selected_candidate_counts": {
            candidate_id: sum(
                item["selected_candidate_id"] == candidate_id
                for item in outcomes
            )
            for candidate_id in sorted(
                {item["selected_candidate_id"] for item in outcomes}
            )
        },
    }
    thresholds = contract["integration_gates"]
    gates = {
        "decision_count_pass": len(outcomes)
        == int(thresholds["decision_count"]),
        "simulation_unit_count_pass": int(thresholds["simulation_unit_count"])
        == 18,
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        >= float(thresholds["selected_candidate_accepted_rate_min"]),
        "certified_stop_available_pass": metrics[
            "certified_stop_available_rate"
        ]
        >= float(thresholds["certified_stop_available_rate_min"]),
        "actual_clearance_pass": metrics[
            "no_actual_footprint_intersection_rate"
        ]
        >= float(thresholds["no_actual_footprint_intersection_rate_min"]),
        "horizon_coverage_pass": metrics[
            "horizon_actor_envelope_coverage"
        ]
        >= float(thresholds["horizon_actor_envelope_coverage_min"]),
        "per_horizon_coverage_pass": min(
            metrics["per_horizon_actor_envelope_coverage"].values()
        )
        >= float(
            thresholds["per_horizon_actor_envelope_coverage_min"]
        ),
        "probe_execution_coordinate_alignment_pass": metrics[
            "maximum_common_prefix_probe_execution_coordinate_spread_m"
        ]
        <= float(
            thresholds[
                "common_prefix_probe_execution_coordinate_spread_max_m"
            ]
        ),
        "probe_execution_speed_alignment_pass": metrics[
            "maximum_common_prefix_probe_execution_speed_spread_mps"
        ]
        <= float(
            thresholds[
                "common_prefix_probe_execution_speed_spread_max_mps"
            ]
        ),
        "selected_trajectory_trace_pass": metrics[
            "maximum_selected_trajectory_trace_error_m"
        ]
        <= float(thresholds["selected_trajectory_trace_error_max_m"]),
        "progress_pass": metrics["minimum_progress_at_horizon_m"]
        >= float(thresholds["progress_at_horizon_min_m"]),
        "planner_latency_pass": metrics["planner_latency_p99_ms"]
        <= float(thresholds["planner_latency_p99_max_ms"]),
        "finite_metrics_pass": nonfinite_count
        <= int(thresholds["nonfinite_metric_count_max"]),
    }
    overall = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "assessment_id": "{}_integration_assessment_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_probe_select_execute_integration_pass".format(
                contract["stage"].lower().replace("-", "_")
            )
            if overall
            else "{}_probe_select_execute_integration_fail".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "formal_result": False,
        "metrics": metrics,
        "gates": gates,
        "overall_pass": overall,
        "execution": {
            "authorized_units": 18,
            "consumed_units": 18,
            "completed_units": 18,
            "completed_decisions": 9,
            "retry_units": 0,
        },
        "outcomes": outcomes,
        "claim": {
            "bounded_crossing_simulation_integration_pass": overall,
            "absolute_safety": False,
            "performance_improvement": False,
            "general_scene_planning": False,
            "real_vehicle_ready": False,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    value = assess(v)
    payload = v.yaml_bytes(value)
    path = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(path.is_file() and path.read_bytes() == payload, "assessment differs")
    else:
        write_once(path, payload)
        resources = [
            v.JOURNAL_RELATIVE,
            v.ASSESSMENT_RELATIVE,
            v.RELEASE_RELATIVE,
            v.REVIEW_RELATIVE,
            v.AUTHORIZATION_RELATIVE,
            Path(
                "src/application/v3_scope_planner/src/"
                "v3_scope_planner/core.py"
            ),
            Path(
                "artifacts/v3/prediction/v3_02_d1/model/"
                "v3_02_d1_candidate_predictor.yaml"
            ),
        ]
        for row in v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)["rows"][::2]:
            root = v.EXECUTION_RELATIVE / "decisions" / row["decision_id"]
            resources.extend(
                [
                    root / "probe/raw_pose_trace.csv",
                    root / "execution/raw_pose_trace.csv",
                    root / "decision_receipt.yaml",
                ]
            )
        freeze = (
            "\n".join(
                "{}  {}".format(
                    v.sha256_bytes(v.read_file_once(W / item)),
                    item.as_posix(),
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} overall_pass={} assessment_sha256={}".format(
            value["status"], value["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
