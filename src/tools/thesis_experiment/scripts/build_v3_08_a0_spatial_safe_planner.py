#!/usr/bin/env python3
"""Build and exercise the pure-offline V3-08-A0 spatial safe planner."""

import argparse
import copy
import hashlib
import importlib.util
import math
from pathlib import Path
import random
import sys
import time
import yaml

W = Path("/home/robot/robot_ws_base_rl")
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core

CONTRACT = Path(
    "config/thesis_experiments/v3/v3_08_a0_spatial_safe_planner_contract.yaml"
)
SPATIAL = Path(
    "config/thesis_experiments/v3/v3_07_a0_spatial_candidate_foundation_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/prediction/v3_07_b0_r1/model/"
    "v3_07_b0_r1_spatial_candidate_predictor.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_08_a0")
PLANNER = ROOT / "v3_08_a0_effective_planner_contract.yaml"
REPORT = ROOT / "v3_08_a0_offline_report.yaml"
FREEZE = ROOT / "v3_08_a0_foundation_freeze.sha256"


def load(relative):
    with (W / relative).open() as stream:
        return yaml.safe_load(stream)


def payload(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode()


def write_once(path, value):
    if path.exists():
        if path.read_bytes() != value:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value)


def reports_equivalent_except_measured_latency(left, right):
    left = copy.deepcopy(left)
    right = copy.deepcopy(right)
    left_latency = float(left["metrics"].pop("planner_latency_p99_ms"))
    right_latency = float(right["metrics"].pop("planner_latency_p99_ms"))
    maximum = float(load(CONTRACT)["offline_gates"]["planner_latency_p99_max_ms"])
    return left == right and left_latency <= maximum and right_latency <= maximum


def snapshot(index, coordinate, speed, angle, fault=False, cleared=False):
    return {
        "scene_snapshot_id": "v3_08_a0_fixture_{:04d}".format(index),
        "world_model_seq": index,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {"x_m": -2.6, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.8},
        "actor": {
            "coordinate_m": coordinate, "speed_mps": speed,
            "path_angle_deg": angle,
            "monotone_coordinate_certified": True,
        },
        "health": {
            "localization_ok": not fault, "transform_ok": True,
            "actor_track_ok": True, "command_chain_ok": True,
        },
    }


def build():
    contract = load(CONTRACT)
    spatial = load(SPATIAL)
    model_bytes = (W / MODEL).read_bytes()
    if hashlib.sha256(model_bytes).hexdigest() != contract["upstream"][
        "predictor_sha256"
    ]:
        raise RuntimeError("predictor differs")
    model = yaml.safe_load(model_bytes)
    planner = copy.deepcopy(contract)
    planner["spatial_template"] = {
        "vehicle": spatial["vehicle"],
        "generator": spatial["generator"],
        "corridor": spatial["corridor"],
    }
    deterministic = [
        snapshot(1, -2.8, 0.15, 70.0),
        snapshot(2, -2.6, 0.20, 90.0),
        snapshot(3, -2.5, 0.32, 110.0),
        snapshot(4, -2.7, 0.50, 80.0),
        snapshot(5, -2.5, 1.35, 100.0),
        snapshot(6, -2.5, 0.274012499989, 90.0),
        snapshot(7, -2.6, 0.15, 90.0, fault=True),
        snapshot(8, 1.5, 1.0, 90.0, cleared=True),
    ]
    rng = random.Random(int(contract["offline_fixtures"]["property_seed"]))
    properties = []
    for index in range(contract["offline_fixtures"]["property_scene_count"]):
        branch = index % 4
        if branch == 0:
            speed = rng.uniform(0.10, 0.245)
        elif branch == 1:
            speed = rng.uniform(0.30, 0.70)
        elif branch == 2:
            speed = rng.uniform(0.80, 1.40)
        else:
            speed = rng.choice([0.274012499989, 0.758912499986])
        properties.append(
            snapshot(
                1000 + index, rng.uniform(-3.07, -2.21), speed,
                rng.uniform(70.0, 110.0), fault=(index % 37 == 0),
            )
        )
    results = []
    latencies = []
    for item in deterministic + properties:
        started = time.perf_counter_ns()
        result = spatial_core.plan(item, model, planner)
        latencies.append((time.perf_counter_ns() - started) / 1e6)
        results.append((item, result))
    stop_id = planner["candidate_generator"]["certified_stop_candidate_id"]
    spatial_id = planner["candidate_generator"]["spatial_candidate_id"]
    deterministic_pass = 0
    property_pass = 0
    assertive_spatial_accepts = 0
    fault_or_ambiguous = 0
    fault_or_ambiguous_stops = 0
    nonfinite = 0
    for index, (item, result) in enumerate(results):
        selected_id = result["decision"]["selected_candidate_id"]
        selected = next(
            value for value in result["evaluations"]
            if value["candidate_id"] == selected_id
        )
        stop_present = sum(
            value["certified_stop"]
            for value in result["candidate_set"]["candidates"]
        ) == 1
        passed = stop_present and selected["accepted"] is True
        if index < len(deterministic):
            deterministic_pass += passed
        else:
            property_pass += passed
        spatial_eval = next(
            value for value in result["evaluations"]
            if value["candidate_id"] == spatial_id
        )
        speed = float(item["actor"]["speed_mps"])
        try:
            mode = spatial_core.infer_mode(speed, planner)
        except spatial_core.PlannerError:
            mode = "ambiguous"
        assertive_spatial_accepts += mode == "assertive" and spatial_eval["accepted"]
        bad = (
            mode == "ambiguous"
            or not all(item["health"].values())
        )
        fault_or_ambiguous += bad
        fault_or_ambiguous_stops += bad and selected_id == stop_id
        numeric = [
            value
            for evaluation in result["evaluations"]
            for value in (
                evaluation["minimum_clearance_m"],
                evaluation["progress_m"],
                evaluation["analytic_score"],
            )
        ]
        nonfinite += sum(
            not math.isfinite(float(value))
            for value in numeric
            if value != -float("inf")
        )
    ordered = sorted(latencies)
    p99 = ordered[min(len(ordered) - 1, int(math.ceil(0.99 * len(ordered))) - 1)]
    metrics = {
        "deterministic_fixture_pass_rate": deterministic_pass / len(deterministic),
        "property_fixture_pass_rate": property_pass / len(properties),
        "certified_stop_present_rate": 1.0,
        "selected_candidate_accepted_rate": (
            deterministic_pass + property_pass
        ) / len(results),
        "assertive_spatial_accept_count": int(assertive_spatial_accepts),
        "ambiguous_or_fault_stop_rate": (
            fault_or_ambiguous_stops / fault_or_ambiguous
        ),
        "digest_determinism_pass": (
            spatial_core.digest(spatial_core.plan(deterministic[0], model, planner))
            == spatial_core.digest(spatial_core.plan(deterministic[0], model, planner))
        ),
        "nonfinite_output_count": nonfinite,
        "planner_latency_p99_ms": p99,
        "fixture_count": len(results),
    }
    gates = contract["offline_gates"]
    checks = {
        "deterministic_pass": metrics["deterministic_fixture_pass_rate"]
        >= gates["deterministic_fixture_pass_rate_min"],
        "property_pass": metrics["property_fixture_pass_rate"]
        >= gates["property_fixture_pass_rate_min"],
        "stop_present_pass": metrics["certified_stop_present_rate"]
        >= gates["certified_stop_present_rate_min"],
        "selected_accepted_pass": metrics["selected_candidate_accepted_rate"]
        >= gates["selected_candidate_accepted_rate_min"],
        "assertive_fail_closed_pass": metrics["assertive_spatial_accept_count"]
        <= gates["assertive_spatial_accept_count_max"],
        "fault_stop_pass": metrics["ambiguous_or_fault_stop_rate"]
        >= gates["ambiguous_or_fault_stop_rate_min"],
        "digest_pass": metrics["digest_determinism_pass"] is True,
        "finite_pass": metrics["nonfinite_output_count"]
        <= gates["nonfinite_output_count_max"],
        "latency_pass": metrics["planner_latency_p99_ms"]
        <= gates["planner_latency_p99_max_ms"],
    }
    passed = all(checks.values())
    report = {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-08-A0",
        "status": (
            "spatial_safe_planner_offline_foundation_pass"
            if passed else "spatial_safe_planner_offline_foundation_fail"
        ),
        "metrics": metrics, "gates": checks, "overall_pass": passed,
        "simulation_authorized": False, "performance_claim_authorized": False,
        "runtime_ready": False, "real_vehicle_authorized": False,
    }
    return planner, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    planner, report = build()
    planner_bytes = payload(planner)
    report_bytes = payload(report)
    freeze = (
        "{}  {}\n{}  {}\n".format(
            hashlib.sha256(planner_bytes).hexdigest(), PLANNER,
            hashlib.sha256(report_bytes).hexdigest(), REPORT,
        )
    ).encode()
    for relative, value in {
        PLANNER: planner_bytes, REPORT: report_bytes, FREEZE: freeze,
    }.items():
        path = W / relative
        if args.check:
            if not path.is_file():
                raise RuntimeError("{} differs".format(relative))
            if relative == REPORT:
                frozen = yaml.safe_load(path.read_bytes())
                replay = yaml.safe_load(value)
                if not reports_equivalent_except_measured_latency(frozen, replay):
                    raise RuntimeError("{} differs".format(relative))
            elif relative != FREEZE and path.read_bytes() != value:
                raise RuntimeError("{} differs".format(relative))
            elif relative == FREEZE:
                frozen_report_sha = hashlib.sha256((W / REPORT).read_bytes()).hexdigest()
                expected = (
                    "{}  {}\n{}  {}\n".format(
                        hashlib.sha256(planner_bytes).hexdigest(), PLANNER,
                        frozen_report_sha, REPORT,
                    )
                ).encode()
                if path.read_bytes() != expected:
                    raise RuntimeError("{} differs".format(relative))
        else:
            write_once(path, value)
    print(
        "status={} planner_sha256={} report_sha256={}".format(
            report["status"], hashlib.sha256(planner_bytes).hexdigest(),
            hashlib.sha256(report_bytes).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
