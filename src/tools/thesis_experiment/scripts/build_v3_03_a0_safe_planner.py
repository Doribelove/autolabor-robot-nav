#!/usr/bin/env python3
"""Build deterministic V3-03-A0 offline verifier/ranker evidence."""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


CONTRACT = Path(
    "config/thesis_experiments/v3/v3_03_a0_safe_planner_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/v3_03_a0_safe_planner_preregistration.yaml"
)
ROOT = Path("artifacts/v3/planner/v3_03_a0")
DETERMINISTIC_REPORT = ROOT / "v3_03_a0_deterministic_fixture_report.yaml"
PROPERTY_REPORT = ROOT / "v3_03_a0_property_report.yaml"
SUMMARY = ROOT / "v3_03_a0_offline_summary.yaml"
CLOSURE = ROOT / "v3_03_a0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_03_a0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_03_a0_independent_review.yaml"

SOURCE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path(
        "config/thesis_experiments/v3/"
        "v3_03_a0_safe_planner_fixtures.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d2/"
        "v3_02_d2_terminal_freeze.sha256"
    ),
    Path("src/application/v3_scope_planner/package.xml"),
    Path("src/application/v3_scope_planner/CMakeLists.txt"),
    Path("src/application/v3_scope_planner/setup.py"),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/__init__.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/core.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_planner_offline_probe.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_a0_safe_planner.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_a0_safe_planner.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_03_a0_safe_planner.py"
    ),
)


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    value = yaml.safe_load(read(path))
    if not isinstance(value, dict):
        raise RuntimeError("YAML root is not mapping: {}".format(path))
    return value


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def receipt(path):
    payload = read(path)
    return {
        "path": path.as_posix(),
        "sha256": sha(payload),
        "size_bytes": len(payload),
    }


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(absolute))
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def validate(contract, preregistration):
    core.require(contract["stage"] == "V3-03-A0", "stage differs")
    core.require(
        contract["status"] == "pure_offline_preregistered",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-03-A0"
        and preregistration["status"]
        == "frozen_before_offline_fixture_materialization",
        "preregistration differs",
    )
    core.require(
        contract["candidate_generator"]["target_speeds_mps"]
        == [0.0, 0.6, 0.8, 1.0, 1.2]
        and contract["candidate_generator"]["certified_stop_required"] is True
        and contract["candidate_generator"]["teb_fallback_required"] is True,
        "candidate roster differs",
    )
    core.require(
        contract["hard_verifier"]["fail_closed_on_nonfinite"] is True
        and contract["hard_verifier"]["fail_closed_on_digest_mismatch"] is True
        and contract["analytic_ranker"]["rejected_candidate_visibility"] is False,
        "safety boundary differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    model_payload = read(Path(contract["upstream"]["predictor_path"]))
    core.require(
        sha(model_payload) == contract["upstream"]["predictor_sha256"],
        "predictor hash differs",
    )


def materialize_snapshot(fixtures, row):
    snapshot = copy.deepcopy(fixtures["defaults"])
    snapshot["scene_snapshot_id"] = row["scene_id"]
    if "actor_override" in row:
        snapshot["actor"].update(row["actor_override"])
    if "health_false" in row:
        snapshot["health"][row["health_false"]] = False
    for field in (
        "snapshot_age_s",
        "predictor_latency_ms",
        "verifier_latency_ms",
    ):
        if field in row:
            snapshot[field] = row[field]
    return snapshot


def selected_evaluation(result):
    selected = result["decision"]["selected_candidate_id"]
    return next(
        item for item in result["evaluations"] if item["candidate_id"] == selected
    )


def deterministic_fixtures(contract, model, fixtures):
    rows = []
    stop_id = contract["candidate_generator"]["certified_stop_candidate_id"]
    for fixture in fixtures["scenes"]:
        snapshot = materialize_snapshot(fixtures, fixture)
        first = core.plan(snapshot, model, contract)
        second = core.plan(snapshot, model, contract)
        selected = selected_evaluation(first)
        expected_stop = fixture["expected"] == "stop"
        checks = {
            "byte_deterministic": core.canonical_bytes(first)
            == core.canonical_bytes(second),
            "certified_stop_present_once": sum(
                item["certified_stop"]
                for item in first["candidate_set"]["candidates"]
            )
            == 1,
            "teb_fallback_present_once": sum(
                item["teb_fallback"]
                for item in first["candidate_set"]["candidates"]
            )
            == 1,
            "all_generated_candidates_kinematically_valid": all(
                not core.validate_candidate(item, snapshot, contract)
                for item in first["candidate_set"]["candidates"]
            ),
            "selected_expectation": (
                first["decision"]["selected_candidate_id"] == stop_id
                if expected_stop
                else first["decision"]["selected_candidate_id"] != stop_id
            ),
            "selected_is_accepted": selected["accepted"] is True,
            "decision_digest_valid": core.digest(
                {
                    key: value
                    for key, value in first["decision"].items()
                    if key != "decision_digest"
                }
            )
            == first["decision"]["decision_digest"],
        }
        rows.append(
            {
                "scene_id": fixture["scene_id"],
                "expected": fixture["expected"],
                "selected_candidate_id": first["decision"][
                    "selected_candidate_id"
                ],
                "accepted_candidate_ids": [
                    item["candidate_id"]
                    for item in first["evaluations"]
                    if item["accepted"]
                ],
                "checks": checks,
                "pass": all(checks.values()),
                "result_digest": core.digest(first),
            }
        )
    return {
        "schema_version": "1.0",
        "stage": "V3-03-A0",
        "report_id": "v3_03_a0_deterministic_fixture_report_1",
        "scene_count": len(rows),
        "passing_scene_count": sum(row["pass"] for row in rows),
        "pass_rate": sum(row["pass"] for row in rows) / len(rows),
        "overall_pass": all(row["pass"] for row in rows),
        "scenes": rows,
    }


def property_fixtures(contract, model):
    specification = contract["fixtures"]
    rng = random.Random(int(specification["property_seed"]))
    count = int(specification["property_scene_count"])
    failures = []
    stop_id = contract["candidate_generator"]["certified_stop_candidate_id"]
    selected_valid = 0
    stop_present = 0
    fault_stop = 0
    fault_count = 0
    tamper_stop = 0
    tamper_count = 0
    nonfinite_outputs = 0
    result_digests = []
    health_fields = contract["hard_verifier"]["required_health_fields"]
    for index in range(count):
        snapshot = {
            "scene_snapshot_id": "property_{:04d}".format(index),
            "world_model_seq": index + 1,
            "snapshot_age_s": 0.05,
            "predictor_latency_ms": 1.0,
            "verifier_latency_ms": 1.0,
            "ego": {
                "x_m": -2.6,
                "y_m": 0.0,
                "yaw_rad": 0.0,
                "speed_mps": 0.8,
            },
            "actor": {
                "coordinate_m": rng.uniform(-3.35, -1.95),
                "speed_mps": rng.uniform(0.12, 1.38),
                "path_angle_deg": rng.uniform(70.5, 109.5),
            },
            "health": {field: True for field in health_fields},
        }
        injected_fault = index % 7 == 0
        if injected_fault:
            snapshot["health"][health_fields[index % len(health_fields)]] = False
            fault_count += 1
        try:
            first = core.plan(snapshot, model, contract)
            second = core.plan(snapshot, model, contract)
            if core.canonical_bytes(first) != core.canonical_bytes(second):
                failures.append([index, "NONDETERMINISTIC"])
                continue
            candidates = first["candidate_set"]["candidates"]
            if sum(item["certified_stop"] for item in candidates) == 1:
                stop_present += 1
            selected = selected_evaluation(first)
            if selected["accepted"]:
                selected_valid += 1
            if (
                injected_fault
                and first["decision"]["selected_candidate_id"] == stop_id
            ):
                fault_stop += 1
            numbers = [
                value
                for evaluation in first["evaluations"]
                for key, value in evaluation.items()
                if key
                in (
                    "minimum_clearance_m",
                    "braking_margin_m",
                    "progress_m",
                    "acceleration_cost",
                    "analytic_score",
                )
            ]
            nonfinite_outputs += sum(not core.finite(value) for value in numbers)
            result_digests.append(core.digest(first))
            if index % 31 == 0:
                tamper_count += 1
                tampered = copy.deepcopy(first["candidate_set"])
                tampered["candidates"][1]["points"][1]["x_m"] += 0.01
                tampered_result = core.evaluate_candidate_set(
                    snapshot, tampered, model, contract
                )
                if (
                    tampered_result["decision"]["selected_candidate_id"]
                    == stop_id
                    and tampered_result["decision"]["fallback_reason"]
                    == "EMERGENCY_STOP_UNVERIFIED"
                ):
                    tamper_stop += 1
                else:
                    failures.append([index, "DIGEST_TAMPER_NOT_STOPPED"])
        except Exception as exc:
            failures.append([index, type(exc).__name__ + ":" + str(exc)])
    pass_count = count - len({item[0] for item in failures})
    return {
        "schema_version": "1.0",
        "stage": "V3-03-A0",
        "report_id": "v3_03_a0_property_report_1",
        "property_seed": specification["property_seed"],
        "scene_count": count,
        "passing_scene_count": pass_count,
        "pass_rate": pass_count / count,
        "certified_stop_present_rate": stop_present / count,
        "selected_candidate_accepted_rate": selected_valid / count,
        "fault_injection_count": fault_count,
        "fault_injection_stop_rate": fault_stop / fault_count,
        "digest_tamper_count": tamper_count,
        "digest_tamper_emergency_stop_rate": tamper_stop / tamper_count,
        "nonfinite_output_count": nonfinite_outputs,
        "result_digest_set_sha256": core.digest(result_digests),
        "failure_count": len(failures),
        "failures": failures,
        "overall_pass": not failures
        and stop_present == count
        and selected_valid == count
        and fault_stop == fault_count
        and tamper_stop == tamper_count
        and nonfinite_outputs == 0,
    }


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    fixtures = load_yaml(Path(contract["fixtures"]["definition_path"]))
    core.require(
        len(fixtures["scenes"]) == contract["fixtures"][
            "deterministic_scene_count"
        ],
        "deterministic fixture count differs",
    )
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    deterministic = deterministic_fixtures(contract, model, fixtures)
    properties = property_fixtures(contract, model)
    gates = contract["offline_gates"]
    gate_values = {
        "deterministic_fixture_pass": deterministic["pass_rate"]
        >= float(gates["deterministic_fixture_pass_rate_min"]),
        "property_fixture_pass": properties["pass_rate"]
        >= float(gates["property_fixture_pass_rate_min"]),
        "stop_present_pass": properties["certified_stop_present_rate"]
        >= float(gates["stop_present_rate_min"]),
        "selected_candidate_pass": properties[
            "selected_candidate_accepted_rate"
        ]
        >= float(
            gates[
                "selected_candidate_accepted_or_emergency_stop_rate_min"
            ]
        ),
        "fault_injection_stop_pass": properties[
            "fault_injection_stop_rate"
        ]
        >= float(gates["fault_injection_stop_rate_min"]),
        "digest_tamper_stop_pass": properties[
            "digest_tamper_emergency_stop_rate"
        ]
        == 1.0,
        "nonfinite_output_pass": properties["nonfinite_output_count"]
        <= int(gates["nonfinite_output_count_max"]),
    }
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-A0",
        "summary_id": "v3_03_a0_offline_summary_1",
        "status": (
            "v3_03_a0_pure_offline_safe_planner_foundation_pass"
            if all(gate_values.values())
            else "v3_03_a0_pure_offline_safe_planner_foundation_fail"
        ),
        "upstream_predictor_sha256": contract["upstream"][
            "predictor_sha256"
        ],
        "deterministic_fixture_report_sha256": sha(
            yaml_bytes(deterministic)
        ),
        "property_report_sha256": sha(yaml_bytes(properties)),
        "gates": gate_values,
        "overall_pass": all(gate_values.values()),
        "claim": {
            "bounded_crossing_offline_verifier_ranker_fixture_pass": all(
                gate_values.values()
            ),
            "absolute_safety": False,
            "navigation_performance": False,
            "general_scene_planning": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    closure = {
        "schema_version": "1.0",
        "stage": "V3-03-A0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in SOURCE_RESOURCES],
    }
    outputs = {
        DETERMINISTIC_REPORT: yaml_bytes(deterministic),
        PROPERTY_REPORT: yaml_bytes(properties),
        SUMMARY: yaml_bytes(summary),
        CLOSURE: yaml_bytes(closure),
    }
    freeze_resources = list(outputs)
    freeze = (
        "\n".join(
            "{}  {}".format(sha(outputs[path]), path.as_posix())
            for path in freeze_resources
        )
        + "\n"
    ).encode("utf-8")
    outputs[EVIDENCE_FREEZE] = freeze
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build()
    for path, payload in outputs.items():
        if args.check:
            core.require(
                (W / path).is_file() and read(path) == payload,
                "persisted output differs: {}".format(path),
            )
        else:
            write_once(path, payload)
    summary = yaml.safe_load(outputs[SUMMARY])
    print(
        "status={} overall_pass={} summary_sha256={}".format(
            summary["status"], summary["overall_pass"], sha(outputs[SUMMARY])
        )
    )


if __name__ == "__main__":
    main()
