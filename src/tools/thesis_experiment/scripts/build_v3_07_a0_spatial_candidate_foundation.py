#!/usr/bin/env python3
"""Build the pure-offline V3-07-A0 spatial candidate foundation."""

import hashlib
import importlib.util
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path("config/thesis_experiments/v3/v3_07_a0_spatial_candidate_foundation_contract.yaml")
PLAN = Path("experiments/manifests/v3/v3_07_a0_spatial_candidate_foundation_plan.yaml")
MODULE = Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py")
ROOT = Path("artifacts/v3/spatial/v3_07_a0")
LEFT = ROOT / "v3_07_a0_spatial_left_v120.yaml"
RIGHT = ROOT / "v3_07_a0_spatial_right_v120.yaml"
REPORT = ROOT / "v3_07_a0_offline_foundation_report.yaml"
FREEZE = ROOT / "v3_07_a0_evidence_freeze.sha256"
REVIEW = ROOT / "v3_07_a0_independent_review.yaml"
TERMINAL = ROOT / "v3_07_a0_terminal_freeze.sha256"


def spatial():
    spec = importlib.util.spec_from_file_location("v3_07_spatial", W / MODULE)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load(path):
    value = yaml.safe_load(read(path))
    if not isinstance(value, dict):
        raise RuntimeError("YAML root differs")
    return value


def dump(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True, width=1000000).encode()


def write_once(path, payload):
    target = W / path
    if target.exists():
        if target.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)


def receipt(path):
    payload = read(path)
    return {"path": path.as_posix(), "sha256": sha(payload), "size_bytes": len(payload)}


def build():
    s = spatial()
    contract = load(CONTRACT)
    plan = load(PLAN)
    s.require(
        contract["stage"] == "V3-07-A0"
        and contract["status"] == "offline_foundation_frozen_before_generation"
        and plan["status"] == "frozen_before_offline_generation",
        "A0 contract differs",
    )
    s.require(
        all(value is False for value in contract["authority"].values()),
        "A0 authority must remain zero",
    )
    left = s.generate("spatial_left_v120", 1.0, contract)
    right = s.generate("spatial_right_v120", -1.0, contract)
    left_audit = s.audit(left, contract)
    right_audit = s.audit(right, contract)
    mirror_error = max(
        max(
            abs(float(a["x_m"]) - float(b["x_m"])),
            abs(float(a["y_m"]) + float(b["y_m"])),
            abs(float(a["yaw_rad"]) + float(b["yaw_rad"])),
            abs(float(a["steering_angle_rad"]) + float(b["steering_angle_rad"])),
        )
        for a, b in zip(left["points"], right["points"])
    )
    rules = contract["offline_gates"]
    audits = [left_audit, right_audit]
    gates = {
        "candidate_count_pass": len(audits) == int(rules["candidate_count"]),
        "mirror_pass": mirror_error <= float(rules["mirror_position_error_max_m"]),
        "bicycle_integration_pass": max(
            row["maximum_bicycle_integration_error_m"] for row in audits
        )
        <= float(rules["bicycle_integration_error_max_m"]),
        "speed_pass": sum(row["speed_limit_violation_count"] for row in audits)
        <= int(rules["speed_limit_violation_count_max"]),
        "acceleration_pass": sum(
            row["acceleration_limit_violation_count"] for row in audits
        )
        <= int(rules["acceleration_limit_violation_count_max"]),
        "steering_pass": sum(row["steering_limit_violation_count"] for row in audits)
        <= int(rules["steering_limit_violation_count_max"]),
        "steering_rate_pass": sum(
            row["steering_rate_limit_violation_count"] for row in audits
        )
        <= int(rules["steering_rate_limit_violation_count_max"]),
        "curvature_pass": sum(row["curvature_limit_violation_count"] for row in audits)
        <= int(rules["curvature_limit_violation_count_max"]),
        "corridor_pass": sum(row["corridor_violation_count"] for row in audits)
        <= int(rules["corridor_violation_count_max"]),
        "terminal_pass": all(
            row["terminal_goal_x_reached"]
            and row["terminal_center_error_m"]
            <= float(contract["corridor"]["terminal_center_error_max_m"])
            and row["terminal_yaw_error_rad"]
            <= float(contract["corridor"]["terminal_yaw_error_max_rad"])
            for row in audits
        ),
    }
    passed = all(gates.values())
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-A0",
        "status": (
            "spatial_candidate_offline_foundation_pass"
            if passed
            else "spatial_candidate_offline_foundation_fail"
        ),
        "candidate_audits": {
            "spatial_left_v120": left_audit,
            "spatial_right_v120": right_audit,
        },
        "mirror_error": mirror_error,
        "gates": gates,
        "overall_pass": passed,
        "prediction_coverage": contract["prediction_coverage"],
        "execution_ready": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "claim": {
            "offline_ackermann_candidate_foundation": passed,
            "interaction_prediction_qualified": False,
            "runtime_safety_qualified": False,
            "performance_improvement": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
    }
    return left, right, report


def main():
    left, right, report = build()
    write_once(LEFT, dump(left))
    write_once(RIGHT, dump(right))
    write_once(REPORT, dump(report))
    sources = [
        CONTRACT,
        PLAN,
        Path("artifacts/v3/analysis/v3_06_d0/v3_06_d0_terminal_freeze.sha256"),
        Path("config/thesis_experiments/v3/v3_00_foundation_contract.yaml"),
        MODULE,
        Path("src/tools/thesis_experiment/scripts/build_v3_07_a0_spatial_candidate_foundation.py"),
        Path("src/tools/thesis_experiment/scripts/review_v3_07_a0_spatial_candidate_foundation.py"),
        Path("src/tools/thesis_experiment/tests/test_v3_07_a0_spatial_candidate_foundation.py"),
        LEFT,
        RIGHT,
        REPORT,
    ]
    freeze = ("\n".join("{}  {}".format(sha(read(path)), path) for path in sources) + "\n").encode()
    write_once(FREEZE, freeze)
    print("status={} report_sha256={}".format(report["status"], sha(dump(report))))


if __name__ == "__main__":
    main()
