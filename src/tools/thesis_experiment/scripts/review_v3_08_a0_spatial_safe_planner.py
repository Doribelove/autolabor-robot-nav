#!/usr/bin/env python3
"""Independent deterministic replay of V3-08-A0."""

import hashlib
import importlib.util
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_08_a0_spatial_safe_planner.py"
REVIEW = Path(
    "artifacts/v3/planner/v3_08_a0/v3_08_a0_independent_review.yaml"
)
TERMINAL = Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_terminal_freeze.sha256")


def main():
    spec = importlib.util.spec_from_file_location("v3_08_a0_builder_review", BUILDER)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    planner, report = builder.build()
    planner_bytes = builder.payload(planner)
    report_bytes = builder.payload(report)
    if (W / builder.PLANNER).read_bytes() != planner_bytes:
        raise RuntimeError("planner replay differs")
    frozen_report_bytes = (W / builder.REPORT).read_bytes()
    frozen_report = yaml.safe_load(frozen_report_bytes)
    if not builder.reports_equivalent_except_measured_latency(
        frozen_report, report
    ):
        raise RuntimeError("report replay differs")
    passed = frozen_report["overall_pass"] is True and report["overall_pass"] is True
    review = {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-08-A0",
        "status": (
            "v3_08_a0_independent_offline_review_pass"
            if passed else "v3_08_a0_independent_offline_review_fail"
        ),
        "planner_sha256": hashlib.sha256(planner_bytes).hexdigest(),
        "report_sha256": hashlib.sha256(frozen_report_bytes).hexdigest(),
        "measured_latency_replayed_under_gate": True,
        "simulation_integration_qualified": passed,
        "performance_claim_authorized": False,
        "runtime_ready": False,
        "real_vehicle_authorized": False,
    }
    review_bytes = yaml.safe_dump(review, sort_keys=False).encode()
    builder.write_once(W / REVIEW, review_bytes)
    resources = [builder.PLANNER, builder.REPORT, builder.FREEZE, REVIEW]
    terminal = (
        "\n".join(
            "{}  {}".format(hashlib.sha256((W / item).read_bytes()).hexdigest(), item)
            for item in resources
        ) + "\n"
    ).encode()
    builder.write_once(W / TERMINAL, terminal)
    print(
        "status={} review_sha256={}".format(
            review["status"], hashlib.sha256(review_bytes).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
