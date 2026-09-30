#!/usr/bin/env python3
"""Independently review V3-07-A0 spatial candidate foundation."""

import hashlib
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_07_a0_spatial_candidate_foundation.py"


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_07_a0_builder_review", BUILDER)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    b = load_builder()
    left, right, report = b.build()
    b.spatial().require(b.read(b.LEFT) == b.dump(left), "left candidate differs")
    b.spatial().require(b.read(b.RIGHT) == b.dump(right), "right candidate differs")
    b.spatial().require(b.read(b.REPORT) == b.dump(report), "A0 report differs")
    b.spatial().require(
        report["overall_pass"] is True
        and report["execution_ready"] is False
        and report["prediction_coverage"]["existing_model_authorized_for_spatial_candidates"] is False,
        "A0 conclusion differs",
    )
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-A0",
        "status": "v3_07_a0_independent_offline_foundation_review_pass",
        "report_sha256": b.sha(b.read(b.REPORT)),
        "evidence_freeze_sha256": b.sha(b.read(b.FREEZE)),
        "exact_candidate_reproduction": True,
        "execution_ready": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = b.dump(review)
    b.write_once(b.REVIEW, payload)
    terminal = (
        "\n".join(
            "{}  {}".format(b.sha(b.read(path)), path)
            for path in [b.LEFT, b.RIGHT, b.REPORT, b.FREEZE, b.REVIEW]
        )
        + "\n"
    ).encode()
    b.write_once(b.TERMINAL, terminal)
    print("status={} review_sha256={}".format(review["status"], hashlib.sha256(payload).hexdigest()))


if __name__ == "__main__":
    main()
