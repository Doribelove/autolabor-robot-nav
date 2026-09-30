#!/usr/bin/env python3
"""Independently reproduce and freeze the V3-06-D0 attribution."""

import hashlib
import importlib.util
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_06_d0_c6_failure_attribution.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_06_d0_builder_for_review", BUILDER
    )
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    b = load_builder()
    expected = b.dump(b.build())
    actual = b.read(b.REPORT)
    b.core.require(actual == expected, "D0 report replay differs")
    report = yaml.safe_load(actual)
    b.core.require(
        report["c6_terminal_result"]["qualified"] is False
        and report["grid_summary"][
            "nearby_speed_search_has_material_room"
        ]
        is False
        and report["diagnosis"]["straight_creep_family_saturated"] is True
        and report["diagnosis"][
            "new_spatial_intervention_data_and_calibration_required"
        ]
        is True,
        "D0 interpretation differs",
    )
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-D0",
        "status": (
            "v3_06_d0_independent_offline_attribution_review_pass"
        ),
        "report_sha256": b.sha(actual),
        "evidence_freeze_sha256": b.sha(b.read(b.FREEZE)),
        "exact_reproduction": True,
        "formal_result": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = b.dump(review)
    b.write_once(b.REVIEW, payload)
    terminal_resources = [b.REPORT, b.FREEZE, b.REVIEW]
    terminal = (
        "\n".join(
            "{}  {}".format(b.sha(b.read(path)), path)
            for path in terminal_resources
        )
        + "\n"
    ).encode()
    b.write_once(b.TERMINAL, terminal)
    print(
        "status={} review_sha256={}".format(
            review["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
