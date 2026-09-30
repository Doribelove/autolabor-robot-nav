#!/usr/bin/env python3
"""Independent deterministic replay of the V3-07-B1 held-out result."""

import hashlib
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / "src/tools/thesis_experiment/scripts/assess_v3_07_b1_heldout.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    assessor = load(ASSESSOR, "v3_07_b1_result_review")
    module = assessor.load(assessor.MODULE, "v3_07_b1_result_review_module")
    value = assessor.assess(module)
    value.pop("_evidence")
    assessment_bytes = module.yaml_bytes(value)
    module.require(
        (W / module.ASSESSMENT_RELATIVE).read_bytes() == assessment_bytes,
        "B1 assessment replay differs",
    )
    passed = value["overall_pass"] is True
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-B1",
        "status": (
            "v3_07_b1_independent_heldout_review_pass"
            if passed else "v3_07_b1_independent_heldout_review_valid_negative"
        ),
        "assessment_sha256": hashlib.sha256(assessment_bytes).hexdigest(),
        "evidence_freeze_sha256": module.sha256_bytes(
            module.read_file_once(W / module.EVIDENCE_FREEZE_RELATIVE)
        ),
        "qualified_for_spatial_safe_planner_integration": passed,
        "performance_claim_authorized": False,
        "runtime_ready": False,
        "real_vehicle_authorized": False,
    }
    review_bytes = module.yaml_bytes(review)
    assessor.write_once(W / module.RESULT_REVIEW_RELATIVE, review_bytes)
    resources = [
        module.ASSESSMENT_RELATIVE, module.EVIDENCE_FREEZE_RELATIVE,
        module.RESULT_REVIEW_RELATIVE,
    ]
    terminal = (
        "\n".join(
            "{}  {}".format(
                module.sha256_bytes(module.read_file_once(W / item)), item
            )
            for item in resources
        ) + "\n"
    ).encode()
    assessor.write_once(W / module.TERMINAL_FREEZE_RELATIVE, terminal)
    print(
        "status={} review_sha256={}".format(
            review["status"], hashlib.sha256(review_bytes).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
