#!/usr/bin/env python3
"""Materialize the preregistered V2-04C dynamic-fusion qualification retry."""

import argparse
import copy
import hashlib
from pathlib import Path

import yaml


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/robot/robot_ws_base_rl"))
    args = parser.parse_args()
    root = args.workspace.resolve()
    amendment_path = (
        root / "config/thesis_experiments/v2/v2_04c_ttc_qualification_r4_amendment.yaml"
    )
    amendment = yaml.safe_load(amendment_path.read_text(encoding="utf-8"))
    if (
        amendment.get("stage") != "V2-04C-Q-R4"
        or amendment.get("simulation_only") is not True
        or amendment.get("runtime_ready") is not False
        or amendment.get("training_allowed") is not False
        or amendment.get("real_vehicle_use_forbidden") is not True
    ):
        raise ValueError("V2-04C R4 boundary drifted")
    for group in ("failed_r3_evidence", "fusion_resources"):
        for name, item in amendment[group].items():
            if not isinstance(item, dict):
                continue
            path = root / item["path"]
            if not path.is_file() or _sha256(path) != item["sha256"]:
                raise ValueError("V2-04C R4 resource {}.{} drifted".format(group, name))
    r3_path = root / amendment["failed_r3_evidence"]["qualification_plan"]["path"]
    plan = copy.deepcopy(yaml.safe_load(r3_path.read_text(encoding="utf-8")))
    plan.update({
        "stage": "V2-04C-Q-R4",
        "plan_id": "fam_teb_v2_04c_ttc_qualification_r4_plan_1",
        "contract": {
            "path": amendment_path.relative_to(root).as_posix(),
            "sha256": _sha256(amendment_path),
        },
    })
    candidate = plan["candidates"][0]
    old_id = candidate["candidate_id"]
    candidate["candidate_id"] = "v2_04c-q-r4-balanced-center"
    candidate["retry_provenance"] = {
        "r3_candidate_id": old_id,
        "single_changed_factor": "tracked_dynamic_costmap_spatial_fusion",
        "episode_timeout_s": 80.0,
    }
    plan["claims"] = {
        "qualification_only": True,
        "dynamic_fusion_only_retry": True,
        "common_random_numbers_with_r1_r2_r3": True,
        "refinement_selection_used": False,
    }
    output = root / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification_r4_plan.yaml"
    output.write_text(yaml.safe_dump(plan, sort_keys=False), encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
