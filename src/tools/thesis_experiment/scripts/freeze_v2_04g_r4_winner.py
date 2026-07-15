#!/usr/bin/env python3
"""Freeze the R4 winner only after every preregistered hard gate passes."""

import argparse
import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R2_BATCH = Path(__file__).with_name("v2_04g_r2_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r2_materializer_for_r4_freeze", R2_BATCH)
_R2 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R2)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--assessment", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, default=WORKSPACE /
        "src/application/teb_mode_manager/config/v2_04g_r4_winner")
    parser.add_argument("--report", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4/v2_04g_r4_winner_freeze_report.yaml")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    assessment = yaml.safe_load(args.assessment.read_text(encoding="utf-8"))
    if prereg.get("stage") != "V2-04G-R4" or assessment.get("stage") != "V2-04G-R4":
        raise ValueError("R4 freeze boundary drifted")
    winner = assessment.get("winner_candidate_id")
    summaries = assessment.get("candidate_summaries", {})
    if not winner or summaries.get(winner, {}).get("all_hard_gates_pass") is not True:
        raise RuntimeError("R4 has no all-hard-gates winner; freeze forbidden")
    if winner not in prereg["candidate_ids"]:
        raise ValueError("R4 winner is outside preregistered candidates")
    with tempfile.TemporaryDirectory() as directory:
        runtime = _R2.materialize_candidates(args.candidate_bank, directory)[winner]
        values = {
            kind: yaml.safe_load(Path(runtime[kind]).read_text(encoding="utf-8"))
            for kind in ("supervisor", "anchor_bank", "mechanism")
        }
    outputs = {}
    for kind, value in values.items():
        frozen = copy.deepcopy(value)
        frozen["status"] = "frozen_after_v2_04g_r4_calibration"
        frozen["runtime_ready"] = False
        frozen["training_allowed"] = False
        target = Path("{}_{}.yaml".format(args.output_prefix, kind))
        _write(target, frozen)
        outputs[kind] = {"path": str(target), "sha256": _sha256(target)}
    report = {
        "schema_version": "2.0", "stage": "V2-04G-R4",
        "status": "winner_frozen", "simulation_only": True,
        "formal_result": False, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "winner_candidate_id": winner,
        "preregistration": {"path": str(args.preregistration),
                            "sha256": _sha256(args.preregistration)},
        "assessment": {"path": str(args.assessment), "sha256": _sha256(args.assessment)},
        "candidate_bank": {"path": str(args.candidate_bank),
                           "sha256": _sha256(args.candidate_bank)},
        "frozen_outputs": outputs,
        "all_hard_gates_pass": True,
        "held_out_validation_preregistration_authorized": True,
        "v2_05_authorized": False, "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    }
    _write(args.report, report)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
