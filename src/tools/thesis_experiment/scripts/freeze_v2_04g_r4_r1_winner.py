#!/usr/bin/env python3
"""Freeze an R4-R1 repair winner only after every preregistered gate passes."""

import argparse
import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MATERIALIZER = Path(__file__).with_name("v2_04g_r4_r1_candidate_materializer.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_r1_materializer_for_freeze", MATERIALIZER)
_MAT = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MAT)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--assessment", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, default=WORKSPACE /
        "src/application/teb_mode_manager/config/v2_04g_r4_r1_winner")
    parser.add_argument("--report", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1/v2_04g_r4_r1_winner_freeze_report.yaml")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    assessment = yaml.safe_load(args.assessment.read_text(encoding="utf-8"))
    bank = yaml.safe_load(args.candidate_bank.read_text(encoding="utf-8"))
    if not (prereg.get("stage") == "V2-04G-R4-R1"
            and assessment.get("stage") == "V2-04G-R4-R1"
            and bank.get("stage") == "V2-04G-R4-R1"):
        raise ValueError("R4-R1 freeze boundary drifted")
    winner = assessment.get("winner_candidate_id")
    summary = assessment.get("candidate_summaries", {}).get(winner, {})
    eligible = {row["candidate_id"]: row["winner_eligible"] for row in bank["candidates"]}
    if not (winner and summary.get("all_hard_gates_pass") is True
            and assessment.get("decision", {}).get("freeze_authorized") is True
            and eligible.get(winner) is True):
        raise RuntimeError("R4-R1 has no eligible all-hard-gates winner; freeze forbidden")
    with tempfile.TemporaryDirectory() as directory:
        runtime = _MAT.materialize_candidates(args.candidate_bank, directory)[winner]
        values = {kind: yaml.safe_load(Path(runtime[kind]).read_text(encoding="utf-8"))
                  for kind in ("supervisor", "anchor_bank", "mechanism")}
    outputs = {}
    for kind, value in values.items():
        frozen = copy.deepcopy(value)
        frozen["status"] = "frozen_after_v2_04g_r4_r1_calibration"
        frozen["runtime_ready"] = False
        frozen["training_allowed"] = False
        target = Path("{}_{}.yaml".format(args.output_prefix, kind))
        _write(target, frozen)
        outputs[kind] = {"path": str(target), "sha256": _sha256(target)}
    report = {
        "schema_version": "2.0", "stage": "V2-04G-R4-R1",
        "status": "winner_frozen", "version": "v2_04g_r4_r1_winner_1",
        "simulation_only": True, "formal_result": False, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "winner_candidate_id": winner,
        "preregistration": {"path": str(args.preregistration),
                            "sha256": _sha256(args.preregistration)},
        "assessment": {"path": str(args.assessment), "sha256": _sha256(args.assessment)},
        "candidate_bank": {"path": str(args.candidate_bank),
                           "sha256": _sha256(args.candidate_bank)},
        "frozen_outputs": outputs, "all_hard_gates_pass": True,
        "held_out_validation_preregistration_authorized": True,
        "v2_05_authorized": False, "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    }
    _write(args.report, report)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
