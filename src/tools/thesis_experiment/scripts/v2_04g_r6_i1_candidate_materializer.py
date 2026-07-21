#!/usr/bin/env python3
"""Materialize the two R6-I1 runtime profiles with one exact field diff."""

import argparse
import copy
import hashlib
import os
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I1"
DESIGN_BANK = WORKSPACE / (
    "experiments/manifests/v2/preregistrations/"
    "v2_04g_r6_semantic_candidates.yaml"
)
SOURCE_ROOT = WORKSPACE / (
    "artifacts/v2/calibration/v2_04g_r5/runtime_candidate_configs/"
    "r5_ttc_control_h500"
)
SOURCE_HASHES = {
    "supervisor.yaml": "a7ab5613e7d7b7a8a943ab8dba288d8a3ad86257ba856bd35b84b01c4476a4cd",
    "anchor_bank.yaml": "5a3ecfc4c9dc8b6bfbb90ad5f036250396eceb99bd0154d1634012bbc4a4a72f",
    "mechanism.yaml": "f3deff87e322e1d009fc30bfef6abb4b44770c455e3db61ff8e22afa041eea5e",
}
DESIGN_BANK_SHA256 = (
    "6732b132067591f71c365463b5677bbe5dc161b9fb5b891ebac8a74b70c63c5d"
)
FROZEN_DYNAMIC = {
    "minimum_track_confidence": 0.45,
    "predicted_ttc_max_s": 5.0,
    "closest_approach_max_m": 1.35,
    "robot_radius_m": 0.62,
    "minimum_relative_speed_mps": 0.05,
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _load_inputs():
    if sha256(DESIGN_BANK) != DESIGN_BANK_SHA256:
        raise ValueError("R6 design candidate bank hash drifted")
    bank = yaml.safe_load(DESIGN_BANK.read_text(encoding="utf-8"))
    candidates = bank.get("candidates")
    if not isinstance(candidates, list) or len(candidates) != 2:
        raise ValueError("R6 design candidate set drifted")
    mapping = {
        row["candidate_id"]: row["conflict_estimator_id"]
        for row in candidates
    }
    expected = {
        "r6_semantics_legacy_control":
            "legacy_class_conditioned_geometry_v1",
        "r6_semantics_circle_contact":
            "shared_circle_envelope_first_contact_v1",
    }
    if mapping != expected:
        raise ValueError("R6 semantic factor levels drifted")
    payloads = {}
    for name, digest in SOURCE_HASHES.items():
        path = SOURCE_ROOT / name
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != digest:
            raise ValueError("frozen R5 source {} drifted".format(name))
        payloads[name] = payload
    return mapping, payloads


def materialize(output_root):
    output = Path(output_root).resolve()
    allowed = (
        WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/"
        "runtime_candidate_configs"
    ).resolve()
    if output != allowed:
        raise ValueError("R6-I1 runtime output root drifted")
    if output.exists():
        raise FileExistsError("R6-I1 runtime profiles already exist")
    mapping, payloads = _load_inputs()
    base = yaml.safe_load(payloads["supervisor.yaml"].decode("utf-8"))
    base["stage"] = STAGE
    base["profile_id"] = "fam_teb_v2_04g_r6_i1_semantic_supervisor"
    base["dynamic"] = copy.deepcopy(FROZEN_DYNAMIC)
    result = {}
    try:
        for candidate_id, estimator_id in mapping.items():
            directory = output / candidate_id
            directory.mkdir(parents=True, exist_ok=False)
            supervisor = copy.deepcopy(base)
            supervisor["dynamic"]["conflict_estimator_id"] = estimator_id
            supervisor_payload = yaml.safe_dump(
                supervisor, sort_keys=False
            ).encode("utf-8")
            _atomic_write(directory / "supervisor.yaml", supervisor_payload)
            _atomic_write(
                directory / "anchor_bank.yaml", payloads["anchor_bank.yaml"]
            )
            _atomic_write(
                directory / "mechanism.yaml", payloads["mechanism.yaml"]
            )
            result[candidate_id] = {
                name: {
                    "path": str((directory / name).relative_to(WORKSPACE)),
                    "sha256": sha256(directory / name),
                }
                for name in (
                    "supervisor.yaml", "anchor_bank.yaml", "mechanism.yaml"
                )
            }
    except BaseException:
        # Materialization is a pre-execution build product.  A partial output
        # cannot be accepted by the reviewer and is left visible for audit.
        raise
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-root",
        default=str(
            WORKSPACE
            / "artifacts/v2/integration/v2_04g_r6_i1/"
              "runtime_candidate_configs"
        ),
    )
    args = parser.parse_args()
    print(yaml.safe_dump(materialize(args.output_root), sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
