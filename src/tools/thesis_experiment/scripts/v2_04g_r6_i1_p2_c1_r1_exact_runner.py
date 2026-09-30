#!/usr/bin/env python3
"""Execute the unique P2-C1-R1 repaired Gazebo calibration release.

The behavior, selection rules, qualification gate, process lifecycle and raw
replay logic are byte-frozen from C1.  This wrapper changes only the independent
stage identity, output roots, evidence adapters and fail-closed release
validator.  The R1 schedule itself binds every timeout to its compiled scene.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_ROOT = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))

from thesis_experiment.v2_04g_r6_i1_p2_c1_r1_release import (  # noqa: E402
    C1R1ReleaseError,
    validate_release,
)


STAGE = "V2-04G-P2-C1-R1"
OUTPUT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_execution"
)
OUTPUT_ROOT = WORKSPACE / OUTPUT_ROOT_RELATIVE
ATTEMPTS_ROOT = OUTPUT_ROOT / "attempts"
JOURNAL_ROOT = OUTPUT_ROOT / "journals"
CONSUMPTION_ROOT = OUTPUT_ROOT / "consumption"
STAGE_REPORT = OUTPUT_ROOT / "p2_c1_r1_stage_report.yaml"
SKIPPED_LEDGER = OUTPUT_ROOT / "p2_c1_r1_skipped_identities.yaml"
FAILURE_INDEX = OUTPUT_ROOT / "p2_c1_r1_failure_index.yaml"
NORMALIZED_MATRIX = OUTPUT_ROOT / "normalized_calibration_matrix.json"
REPLAY_REPORT = OUTPUT_ROOT / "p2_c1_r1_raw_replay_report.yaml"
ASSESSMENT_REPORT = OUTPUT_ROOT / "p2_c1_r1_calibration_selection.yaml"

NAVIGATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_navigation_episode.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_activation_probe_listener.py"
)
FIXED_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_fixed_snapshot.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_assess.py"
)
FROZEN_RUNNER = Path(__file__).with_name(
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
FROZEN_ASSESSOR = Path(__file__).with_name(
    "v2_04g_r6_i1_p2_c1_r1_assess.py"
)


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_BASE = _load("_p2_c1_r1_frozen_c1_runner", FROZEN_RUNNER)
ASSESSOR = _load("_p2_c1_r1_assessor", FROZEN_ASSESSOR)


def _fixed_startup_config(row, closure):
    """Resolve both semantic and direct circle probes through their startup key."""
    method = _BASE._method_for_row(row)  # pylint: disable=protected-access
    binding = (
        method["fixed_startup_config"]
        if "fixed_startup_config" in method
        else method["materialized_config"]
    )
    payload = _BASE.P1._rehash_relative(  # pylint: disable=protected-access
        Path(binding["path"]), closure
    )
    if _BASE._sha(payload) != binding["sha256"]:  # pylint: disable=protected-access
        raise _BASE.C1ExecutionError("fixed startup config differs")
    return str(WORKSPACE / binding["path"])


def _configure_frozen_runner() -> None:
    _BASE.STAGE = STAGE
    _BASE.OUTPUT_ROOT_RELATIVE = OUTPUT_ROOT_RELATIVE
    _BASE.OUTPUT_ROOT = OUTPUT_ROOT
    _BASE.ATTEMPTS_ROOT = ATTEMPTS_ROOT
    _BASE.JOURNAL_ROOT = JOURNAL_ROOT
    _BASE.CONSUMPTION_ROOT = CONSUMPTION_ROOT
    _BASE.STAGE_REPORT = STAGE_REPORT
    _BASE.SKIPPED_LEDGER = SKIPPED_LEDGER
    _BASE.FAILURE_INDEX = FAILURE_INDEX
    _BASE.NORMALIZED_MATRIX = NORMALIZED_MATRIX
    _BASE.REPLAY_REPORT = REPLAY_REPORT
    _BASE.ASSESSMENT_REPORT = ASSESSMENT_REPORT
    _BASE.NAVIGATION_RELATIVE = NAVIGATION_RELATIVE
    _BASE.QUALIFICATION_RELATIVE = QUALIFICATION_RELATIVE
    _BASE.CONTROL_RELATIVE = CONTROL_RELATIVE
    _BASE.LISTENER_RELATIVE = LISTENER_RELATIVE
    _BASE.FIXED_RELATIVE = FIXED_RELATIVE
    _BASE.ASSESSOR_RELATIVE = ASSESSOR_RELATIVE
    _BASE.ASSESSOR = ASSESSOR
    _BASE.validate_release = validate_release
    _BASE._fixed_config = _fixed_startup_config  # pylint: disable=protected-access


_configure_frozen_runner()

CIRCLE_METHODS = _BASE.CIRCLE_METHODS
FIXED_METHODS = _BASE.FIXED_METHODS
P1 = _BASE.P1
C1ExecutionError = _BASE.C1ExecutionError


def configure_static_bindings(methods, closure) -> None:
    """Configure pure command builders for independent offline review."""
    _configure_frozen_runner()
    _BASE.METHOD_BINDINGS = {
        row["method_id"]: row for row in methods["methods"]
    }
    _BASE._CURRENT_CLOSURE = dict(closure)  # pylint: disable=protected-access


def identity(row):
    return _BASE._identity(row)  # pylint: disable=protected-access


def profile_id(row):
    return _BASE._profile_id(row)  # pylint: disable=protected-access


def semantic(row):
    return _BASE._semantic(row)  # pylint: disable=protected-access


def runtime(row, closure):
    return _BASE._runtime(row, closure)  # pylint: disable=protected-access


def base_command(row, scene, world_path, runtime_binding, executables):
    return _BASE._base_command(  # pylint: disable=protected-access
        row, scene, world_path, runtime_binding, executables
    )


def transaction_command(row, runtime_binding, executables):
    return _BASE._transaction_command(  # pylint: disable=protected-access
        row, runtime_binding, executables
    )


def episode_command(row, instance_path, digest, output, executables):
    return _BASE._episode_command(  # pylint: disable=protected-access
        row, instance_path, digest, output, executables
    )


def probe_command(row, output, executables):
    return _BASE._probe_command(row, output, executables)  # pylint: disable=protected-access


def execute(expected_release_sha256):
    _configure_frozen_runner()
    return _BASE.execute(expected_release_sha256)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; use the offline reviewer for validation")
    try:
        report = execute(args.release_sha256)
    except (C1R1ReleaseError, C1ExecutionError, P1.P1ExecutionError) as exc:
        print("P2-C1-R1 terminal failure: {}".format(exc), file=sys.stderr)
        return 2
    print(
        yaml.safe_dump(
            {
                "status": report["status"],
                "consumed": report["evidence_units_consumed"],
                "completed": report["evidence_units_completed"],
                "skipped": report["evidence_units_skipped"],
                "selected_circle_candidate": report["selected_circle_candidate"],
                "selected_primary_fair_baseline": report[
                    "selected_primary_fair_baseline"
                ],
                "held_out_test_design_eligible": report[
                    "held_out_test_design_eligible"
                ],
            },
            sort_keys=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
