#!/usr/bin/env python3
"""Read-only machine review of the exact R6-I5 scene materialization."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


sys.dont_write_bytecode = True
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_scene_materializer as materializer


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(materializer.DEFAULT_WORKSPACE))
    parser.add_argument("--derivation-sha256", required=True)
    args = parser.parse_args()
    try:
        receipt = materializer.review_materialization(
            Path(args.workspace), args.derivation_sha256
        )
        print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
        return 0
    except (materializer.R6I5SceneMaterializationError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "status": "invalid",
                    "stage": materializer.STAGE,
                    "error": str(exc),
                    "evidence_units_consumed": 0,
                    "release_created": False,
                    "attempt_or_journal_created": False,
                    "ros_or_gazebo_started": False,
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
