#!/usr/bin/env python3
"""Replay frozen C1 logic under the independent P2-C1-R1 stage identity."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-P2-C1-R1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_p2_c1_assess.py")
_SPEC = importlib.util.spec_from_file_location("p2_c1_r1_frozen_c1_assessor", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE

identity = _BASE.identity
replay_one = _BASE.replay_one
build_assessment = _BASE.build_assessment


def main():
    return _BASE.main()


if __name__ == "__main__":
    raise SystemExit(main())
