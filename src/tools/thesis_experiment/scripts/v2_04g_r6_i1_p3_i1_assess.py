#!/usr/bin/env python3
"""Replay the frozen C1 qualification assessor under P3-I1 stage."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-P3-I1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_p2_c1_assess.py")
_SPEC = importlib.util.spec_from_file_location("p3_i1_frozen_c1_assessor", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE

identity = _BASE.identity
replay_one = _BASE.replay_one


def main():
    return _BASE.main()


if __name__ == "__main__":
    raise SystemExit(main())
