#!/usr/bin/env python3
"""Run the frozen R6 transaction control with a P1-E1 evidence identity."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-P1-E1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_runtime_control.py")
_SPEC = importlib.util.spec_from_file_location("p1_e1_frozen_r6_runtime_control", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE


def main():
    return _BASE.main()


if __name__ == "__main__":
    raise SystemExit(main())
