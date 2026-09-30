#!/usr/bin/env python3
"""Bind the frozen Q1 qualification probe to the P4-I1 evidence stage."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-P4-I1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_p2_q1_qualification_probe.py")
_SPEC = importlib.util.spec_from_file_location("p4_i1_frozen_q1_probe", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE


def main():
    return _BASE.main()


if __name__ == "__main__":
    raise SystemExit(main())
