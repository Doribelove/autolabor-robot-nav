#!/usr/bin/env python3
"""Run the frozen R6 activation listener with a P2-C1-R1 identity."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-P2-C1-R1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location("p2_c1_r1_frozen_r6_listener", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE
_BASE._BASE.STAGE = STAGE  # pylint: disable=protected-access
_BASE._FROZEN.STAGE = STAGE  # pylint: disable=protected-access


def main():
    return _BASE.main()


if __name__ == "__main__":
    raise SystemExit(main())
