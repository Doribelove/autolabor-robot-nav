#!/usr/bin/env python3
"""R4-R1 wrapper over the frozen R4 atomic readiness listener."""

import importlib.util
from pathlib import Path


SOURCE = Path(__file__).with_name("v2_04g_r4_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_frozen_listener_r4_r1", SOURCE)
_R4 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R4)
_R4._FROZEN.STAGE = "V2-04G-R4-R1"
_R4._FROZEN._FROZEN.STAGE = "V2-04G-R4-R1"


if __name__ == "__main__":
    raise SystemExit(_R4._FROZEN.main())
