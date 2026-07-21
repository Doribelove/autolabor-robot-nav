#!/usr/bin/env python3
"""V2-04G-R4 wrapper over the frozen taxonomy plus atomic-input gate."""

import importlib.util
from pathlib import Path


SOURCE = Path(__file__).with_name("v2_04g_r3_r1_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r3_r1_frozen_listener_r4", SOURCE)
_FROZEN = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_FROZEN)
_FROZEN.STAGE = "V2-04G-R4"
_FROZEN._FROZEN.STAGE = "V2-04G-R4"


if __name__ == "__main__":
    raise SystemExit(_FROZEN.main())
