#!/usr/bin/env python3
"""R5 wrapper over the frozen transaction-taxonomy readiness listener."""

import importlib.util
from pathlib import Path


STAGE = "V2-04G-R5"
SOURCE = Path(__file__).with_name("v2_04g_r3_r1_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r3_r1_frozen_listener_for_r5", SOURCE
)
_FROZEN = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_FROZEN)
_FROZEN.STAGE = STAGE
_FROZEN._FROZEN.STAGE = STAGE


if __name__ == "__main__":
    raise SystemExit(_FROZEN.main())
