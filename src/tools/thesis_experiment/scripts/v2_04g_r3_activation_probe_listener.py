#!/usr/bin/env python3
"""V2-04G-R3 identity wrapper over the frozen R2-R1 taxonomy listener."""

import importlib.util
from pathlib import Path


SOURCE = Path(__file__).with_name(
    "v2_04g_r2_r1_activation_probe_listener.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r2_r1_frozen_taxonomy_listener", SOURCE
)
_FROZEN = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_FROZEN)
_FROZEN.STAGE = "V2-04G-R3"


if __name__ == "__main__":
    raise SystemExit(_FROZEN.main())
