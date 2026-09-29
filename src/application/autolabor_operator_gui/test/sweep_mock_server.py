#!/usr/bin/env python3
"""Loopback-only fixture for Qt to Python to TCP integration tests."""
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src" / "SweepDeviceControl"))
from test_sweep_test import DeviceServer

with DeviceServer(fragmented=True) as server:
    print(server.port, flush=True)
    while True:
        time.sleep(1)
