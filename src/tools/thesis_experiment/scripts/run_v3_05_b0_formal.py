#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
V3 = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_b0.py"
spec = importlib.util.spec_from_file_location("v3_05_b0_runner", SOURCE)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.M = V3
driver.main()
