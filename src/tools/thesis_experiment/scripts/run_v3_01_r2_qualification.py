#!/usr/bin/env python3
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
SOURCE = W / "src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"
R2 = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"
spec = importlib.util.spec_from_file_location("v3_01_r1_runner", SOURCE)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.M = R2
driver.main()
